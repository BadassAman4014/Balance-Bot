import math

# Initialize dummy's yaw variable
dummy_yaw = 0.0

def sysCall_init():
    sim = require('sim')
    global motor1, motor2, wheel1, wheel2, dummy, body, mOr1, mOr2, bOr, xRef, D_Ref, Ref, startSimulation, stopSim
    global Kp_outer, Ki_outer, Kd_outer, Kp_inner, Ki_inner, Kd_inner, Imax_outer, Imax_inner, ref
    global prevErr_outer, prevErr_inner, I_outer, I_inner, pid_inner, turn_velocity, PrevBodyPos, speedTurn, dummy_yaw

    # Speed for turning
    speedTurn = 8
    # Getting handles for objects
    motor1 = sim.getObject("/body/right_joint")
    motor2 = sim.getObject("/body/left_joint")
    dummy = sim.getObject("/XYZCameraProxy")
    body = sim.getObject("/body")

    # Set initial dummy yaw angle based on current body orientation
    PrevBodyPos = sim.getEulerAnglesFromMatrix(sim.getObjectMatrix(body, dummy))
    dummy_yaw = PrevBodyPos[2]  # Initial yaw

    # Set reference positions for x and y axis
    xRef = sim.getObjectPosition(body, dummy)[1]
    D_Ref = sim.getObjectPosition(body, dummy)
    Ref = sim.getObjectPosition(body, dummy)
    
    # Initialize start/stop flags
    startSimulation = True
    stopSim = False

    # Outer PID (position control)
    Kp_outer = 25.0
    Ki_outer = 1.5
    Kd_outer = 0.54
    Imax_outer = 100

    # Inner PID (velocity control)
    Kp_inner = 20.0
    Ki_inner = 0.8
    Kd_inner = 2.5
    Imax_inner = 100

    ref = 0.0

    # Initialize PID variables
    prevErr_outer = 0
    prevErr_inner = 0
    I_outer = 0
    I_inner = 0
    pid_inner = 0
    turn_velocity = 0

def sysCall_actuation():
    message, data, data2 = sim.getSimulatorMessage()
    global ref
    drive_needed = False  # Flag to check if drive() needs to be called
    
    if message == sim.message_keypress:
        # Handle Prismatic Joint velocity based on key 'q' and 'e' (closing and opening)
        if data[0] == 113:  # 'q' key for closing the gripper
            sim.setJointTargetVelocity(sim.getObject('/Prismatic_joint'), -0.30)  # Set a negative velocity to close
            drive_needed = True
        elif data[0] == 101:  # 'e' key for opening the gripper
            sim.setJointTargetVelocity(sim.getObject('/Prismatic_joint'), 0.30)   # Set a positive velocity to open
            drive_needed = True
            
        # Handle Arm Joint velocity based on key 'w' and 's' (raising and lowering)
        if data[0] == 119:  # 'w' key for raising the gripper
            sim.setJointTargetVelocity(sim.getObject('/arm_joint'), -8)             # Positive velocity to raise
            drive_needed = True
        elif data[0] == 115:  # 's' key for lowering the gripper
            sim.setJointTargetVelocity(sim.getObject('/arm_joint'), 8)            # Negative velocity to lower
            drive_needed = True
            
        # Handle reference and drive functions based on arrow keys
        if data[0] == 2008:  # forward up arrow
            ref += 0.002  # Increase reference
            drive_needed = True
        elif data[0] == 2007:  # backward down arrow
            ref -= 0.002  # Decrease reference
            drive_needed = True
        elif data[0] == 2010:  # left arrow key
            Ldrive()  # Execute left drive function
        elif data[0] == 2009:  # right arrow key
            Rdrive()  # Execute right drive function
    else:
        # If no key is pressed, stop the joints by setting velocity to 0
        sim.setJointTargetVelocity(sim.getObject('/Prismatic_joint'), 0)
        sim.setJointTargetVelocity(sim.getObject('/arm_joint'), 0)
        drive_needed = True  # Ensure default drive function is executed
    
    # Call drive() only if drive_needed is True
    if drive_needed:
        drive()

def sysCall_sensing():
    drive()

def sysCall_cleanup():
    pass

def drive():
    global ref, Imax_outer, Imax_inner, Kp_outer, Ki_outer, Kd_outer, Kp_inner, Ki_inner, Kd_inner
    global prevErr_outer, prevErr_inner, I_outer, I_inner, pid_inner
    global dummy_yaw, PrevBodyPos
    dummy_yaw = sim.getObjectPosition(body, dummy)[2]
    
    # Get the current body position (Euler angles)
    BodyPos = sim.getEulerAnglesFromMatrix(sim.getObjectMatrix(body, dummy))

    # Calculate the change in yaw
    yaw_change = BodyPos[2] - PrevBodyPos[2]

    # Update dummy_yaw based on the change in the body yaw
    dummy_yaw += yaw_change

    # Update BodyPos for the next iteration
    PrevBodyPos = BodyPos
    
    # Use dummy_yaw for balancing calculations instead of manipulating the dummy's orientation
    yaw = dummy_yaw
    pitch = BodyPos[0]
    roll = BodyPos[1]
    xErr = xRef - sim.getObjectPosition(body, dummy)[0]

    # Adjust roll based on pitch and yaw
    roll -= pitch * math.sin(yaw)
    pitch += roll * math.sin(yaw)

    ## Outer PID loop (position control)
    err_outer = (ref + xErr) - roll
    P_outer = Kp_outer * err_outer
    I_outer += Ki_outer * err_outer
    D_outer = Kd_outer * (err_outer - prevErr_outer)
    ref_velocity = P_outer + I_outer + D_outer
    prevErr_outer = err_outer

    # Integral windup handling for outer loop
    if abs(I_outer) > Imax_outer:
        I_outer = Imax_outer * (I_outer / abs(I_outer))

    ## Inner PID loop (velocity control)
    err_inner = ref_velocity - sim.getObjectPosition(body, -1)[0]  # Adjust as per velocity calculation
    P_inner = Kp_inner * err_inner
    I_inner += Ki_inner * err_inner
    D_inner = Kd_inner * (err_inner - prevErr_inner)
    pid_inner = P_inner + I_inner + D_inner
    prevErr_inner = err_inner

    # Integral windup handling for inner loop
    if abs(I_inner) > Imax_inner:
        I_inner = Imax_inner * (I_inner / abs(I_inner))

    # Apply inner PID output to the motors
    sim.setJointTargetVelocity(motor1, pid_inner)
    sim.setJointTargetVelocity(motor2, pid_inner)

def Ldrive():
    sim.setJointTargetVelocity(motor1, pid_inner + speedTurn)
    sim.setJointTargetVelocity(motor2, pid_inner - speedTurn)

def Rdrive():
    sim.setJointTargetVelocity(motor1, pid_inner - speedTurn)
    sim.setJointTargetVelocity(motor2, pid_inner + speedTurn)
