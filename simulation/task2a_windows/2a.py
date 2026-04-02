import math

def sysCall_init():
    sim = require('sim')
    global motor1, motor2, wheel1, wheel2, dummy, body, mOr1, mOr2, bOr, xRef, D_Ref, Ref, startSimulation, stopSim
    global Kp_outer, Ki_outer, Kd_outer, Kp_inner, Ki_inner, Kd_inner, Imax_outer, Imax_inner, ref
    global prevErr_outer, prevErr_inner, I_outer, I_inner, pid_inner, turn_velocity, PrevBodyPos
    global speedTurn

    # Speed for turning
    speedTurn = 8
    # Getting handles for objects
    motor1 = sim.getObject("/body/right_joint")
    motor2 = sim.getObject("/body/left_joint")
    dummy = sim.getObject("/XYZCameraProxy")
    body = sim.getObject("/body")

    # Initial object orientations and positions
    mOr1 = sim.getObjectMatrix(motor1, dummy)
    mOr2 = sim.getObjectMatrix(motor2, dummy)
    bOr = sim.getObjectMatrix(body, dummy)

    # Set reference positions for x and y axis
    xRef = sim.getObjectPosition(body, dummy)[0]
    D_Ref = sim.getObjectPosition(body, dummy)
    Ref = sim.getObjectPosition(body, dummy)
    
    # Initialize start/stop flags
    startSimulation = True
    stopSim = False

    # Outer PID (position control)
    Kp_outer = 23.0  # Increase from previous values
    Ki_outer = 1.0   # Slightly increase to help with steady-state error
    Kd_outer = 2.0   # Increase for more damping
    Imax_outer = 100  # Reduced from 500 to limit windup

    # Inner PID (velocity control)
    Kp_inner = 11.0  # Increase for faster response
    Ki_inner = 0.5   # Keep low to prevent windup
    Kd_inner = 5.0   # Increase for smoother control
    Imax_inner = 100  # Reduced from 500 to limit windup

    ref = 0.0

    # Initialize PID variables
    prevErr_outer = 0
    prevErr_inner = 0
    I_outer = 0
    I_inner = 0
    pid_inner = 0
    turn_velocity = 0

    # Initialize PrevBodyPos to the current body position
    PrevBodyPos = sim.getEulerAnglesFromMatrix(sim.getObjectMatrix(body, dummy))

def sysCall_actuation():
    message, data, data2 = sim.getSimulatorMessage()
    global ref
    
    if message == sim.message_keypress:
        # Handle Prismatic Joint velocity based on key 'q' and 'e' (closing and opening)
        if data[0] == 113:  # 'q' key for closing the gripper
            sim.setJointTargetVelocity(sim.getObject('/Prismatic_joint'), -0.10)  # Set a negative velocity to close
            drive()
        elif data[0] == 101:  # 'e' key for opening the gripper
            sim.setJointTargetVelocity(sim.getObject('/Prismatic_joint'), 0.10)   # Set a positive velocity to open
            drive()
            
        # Handle Arm Joint velocity based on key 'w' and 's' (raising and lowering)
        if data[0] == 119:  # 'w' key for raising the gripper
            sim.setJointTargetVelocity(sim.getObject('/arm_joint'), -8)             # Positive velocity to raise
            drive()
            
        elif data[0] == 115:  # 's' key for lowering the gripper
            sim.setJointTargetVelocity(sim.getObject('/arm_joint'), 8)            # Negative velocity to lower
            drive()
            
        # Handle reference and drive functions based on arrow keys
        if data[0] == 2008:  # forward up arrow
            ref += 0.002  # Increase reference
            drive()  # Execute drive function
        elif data[0] == 2007:  # backward down arrow
            ref -= 0.002  # Decrease reference
            drive()  # Execute drive function
        elif data[0] == 2010:  # left arrow key
            Ldrive()  # Execute left drive function
        elif data[0] == 2009:  # right arrow key
            Rdrive()  # Execute right drive function
    else:
        # If no key is pressed, stop the joints by setting velocity to 0
        sim.setJointTargetVelocity(sim.getObject('/Prismatic_joint'), 0)
        sim.setJointTargetVelocity(sim.getObject('/arm_joint'), 0)
        # Execute default drive function
        drive()

def sysCall_sensing():
        drive()

def sysCall_cleanup():
    pass

def drive():
    global ref, Imax_outer, Imax_inner, Kp_outer, Ki_outer, Kd_outer, Kp_inner, Ki_inner, Kd_inner
    global prevErr_outer, prevErr_inner, I_outer, I_inner, pid_inner
    global change, PrevBodyPos, DummyPos
    
    # Get the current body position (Euler angles)
    BodyPos = sim.getEulerAnglesFromMatrix(sim.getObjectMatrix(body, dummy))
    DummyPos = sim.getEulerAnglesFromMatrix(sim.getObjectMatrix(dummy))

    # Subtract corresponding elements of BodyPos and PrevBodyPos
    change = [BodyPos[i] - PrevBodyPos[i] for i in range(len(BodyPos))]

    # Update only the specific value in DummyPos (for example, changing the pitch at index 1)
    index_to_change = 2  # Change this to the index you want to modify
    DummyPos[index_to_change] += change[index_to_change]

    # Set the updated orientation to the dummy
    sim.setObjectOrientation(dummy, -1, DummyPos)

    # Update BodyPos again if necessary
    BodyPos = sim.getEulerAnglesFromMatrix(sim.getObjectMatrix(body, dummy))

    
    yaw = BodyPos[2]
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
    
    # Update PrevBodyPos for the next iteration
    PrevBodyPos = BodyPos

def Ldrive():
    sim.setJointTargetVelocity(motor1, pid_inner + speedTurn)
    sim.setJointTargetVelocity(motor2, pid_inner - speedTurn)

def Rdrive():
    sim.setJointTargetVelocity(motor1, pid_inner - speedTurn)
    sim.setJointTargetVelocity(motor2, pid_inner + speedTurn)
