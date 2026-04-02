import math

def sysCall_init():
    sim = require('sim')
    global motor1, motor2, wheel1, wheel2, dummy, body, mOr1, mOr2, wOr1, wOr2, bOr, xRef, yRef, startSimulation, stopSim, Kp_outer, Ki_outer, Kd_outer, Kp_inner, Ki_inner, Kd_inner, Imax_outer, Imax_inner, ref, prevErr_outer, prevErr_inner, I_outer, I_inner, pid_inner, turn_velocity

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
    yRef = sim.getObjectPosition(body, dummy)[1]

    # Initialize start/stop flags
    startSimulation = True
    stopSim = False

    # Outer PID (position control)
    Kp_outer = 15.0
    Ki_outer = 0.5
    Kd_outer = 0.1
    Imax_outer = 500

    # Inner PID (velocity control)
    Kp_inner = 10.0
    Ki_inner = 0.5
    Kd_inner = 0.1
    Imax_inner = 500

    ref = 0.0

    # Initialize PID variables
    prevErr_outer = 0
    prevErr_inner = 0
    I_outer = 0
    I_inner = 0
    pid_inner = 0  # Initialize pid_inner as a global variable
    turn_velocity = 0  # Initialize turn_velocity variable

def sysCall_actuation():
    global prevErr_outer, prevErr_inner, I_outer, I_inner, stopSim
    # Call the drive function in each actuation step
    #drive(ref)

def sysCall_sensing():
    ############### Keyboard Input ##############
    message, data, data2 = sim.getSimulatorMessage()
    global ref, pid_inner, turn_velocity
    #print(data[0])
    if (message == sim.message_keypress):
        if (data[0] == 2008):  # forward up arrow
            # Increase forward velocity reference to move forward
            forward_velocity_ref = 0.005  # Adjust value for desired forward speed
            ref += forward_velocity_ref

            print('forward')

        if (data[0] == 2007):  # backward down arrow
            # Decrease forward velocity reference to move backward
            backward_velocity_ref = -0.005  # Adjust value for desired backward speed
            ref += backward_velocity_ref
            print('backward')

        if (data[0] == 2010):  # left arrow key
            print('left')
            Ldrive()
            
        if (data[0] == 2009):  # right arrow key
            print('right')
            Rdrive()
    else:
        drive()
    # Call drive function once, after processing input
    
    #########################################

def sysCall_cleanup():
    pass

def drive():
    global ref, Imax_outer, Imax_inner, Kp_outer, Ki_outer, Kd_outer, Kp_inner, Ki_inner, Kd_inner, stopSim, prevErr_outer, prevErr_inner, I_outer, I_inner, pid_inner
    yaw = sim.getEulerAnglesFromMatrix(sim.getObjectMatrix(body, dummy))[2]
    pitch = sim.getEulerAnglesFromMatrix(sim.getObjectMatrix(body, dummy))[0]
    roll = sim.getEulerAnglesFromMatrix(sim.getObjectMatrix(body, dummy))[1]
    xErr = xRef - sim.getObjectPosition(body, dummy)[0]
    
    # Adjust roll based on pitch and yaw
    roll -= pitch * math.sin(yaw)
    pitch += roll * math.sin(yaw)

    ## Outer PID loop: calculates desired velocity based on roll and pitch (position control)
    # Add a small forward movement by increasing the reference value
    err_outer = (ref + xErr) - roll
    P_outer = Kp_outer * err_outer
    I_outer += Ki_outer * err_outer
    D_outer = Kd_outer * (err_outer - prevErr_outer)
    ref_velocity = P_outer + I_outer + D_outer
    prevErr_outer = err_outer

    # Integral windup handling for outer loop
    if abs(I_outer) > Imax_outer:
        I_outer = Imax_outer * (I_outer / abs(I_outer))

    ## Inner PID loop: controls the motor speed based on desired velocity (velocity control)
    err_inner = ref_velocity - sim.getObjectPosition(body, -1)[0]  # Adjust this as per your velocity calculation
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

    sim.setJointTargetVelocity(motor1, pid_inner + 9)
    sim.setJointTargetVelocity(motor2, pid_inner - 9)

    
def Rdrive():
    sim.setJointTargetVelocity(motor1, pid_inner - 9)
    sim.setJointTargetVelocity(motor2, pid_inner + 9)
