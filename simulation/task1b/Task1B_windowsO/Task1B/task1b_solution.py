import math

def sysCall_init():
    sim = require('sim')

    # Variables
    global right_motor, left_motor, theta, floor, body
    global start_time, prev_time
    global ref, Kp, Ki, Kd, I, prev_err, Imax, xRef
    global tuning_mode, oscillation_start_time, oscillation_amplitude, prev_err_sign, Ku, Tu

    start_time = sim.getSimulationTime()
    prev_time = start_time
    
    right_motor = sim.getObject('/body/right_joint')
    left_motor = sim.getObject('/body/left_joint')
    body = sim.getObject('/body')
    floor = sim.getObject('/Floor')    
    theta = sim.getObjectPosition(body, floor)

    # Initialize PID gains and tuning variables
    ref = 0
    Kp = 900   # Start with a reasonable guess for Kp
    Ki = 0     # Start with Ki and Kd at 0 for tuning
    Kd = 0   
    I = 0      # Initialize integral term
    prev_err = 0
    Imax = 99  # Limit integral windup
    xRef = sim.getObjectPosition(body, -1)[0]

    # Auto-tuning parameters
    tuning_mode = True
    oscillation_start_time = None
    oscillation_amplitude = 0
    prev_err_sign = None
    Ku = 0     # Ultimate gain (to be tuned)
    Tu = 0     # Oscillation period (to be determined)

def sysCall_actuation():
    global prev_err, I, prev_time, tuning_mode, oscillation_start_time, Ku, Tu, prev_err_sign
    global Kp, Ki, Kd  # Declare the global variables here
    
    # Get the current time and time step
    current_time = sim.getSimulationTime()
    dt = current_time - prev_time
    if dt <= 0:  # Avoid division by zero
        dt = 0.0004

    # Get yaw, pitch, roll
    yaw = sim.getEulerAnglesFromMatrix(sim.getObjectMatrix(body, floor))[2]
    pitch = sim.getEulerAnglesFromMatrix(sim.getObjectMatrix(body, floor))[1]
    roll = sim.getEulerAnglesFromMatrix(sim.getObjectMatrix(body, floor))[0]
    xErr = xRef - sim.getObjectPosition(body, -1)[0]
    
    # Calculate the combined error (roll angle + position error)
    err = (ref + xErr) - roll

    # Proportional term
    P = Kp * err

    # Auto-tuning logic (Ziegler-Nichols)
    if tuning_mode:
        err_sign = math.copysign(1, err)  # Determine the sign of the error
        if prev_err_sign is None:
            prev_err_sign = err_sign

        if prev_err_sign != err_sign:  # Detect zero-crossing (oscillation point)
            if oscillation_start_time is None:
                oscillation_start_time = current_time  # Start timing oscillations
            else:
                Tu = current_time - oscillation_start_time  # Calculate oscillation period
                oscillation_start_time = current_time

            Ku = Kp  # Ultimate gain (when oscillations are detected)
            prev_err_sign = err_sign

        if Tu > 0 and Ku > 0:  # Once we've detected oscillations, calculate PID gains
            Kp = 0.6 * Ku
            Ki = 1.2 * Ku / Tu
            Kd = 3 * Ku * Tu / 40

            tuning_mode = False  # Disable tuning once parameters are set

    # Integral term (accumulate over time)
    I += Ki * err * dt
    if abs(I) > Imax:  # Cap the integral term to prevent windup
        I = Imax * (I / abs(I))
    
    # Derivative term (change in error over time)
    D = Kd * (err - prev_err) / dt
    prev_err = err  # Update the previous error
    
    # PID output
    pid = P + I + D
    
    # Set motor velocities (one forward, one backward for balancing)
    sim.setJointTargetVelocity(right_motor, pid)
    sim.setJointTargetVelocity(left_motor, pid)
    
    # Print the PID values for debugging
    print(f"PID: {pid}, P: {P}, I: {I}, D: {D}, Kp: {Kp}, Ki: {Ki}, Kd: {Kd}")
    
    # Update the previous time
    prev_time = current_time

def sysCall_sensing():
    global theta 
    theta = sim.getObjectOrientation(body, floor)
    # Add any sensing or feedback logic if needed

def sysCall_cleanup():
    # Cleanup logic after the simulation
    pass
