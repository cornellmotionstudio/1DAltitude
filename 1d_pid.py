import serial 
import time

from yamspy import MSPy

# Adjust port to your Arduino's port and drone serial port
ARDUINO_SERIAL = serial.Serial(port='COM7', baudrate=115200, timeout=.1)
DRONE_SERIAL = 'COM12'

# PID Tuning Variables
kp = 1.0
ki = 0.1
kd = 0.05
previous_error = 0
integral = 0
dt = 0.1

# Plotting Variables
time_steps = []
throttle_values = []
control_values = []
target_values = []
measured_values = []

target_altitude = 50.0 # Target in mm

def pid_controller(target, measured, kp, ki, kd, previous_error, integral, dt):
    error = target - measured
    integral += error * dt
    derivative = (error - previous_error) / dt
    control = kp * error + ki * integral + kd * derivative
    return control, error, integral

def control_loop():
    global previous_error, integral, dt

    CMDS = {
        'roll':     1500,
        'pitch':    1500,
        'throttle': 900,
        'yaw':      1500,
        'aux1':     1000,
        'aux2':     1000
        }
    
    # Assuming flight controller set to AETR
    CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']
    
    with MSPy(device=DRONE_SERIAL, loglevel='WARNING', baudrate=115200) as board:
        if board == 1: #error occured
            print("Error connecting to drone, check serial port and connection.")
            return 1
        
        # Necessary to send some messages or RX failsage will be activated
        command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                    'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                    'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']

        for msg in command_list: 
            if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                dataHandler = board.receive_msg()
                board.process_recv_data(dataHandler)

        print("flightControllerIdentifier: {}".format(board.CONFIG['flightControllerIdentifier']))
        print("boardName: {}".format(board.CONFIG['boardName']))
        print("name: {}".format(board.CONFIG['name']))

        CMDS['aux1'] = 1800 # Arm command
        CMDS['aux2'] = 1500 # Enable Horizon Mode

        start_time = time.time()
        try: 
            while True:
                # Send messages to board
                if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)

                # Read measured altitude from Arduino
                measured_altitude = ARDUINO_SERIAL.readline().decode('utf-8').strip()
                if measured_altitude: measured_altitude = float (measured_altitude)
                else: 
                    print("No altitude data received from Arduino.")
                    continue

                # Calculate PID control output
                control, error, integral = pid_controller(
                    target_altitude, float(measured_altitude), kp, ki, kd, previous_error, integral, dt)
                
                # Update throttle based on PID control output
                throttle = int(CMDS['throttle'] + control)
                throttle = max(900, min(throttle, 2000))
                previous_error = error

                # Plot data
                time_steps.append(time.time() - start_time)
                throttle_values.append(throttle)
                control_values.append(control)
                target_values.append(target_altitude)
                measured_values.append(measured_altitude)

                time.sleep(dt)
        except KeyboardInterrupt:
            print("Exiting control loop")
            CMDS['aux1'] = 1000 # Disarm command
            board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER])  
            

if __name__ == "__main__":
    control_loop()