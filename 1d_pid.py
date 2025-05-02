import time
import curses
import serial
from collections import deque
from itertools import cycle

from yamspy import MSPy

DRONE_SERIAL = "COM7"
ARDUINO_SERIAL = serial.Serial(port='COM3', baudrate=115200, timeout=0.02)

# PID Tuning Variables
kp = 0.03
ki = 0.01
kd = 0.02
previous_error = 0
integral = 0
dt = CTRL_LOOP_TIME = 0.025

# Plotting Variables
time_steps = []
throttle_values = []
control_values = []
target_values = []
measured_values = []

target_altitude = 100.0  # Target altitude in mm

def pid_controller(target, measured, kp, ki, kd, previous_error, integral, dt):
    error = target - measured
    integral += error * dt
    derivative = (error - previous_error) / dt
    control = kp * error + ki * integral + kd * derivative
    return control, error, integral

def run_curses(external_function):
    result=1

    try:
        # get the curses screen window
        screen = curses.initscr()

        # turn off input echoing
        curses.noecho()

        # respond to keys immediately (don't wait for enter)
        curses.cbreak()

        # non-blocking
        screen.timeout(0)

        # map arrow keys to special values
        screen.keypad(True)

        screen.addstr(1, 0, "Press 'q' to quit, 'r' to reboot, 'a' to arm, 'd' to disarm, 'm' to change mode (manual or PID) and arrow keys to control", curses.A_BOLD)
        
        result = external_function(screen)

    finally:
        # shut down cleanly
        curses.nocbreak(); screen.keypad(0); curses.echo()
        curses.endwin()
        if result==1:
            print("An error occurred... probably the serial port is not available ;)")

def keyboard_controller(screen):
    global target_altitude, kp, ki, kd, previous_error, integral, dt
    CMDS = {
            'roll':     1500,
            'pitch':    1500,
            'throttle': 900,
            'yaw':      1500,
            'aux1':     1000,
            'aux2':     1500
            }

    # This order is the important bit: it will depend on how your flight controller is configured.
    CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

    # "print" doesn't work with curses, use addstr instead
    try:
        screen.addstr(15, 0, "Connecting to the FC...")

        with MSPy(device=DRONE_SERIAL, loglevel='WARNING', baudrate=115200) as board:
            if board == 1: # an error occurred...
                return 1

            screen.addstr(15, 0, "Connecting to the FC... connected!")
            screen.clrtoeol()
            screen.move(1,0)

            # It's necessary to send some messages or the RX failsafe will be activated
            # and it will not be possible to arm.
            command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                            'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                            'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']

            for msg in command_list: 
                if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)

            screen.addstr(15, 0, "apiVersion: {}".format(board.CONFIG['apiVersion']))
            screen.clrtoeol()
            screen.addstr(15, 50, "flightControllerIdentifier: {}".format(board.CONFIG['flightControllerIdentifier']))
            screen.addstr(16, 0, "flightControllerVersion: {}".format(board.CONFIG['flightControllerVersion']))
            screen.addstr(16, 50, "boardIdentifier: {}".format(board.CONFIG['boardIdentifier']))
            screen.addstr(17, 0, "boardName: {}".format(board.CONFIG['boardName']))
            screen.addstr(17, 50, "name: {}".format(board.CONFIG['name']))

            mode = 'MANUAL'
            cursor_msg = ""
            measured_altitude = 0.0
            last_loop_time = time.time()
            while True:
                start_time = time.time()

                char = screen.getch() # get keypress
                curses.flushinp() # flushes buffer
                

                #
                # Key input processing
                #

                #
                # KEYS (NO DELAYS)
                #
                if char == ord('q') or char == ord('Q'):
                    break

                elif char == ord('d') or char == ord('D'):
                    cursor_msg = 'Sending Disarm command...'
                    CMDS['aux1'] = 1000

                elif char == ord('r') or char == ord('R'):
                    screen.addstr(3, 0, 'Sending Reboot command...')
                    screen.clrtoeol()
                    board.reboot()
                    time.sleep(0.5)
                    break

                elif char == ord('a') or char == ord('A'):
                    cursor_msg = 'Sending Arm command...'
                    CMDS['aux1'] = 1800

                elif char == ord('m') or char == ord('M'):
                    if mode == 'MANUAL':
                        mode = 'PID'
                        cursor_msg = 'Mode changed to PID!'
                    else:
                        mode = 'MANUAL'
                        cursor_msg = 'Mode changed to MANUAL!'


                #
                # The code below is expecting the drone to have the
                # modes set accordingly since everything is hardcoded.
                #
                if mode == 'MANUAL':
                    if char == ord('w') or char == ord('W'):
                        CMDS['throttle'] = CMDS['throttle'] + 10 if CMDS['throttle'] + 10 <= 2000 else CMDS['throttle']
                        cursor_msg = 'W Key - throttle(+):{}'.format(CMDS['throttle'])

                    elif char == ord('e') or char == ord('E'):
                        CMDS['throttle'] = CMDS['throttle'] - 10 if CMDS['throttle'] - 10 >= 1000 else CMDS['throttle']
                        cursor_msg = 'E Key - throttle(-):{}'.format(CMDS['throttle'])

                    elif char == curses.KEY_RIGHT:
                        CMDS['roll'] = CMDS['roll'] + 10 if CMDS['roll'] + 10 <= 2000 else CMDS['roll']
                        cursor_msg = 'Right Key - roll(-):{}'.format(CMDS['roll'])

                    elif char == curses.KEY_LEFT:
                        CMDS['roll'] = CMDS['roll'] - 10 if CMDS['roll'] - 10 >= 1000 else CMDS['roll']
                        cursor_msg = 'Left Key - roll(+):{}'.format(CMDS['roll'])

                    elif char == curses.KEY_UP:
                        CMDS['pitch'] = CMDS['pitch'] + 10 if CMDS['pitch'] + 10 <= 2000 else CMDS['pitch']
                        cursor_msg = 'Up Key - pitch(+):{}'.format(CMDS['pitch'])

                    elif char == curses.KEY_DOWN:
                        CMDS['pitch'] = CMDS['pitch'] - 10 if CMDS['pitch'] - 10 >= 1000 else CMDS['pitch']
                        cursor_msg = 'Down Key - pitch(-):{}'.format(CMDS['pitch'])
                else:
                    # Read measured altitude from Arduino
                    line = ARDUINO_SERIAL.readline()
                    try:
                        measured_altitude = float(line.decode().strip())
                    except:
                        continue
                
                    # Calculate PID control output
                    control, error, integral = pid_controller(
                        target_altitude, float(measured_altitude), kp, ki, kd, previous_error, integral, dt)
                    
                    # Update throttle based on PID control output
                    throttle = int(CMDS['throttle'] + control)
                    throttle = max(900, min(throttle, 1500))
                    CMDS['throttle'] = throttle
                    previous_error = error

                screen.addstr(5, 0, f"[PID] Target Altitude: {target_altitude:.1f} mm       ")
                screen.addstr(6, 0, f"[PID] kp: {kp:.3f}, ki: {ki:.3f}, kd: {kd:.3f}        ")
                screen.clrtoeol()
                screen.addstr(7, 0, f"[PID] Measured Altitude: {measured_altitude:.1f} mm       ")
                screen.addstr(8, 0, f"[PID] Throttle: {CMDS['throttle']}       ")

                #
                # IMPORTANT MESSAGES (CTRL_LOOP_TIME based)
                #
                if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                    last_loop_time = time.time()
                    # Send the RC channel values to the FC
                    if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
                
    
                screen.addstr(2, 0, f"[MODE] Current Mode: {mode}        ", curses.A_BOLD)
                screen.addstr(3, 0, cursor_msg)
                screen.clrtoeol()

                    
                end_time = time.time()
                if (end_time-start_time)<CTRL_LOOP_TIME:
                    time.sleep(CTRL_LOOP_TIME-(end_time-start_time))

    finally:
        screen.addstr(5, 0, "Disconneced from the FC!")
        screen.clrtoeol()

if __name__ == "__main__":
    run_curses(keyboard_controller)


