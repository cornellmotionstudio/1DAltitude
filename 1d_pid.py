import time
import curses
import serial
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from itertools import cycle
import threading

from yamspy import MSPy

DRONE_SERIAL = "COM7"
ARDUINO_SERIAL = serial.Serial(port='COM3', baudrate=115200, timeout=0.02)

# PID Tuning Variables
kp = 1.0
ki = 0.01
kd = 0.3

previous_error = 0
integral = 0
MAX_INTEGRAL = 100
dt = CTRL_LOOP_TIME = 0.02

# Plotting Variables
time_steps = []
throttle_values = []
control_values = []
target_values = []
measured_values = []

target_altitude = 100.0  # Target altitude in mm
previous_target = 0

def pid_controller(target, measured, kp, ki, kd, previous_error, integral, dt):
    error = target - measured
    integral += error * dt
    integral = max(min(integral, MAX_INTEGRAL), -MAX_INTEGRAL)
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

        screen.addstr(1, 0, "Press 'q' to quit, 'r' to reboot, 'a' to arm, 'd' to disarm, 't' to change target, 'm' to change mode (manual or PID) and arrow keys + 'w'/'e' to control", curses.A_BOLD)
        
        result = external_function(screen)

    finally:
        # shut down cleanly
        curses.nocbreak(); screen.keypad(0); curses.echo()
        curses.endwin()
        if result==1:
            print("An error occurred... probably the serial port is not available ;)")

def keyboard_controller(screen):
    global target_altitude, previous_target, kp, ki, kd, previous_error, integral
    CMDS = {
            'roll':     1500,
            'pitch':    1500,
            'throttle': 900,
            'yaw':      1500,
            'aux1':     1000,
            'aux2':     1500 # Horizon mode
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
            init_time = time.time()
            while True:
                start_time = time.time()

                char = screen.getch() # get keypress
                curses.flushinp() # flushes buffer

                # Read measured altitude from Arduino
                line = ARDUINO_SERIAL.readline()
                try:
                    measured_altitude = float(line.decode().strip())
                except:
                    continue
                

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
                elif char == ord('t') or char == ord('T'):
                    curses.echo()
                    screen.addstr(10, 0, "Enter new target altitude (mm): ")
                    screen.clrtoeol()
                    screen.refresh()
                    try:
                        input_str = screen.getstr(10, 34, 10).decode().strip()
                        new_target = float(input_str)
                        target_altitude = new_target
                        cursor_msg = f"Target altitude updated to: {target_altitude} mm"
                    except:
                        cursor_msg = "Invalid input for target altitude"
                    screen.move(10, 0)
                    screen.clrtoeol()
                    curses.noecho()


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
                    # Calculate PID control output
                    if abs(target_altitude - previous_target) > 5:
                        integral = 0
                    previous_target = target_altitude
                    control, error, integral = pid_controller(target_altitude, float(measured_altitude), kp, ki, kd, previous_error, integral, dt)
                    
                    # Update throttle based on PID control output
                    base_throttle = 1350  # Base throttle value
                    throttle = int(base_throttle + control)
                    throttle = max(900, min(throttle, 1600))
                    CMDS['throttle'] = throttle
                    previous_error = error

                    time_steps.append(time.time()-init_time)
                    measured_values.append(measured_altitude)
                    target_values.append(target_altitude)
                    throttle_values.append(CMDS['throttle'])
                    control_values.append(control)
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
                screen.addstr(5, 0, f"[PID] Target Altitude: {target_altitude:.1f} mm       ")
                screen.addstr(6, 0, f"[PID] kp: {kp:.3f}, ki: {ki:.3f}, kd: {kd:.3f}        ")
                screen.clrtoeol()
                screen.addstr(7, 0, f"[PID] Measured Altitude: {measured_altitude:.1f} mm       ")
                screen.addstr(8, 0, f"[PID] Throttle: {CMDS['throttle']}       ")

                    
                end_time = time.time()
                if (end_time-start_time)<CTRL_LOOP_TIME:
                    time.sleep(CTRL_LOOP_TIME-(end_time-start_time))

    finally:
        screen.addstr(5, 0, "Disconneced from the FC!")
        screen.clrtoeol()


if __name__ == "__main__":

    run_curses(keyboard_controller)

    # Now plot the data after curses exits
    fig, ax1 = plt.subplots()
    fig.suptitle("Real-Time Altitude PID Control [kp={}, ki={}, kd={}]".format(kp, ki, kd))
    
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Altitude (mm)")
    ax1.set_ylim(0, 320)
    line_measured, = ax1.plot(time_steps, measured_values, label="Measured Altitude", linewidth=2)
    line_target, = ax1.plot(time_steps, target_values, label="Target Altitude", linestyle='--')
    ax1.legend(loc="upper left")
    ax1.grid(True)

    ax2 = ax1.twinx()
    ax2.set_ylabel("Throttle / Control")
    ax2.set_ylim(800, 1700)
    ax2.plot(time_steps, throttle_values, label="Throttle", color='orange', alpha=0.6)
    ax2.legend(loc="upper right")

    plt.show()
