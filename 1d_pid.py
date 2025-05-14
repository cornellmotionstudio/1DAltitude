import time
import curses
import serial
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from itertools import cycle
import threading
from yamspy import MSPy

# -------------------- Configuration -------------------- #
DRONE_SERIAL = "COM7"
ARDUINO_SERIAL = serial.Serial(port='COM3', baudrate=115200, timeout=0.02)
SENSOR_DISTANCE = 305 # mm (sensor max range)

# PID Variables
kp = 15.0 # 1
ki = 0 # 0.01
kd = 0.0 #0.3
previous_error = 0
integral = 0
dt = CTRL_LOOP_TIME = 0.02

# Throttle Maping
T_min = 900
T_max = 1600
T_hover = 1350

# Target Altitude
target_altitude = 100.0  # mm

# -------------------- Logging Variables -------------------- #
# Plotting Variables
time_steps = []
throttle_values = []
target_values = []
measured_values = []
p_terms = []
i_terms = []
d_terms = []
u_pid_values = []
battery_voltages = []

# -------------------- PID Controller -------------------- #
def pid_controller(target, measured, kp, ki, kd, previous_error, integral, dt):
    error = target - measured
    integral += error * dt
    derivative = (error - previous_error) / dt

    p_term, i_term, d_term = kp * error, ki * integral, kd * derivative
    u_pid = p_term + i_term + d_term

    p_terms.append(p_term)
    i_terms.append(i_term)
    d_terms.append(d_term)

    return u_pid, error

# -------------------- Normalize PID Output to Throttle -------------------- #
def u_pid_to_throttle(u_pid):
    # Normalize the u_pid to [-1, 1]
    MAX_ERROR = SENSOR_DISTANCE
    MAX_INTEGRAL = MAX_ERROR * 10 # assume 10 seconds of integration (not sure)
    MAX_DERIVATIVE = MAX_ERROR / dt

    u_pid_max = abs(kp * MAX_ERROR + ki * MAX_INTEGRAL + kd * MAX_DERIVATIVE)
    u_pid_normalized = max(-1.0, min(u_pid / u_pid_max, 1.0))

    # Map u_pid = -1 -> T_min, 0 -> T_hover, +1 -> T_max
    if u_pid_normalized >= 0:
        throttle = T_hover + u_pid_normalized * (T_max - T_hover)
    else:
        throttle = T_hover + u_pid_normalized * (T_hover - T_min)

    return int(throttle)

# -------------------- Plotting Function -------------------- #
def plot_pid_data(time_steps, measured_values, target_values, throttle_values,
                  u_pid_values, p_terms, i_terms, d_terms, battery_voltages,
                  kp, ki, kd):
    import matplotlib.pyplot as plt

    # First plot: Measured/Target Altitude + Throttle
    fig, ax1 = plt.subplots()
    fig.suptitle(f"Real-Time Altitude PID Control [kp={kp}, ki={ki}, kd={kd}]")

    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Altitude (mm)")
    ax1.set_ylim(0, 320)
    ax1.plot(time_steps, measured_values, label="Measured Altitude", linewidth=2)
    ax1.plot(time_steps, target_values, label="Target Altitude", linestyle='--')
    ax1.legend(loc="upper left")
    ax1.grid(True)

    ax2 = ax1.twinx()
    ax2.set_ylabel("Throttle")
    ax2.set_ylim(800, 1700)
    ax2.plot(time_steps, throttle_values, label="Throttle", color='orange', alpha=0.6)
    ax2.legend(loc="upper right")

    # Second plot: PID terms and u_pid
    fig2, ax3 = plt.subplots()
    fig2.suptitle("PID Terms and u_pid")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Control Output")
    ax3.set_ylim(-1000, 1000)
    ax3.plot(time_steps, u_pid_values, label="u_pid", color='green', linewidth=2)
    ax3.plot(time_steps, p_terms, label="P Term", linestyle='--', color='red')
    ax3.plot(time_steps, i_terms, label="I Term", linestyle='--', color='blue')
    ax3.plot(time_steps, d_terms, label="D Term", linestyle='--', color='purple')
    ax3.legend(loc="upper left")
    ax3.grid(True)

    # Third plot: Battery voltage
    fig3, ax4 = plt.subplots()
    fig3.suptitle("Battery Voltage")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Voltage (V)")
    ax4.set_ylim(0, 20)
    ax4.plot(time_steps, battery_voltages, label="Battery Voltage", color='orange', linewidth=2)
    ax4.legend(loc="upper left")
    ax4.grid(True)

    # Save and show
    plt.savefig(f"PID_Altitude_Control_{kp}_{ki}_{kd}.png")
    plt.show()

# -------------------- Curses UI Handling -------------------- #
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

# -------------------- Drone Control Logic -------------------- #
def keyboard_controller(screen):
    global target_altitude, kp, ki, kd, previous_error, integral

    CMDS = {
            'roll':     1500,
            'pitch':    1500,
            'throttle': 900,
            'yaw':      1500,
            'aux1':     1000,
            'aux2':     1500 # Horizon mode
            }

    CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

    try:
        screen.addstr(15, 0, "Connecting to the FC...")

        with MSPy(device=DRONE_SERIAL, loglevel='WARNING', baudrate=115200) as board:
            if board == 1: # an error occurred...
                return 1

            screen.addstr(15, 0, "Connecting to the FC... connected!")
            screen.clrtoeol()
            screen.move(1,0)

            # FC Setup
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
                char = screen.getch() 
                curses.flushinp() 

                # Read measured altitude from Arduino
                line = ARDUINO_SERIAL.readline()
                try:
                    measured_altitude = float(line.decode().strip())
                except:
                    continue

                # Disarm after 20 seconds
                if (time.time()-init_time) > 20:
                    CMDS['aux1'] = 1000 # Disarm command
                
                # --- Keypress Handling ---
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

                 # --- Manual or PID mode ---
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
                    # MATH
                    u_pid, error = pid_controller(target_altitude, measured_altitude, kp, ki, kd, previous_error, integral, dt)
                    previous_error = error
                    
                    # ENGINEERING
                    throttle = u_pid_to_throttle(u_pid)
                    CMDS['throttle'] = throttle

                    # DATA
                    time_steps.append(time.time()-init_time)
                    measured_values.append(measured_altitude)
                    target_values.append(target_altitude)
                    throttle_values.append(CMDS['throttle'])
                    u_pid_values.append(u_pid)
                    battery_voltages.append(board.ANALOG['voltage'])

                # Send RC commands to the FC
                if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                    last_loop_time = time.time()
                    # Send the RC channel values to the FC
                    if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
                        
                # Update the screen
                screen.addstr(2, 0, f"[MODE] Current Mode: {mode}        ", curses.A_BOLD)
                screen.addstr(3, 0, cursor_msg)
                screen.clrtoeol()
                screen.addstr(5, 0, f"[PID] Target Altitude: {target_altitude:.1f} mm       ")
                screen.addstr(6, 0, f"[PID] kp: {kp:.3f}, ki: {ki:.3f}, kd: {kd:.3f}        ")
                screen.clrtoeol()
                screen.addstr(7, 0, f"[PID] Measured Altitude: {measured_altitude:.1f} mm       ")
                screen.addstr(8, 0, f"[PID] Throttle: {CMDS['throttle']}       ")

                # Control loop timing    
                end_time = time.time()
                if (end_time-start_time)<CTRL_LOOP_TIME:
                    time.sleep(CTRL_LOOP_TIME-(end_time-start_time))

    finally:
        screen.addstr(5, 0, "Disconneced from the FC!")
        screen.clrtoeol()


# -------------------- Main -------------------- #
if __name__ == "__main__":
    run_curses(keyboard_controller)

    # Plot after exiting the curses UI
    plot_pid_data(time_steps, measured_values, target_values, throttle_values,
                  u_pid_values, p_terms, i_terms, d_terms, battery_voltages,
                  kp, ki, kd)