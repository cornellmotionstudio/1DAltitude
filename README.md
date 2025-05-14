# 1DAltitude

<p align="center">
    <img src="https://github.com/user-attachments/assets/2ca8a1cd-e680-4faa-853f-ddc64fb71ed2" width="400">
</p>

 
 The 1D Altitude project aims to develop an altitude control interface for Dylan to accurately measure drone height and to command drone throttle to a target altitude.
 
 `arduino_sensor` contains the code run on an Arduino attached to a linear distance sensor (potentiometer) and transmits altitude measurement data to the Python program `1d_pid.py`. This Python program contanis a basic UI developed in curses, allowing for manual and PID control of a connected quadcopter. Depending on the measured altitude and the target altitude, `1d_pid.py` calculates how much throttle to send to the drone in order to reach the target altitude. Serial communication is used to transmit measured altitude data between Arduino and Python and [YAMPSpy](https://github.com/thecognifly/YAMSPy) is used to communicate drone commands over Multiwii Serial Protocol to a Betaflight supported flight controller. After exiting out of the UI, a plot is generated detailing the measured and target altitudes, PID control output, and throttle output. 

 ### Example PID Control Graphs
 <p align="center">
    <img src="https://github.com/user-attachments/assets/872267cb-f86e-4a8f-89ad-53f5da0cdd14" width="400">
</p>

 ### UI and Controls
  <p align="center">
    <img src="https://github.com/user-attachments/assets/74716d57-6a71-4c94-a392-4c92d2dc499d" width="400">
</p>

* `a` - Arm 
* `r` - Reboot
* `d` - Disarm
* `m` - Mode (Manual and PID control)

**Manual Mode**
* `w/e` - Increase/decrease throttle
* `Arrow Keys` - Pitch and Roll

**PID Control Mode**
* `t` - Set target altitude
