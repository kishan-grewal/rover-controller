# Rover Controller - Code Usage and Customisation

Contains code for controlling the rover integrating modules for line following, wall following, PID control, distance sensing, and WiFi communication The following sections describe how to use and change each part of the code

## Repository Structure
- rover_controller.ino - Main logic integrates modules
- pid_controller.cpp / pid_controller.h - Contains PID logic
- QTRSensorArray.cpp / QTRSensorArray.h - Reads line sensor values
- distance_sensor.cpp / distance_sensor.h - Reads and smooths distance sensor values
- wifi_logic.cpp / wifi_logic.h - Manages WiFi connection and UDP commands

## Required Libraries
Before uploading the code ensure the following libraries are installed in the Arduino IDE
- WiFi, WiFiUDP (by downloading Arduino Mbed OS Giga Boards in board manager)
- Average (by Rob Tillaart https://github.com/MajenkoLibraries/Average/tree/master/src)

## Code Use and Modification

### Source Files - Customisation Options

#### pid_controller.cpp / pid_controller.h
- kp ki kd gains can be changed in the PIDController constructor to adjust control response
- reset() clears integral and previous error terms when needed
- no output clamping in PID logic so limit output in main code if needed

#### QTRSensorArray.cpp / QTRSensorArray.h
- sensorMin and sensorMax in calibrate() define raw to normalised mapping range adjust as needed
- isLineDetected(confidence) checks line detection with confidence threshold adjust for sensitivity
- readLineBlack() uses weighted average logic for line position adjust weighting or sensor layout if necessary
- replace hardcoded calibration with dynamic calibration if needed for changing surfaces

#### distance_sensor.cpp / distance_sensor.h
- alpha in constructor controls smoothing factor lower for more smoothing higher for faster response
- bufferSize in constructor controls moving average buffer length increase for stability decrease for responsiveness
- calculateDistance() formula converts voltage to distance change if different sensor or mapping is used
- sampling interval in update() controls frequency of distance checks adjust to balance responsiveness and stability

#### wifi_logic.cpp / wifi_logic.h
- ssid and password control network credentials change to match WiFi network
- deviceIP gateway subnet define static IP configuration change as needed for network setup
- reconnectInterval sets time between reconnect attempts after WiFi drops change to control recovery time
- processUdpPackets() parses Stop command and PID updates adjust parsing logic if adding more commands
- Stop command disables system cannot be overridden by hardware button

### Main Control Logic - rover_controller.ino

#### System Parameters
- BUTTON_PIN defines the digital input pin for the mechanical kill switch change pin number if hardware changes
- DEBOUNCE_TIME controls button debounce delay change if button bounces more or less than expected
- MOTOR_SPEED_MIN and MOTOR_SPEED_MAX set motor speed limits change to adjust driving range
- baseSpeed sets default driving speed can be adjusted to change forward speed
- line_threshold defines the minimum normalised sensor value to detect line adjust for surface contrast
- threshold in isLineDetected() sets overall line detection confidence adjust for sensitivity
- LOST_LINE_TIMEOUT defines timeout for lost line detection adjust to control how long robot waits before stopping
- LOOP_INTERVAL controls main loop timing adjust to change system refresh rate

#### Motor Control and Movement
- setDrive() maps final left and right motor speeds applies constraints adjust for scaling or clamping
- max acceleration and deceleration in Motoron setup control motor response adjust for desired performance
- MotoronI2C setup includes device addresses and I2C bus assignment change addresses or buses as needed for hardware configuration

#### Line and Turn Detection
- last_pos stores last valid line position for fallback when line is lost adjust logic if needed
- calculatePos() combines left and right QTR data to compute position adjust weights x and y if needed
- lineAverage() computes average normalised reading for a side adjust usage as needed
- turn detection uses difference between ave_R buffer and current sensor sum adjust buffer size or detection logic for cornering behaviour

#### Button Handling and State
- robot_enabled flag toggles system state on button press adjust logic to match desired behaviour
- button handling logic reads state and applies debounce delay controlled by DEBOUNCE_TIME adjust if button is noisy or different hardware used
- Stop command from WiFi sets robot_enabled false which cannot be overridden by button
- sensor error fallback behaviour can be added to stop robot or switch modes on invalid readings

#### Optional Diagnostics
- sensor output printing can be enabled or disabled for debugging adjust interval or remove as needed
- commented-out code for LDR readings can be used for light-based control behaviour adjust as needed
