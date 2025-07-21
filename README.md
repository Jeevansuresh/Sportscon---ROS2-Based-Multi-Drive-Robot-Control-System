# Sportscon---ROS2-Based-Multi-Drive-Robot-Control-System

## Node Architecture

- **Joystick Control Node**
    - Receives and interprets joystick or gamepad inputs from the user.
    - Converts input into unified velocity commands: linear (x, y) and angular (ω*ω*).
    - Publishes velocity commands to the appropriate drive node topics.
    - Acts as the central coordinator for robot motion control.
- **Drive Nodes (Three Required)**
    - **Differential Drive Node**
    - **Omnidirectional Drive Node**
    - **Mecanum Drive Node**
    - Each drive node subscribes to the velocity commands published by the joystick control node.
    - Computes PWM signals for its specific drive kinematics and publishes these to hardware interfaces.

## Node Responsibilities

| **Node** | **Purpose** | **Inputs** | **Outputs** |
| --- | --- | --- | --- |
| Joystick Control Node | User input handling & velocity command coordination | Joystick/gamepad controls | Velocity commands |
| Differential Drive | Computes PWM for differential drive kinematics | Velocity commands (v, ω) | Wheel PWMs |
| Omnidirectional Drive | Computes PWM for omni wheel kinematics | Velocity commands (v_x, v_y, ω) | Wheel PWMs |
| Mecanum Drive | Computes PWM for mecanum drive kinematics | Velocity commands (v_x, v_y, ω) | Wheel PWMs |
