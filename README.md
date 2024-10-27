### Project: Modeling and Simulation of Quadcopter Using PID Controller

#### Overview
This project focuses on designing, modeling, and simulating a quadcopter, also known as a quadrotor, with the aim of achieving precise control of its movement and stability using a **PID (Proportional-Integral-Derivative) controller**. Quadcopters are a prevalent type of **Unmanned Aerial Vehicle (UAV)**, highly valued in various applications for their ability to take off and land vertically (VTOL) and navigate complex environments. The project utilizes MATLAB Simulink for simulation and implements the control system on a hardware prototype using the Ardupilot Mega controller board. Through PID tuning, the quadcopter achieves stability in pitch, roll, and yaw, essential for real-time UAV control.

---

#### Objectives
- Develop a **mathematical model** of the quadcopter dynamics to understand and control its behavior.
- Design a **PID control strategy** to manage the quadcopter’s pitch, roll, and yaw for stable flight.
- Simulate quadcopter dynamics using **MATLAB Simulink** to validate the PID control system.
- Construct a physical prototype of the quadcopter, embedding the PID control logic, and perform real-time tests to achieve stable flight.

---

#### System Overview
The quadcopter system comprises **four motors** arranged symmetrically, with two rotating clockwise and two counterclockwise to balance rotational torques and maintain stability. It is controlled via a **radio controller** which communicates wirelessly through a transceiver. Key components of the system include:
- **Ardupilot Mega board** with an **ATMEGA 2560 microcontroller** to manage signal processing and control outputs.
- **Inertial Measurement Unit (IMU)**, which incorporates accelerometers and gyroscopes to measure the quadcopter’s orientation and angular velocities, providing feedback for stability.
- **BLDC Motors** and **Electronic Speed Controllers (ESCs)** to control motor speeds based on PWM signals, allowing directional adjustments (up, down, left, right, forward, and backward).

---

#### Modeling and Simulation

1. **Mathematical Modeling**:  
   The project begins with deriving mathematical equations that represent the quadcopter's motion in three degrees of freedom: pitch, roll, and yaw. The **Newton-Euler formalism** is used to obtain the dynamic equations of the quadcopter. The key parameters include:
   - **Moment of Inertia** along each axis (I<sub>xx</sub>, I<sub>yy</sub>, I<sub>zz</sub>)
   - **Thrust and drag coefficients** (b and d)
   - **Arm length** (distance from each motor to the center)

2. **PID Controller Design**:
   - **Control Strategy**: A PID controller is used to manage the quadcopter's pitch, roll, and yaw. By adjusting proportional (K<sub>p</sub>), integral (K<sub>i</sub>), and derivative (K<sub>d</sub>) gains, the PID controller stabilizes the quadcopter's movements to maintain a desired orientation.
   - **Laplace Transform**: The system's differential equations are converted into the Laplace domain, allowing for transfer function modeling of the pitch, roll, and yaw controllers.

3. **Simulation in MATLAB Simulink**:
   - **Simulation Model**: The MATLAB Simulink model includes three PID controllers for pitch, roll, and yaw, each controlling a specific degree of motion. A step input simulates changes in each axis, and the controllers respond by adjusting motor speeds to achieve the desired stability.
   - **Result Analysis**: Simulation results showcase the quadcopter’s responses to pitch, roll, and yaw inputs, demonstrating the effectiveness of the PID controller in stabilizing these movements. The simulation graphs reveal how quickly the quadcopter can return to its set orientation after a disturbance.

---

#### Hardware Implementation

After verifying the control strategy through simulation, a physical prototype was constructed using:
- **Quadcopter Frame**: Made from lightweight materials, with dimensions of 450mm in width and 55mm in height, ensuring adequate space and stability.
- **BLDC Motors**: Four motors rated at 12V and 1000 rpm/volt, equipped with propellers for controlled lift.
- **Ardupilot Mega Flight Controller**: The pre-tuned PID logic is uploaded to this microcontroller, enabling real-time control of the quadcopter.
- **2.4 GHz Radio Controller**: Allows remote control input for adjusting the quadcopter’s position and speed.
- **Battery**: A 2300mAh Li-Po battery powers the quadcopter, ensuring sufficient energy for sustained flight.

**Hardware Testing**: The quadcopter prototype was tested to evaluate the real-world performance of the PID controller. After refining the P, I, and D gains, stable flight control was achieved. The hardware prototype successfully responded to control inputs, demonstrating stable pitch, roll, and yaw control.

---

#### Results
- **Simulation Results**: The MATLAB Simulink model provided precise control over the quadcopter’s movements, with the PID controller maintaining stability across all axes. The pitch and roll responses were nearly identical due to symmetrical moment of inertia values, while the yaw response differed due to variations in drag and inertia along the z-axis.
- **Prototype Testing**: The hardware implementation of the PID control on the Ardupilot Mega board provided similar stability results as seen in the simulation. The quadcopter prototype successfully maintained stability in flight, with desired movements achieved by varying motor speeds.

---

#### Conclusion and Future Work
This project successfully demonstrates the design and implementation of a PID-controlled quadcopter. The modeling, simulation, and physical testing validate the PID controller’s ability to stabilize pitch, roll, and yaw movements in real-time. Future enhancements could involve using more advanced control techniques, such as:
- **Adaptive PID Control**: Incorporate adaptive tuning methods (e.g., Ziegler-Nichols) for dynamic adjustments to the PID gains, improving response time and stability in varying environmental conditions.
- **Sensor Fusion**: Integrate additional sensors (e.g., GPS and barometers) for more comprehensive control and positioning, enabling applications beyond basic hovering, such as autonomous navigation.
