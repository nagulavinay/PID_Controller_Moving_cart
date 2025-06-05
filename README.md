# PID_Controller_Moving_cart

# Overview
This repository contains the implementation and simulation of a PID controller for stabilizing an inverted pendulum system, developed as part of the Embedded Systems Laboratory at our university. The code is written in C++ and simulates real-world control challenges such as delay and jitter in sensor-to-controller communication.

# Objectives
Understand the dynamics and instability of the inverted pendulum system.

Implement a discrete-time Proportional-Integral-Derivative (PID) controller in C++.

Tune the PID parameters (Kp, Ki, Kd) for optimal stability and response.

Simulate the effects of different reference angles, as well as delay and jitter in the control loop.

Analyze the performance, stability, and challenges of PID control for dynamic systems.

# Significance
Stability in control systems is crucial for predictable, accurate, and safe operation especially in robotics, automation, and industrial environments. PID controllers remain the industry standard for such applications due to their simplicity and effectiveness.

# Methodology
Sensors: Simulated sensors provide the pendulum angle and cart position.

Controller: The PID controller calculates the corrective force based on the error between the desired and actual state.

Discrete-Time Implementation: The algorithm is tailored for digital simulation with real-time step updates.

Delay & Jitter Simulation: Communication delays and timing jitter are introduced in the simulation to mimic real-world conditions and test controller robustness.

# Repository Structure
Controller.h & Controller.cpp:
Contains the PID controller class and logic, including parameter initialization, discrete-time PID output calculation, and output clamping.

Simulator.cpp:
Handles the simulation of the inverted pendulum, including the introduction of delay and jitter in the sensor data, as well as the main update loop.

README.md:
Project documentation and instructions.

Other files:
Additional configuration files and code as required by the assignment.

# Key Features
Modular C++ code with clear separation of simulation and controller logic.

Functions to update controller parameters and output limits at runtime.

Simulation of delay and jitter in the control loop, with wraparound buffer management.

Results and observations discussed in detail in the lab report.

# Results
PID gains achieved for best performance:

Kp: 6171.08

Ki: 231.21

Kd: 214.98

The tuned PID controller effectively stabilized the pendulum at reference angles 0, +0.1, and –0.1 radians, even under delay and jitter conditions.
