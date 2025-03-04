# Dynamic Modeling of a 6-DOF Robotic Arm

## Project Overview

This project focuses on the dynamic modeling of a 6-degree-of-freedom (6-DOF) robotic arm for industrial material handling applications. The goal is to analyze the forces and torques required at each joint to transport payloads along predefined trajectories efficiently. The study includes forward kinematics, trajectory planning, and dynamic modeling, implemented in MATLAB.

## Key Features

- **Forward Kinematics**: Uses Denavit-Hartenberg (DH) parameters to determine the end-effector position.

- **Trajectory Planning**: Implements quintic polynomial interpolation for smooth motion.

- **Dynamic Model**: Uses Lagrangian dynamics to compute joint torques based on robot inertia, mass, and gravitational effects.

- **Simulation in MATLAB**: Evaluates torque requirements for different payloads and trajectories.


## Requirements

- MATLAB (Tested on MATLAB R2023)

- Symbolic Math Toolbox (for dynamic modeling)

## Running the Code

Clone this repository:

git clone https://github.com/NateDixon47/Dynamics-Project.git

Open MATLAB and navigate to the project directory.

Run Final.m to execute the full simulation.

Modify payload parameters in dynamicModel.m to test different cases.

## Results & Findings

- Simulations showed expected torque variations across different payloads.

- Joints with the longest moment arms exhibited the highest torques

- The model demonstrated scalability for handling increased payloads, making it useful for industrial robotics applications.