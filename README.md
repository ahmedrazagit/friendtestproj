# Fuzzy Logic Parking System

This project demonstrates the implementation of a fuzzy logic-based control system for autonomous parking. The system supports both parallel and perpendicular parking, using a tank robot model in CoppeliaSim.

## Table of Contents
- [Features](#features)
- [Demonstrations](#demonstrations)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)


## Features
- Fuzzy logic model for velocity control based on distance to obstacles.
- Parallel parking and exiting functionality.
- Perpendicular parking capability.
- Visualization of distance and velocity data.


## Demonstrations

### Parallel Parking and Exiting Demo
[![Parallel Parking Demo](https://github.com/user-attachments/assets/5e4e423c-e734-405c-abf5-b57d54f46a4c)](https://github.com/user-attachments/assets/5e4e423c-e734-405c-abf5-b57d54f46a4c)

### Perpendicular Parking Demo
[![Perpendicular Parking Demo](https://github.com/user-attachments/assets/2582ed66-9721-40a2-92f3-41ad776a1b4e)](https://github.com/user-attachments/assets/2582ed66-9721-40a2-92f3-41ad776a1b4e)

## Installation

1. **Download CoppeliaSim**
   - Visit [Coppelia Robotics](https://www.coppeliarobotics.com) to download and install CoppeliaSim for your platform.

2. **Clone the Repository**
   ```bash
   git clone <repository_url>
   cd <repository_folder>
   ```

3. **Install Python Dependencies**
   ```bash
   pip install -r requirements.txt
   ```

4. **Open the CoppeliaSim Scene**
   - Navigate to the project folder and double-click on `map.tt` to open it in CoppeliaSim.
   - Then click the start button from the menu bar.

## Usage

1. **Run the Python Script**
   - Open a terminal in the project folder and execute:
     ```bash
     python start.py
     ```

2. **Follow Terminal Instructions**
   - The terminal will display instructions to guide the simulation.

3. **Observe the Parking Simulation**
   - The robot will perform parking maneuvers in the simulation environment. Adjust parameters in the `start.py` file for different parking scenarios.

## Project Structure

```
.
├── map.tt                # CoppeliaSim scene file
├── start.py              # Main Python script to run the simulation
├── fuzzy_model.py        # Fuzzy logic model implementation
├── requirements.txt      # Python dependencies
├── README.md             # Project documentation


