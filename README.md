# Thesis: Development of an Emergency Collision Avoidance System for Autonomous Vehicles

## Introduction
This repository contains the code and documentation for my thesis on the development of an emergency collision avoidance system for autonomous vehicles. The thesis aims to address the pressing need for sophisticated collision avoidance systems in autonomous vehicles to ensure safety and reduce the likelihood of accidents.

## Motivation
In the constantly changing field of autonomous vehicle technology, safety is still the top priority. Ensuring autonomous vehicles can navigate complex environments and react to unforeseen obstacles becomes increasingly important as they become a vital part of our transportation ecosystem. The requirement for sophisticated emergency collision avoidance systems is at the heart of this problem. This thesis is driven by the urgent need to give autonomous cars the capacity to respond quickly and efficiently to unforeseen obstacles and possible collision situations.

## Objective
This thesis’s main goal is to create, develop, and assess a sophisticated emergency collision avoidance system specifically for self-driving cars. This system will be painstakingly designed to allow self-driving cars to react appropriately to unforeseen obstacles or circumstances, reducing the likelihood of collisions and improving overall safety. In order to accomplish this main goal, we will start a thorough investigation that entails:
- The development of basic and emergency operating modes for autonomous vehicles.
- Addressing safety challenges through the design of collision avoidance strategies and integration of redundant on-board systems.
- Discussing different controlling techniques and algorithms.
- Seamless integration of Advanced Driver Assistance Systems (ADAS).
- Extensive simulation experiments conducted primarily within the MATLAB/Simulink environment to validate the feasibility and effectiveness of the emergency collision avoidance system.
- Using the system designed in MATLAB and validating it in a Real-Time simulation (RTL) environment using CarMaker software.

Eventually, the goal of this thesis is to advance the field of autonomous vehicle safety protocols and open the door to safer and more efficient transportation in a society that is becoming more and more automated.

## Repository Structure
- `code/`: Contains the MATLAB/Simulink code for implementing and testing the emergency collision avoidance system.
- `Development and Evaluation of an Emergency Collision Avoidance System for Autonomous Vehicles.pdf/`: Includes the compiled PDF document for the thesis documentation.
- `data/`: Any relevant data files or datasets used in the research.
- `references/`: Bibliography and reference files.
- `LICENSE`: License information for the repository.
- `README.md`: This README file providing an overview of the repository.

## How to Use
- Clone or download the repository to your local machine.
- Navigate to the relevant directories (`code/`, `documentation/`, etc.) to access the files.
- Refer to the documentation for detailed information on the research methodology, implementation details, and experimental results.

### CarMaker Integration
- The `CarMaker_test` folder contains the integrated model with Simulink, located at `CarMaker_Simulink_thesis/CarMaker_test/src_cm4sl/generic_20231123_1032.mdl`.
- You need to first run `cmenv.m` and then the model itself.
- After running the model (it will open in Simulink), click "Launch CarMaker GUI" and then run `TwoDLook_uptable.m` to load the necessary parameters.
- Note: You must have a CarMaker license to use these features.
- At `CarMaker_Simulink_thesis/CarMaker_test/SimOutput/Videos`, you can see the output videos for two test runs at a speed of 115 km/h. (More description can be found in the thesis PDF file)

## Acknowledgments
Special thanks to CarMaker for providing the license necessary to validate the model.

## License
A CarMaker license is needed to run the project. More information can be found on their website: [CarMaker Licenses](https://www.ipg-automotive.com/en/support/licenses/).

## Contact
For any inquiries or feedback, please feel free to contact me at [abdalfatah2000gg@gmail.com](mailto:abdalfatah2000gg@gmail.com).


