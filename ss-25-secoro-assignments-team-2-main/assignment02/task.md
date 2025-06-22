# Assignment 02
 
Following the previous assignment, where you used the client's email to identify system requirements and acceptance criteria for a multi-robot indoor delivery solution, this assignment focuses on modeling the **physical indoor environment** in which the robots will operate.

## Objective
As a next step toward implementing the solution, it is essential to simulate the environment that the robots will perceive and operate in. This simulated environment should reflect real-world variations and constraints to allow thorough testing of your system before real-world deployment.

You will gather additional information about the operational environment, simulate the layout using a **domain-specific language (DSL)**—**FloorPlan DSL**—and visualize how robots interpret and interact with that space. This will later support the implementation of navigation, obstacle avoidance, and interaction with delivery targets.

## Task

#### 1. Clarify with the Client and Update System Requirements
- To better understand the challenges faced by mobile robot software developers and the importance of diverse test environments, refer to [1]. This will help refine your system requirements and assist in coming up with the follow-up questions for the client
- Brainstorm and compile a comprehensive list of **follow-up questions** for the client that are necessary to accurately model the delivery environment. This could include desired information about room dimensions, lighting conditions, communication dead zones, perceivable and non-perceivable features, to name a few
- Reflect on the functional and non-functional requirements identified from the Assignment 01 and make suitable revisions to them based on the feedback during the oral examination

#### 2. Environment Modeling using FloorPlan-DSL
- Use the **FloorPlan-DSL** to model the office environment, incorporating insights from the client answers to your questions, feedback on first assignment, and from the initial client email
- Refer to [2] for guidance on the tooling and its role in testing various simulation scenarios, which will be covered during the tutorial session before the release of this assignment 
- Ensure the model includes possible variations (e.g., narrow corridors, open spaces, movable obstacles) that reflect real-world testing challenges and help validate all acceptance criteria. You could also build multiple of them for particular set of tests

#### 3. Visualization and Simulation
- Using the generated environment files, **spawn TurtleBot 4 robot** (optionally two of them) in the simulation environment
- Visualize the robots' sensor data using **Rviz2**

## Deliverables
- Submit all questions you have for your client on the forum on stud ip by the **7th of May**, 17:00 HRS
- FloorPlan DSL input file (textx format)
- An image which shows the spawned robots in the environment, an image showing the sensor data in rviz2 and the corresponding launch files
- Generated environment files, including the 3d mesh files, map files, and gazebo world files representing the modeled world
- Slides for your presentation explaining the decisions made while modeling the environment

**Note:** The grading will be entirely based on your presentation. All materials used in the presentation must be submitted by **22:00 HRS on May 13th, 2025** in the `submission_files` folder under the respective assignment.

## References
[1] Parra, S., Schneider, S., & Hochgeschwender, N. (2023, October). A thousand worlds: scenery specification and generation for simulation-based testing of mobile robot navigation stacks. In 2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 5537-5544). IEEE.

[2] Ortega, A., Parra, S., Schneider, S., & Hochgeschwender, N. (2024). Composable and executable scenarios for simulation-based testing of mobile robots. Frontiers in Robotics and AI, 11, 1363281.