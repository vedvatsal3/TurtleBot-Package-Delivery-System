# Assignment 03

As you now have different test environments ready for your software, this assignment focuses on making key design decisions and developing a functional prototype in the simulated environment.

## Objective
With the test environments in place, the next step is to identify the must have requirements for successful application and define a clear software architecture to support them, while keeping optional requirements in mind for future integration. Developing a working prototype will enable you to iteratively evaluate and refine your design choices. This process lays the foundation for robust testing, feature integration, and future system scaling.

## Task
1. **Review and Refine Requirements**: Reassess the feedback from the first assignment and any additional insights from the client. Distinguish between **must-have** and **optional** features for your prototype based on practical constraints and client expectations
2. **Analysis of missing software components and tool selection:** Review what is already available and what software components are missing. Identify appropriate tools, libraries, and frameworks needed to complete the prototype. If not already done, refer to the worksheets to setup basic navigation functionalities
3. **Prototype**: Your prototype should implement the following core features:
	- **Robot deployment:** Spawn two or more robots at different locations in the indoor environment, preferably at the docking stations
	- **Docking station:** Define same number of docking stations as robots. They can be for example represented as named locations specified in a config file. Note that the robots could dock to any of the docking station, and they are not one to one mapped
	- **Interaction logic (e.g., receiving tasks, confirming delivery, etc.):** an interface to specify the the delivery task, which includes pickup and delivery location. This should allow appropriate robot to execute the task while having a functionality to confirm whether the object was successfully picked up and delivered. These confirmations could be implemented as user acknowledgments

## Deliverables
- A **presentation** outlining the identified functionalities, system components, and the tools and libraries used to implement them. You may use a format of your choice—slides or a structured document—that is suitable for presenting
- A **system architecture diagram** showing how different modules interact
- **Detailed instructions** to run the prototype. Be prepared to demonstrate your prototype during the oral examination

**Note:** Grading will be based on your final presentation. All materials used in your presentation must be submitted by **22:00 HRS on June 6th, 2025**, under the `submission_files` folder under the respective assignment

**PS:** For support or to report bugs, please join the Discord server: https://discord.gg/NGWmp9qW