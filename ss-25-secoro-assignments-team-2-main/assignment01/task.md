# Assignment 01

You are a team of robotic software developers. Based on your client’s request, you are tasked with defining the requirements and developing a fully functional solution for a multi-robot delivery system.

In addition to the features explicitly requested by the client, you should also identify and implement critical functionality related to **safety, security, and reliability**, even if it was not directly mentioned. You might also have to consider the characteristics of the robot and the environment while doing so. These aspects are essential for ensuring the trustworthiness and acceptance of autonomous robotic systems in real-world environments.

## Outline 

The following is a description of the scenario provided by a client seeking an indoor, multi-robot delivery solution for an indoor office environment.

> **To whom it may concern,**
>
> I am an employee of a research group and responsible for improving the efficiency of our internal operations. With the increasing exchange of mails, shared tools, and devices among our team, we are exploring an automated solution to support internal logistics. Our goal is to simplify and automate the delivery of small objects between rooms and staff members, thereby improving team productivity.
>
> Our office layout consists of several rooms connected via a corridor. On a daily basis, we frequently need to deliver items to specific employees or rooms for meetings, collaborative projects, or routine deliveries.
>
> To address this, we are interested in a system that utilizes two real TurtleBot 4 robots capable of autonomous indoor navigation to carry out package delivery tasks. These robots should:
> - Pick up packages from any employee intending to send an object.
> - Navigate through the office environment while avoiding obstacles.
> - Deliver the packages reliably to a target room or a specific person.
>
> We already have two operational TurtleBot 4 platforms. The key features we require from the system are as follows:
> - Robots must be able to **navigate autonomously** within the office environment.
> - Deliveries should be performed in the **most efficient and timely manner** possible.
> - Users should be able to **specify a target room or a staff member** as the delivery destination.
> - If the **destination becomes inaccessible**, the robot must **return the package to the sender**.
> - The robots must **avoid collisions** with people or objects.
> - Robots should operational all the time.
> - The system should ensure that **packages are delivered to the correct location or person**
> - The software system should be **scalable in the future to support additional robots** as the office expands.
>
> We would be happy to hear proposals from you on how this solution could be implemented to meet our needs.

## Task

Based on the client request above, your task is to define a detailed set of requirements for the multi-robot package delivery system. For guidance on writing effective requirements, please refer to [1][2][3] in the Resources section where different approaches are mentioned.

Each requirement must include clear acceptance criteria to verify its successful implementation. In addition, you should define a set of project milestones along with a projected timeline indicating when each milestone is expected to be achieved.

Further details are provided below:

1. **Extract and define a complete list of functional and non-functional requirements.**
    - _Example:_ A functional requirement could be: “Robots shall detect obstacles and stop within 0.5 seconds of proximity alert.”
    - _Example:_ A non-functional requirement might be: “The system shall allow delivery requests to be initiated via a web interface.”
        
2. **Formulate clear and testable acceptance criteria for each requirement.**
    - Each requirement must include conditions under which it can be considered fulfilled. This can be more than one and should include all edge cases.
    - _Example:_ For the obstacle avoidance requirement, an acceptance criterion might be: “During a test run with simulated dynamic obstacles, the robot does not collide in 10 out of 10 trials.”
        
3. **Define a set of development milestones that cover the complete development lifecycle.**
    - Milestones should include phases such as requirement analysis, system design, implementation, integration, testing, and deployment.
    - _Example Milestone 1:_ "Navigation system functional in simulation by Week 3."
    - _Example Milestone 2:_ "Package pickup and drop-off routine integrated with destination input by Week 5."
        
4. **Propose a realistic timeline for completing the project.**
    - Consider factors such as team capacity, system complexity, and time needed for testing and debugging while deciding the timeline for the milestones. 
        
5. **Be prepared to justify your choices during milestone reviews.**
    - You will present all your requirements, milestones, and timeline, either as a document or as slides. Any of the team members should be able to explain any questions the examiners have. Evaluation will be individual grading and this will be considered towards your final grade.
    - **Evaluation**: Each student will be graded individually. The assessment will focus on the **completeness**, **clarity**, and **feasibility** of your requirements, acceptance criteria, milestones, and timeline. You will be challenged on your decisions to encourage deeper reflection and critical thinking.

You are free to select an appropriate format for expressing the requirements. This could be *user stories* [4], *behaviour-driven development* [5], *EARS* [3] or a mix of different formats maybe in combination with risk assessment techniques as presented during the lecture.  

## Deliverable
- The grading will be completely dependent on your presentation. All material used for presentation should be submitted by **22:00 HRS on the 29th of April, 2025** under submission_files folder under respective assignments. Please make sure to keep the submissions concise which allows you to present the submission completely in designated time.

The grading will be entirely based on your presentation. All materials used in the presentation must be submitted by **22:00 HRS on April 29th, 2025** in the `submission_files` folder under the respective assignment. Please ensure your submission is concise and structured in a way that allows you to cover all content within the allotted presentation time (~25 minutes per team -> 5 minutes per person).

## Resources
[1] https://mlip-cmu.github.io/book/06-gathering-requirements.html?highlight=requirement#requirements-evaluation

[2] https://docs.google.com/presentation/d/1lHlHBVp0VXUlOp3s9hLqi5Qc45CaguopoqGNud8i1j4/edit#slide=id.g20758b59f60_0_35

[3] https://medium.com/paramtech/ears-the-easy-approach-to-requirements-syntax-b09597aae31d

[4] https://dl.acm.org/doi/10.5555/984017

[5] https://www.oreilly.com/library/view/bdd-in-action/9781617297533/