# ROS 2 Actions: Part 6 Research Question Answers

## 1. Describe a robotics task where an Action is superior to a Service. Explain why.

A robotics task where an Action is superior to a Service is navigating to a waypoint autonomously.
Driving to the waypoint takes time, requiring constant progress monitoring, and may need to be interrupted or stopped if a new command is issued or an exception is detected (object found or obstruction, for example).
This is because Services are meant for instant or short tasks, while Actions support long-running functions, feedback during execution, and the ability to cancel or preempt a request/command.
Since navigation requires all three, Actions are superior to Services in this task.

## 2. What are the three main components of an Action (goal, result, feedback)? Describe the purpose of each.

| Component | Purpose |
|----------|---------|
| **Goal** | Defines what the robot is to do (navigate to a target location, grasp an object). |
| **Feedback** | Provides updates about progress while the action is running (current position, completion progress). |
| **Result** | Reports the final outcome (location reached, task failed). |

## 3. Why is the ability to preempt (cancel) a goal crucial in robotics?

The ability to preempt or cancel a goal is crucial because robots must react to changing environments and higher-priority commands.
Examples include stopping motion if a human steps in front of the robot, canceling current navigation if a new destination is issued, and ending a task if an emergency or safety stop occurs.
Without preemption, the robot may continue executing obsolete or unsafe commands, leading to unintentional behavior or accidents.
