# ChaseR

An AI driven Unreal Engine 5 project done as coursework for the graduate course 'Game Artificial Intelligence' demonstrating advanced pathfinding, decision-making, and behavior tree techniques. This repository also documents the step-by-step learning and development process for multiple class assignments.


# Table of Contents

-   [Overview](#overview)
-   [Implementation Overview](#implementation-overview)
-   [Installation & Setup](#Installation-&-Setup)


## Overview

**ChaseR** is a  **Unreal Engine 5 (UE5)** project that merges **C++** and **Blueprint**.

  The toy game starts with 2 basic characters- A **player** and an **evil R2D2 robot**. 

Throughout its development, the AI evolves from basic movement demos  of the robot to robust pathfinding and behavior trees. Some cool things that I implemented are:

- **Robust A\* pathfinding** (with path smoothing) navigating complex environments  
- **Dijkstra-driven positioning** empowering AI to hold, hide, or flee strategically  
- **Behavior Trees** orchestrating unique AI archetypes (Leader, Soldier, Fodder, etc.)  
- **Engaging player interaction**, complete with projectile combat and health systems  


## Implementation Overview

Each subsection in the implementation overview corresponds to a different assignment.

### Figure 8
Unreal Engine 5 is overwhelming to begin with and so thing I implemented was a simple figure-8 movement for the robot.  This was done through a simple sinusodial-like movement. The biggest challenge here was the **Drifting** problem wherein the bot would drift in direction while moving in the figure-8.



https://github.com/user-attachments/assets/f8ae9ac1-3af4-4232-8da8-cda93057e6d7



### A* Pathfinding and Chase Behavior
To enable **chase behavior**, I integrated A* pathfinding into the AI using the *UGAPathComponent*. The AI now dynamically calculates the best path towards the target. I implemented a custom **Line-Trace Algorithm** to ensure that the robot avoids any obstacles in its path. After the robot discovers a path, A **smoothing function** was added to ensure fluid movement along the path.



https://github.com/user-attachments/assets/8e07c749-23ae-4221-8d0f-c7aaefaf30fc


### Spatial Functions

I used **Dijkstra’s algorithm** to calculate the shortest path to the target, creating a **distance map**. For each potential position, I evaluated **line-of-sight (LOS)**, **target distance**, and **path distance** in a separate **GridMap**.

These values were processed through the **spatial function's response curve** to determine the best position based on the AI's behavior, such as **hide**, **hold**, or **flee**. This allowed the AI to choose the optimal destination based on its current goal.


https://github.com/user-attachments/assets/c5a402de-35e6-41ca-807c-feadf14ba273


In the video, the first clip shows the hold behaviour, the second shows the flee behaviour and the third shows the hide behaviour. The Colors here indicate the values evaluated using Spatial functions of these different behaviours. Red means higher value i.e. the AI robot wants to go to the red squares.

### Behavior Trees
Using Behavior Trees, I designed AI behavior for different character types: Leader, Soldier, and Fodder. Each type had distinct actions such as charging or self-destructing based on the situation. The trees were driven by a blackboard, allowing the AI to dynamically adjust its behavior in response to the game environment.

## Installation & Setup

1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/ChaseR.git
   cd ChaseR
Open in Unreal Engine 5:

Double-click the .uproject file, or open it through the Epic Games Launcher.
Let the project compile any necessary modules.
Compile:

If using an IDE like Visual Studio or Rider, open the .sln file and build it.
Alternatively, let Unreal Editor handle the compile on project load.
Run:

Press Play in the Unreal Editor to see the AI in action.
