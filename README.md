# RRT* Path Planning Algorithm with Visualization

This script implements the **Rapidly-exploring Random Tree Star (RRT\*)** path planning algorithm
with real-time visualization using `matplotlib`. RRT\* is an extension of the RRT algorithm that improves the path by rewiring the tree, aiming to find an optimal path from a start point to a goal in an environment with obstacles.

## Features

- **RRT\* Algorithm**: Efficiently finds an optimal path to the goal by iteratively expanding a tree from the start position.
- **Random Obstacles**: Generates random circular obstacles in the environment to simulate real-world constraints.
- **Real-time Visualization**: Animates the tree growth and final path using `matplotlib`.
- **Collision Detection**: Ensures the path avoids obstacles during the search process.

## Getting Started

### Prerequisites

Make sure you have Python installed along with the following libraries:

- `numpy`
- `matplotlib`

You can install the dependencies by running:

```bash
pip install numpy matplotlib
```

## Running the Code
### Clone the repo to your local machine
```bash
git clone https://github.com/Jefferson-Aggor/rrt-.git
```

### Navigate to the project directory
```bash
cd rrt-
```

### Run the main.py script to execute the RRT* algorithm with real-time visualization
```bash 
python rrt.py
```

## How it Works
- `Start and Goal: 
The algorithm starts from a specified start point and attempts to reach a goal point in a 2D grid environment.`

- `Obstacles:
Circular obstacles are randomly generated with random positions and sizes. The algorithm will attempt to avoid these obstacles while planning the path.`

- `Tree Expansion: 
The RRT* algorithm iteratively generates random nodes, connects them to the nearest node in the tree, and rewires the tree if a more optimal path is found.`

- `Path Visualization: 
As the tree grows, the nodes and edges are displayed in real-time. Once the goal is reached, the final path is highlighted.`

## Parameters
You can modify the following parameters in the code to suit your environment:

- `start: Start position, e.g., [1, 1]`
- `goal: Goal position, e.g., [18, 18]`
- `num_obstacles: Number of random obstacles in the environment.`
- `map_size: The size of the environment, e.g., [20, 20]`
- `step_size: Step size for the algorithm to expand the tree.`
- `max_iter: Maximum number of iterations before stopping the algorithm.`

## Visualization
![image](https://github.com/Jefferson-Aggor/rrt-star/blob/main/rrt_star_animation.gif)
