# Robot Simulator

2D online Robot Simulator platform for testing state-of-the-art path planning algorithms in environments with dynamic obstacles.

## Installation

    $ git clone https://github.com/ggldnl/Robot-Simulator
    $ cd Robot-Simulator
    $ conda env create -f environment.yml
    $ conda activate robot_simulator

## Algorithms

```
Search-based Planning
├── Breadth-First Searching (BFS)
├── Depth-First Searching (DFS)
├── Best-First Searching
└── A*
Sampling-based Planning
├── RRT
├── Dynamic-RRT
└── RRT*
```

## TODO

- [ ] Introduce new frontend and polling mechanism to allow the frontend to poll the server for data
- [ ] Update documentation
- [ ] Add save capabilities
- [x] Add load capabilities
- [ ] Fix inverted angles in link origin (URDF parser)
- [ ] Add typing and input checking
- [ ] Provide more control on map generation (e.g. number of each type of obstacles, how to spatially align the static ones and so on...)
- [ ] Interact with obstacles on mouse click (accelerate them, stop them, ...) 
- [ ] CRUCIAL to refactor search algorithm and introduce a class hierarchy to share code (e.g. nodes, drawing geometry production method, ...)
- [ ] Add missing search algorithms
