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

- [x] Strip unnecessary stuff from robot stack (e.g. sensors, motors, etc)
- [ ] Rework map and remove moving obstacles. Add randomly spawned obstacles instead
- [x] Add obstacles on mouse click
- [ ] Add missing search algorithms
- [ ] Redefine world serialization
- [ ] Finish left sidebar
- [x] Add another load button to left sidebar to load a custom URDF
- [x] Make the save button actually save the last map data received
- [ ] Fix inverted angles in link origin (URDF parser)
- [ ] Add typing and input checking 
- [ ] Update documentation
