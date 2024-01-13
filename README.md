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
- [x] Rework map and remove moving obstacles. Add obstacles on mouse click instead
- [x] Add obstacles on mouse click
- [x] Redefine/Uniform websocket channels names
- [ ] Fix bug with region query in Map class
- [ ] Uniform query and query_region methods in Map class
- [ ] Rework world json to use map serialization instead of plain polygons serialization
- [ ] Add missing search algorithms
- [x] Redefine world serialization
- [ ] Provide thread safe rtree implementation
- [x] Finish left sidebar
- [x] Add another load button to left sidebar to load a custom URDF
- [x] Make the save button actually save the last map data received
- [ ] Find a way to specify obstacles shape (rectangle, polygon, ...)
- [ ] Find a way to choose the behavior of the map (moving/spawning obstacles)
- [ ] Fix inverted angles in link origin (URDF parser)
- [ ] Add typing and input checking 
- [ ] Update documentation
