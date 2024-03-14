# Robot Simulator

2D online Robot Simulator platform for testing state-of-the-art path planning algorithms in environments with dynamic obstacles.

## Installation

    $ git clone https://github.com/ggldnl/Robot-Simulator
    $ cd Robot-Simulator
    $ conda env create -f environment.yml
    $ conda activate robot_simulator

## How to run

To run the application, from the main folder, simply do the following:

```
python application.py
```

This will launch a flask server; open the address on the console from a web browser to reach the web page.

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
- [x] Fix bug with region query in Map class
- [x] Uniform query and query_region methods in Map class
- [x] Provide thread safe rtree implementation
- [x] Rework world json to use map serialization instead of plain polygons serialization
- [x] Fix bug with search algorithms and new map (seems like sampling doesn't care about obstacles)
- [ ] Algorithms can't find the goal if we set it to positions outside the grid
- [ ] Add missing search algorithms
- [x] Redefine world serialization
- [x] Finish left sidebar
- [x] Add another load button to left sidebar to load a custom URDF
- [x] Make the save button actually save the last map data received
- [ ] Provide multi robot support
- [ ] Fix bugs with urdf loading (robot stops moving after few steps, maybe step size)
- [ ] Find a way to specify obstacles shape (rectangle, polygon, ...)
- [ ] Find a way to choose the behavior of the map (moving/spawning obstacles)
- [ ] Fix inverted angles in link origin (URDF parser)
- [ ] Add typing and input checking 
- [ ] Update documentation
