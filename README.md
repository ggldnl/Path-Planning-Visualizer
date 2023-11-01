# Robot Simulator

2D online Robot Simulator platform for testing state-of-the-art path planning algorithms in environments with dynamic obstacles.

## Installation

    $ git clone https://github.com/ggldnl/Robot-Simulator
    $ cd Robot-Simulator
    $ conda env create -f environment.yml
    $ conda activate robot_simulator

## TODO

- [x] write URDF for cobalt
- [x] define environment
- [ ] remove annoying checkbox (upper left corner) that loads before the rest of the page 
- [ ] redefine communication between backend and frontend (make it lighter)
- [x] replace buttons with radio buttons for controller and planner menus
- [x] highlight the current choice in controller and planner menus
- [x] disable the planner menu if the current planner is joystick
- [ ] update documentation
- [ ] add save capabilities
- [x] add load capabilities
- [ ] ~~add sensors support to URDF parser~~
- [ ] fix inverted angles in link origin (URDF parser)
- [x] scale the map to match real world measures
- [ ] move from current, complex, translation and scale logic to
    translate() and scale() on sort of a contextManager object in javascript
- [ ] add typing and input checking
- [ ] add radar in gui to see sensors read from the robot
- [ ] provide more control on map generation (e.g. number of each type of obstacles, how to spatially align the static ones and so on...)
- [ ] interact with obstacles on mouse click (accelerate them, stop them, ...) 





