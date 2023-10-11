# Robot Simulator

2D online Robot Simulator platform for testing state of the art path planning algorithms in environments with dynamic obstacles.

## Roadmap

- [ ] pass the points of the polygon directly from the backend 
    to test the drawing capability for custom shapes
- [x] scale the polygons on the frontend such that their dimension
    is correct at each scale
- [x] find a way to buffer the shapes (frame) and send them 
    all at the same time to the frontend
- [ ] define the control loop
- [ ] move from current, complex, translation and scale logic to
    translate() and scale() on the canvas object
