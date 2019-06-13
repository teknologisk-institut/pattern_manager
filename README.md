## Pattern Manager

This package implements a ROS package for defining, configuring, and working with patterns in robotics applications. This allows a robotics application developer to easily define a pattern, or group of patterns, for batch processing of structured parts, e.g. palletizing operations. 

### Status

We have implemented the ROS package purely in Python, with the key components necessary for working with patterns throughout the rest of the FTP:
- A Pattern parent class is implemented, allowing holding and iteration through a set of positions (internally stores as a list of geometry_msgs/Transform), as well as configuration of many additional parameters (parent frame, offset from parent frame, iteration order, etc.).
- A set of standard pattern implementations (linear, rectangular, circular, scatter) are implemented as plugins, allowing users to implement new pattern types by only writing a single function.
- A Group class is implemented, that can contain a set of patterns, or a set of groups, which allows easy structuring of multiple patterns in an application.
- A ROS node is available, at this point mostly for demonstration purposes, with services for selecting active pattern or group, iteration, etc. Patterns are published as Rviz markers, and the current active position, in the active pattern or group, is broadcast to the TF tree.  

To try out the demo mode of the ROS node, do the following:
- Download this repository to your Catkin workspace, install the dependencies (see below) and build.
- Start the node `rosrun pattern_manager pattern_manager`
- Start rviz, set fixed frame to `base_link`, add a TF view, and a MarkerArray, subscribing to `/pattern_manager/pattern_markers`
- From the command line:
  - Iterate through the active group through the service `/pattern_manager/iterate`
  - Select an active group (available groups are "g1", "g2", "g3" or "g4", see structure below) through `/pattern_manager/set_active` 
  - Reset a group through `/pattern_manager/reset`

#### Planned features

The following is implemented over the next couple of months:
- Implementation of format for defining parameters for an existing pattern type in a yaml file, and load this file to the pattern set through a ROS service
- Implement graphical interfaces in Rviz, that allow adding, deleting, configuring patterns through their (interactive) markers, as well as grouping/ungrouping patterns
- Implement methods for easily touching up a pattern from a set of known poses in the pattern
- Implement proper tests of modules and ROS nodes, establish coverage, enforced code scanners, implement CI, release to build farm
- Ensure that the full software works with ROS2 through the ros1_bridge ROS2 package
- Generate extensive behaviour and API documentation, as well as write tutorials for the ROS wiki page.

### Dependencies

We have not fully merged the dependencies into the rosdep repositories, so you will need to install the following Python dependencies, e.g. through `pip`:
```
pip install pluginlib bidict
```

### Example group and pattern tree-structure

Patterns can be organized in groups, that can themselves be organized in groups. This enables a user to combine multiple (groups of) patterns, which in turn allows seamless iteration through multiple patterns, as if it was one pattern. 

The following is the structure in the example implemented in the node:

    g0                          # Root (not accessible)
    ├── g3                      # Group "g3"
    |   ├── g1                  # Group "g1"
    │   |   ├── p1              # Pattern, linear
    |   │   │   ├── tf1         # Transform
    |   │   │   ├── tf2
    |   │   │   └── ...
    |   │   ├── p2              # Pattern, rectangular
    |   │   │   ├── tf1         # Transform
    |   │   │   ├── tf2
    |   │   │   └── ...
    │   └── g2                  # Group "g2"
    │       ├── p3              # Pattern, rectangular
    |       │   ├── tf1         # Transform
    |       │   ├── tf2
    |       │   └── ...
    |       └── p4              # Pattern, rectangular
    |           ├── tf1         # Transform
    |           ├── tf2
    |           └── ...
    └── g4                      # Group #g4"
        ├── p5                  # Pattern, circular
        │   ├── tf1             # Transform
        │   ├── tf2
        │   └── ...
        └── p6                  # Pattern, scatter
            ├── tf1             # Transform
            ├── tf2
            └── ...   


### Python module class diagram

![alt text](https://github.com/teknologisk-institut/pattern_manager/blob/master/doc/images/pm.svg "Logo Title Text 1")

### Acknowledgement

Supported by [ROSIN - ROS-Industrial Quality-Assured Robot Software Components](http://rosin-project.eu/) 

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 732287.
