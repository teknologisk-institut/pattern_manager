## Pattern Manager

This package implements a ROS package for defining, configuring, and working with patterns in robotics applications. This allows a robotics application developer to easily define a pattern, or group of patterns, for batch processing of structured parts, e.g. palletizing operations. 

### Status

We have implemented the ROS package purely in Python, with the key components necessary for working with patterns throughout the rest of the FTP:
- An `XForm` class is implemented as a tree node. Each transform is contained within an `XForm` node. The `XForm` class contains all necessary functions to manage and configuring the transform tree.
- how are transforms grouped?(parent, ref_frame)
- `XForm` nodes contain an attribute describing whether the node is 'active' or not. This property affords the functionality of iterating through transform, as well choosing which transforms should be iterable.
- Patterns of transforms simply consist of a parent `XForm` with a number of child `XForm`'s positioned in the respective pattern shape.
- Patterns are generated from a set of distinct pattern subclasses, which each have a unique reimplementation of a generator function.
- Patterns are implemented as plugins (via `pluginlib`), allowing for easy extensibility of the set of pattern types.
- A standard set of pattern plugins are implemented for generating some primitive pattern types (linear, rectangular, circular, and scatter).
- Patterns can be grouped be creating new `XForm` object and assigning it as the parent of the various pattern parents (see `grp` in <a href="#example-tree-structure">Pattern Manager</a>).


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
- Generate extensive behaviour and API documentation, as well as write tutorials for the ROS wiki page.

### Dependencies

We have not fully merged the dependencies into the rosdep repositories, so you will need to install the following Python dependencies, e.g. through `pip`:
```
pip install pluginlib
```

### Example tree structure

Patterns can be organized in groups, that can themselves be organized in groups. This enables a user to combine multiple (groups of) patterns, which in turn allows seamless iteration through multiple patterns, as if it was one pattern. 

The following is the structure in the example implemented in the node:
```

    root [tf0]                      # <transform-name> [<transform-number>]
    ├── grp1 [tf1]                    
    │   ├── lin1 [tf2]              # linear pattern of transforms
    │   │   ├── lin1_1 [tf3]           
    │   │   ├── lin1_2 [tf4]
    │   │   ├── lin1_3 [tf5]
    │   │   └── ...
    │   └── ...
    ├── grp2 [tf6]
    │   ├── grp3 [tf7]
    │   │   └── rect1 [tf8]         # rectangular pattern of transforms
    │   │       ├── rect1_1 [tf9]
    │   │       └── rect1_2 [tf10]
    │   │       └── ...
    │   └── ...
    └── ...
```

### Acknowledgement

Supported by [ROSIN - ROS-Industrial Quality-Assured Robot Software Components](http://rosin-project.eu/) 

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 732287.
