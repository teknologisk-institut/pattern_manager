## Pattern Manager

This package implements a ROS package for defining, configuring, and working with patterns in robotics applications. This allows a robotics application developer to easily define a pattern, or group of patterns, for batch processing of structured parts, e.g. palletizing operations. 

### Status

We have implemented the ROS package purely in Python, with the key components necessary for working with patterns throughout the rest of the FTP:

#### Structure and Behaviour
- All patterns and transforms are represented by a single class: `XForm`.
- The `XForm` (as in transform) class is implemented as a tree node. Each transform is contained within an `XForm` node. The `XForm` class contains all necessary functions to manage and configure the transform tree structure.
- Patterns of transforms simply consist of a parent `XForm` object with any number of child-`XForm` objects (hence a tree structure) positioned in the respective pattern shape, relative to the parent.
- `XForm` nodes contain an attribute describing whether the node is “active”, or not. Setting this property allows choosing which transforms should be iterated and affords the functionality of iterating through transforms sequentially.
- Pattern classes are implemented as plugins, allowing for easy extensibility of the set of pattern types.
- A standard set of pattern plugins are implemented for generating some primitive pattern types (i.e. linear, rectangular, circular, and scatter).
- With the flexible nature of the tree structure it is possible to group transforms and/or patterns by creating a new `XForm` object and assigning it as the parent of various patterns or sub-groups (see `grp1` and `grp2` <a href="#example-tree-structure">here</a>).
- The `XForm` tree can be stored as a .yaml file, which in addition to allowing users to persistently store their configuration, also allows for editing of the tree by hand using a text editor.
- Saved (or manually constructed) .yaml files can also be loaded to generate a reflective `XForm` tree.
- All required functionality for creating and managing patterns and transforms is enabled through ROS services.
- Geometric information about the position and orientation of each transform is published and broadcasted to the frame and coordinate management tool, tf. The data broadcasted about the transforms from within the node can therefore be tracked and visualized at any time using tools such as the ROS 3D visualization tool, RViz.
- The node also publishes Marker Messages which are used to draw primitive, color-coded, markers in RViz at each transform position. These markers are used to indicate whether a transform is active and if it is the next transform in the iteration.

#### Graphical User Interface
- A considerable effort was also put into developing a GUI (<a href="https://github.com/teknologisk-institut/rqt_pattern_manager">rqt_pattern_manager</a>) to enable simple and intuitive interaction with the Pattern Manager node.
- As the GUI was designed as an RQT plugin it is very easy to integrate into existing ROS software, which makes particularly good sense in the case of RViz, which accepts RQT plugins as extended program functionality. This means the GUI can be opened from within RViz as part of the editor’s environment.
- All the same functionalities available by interacting directly with the Pattern Manager ROS node, are available from within the RQT plugin.
- The GUI additionally allows for drag-and-drop functionality for reordering transforms and re-parenting transforms and patterns of transforms.

#### Documentation, testing, integration
- The Pattern Manager ROS package has been thoroughly documented:
- The source code has been documented with reStructuredText docstrings.
- API documentation has been generated using rosdoc_lite (utilizing Sphinx).

#### Work-in-progress
- Code coverage
- Extensive behaviour and API documentation, and tutorials for the ROS wiki page.

### Example tree structure

Patterns of transforms can be grouped with other patterns. Additionally, these groupings can also be sub-grouped. This enables the user to combine multiple (groups of) patterns, which in turn allows seamless iteration through multiple patterns, as if it was one pattern.

The following is an example of such a structure:
```
    root [tf0]                      # <transform-name> [<transform-number>]
    ├── grp1 [tf1]                  # transform as pattern group/container
    │   ├── lin1 [tf2]              # ex. linear pattern of transforms
    │   │   ├── lin1_1 [tf3]           
    │   │   ├── lin1_2 [tf4]
    │   │   ├── lin1_3 [tf5]
    │   │   └── ...
    │   └── lin2 [tf11]
    │       ├── lin2_1 [tf12]           
    │       ├── lin2_2 [tf13]
    │       └── lin2_3 [tf14]
    ├── grp2 [tf6]
    │   ├── grp3 [tf7]
    │   │   ├── rect1 [tf8]         # ex. rectangular pattern of transforms
    │   │   │   ├── rect1_1 [tf9]
    │   │   │   ├── rect1_2 [tf10]
    │   │   │   └── ...
    │   │   └── ...
    │   └── ...
    └── ...
```

### Acknowledgement

Supported by [ROSIN - ROS-Industrial Quality-Assured Robot Software Components](http://rosin-project.eu/) 

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 732287.
