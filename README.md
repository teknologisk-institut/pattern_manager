## Pattern Manager

This package implements a ROS package for defining, configuring, and working with patterns in robotics applications. This allows a robotics application developer to easily define a pattern, or group of patterns, for batch processing of structured parts, e.g. palletizing operations. 

### Status

We have implemented the ROS package purely in Python, with the key components necessary for working with patterns throughout the rest of the FTP:
- An `XForm` class is implemented as a tree node. Each transform is contained within an `XForm` node. The `XForm` class contains all necessary functions to manage and configuring the transform tree.
- `XForm` nodes contain an attribute describing whether the node is 'active' or not. This property affords the functionality of iterating through transform, as well choosing which transforms should be iterable.
- Patterns of transforms simply consist of a parent `XForm` with a number of child `XForm`'s positioned in the respective pattern shape.
- Patterns are generated from a set of distinct pattern subclasses, which each have a unique reimplementation of a generator function.
- Patterns are implemented as plugins (via `pluginlib`), allowing for easy extensibility of the set of pattern types.
- A standard set of pattern plugins are implemented for generating some primitive pattern types (linear, rectangular, circular, and scatter).
- Patterns and transforms can be grouped by creating a new `XForm` object and assigning it as the parent of the various pattern/transform parents (see `grp1` and `grp2` in <a href="#example-tree-structure">here</a>).
- The XForm tree can be saved as a .yaml file, which allows for editing the tree in a text editor. Saved or manually constructed .yaml files can also be loaded to generate a reflective `XForm` tree.
- An RQT graphical user interface plugin has been designed (rqt_pattern_manager) for intuitive interaction with the Pattern Manager ROS node. The GUI affords then same functionalities as the ros services of this package.

#### Work-in-progress

- Extensive behaviour and API documentation, as well as writing tutorials for the ROS wiki page.

### Dependencies

We have not fully merged the dependencies into the rosdep repositories, so you will need to install the following Python dependencies, e.g. through `pip`:
```
pip install pluginlib
```

### Example tree structure

Patterns of transforms can be grouped with other patterns of transforms. Additionally, this groupings can also be grouped into . This enables a user to combine multiple (groups of) patterns, which in turn allows seamless iteration through multiple patterns, as if it was one pattern. 

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
