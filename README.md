# eskf_odometry

# ROS wrapper of the Error-State Kalman Filter based Odometry


This node is a ROS wrapper of the low-level library **eskf_odometry**.

## Dependencies

### eigen3 Headers

**Install** eigen3 in your sistem **following the instructions of the ReadMe file** provided in the tuxfamily package.
You can download it at http://eigen.tuxfamily.org

### atools Library

You have to install Angel Santamaria-Navarro tools library:

```
  $ git clone https://github.com/angelsantamaria/atools.git
  $ cd atools/build
  $ cmake ..
  $ make
  $ sudo make install
```

### ROS

  * **std_msgs**
  * **sensor_msgs**
  * **nav_msgs**
  * **tf**
  * **iri_base_algorithm**: ``` git clone https://gitlab.iri.upc.edu/labrobotica/ros/iri_core/iri_base_algorithm.git ```
  * **px_comm**: ```git clone https://github.com/cvg/px-ros-pkg.git```

## License

Copyright (C) 

Author Angel Santamaria-Navarro (asantamaria@iri.upc.edu)

All rights reserved.

This file is part of eskf_odometry_ros ROS node.

eskf_odometry_ros is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>