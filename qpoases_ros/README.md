# qpoases_ros

The qpOASES quadratic programming solver in a ROS package.

This ROS package pulls a revision of the code from the project's SVN and builds
the C++ library and Python wrappers.

**Note that install rules are missing for now.**

#### About qpOASES

Look at the [project home page](https://projects.coin-or.org/qpOASES) for the
best description of the solver's capabilities.

There is also a
[User’s Manual](http://www.coin-or.org/qpOASES/doc/3.0/manual.pdf).

#### C++ library

*Currently useless. Catkin seems to expect all include directories
(`INCLUDE_DIRS` in `catkin_package`) to be relative to the package root
(`CMAKE_CURRENT_SOURCE_DIR`). QpOASES actual files get checked out into the
build tree though.*

#### Python library

Will end up as `qpoases` in the workspace scope (so do `import qpoases`). After
a build you should be able to run the Python examples. Examples get checked out
during the build process and end up (relative to your workspace) in

```
build/qpoases_ros/qpoases_svn/src/qpoases_svn/interfaces/python/examples/
```
