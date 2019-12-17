# UNDER CONSTRUCTION

# Description

A collection of tools for making complex "selections" in RVIZ with a lasso tool ala photoshop or blender.
This works by recording the polygon that the user drew along with the pose of the users camera.
This info is published over ROS and external nodes can use it.

Try the demo:
```
rosrun rviz_lasso_tool rviz_lasso_tool_demo _pcd_path:=PATH_TO_YOUR_PCD_FILE
```
or if that encountered the error “Found the following, but they’re either not files or not executable”, run below instead.
```
./rviz_lasso_tool_demo _pcd_path:=/home/ruan/22.24.pcd
```

And in another thread, run Rviz, load the rviz lasso tool and go to town.
Press control to add to the current selection and shift to remove from it.

on 18.04, unset PYTHONPATH, then start gdb to debug, can fix gdb pytthon issue:
Unexpected GDB stderr: "Python Exception <class 'AttributeError'> module 'enum' has no attribute 'IntFlag'.
