# Octomap mapping
## How to start everything
### Terminal 1:
##### 
```
> ./start_px4.sh
# wait for gazebo and px4 ground control to startup
> commander set_ekf_origin 47.397742 8.545594 488.0
# it should say 'home set'
```
###  Terminal 2:
##### 
``` 
> cd __directory__
> ros2 launch tfstarter
```
### Terminal 3:
#####
```
> cd __directory__
> python3 keyboardcontrol.py
``` 

## Configure parameters
### tf starter starts :
1. #### static transforms
2. #### gazebo -> ros2 bridge topics for depth_camera/points and clock
3. #### octomap_server node
4. #### rviz (change path to rviz config path)