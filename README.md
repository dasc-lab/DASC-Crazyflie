# Setup Guide
1. Open the crazyswarm_workspace folder.
2. Type the following commands.
```
xhost +
docker build .
docker compose up -d
docker exec -it crazyswarm_workspace-ros-1 bash
```

# How to Start
1. Open the crazyswarm_workspace folder.
2. Type the following commands.
```
xhost +
docker compose up -d
docker exec -it crazyswarm_workspace-ros-1 bash
```

# How to Use
1. All the scripts are located in the directory ~/crazyswarm/ros_ws/src/crazyswarm/scripts.
You can access the directory by typing 
```
cd ~/crazyswarm/ros_ws/src/crazyswarm/scripts
```
or
```
cd $SHORT
```

2. In order to run the crazyswarm node, we need to type
```
roslaunch crazyswarm hover_swarm.launch
```
If there is an error with an exit code -6, make sure you open another command propt and type ```xhost +```

3. Once the node is running, open another command propt and get into the docker file again, go to the scripts folder, and run the python script.
4. In case you would like to just run the simulation, type `python3 'python_script_name'.py --sim`
