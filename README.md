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
1. All the scripts are located in the directory `scripts` folder in `~/crazyswarm/ros_ws/src/crazyswarm/scripts`
You can access this folder by typing 
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
3. Once the node is running, open another command propt and get into the docker file again, go to the `scripts` folder, and run the python script
4. In case you would like to just run the simulation, type `python3 'python_script_name'.py --sim`


# Flying Drones
1. First, you want to set up the initial positions of the crazyflies. We can do this by modifying `allCrazyflies.yaml` file in `~/crazyswarm/ros_ws/src/crazyswarm/launch/` 
2. In one comamnd prompt, go to the `scripts` folder and run `chooser.py` script to select and deselect the crazyflies you would like to include in deployment. Selecting and Deselecting modifies `crazyflies.yaml` file in the same folder
3. Launch the crazyswarm node in another command prompt
4. Now you are ready to run your script


# Common Debugging
In case you get an error with an exit code -6 after launching the crazyswarm node, make sure 
1. You have typed ```xhost +``` in a command prompt
2. Connected to the right WIFI
3. Vicon cameras are on and can sense the crazyflies
4. Connected to the right vicon system (You can check this by looking at the entry for `motion_capture_host_name` in `hover_swarm.yaml` file)
5. Chooser GUI has been termianted
6. Crazyradio PA is correctly flashed (the flashed ones have `ok` stickers)

If the crazyflies flip over as soon as they take off, check 
1. Crazyradio PA is correctly flashed (the flashed ones have `ok` stickers)
2. Propellers are placed correctly (check https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/)
3. Crazyflies are using the PID controller when doing velocity commands (You can check this by looking at the entry for `controller` in `hover_swarm.launch` file)

