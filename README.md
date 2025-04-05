# Setup Guide
1. Download Docker from [here](https://docs.docker.com/engine/install/ubuntu/)
2. Get into the directory where `Dockerfile` is located
3. On the command prompt, type the following commands
```
xhost +
docker build .
docker compose up -d
docker exec -it {name of the folder}-ros-1 bash
```
# How to Start
On the command prompt, type the following commands.
```
xhost +
docker compose up -d
docker exec -it {name of the folder}-ros-1 bash
cd ~/crazyswarm
./Build.sh
```
For more information, you can look at the instructions in [here](https://crazyswarm.readthedocs.io/en/latest/installation.html)
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
3. Launch the crazyswarm node 
4. Now you are ready to run your script
5. To test hovering and landing, you can run `first_test.py` in the `scripts` folder.

# Emegency Shutdown
1. We have `emergency_break.py` that constantly publishes the status=True of `Crazyswarm` class
2. If you interrupt `emergency_break.py`, it will start publishing status=False, which can be used to terminate the ongoing script


# Common Debugging
In case you get an error with an exit code -6 after launching the crazyswarm node, make sure 
1. You have typed ```xhost +``` in a command prompt
2. Yoou are connected to the right WiFi
3. Vicon cameras are on and can sense the crazyflies (one way to check is run `roslaunch crazyswarm mocap_helper.launch`)
4. You are connected to the right vicon system (You can check this by looking at the entry for `motion_capture_host_name` in `hover_swarm.yaml` file)
5. Your chooser GUI is not on
6. Your computer can recognize the Crazyradio PA, and the radio is also correctly flashed (the flashed ones have `ok` stickers)

If the crazyflies flip over as soon as they take off, check 
1. Crazyradio PA is correctly flashed (the flashed ones have `ok` stickers)
2. Propellers are placed correctly (check [here](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/) for more information)
3. Crazyflies are using the PID controller when doing velocity commands (You can check this by looking at the entry for `controller` in `hover_swarm.launch` file)

