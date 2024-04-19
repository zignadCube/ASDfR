# Steps to compile and run our code for assignment 3.5

## Steps to compile the code

1. Unzip all packages in this zip file the workspace source directory <asdfr_ws>
2. Go to <asdfr_ws>  

        cd <asdfr_ws>

3. Run the command to build:

        colcon build

4. Run the command:  

        source install/local_setup.bash

You are now ready to run the code. This is described in the next part.

## Steps to run the code

1. Compile and run the code of the xenomai thread as described in steps 4 to 6 in ToDo.txt from Canvas.

2. Run the Ros to Xenomai bridge by running this command in a different terminal:

        ./RosXenoBridge.bash

3. Run the launch file for the rest of the nodes in another different terminal:

        ros2 launch src/launch/launch.py  
