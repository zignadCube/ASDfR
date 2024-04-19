# Steps to compile and run our code for assignment 3.2

## Steps to compile the code

1. Unzip all packages in this zip file the workspace source directory <asdfr_ws>/src
2. Go to <asdfr_ws>  

        cd <asdfr_ws>

3. Run the command to build:

        colcon build

4. Run the command:  

        source install/local_setup.bash

In further steps we assume that the cam2image.yaml file is present and that both the cam2image.yaml and the videoserver have the correct IP.

You are now ready to run the code. This is described in the next part.

## Steps to run the code

1. Run the launch file for the assignment

        ros2 launch src/launch/launch_3.2.py  
