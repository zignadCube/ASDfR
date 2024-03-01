# Steps to compile and run our code for assignment 1

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

1. On the host device start the videoserver.py  

        python3 videoserver.py

2. Run the launch file for the corresponding assignment. Where x.x.x should be changed to the corresponding assingment, the options are 1.1.2, 1.1.3, 1.1.4, 1.2.1, 1.2.2 or 1.2.3

        ros2 launch src/launch/launch_x.x.x.py  

Optional for assignment 1.1.3:
- To change the parameter before launching the launch file
    1. Go to the launch directory

            cd src/launch

    2. Open the launch file for 1.1.3

            nano launch_1.1.3.py

    3. Change the parameter threshold in line 24 to another value

            parameters = [{"threshold": 150}]

    4. Close the file and save it with: ctrl+x y enter
    5. Go back to the workspace directory

            cd ../..

    6. Run the launch file as normal

            ros2 launch src/launch/launch_1.1.3.py

- To change the parameter during runtime after launching the launch file
    1. In a new terminal run the commands

            cd <asdfr_ws>
            source install/local_setup.bash
            ros2 param set brightness_node threshold 150