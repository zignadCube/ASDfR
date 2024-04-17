# Steps to compile and run our code for assignment 2

## Steps to run the code of 2.1

1. Unzip this zipfile and change the directory to the just unzipped folder, likely called 18-Set2 and into the subdirectory timing_21

        cd 18-Set2/timing_21

2. Compile the code with make

        make

3. Run the code

        ./timing_21

## Steps to run the code of 2.2

1. Unzip this zipfile and change the directory to the just unzipped folder, likely called 18-Set2 and into the subdirectory timing_22

        cd 18-Set2/timing_22

2. Compile the code with make

        make

3. Run the code

        ./timing_22

## Steps to run the code of 2.3

1. Unzip this zipfile and change the directory to the just unzipped folder, likely called 18-Set2
2. Change directory to the just unzipped folder likely called 18-Set2  

        cd 18-Set2

3. Run the command to build:

        colcon build

4. Run the command:  

        source install/local_setup.bash

5. Run the launch file for assignment 2.3 by using the following command. 

        ros2 launch src/launch/launch_timing.py  

6. The code runs for 10000 messages and writes the measurements to a txt file in the current directory. The program is finished after is said: End of writing to file. After this the program can be closed with ctrl+c