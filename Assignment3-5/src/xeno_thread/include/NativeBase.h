#pragma once

#include <cstdio>
#include <errno.h>
#include <string.h>
#include <time.h>

#include "ico_io_v2.h"
#include <evl/thread.h>
#include <evl/evl.h>
#include "custom_msgs/msg/ros2_xeno.hpp"
#include "custom_msgs/msg/xeno2_ros.hpp"

#include "config.h"

#define MSEC        1000000L
#define SEC         1000000000L
class NativeBase 
{
public:
    //Constructor
    NativeBase();
    virtual ~NativeBase();

    //Variables
    long int cycleCount;
    IcoIO ico_io;

    //Functions
    void run();
    void setKillCondition(volatile bool* condition);
private:
    //Variables
    struct timespec runTime;
    struct timespec startTime;
    struct timespec stopTime;

    bool init_flag;
    int wr_counter;

    int rfd;            // File descriptor for Ros2Xeno crossbuffer
    int xfd;            // File descriptor for Xeno2Ros crossbuffer

    volatile bool* killCondition;
    //Functions
    void init();
protected:
    //Variables
    int cycleTime;
    int missedCycles;
    int wr_delay;
    int cycle_freq;

    IcoIO::IcoWrite FpgaOutput;             // This is data that has to be written to the FPGA, aka PWM + dig. output data
    IcoIO::IcoRead FpgaInput;               // This is the input data that the FPGA has collected, aka encoder + dig. input data

    custom_msgs::msg::Ros2Xeno RosData;      // Data gotten from ROS implementation
    custom_msgs::msg::Xeno2Ros XenoData;     // Data from this Xenomai thread
    
    //Functions
    virtual int preProc() = 0;
    virtual int calc() = 0;
    virtual int postProc() = 0;
};
