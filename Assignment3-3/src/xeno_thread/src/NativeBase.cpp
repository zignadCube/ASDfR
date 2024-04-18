#include "NativeBase.h"


NativeBase::NativeBase() 
{
    printf("%s: Constructing rampio\n", __FUNCTION__);
    cycleTime = SEC/CYCLE_TIME_FREQ;
    cycle_freq = CYCLE_TIME_FREQ;
    if(CYCLE_TIME_FREQ<XENO2ROS_FREQ)
        error(1,0,"XENO2ROS_FREQ is bigger than CYCLE_TIME_FREQ");
    if(XENO2ROS_FREQ == 0)
        wr_delay = 9999999;
    else
        wr_delay = CYCLE_TIME_FREQ/XENO2ROS_FREQ;
}

NativeBase::~NativeBase()
{
    printf("%s: Destructing rampio\n", __FUNCTION__);
}


void NativeBase::init()
{
    int ret;
    struct evl_sched_attrs attrs;

    // Create Ros to Xenomai, cross bufffer
    rfd = evl_create_xbuf(0, sizeof(RosData) * ROS2XENO_XBUF_SIZE, EVL_CLONE_PUBLIC | EVL_CLONE_NONBLOCK , "Ros-Xeno");
    if (rfd < 0)
        error(1, errno, "evl_create_xbuf(Ros-Xeno)");
    #if DEBUG_X_BUF
        printf("Ros-Xeno x-buffer");
        printf("File descriptor :    %d\n", rfd);
        printf("Buffer size     :    %d\n", ROS2XENO_XBUF_SIZE);
        printf("Msg size        :    %ld\n", sizeof(RosData));
    #endif

    // Create Xenomai to Ros, cross bufffer
    xfd = evl_create_xbuf(sizeof(XenoData) * XENO2ROS_XBUF_SIZE, 0, EVL_CLONE_PUBLIC , "Xeno-Ros");
    if (xfd < 0)
        error(1, errno, "evl_create_xbuf(Xeno-Ros)");
    #if DEBUG_X_BUF
        printf("Xeno-Ros x-buffer\n");
        printf("File descriptor :    %d\n", xfd);
        printf("Buffer size     :    %d\n", XENO2ROS_XBUF_SIZE);
        printf("Msg size        :    %ld\n", sizeof(XenoData));
    #endif

    //Attack thread to evl core
    int tfd = evl_attach_self("XenoThread");
    if (tfd < 0)
        error(1, errno, "evl_attach_self()");
    evl_printf("Attached to evl.\n");

    attrs.sched_policy = SCHED_FIFO;
    attrs.sched_priority = 1;

    ret = evl_set_schedattr(tfd, &attrs);
    if(ret < 0)
        error(1, errno, "evl_set_schedattr()");

    //Initiliaze SPI module for OOB use and init the FPGA
    ico_io.spi_init();
    ico_io.init();
    init_flag = true;

    ico_io.reset();
    
    FpgaOutput.pwm1 = 0;
    FpgaOutput.pwm2 = 0;
    FpgaOutput.pwm3 = 0;
    FpgaOutput.pwm4 = 0;
    FpgaOutput.val1 = 0;
    FpgaOutput.val2 = 0;
    FpgaOutput.val3 = 0;
    FpgaOutput.val4 = 0;
    FpgaInput.channel1 = 0;
    FpgaInput.channel1_1 = 0;
    FpgaInput.channel1_2 = 0;
    FpgaInput.channel2 = 0;
    FpgaInput.channel2_1 = 0;
    FpgaInput.channel2_2 = 0;
    FpgaInput.channel3 = 0;
    FpgaInput.channel3_1 = 0;
    FpgaInput.channel3_2 = 0;
    FpgaInput.channel4 = 0;
    FpgaInput.channel4_1 = 0;
    FpgaInput.channel4_2 = 0;

    return;
}


void NativeBase::run()
{  
    int res = 0;
    cycleCount = 0;
    struct timespec timeout;
    struct timespec sleepTime;


    //Check if init is called, if not call init
    if(!init_flag)
        init();

    evl_printf("Cycle frequency :    %d Hz\n",cycle_freq);
    evl_printf("Cycle time :         %d msec\n",cycleTime/MSEC);
    evl_printf("Write delay :        %d cycles\n",wr_delay);

    evl_read_clock(EVL_CLOCK_MONOTONIC, &timeout);
    startTime = timeout;

    struct evl_poll_event pstr;
    int ret;
    bool flag;
    evl_printf("Starting control loop.\n");
    evl_read_clock(EVL_CLOCK_MONOTONIC, &timeout);
    startTime = timeout;
    while(1)
    {
        //Communication with the FPGA
        ico_io.update_io(FpgaOutput, &FpgaInput);
        

        ret = oob_read(rfd, &RosData, sizeof(RosData));

        flag = (ret || flag) && (XENO2ROS_FREQ != 0);
        
        wr_counter++;
        
        if(flag&&(wr_counter == wr_delay))
        {
            oob_write(xfd, &XenoData, sizeof(XenoData));
            wr_counter = 0;
        }

        //Check if kill condition has been met
        #if ENABLE_KILL_CONDITION
            if(*killCondition)
                break;
        #endif
        
        //Call the needed functions
        res = preProc();
        if(res<0)
            break;

        res = calc();
        if(res<0)
            break;

        res = postProc();
        if(res<0)
            break;

        cycleCount = cycleCount + 1;
        
        //Calculate new wake up time and check if any cycles are missed
        evl_read_clock(EVL_CLOCK_MONOTONIC, &sleepTime);
        while (true)
        {        
            timeout.tv_nsec = timeout.tv_nsec + cycleTime;
            if(timeout.tv_nsec >= SEC)
            {
                timeout.tv_nsec = timeout.tv_nsec - SEC;
                timeout.tv_sec = timeout.tv_sec + 1;
            }
            if((sleepTime.tv_sec < timeout.tv_sec) ||((sleepTime.tv_sec == timeout.tv_sec)&&(sleepTime.tv_nsec < timeout.tv_nsec)))
                break;

            missedCycles++;
        }

        #if DEBUG_CONTROL_LOOP
            evl_printf("I start sleeping at:  %ds, %dns \n", sleepTime.tv_sec, sleepTime.tv_nsec);
            evl_printf("I am going to sleep till:  %ds, %dns \n", timeout.tv_sec, timeout.tv_nsec);
        #endif
        evl_sleep_until(EVL_CLOCK_MONOTONIC, &timeout);
    }

    //Run time calculations
    evl_read_clock(EVL_CLOCK_MONOTONIC, &stopTime);
    runTime.tv_sec = stopTime.tv_sec - startTime.tv_sec;
    if (stopTime.tv_nsec < startTime.tv_nsec)
    {
        runTime.tv_nsec = SEC + stopTime.tv_nsec - startTime.tv_nsec;
        runTime.tv_sec -=1;
    }
    else
        runTime.tv_nsec = stopTime.tv_nsec - startTime.tv_nsec;
    
    evl_printf("Runtime: %d sec, %d nsec \n", runTime.tv_sec, runTime.tv_nsec);
    evl_printf("Cycle count: %ld\n", cycleCount);
    evl_printf("Missed cycles count: %d\n", missedCycles);

    return;
}

void NativeBase::setKillCondition(volatile bool* condition)
{
    killCondition = condition;
    return;
}

    