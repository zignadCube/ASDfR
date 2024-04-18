#include "Native20Sim.h"

Native20Sim::Native20Sim() 
{
    printf("%s: Constructing rampio\n", __FUNCTION__);
    #if OVERWRITE_CYCLE_TIME
        cycleTime = controller.GetStepSize() * SEC;
        cycle_freq = 1/controller.GetStepSize();
        if(cycle_freq < XENO2ROS_FREQ)
            error(1,0,"XENO2ROS_FREQ is bigger than cycle_freq");
        wr_delay = cycle_freq/XENO2ROS_FREQ;
    #endif
}


Native20Sim::~Native20Sim()
{
    printf("%s: Destructing rampio\n", __FUNCTION__);
}


int Native20Sim::calc()
{
    #if DEBUG_20SIM_IO
        evl_printf("20Sim model input:\n");
        for (int i = 0; i < SIZE_U; i++)
        {
            evl_printf("%f, ",u[i]);
        }
        evl_printf("\n");
    #endif

    //Calculation of the 20Sim model
    controller.Calculate (&u[0], &y[0]);

    #if DEBUG_20SIM_IO
        evl_printf("20Sim model output:\n");
        for (int i = 0; i < SIZE_Y; i++)
        {
            evl_printf("%f, ",y[i]);
        }
        evl_printf("\n");
    #endif

    return 0;
}
