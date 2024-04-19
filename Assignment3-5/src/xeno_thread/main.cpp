#include <iostream>
#include <string>
#include <sys/types.h>
#include <pthread.h>
#include <fstream>

#include "MyApp.h"

volatile bool exitBool = false;

static void *XenoThread(void *arg)
{
    MyApp rt;
    rt.setKillCondition(&exitBool);
    rt.run();
    return NULL;
}

int main(int argc, char *argv[])
{
    pthread_t th;
    
    if(evl_init())
        error(1, errno, "evl_init()");

    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(1, &cpu_set);

    pthread_attr_t my_attr;
    sched_param my_param;
    mlockall(MCL_CURRENT | MCL_FUTURE);

    pthread_attr_init(&my_attr);
    my_param.sched_priority = 1;
    pthread_attr_setaffinity_np(&my_attr, sizeof(cpu_set), &cpu_set);
    pthread_attr_setinheritsched(&my_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&my_attr, SCHED_FIFO);
    pthread_attr_setschedparam(&my_attr, &my_param);

    int ret = pthread_create(&th, &my_attr, XenoThread, NULL);
    if (ret < 0)
        error(1, errno, "pthread_create(): %d", ret);

    while (!exitBool)
    {
        std::string s;
        std::cin >> s;
        exitBool = s[0] == 'x';
    }

    pthread_join(th, NULL);
    return 0;
}
