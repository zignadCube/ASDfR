#include <error.h>
#include <stdio.h>
#include <evl/evl.h>
#include <pthread.h>
//#include <native/timer.h>
#include <time.h>

void *measurement_thread(void *arg) {
    struct sched_param param;
	int ret, tfd;

	param.sched_priority = 0;
	ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    

    tfd = evl_attach_self("measurement-thread:%d", getpid());


    long num = (long) arg;
    struct timespec threadStart, threadEnd, wakeupTime;
    long measurements[num];

    for(int j = 0; j < num; j++){
        clock_gettime(CLOCK_MONOTONIC_RAW, &threadStart); //start measurement
        if (threadStart.tv_nsec + 1000000 >= 1000000000) {
            wakeupTime.tv_sec = threadStart.tv_sec + 1;
            wakeupTime.tv_nsec = threadStart.tv_nsec + 1000000 - 1000000000;
        } else {
            wakeupTime.tv_sec = threadStart.tv_sec;
            wakeupTime.tv_nsec = threadStart.tv_nsec + 1000000;
        }
        
        // Calculation
        for(int i = 0; i < 10000; i++){
            int a = i*i;
        }
        
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);
        clock_gettime(CLOCK_MONOTONIC_RAW, &threadEnd); //end measurement

        long time_diff = (threadEnd.tv_sec - threadStart.tv_sec) * 1000000000 + (threadEnd.tv_nsec - threadStart.tv_nsec);
        measurements[j] = time_diff;
    }

    // Write measurements to file
    printf("Done measuring\nWriting measurements to file\n");
    FILE *fptr;
    fptr = fopen("measurements_propper.txt", "w");
    for(int i = 0; i < num; i++){
        fprintf(fptr, "%lli,", measurements[i]);
    }   
    fclose(fptr);
        
    return arg;
}


int main(int argc, char *const argv[])
{
	int ret = evl_init();
	if (ret) {
		error(1, -ret, "evl_init() failed");
    }

    pthread_t measurement_thread_;
    long num = 1000; // default 10000 cycles on the for loop

    // Set CPU core for thread
    pthread_t thread;
    cpu_set_t cpuset;

    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);

    // create thread
    pthread_create(&measurement_thread_, NULL, *measurement_thread, (void *)num);
    //pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    sched_setaffinity(thread, sizeof(cpu_set_t), &cpuset);
    printf("Measurement thread created\n");

    // wait for thread
    pthread_join(measurement_thread_, NULL);
    printf("Measurement thread ended\n");

	return 0;
}
