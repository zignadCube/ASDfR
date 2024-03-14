#include <error.h>
#include <stdio.h>
#include <evl/evl.h>
#include <pthread.h>
//#include <native/timer.h>
#include <time.h>

void *measurement_thread(void *arg) {
    long num = (long) arg;

    struct sched_param param;
	int ret, tfd;

	param.sched_priority = 0;
	ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    

    tfd = evl_attach_self("measurement-thread:%d", getpid());


    struct timespec time_;
    time_.tv_nsec = 1000000;

    struct timespec threadStart, threadEnd;

    int array[100];
    for(int i = 0; i < 100; i++){
        array[i] = i;
    }

    long long total = 0;
    long measurements[10];

    for(int j = 0; j < 100; j++){
        clock_gettime(CLOCK_MONOTONIC_RAW, &threadStart); //start measurement
        for(int i = 0; i < num; i++){
            for(int i = 0; i < 100; i++){
                array[i] = array[i]*3;
            }
            clock_nanosleep(CLOCK_MONOTONIC, 0, &time_, NULL);
        }
        clock_gettime(CLOCK_MONOTONIC_RAW, &threadEnd); //end measurement

        long time_diff = (threadEnd.tv_sec - threadStart.tv_sec) * 1000000000 + (threadEnd.tv_nsec - threadStart.tv_nsec);

        total += time_diff;
        measurements[j] = time_diff;

        //printf("Loop %d took %ld microseconds\n", j, time_diff/1000);
        printf("%ld, ", time_diff/1000);
    }

    long mean = total/10;
    long sum = 0;

    for(int i = 0; i < 10; i++){
        sum += (measurements[i] - mean)*(measurements[i] - mean);
    }
    long variation = sum/9;

    printf("Mean: %ld us, variation: %ld us\n", mean/1000, variation/1000);

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
