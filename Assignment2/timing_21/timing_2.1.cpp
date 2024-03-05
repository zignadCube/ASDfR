#include <ctype.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

void * measurement_thread(void *arg){
    long num = (long) arg;

    struct timespec time_;
    time_.tv_nsec = 1000000;

    struct timespec threadStart, threadEnd;

    int array[100];
    for(int i = 0; i < 100; i++){
        array[i] = i;
    }

    clock_gettime(CLOCK_MONOTONIC_RAW, &threadStart); //start measurement
    for(int i = 0; i < num; i++){
        for(int i = 0; i < 100; i++){
            array[i] = array[i]*3;
        }
        if(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &time_, NULL) != 0){
            printf("Error in clock_nanosleep\n");
            return arg;
        }
    }
    clock_gettime(CLOCK_MONOTONIC_RAW, &threadEnd); //end measurement

    long time_diff = (threadEnd.tv_sec - threadStart.tv_sec) * 1000000000 + (threadEnd.tv_nsec - threadStart.tv_nsec);

    printf("Loop took %ld microseconds\n", time_diff/1000);
    return arg;
}

int main(int argc, char *argv[]){
    pthread_t measurement_thread_;
    long num = 10000; //default 10000 cycles on the for loop
    if(argc >= 2){
        num = atoi(argv[1]);
    }

    // create thread
    pthread_create(&measurement_thread_, NULL, *measurement_thread, (void *) num);
    printf("Measurement thread created\n");

    // wait for thread
    pthread_join(measurement_thread_, NULL);
    printf("Measurement thread ended\n");

    return 0;
}