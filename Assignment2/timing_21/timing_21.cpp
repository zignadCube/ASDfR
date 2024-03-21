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
    struct timespec threadStart, threadEnd, wakeupTime;
    long long measurements[num];


    for(int j = 0; j < num; j++){
        clock_gettime(CLOCK_MONOTONIC, &threadStart); //start measurement
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
        clock_gettime(CLOCK_MONOTONIC, &threadEnd); //end measurement

        long time_diff = (threadEnd.tv_sec - threadStart.tv_sec) * 1000000000 + (threadEnd.tv_nsec - threadStart.tv_nsec);
        measurements[j] = time_diff;
    }

    // Write measurements to file
    printf("Done measuring\nWriting measurements to file\n");
    FILE *fptr;
    fptr = fopen("measurements_w_stress.txt", "w");
    for(int i = 0; i < num; i++){
        fprintf(fptr, "%lli,", measurements[i]);
    }   
    fclose(fptr);

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