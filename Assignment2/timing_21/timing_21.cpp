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
    struct timespec Timestamp, wakeupTime;
    long long timestamps[num+1];

    clock_gettime(CLOCK_MONOTONIC, &Timestamp);
    timestamps[0] = Timestamp.tv_nsec + Timestamp.tv_sec * 1000000000;
    for(int j = 0; j < num; j++){
        if (Timestamp.tv_nsec + 1000000 >= 1000000000) {
            wakeupTime.tv_sec = Timestamp.tv_sec + 1;
            wakeupTime.tv_nsec = Timestamp.tv_nsec + 1000000 - 1000000000;
        } else {
            wakeupTime.tv_sec = Timestamp.tv_sec;
            wakeupTime.tv_nsec = Timestamp.tv_nsec + 1000000;
        }
    
        // Calculation
        for(int i = 0; i < 10000; i++){
            int a = i*i;
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);
        clock_gettime(CLOCK_MONOTONIC, &Timestamp);
        timestamps[j+1] = Timestamp.tv_nsec + Timestamp.tv_sec * 1000000000;
    }

    // Write measurements to file
    printf("Done measuring\nWriting measurements to file\n");
    FILE *fptr;
    fptr = fopen("measurements_w_stress.txt", "w");
    for(int i = 0; i < num; i++){
        fprintf(fptr, "%ld,", timestamps[i+1] - timestamps[i]);
    }   
    fclose(fptr);
    printf("Last measurement: %lli\n", timestamps[num]);

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