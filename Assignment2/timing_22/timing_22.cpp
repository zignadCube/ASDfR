#include <error.h>
#include <stdio.h>
#include <evl/evl.h>
#include <pthread.h>
//#include <native/timer.h>
#include <time.h>

#define MILLION 1000000L
#define BILLION 1000000000L

void *measurement_thread(void *arg) {
    struct sched_param param;
	int ret, tfd;

    tfd = evl_attach_self("measurement-thread:%d", getpid());

    struct evl_sched_attrs attrs;
	attrs.sched_policy = SCHED_FIFO;
	attrs.sched_priority = 99;
    ret = evl_set_schedattr(tfd, &attrs);

    long num = (long) arg;
    struct timespec Timestamp, wakeupTime;
    long long timestamps[num+1];

    evl_read_clock(EVL_CLOCK_MONOTONIC, &Timestamp);
    timestamps[0] = Timestamp.tv_nsec + Timestamp.tv_sec * 1000000000;

    for(int j = 0; j < num; j++){
        evl_read_clock(EVL_CLOCK_MONOTONIC, &Timestamp); //start measurement
        if (Timestamp.tv_nsec + MILLION >= BILLION) {
            wakeupTime.tv_sec = Timestamp.tv_sec + 1;
            wakeupTime.tv_nsec = Timestamp.tv_nsec + MILLION - BILLION;
        } else {
            wakeupTime.tv_sec = Timestamp.tv_sec;
            wakeupTime.tv_nsec = Timestamp.tv_nsec + MILLION;
        }
        
        // Calculation
        for(int i = 0; i < 10000; i++){
            int a = i*i;
        }
        
        evl_sleep_until(EVL_CLOCK_MONOTONIC, &wakeupTime);
        evl_read_clock(EVL_CLOCK_MONOTONIC, &Timestamp); //end measurement

        timestamps[j+1] = Timestamp.tv_nsec + Timestamp.tv_sec * 1000000000;
    }

    // Write measurements to file
    printf("Done measuring\nWriting measurements to file\n");
    FILE *fptr;
    fptr = fopen("measurements_w_stress.txt", "w");
    for(int i = 0; i < num; i++){
        fprintf(fptr, "%lld,", timestamps[i+1] - timestamps[i]);
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
    long num = 10000; // default 10000 cycles on the for loop

    // Set CPU core for thread
    pthread_t thread;
    cpu_set_t cpuset;

    // CPU_ZERO(&cpuset);
    // CPU_SET(1, &cpuset);

    // create thread
    pthread_create(&measurement_thread_, NULL, *measurement_thread, (void *)num);
    //pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    // sched_setaffinity(thread, sizeof(cpu_set_t), &cpuset);
    printf("Measurement thread created\n");

    // wait for thread
    pthread_join(measurement_thread_, NULL);
    printf("Measurement thread ended\n");

	return 0;
}
