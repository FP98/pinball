#ifndef PTASK_H
#define PTASK_H
#include <pthread.h>
#include <time.h>
#define MAX_TASKS 23		     //3 task + MAX_BALLS
 
struct task_par {         	//Array di strutture parametri task
    int arg;				           //task index
    int period;            //periodo [ms]
    int dline;				         //deadline relativa [ms]	
    int prio;				          //priorita' (0-99)
    int dmiss;             //numero di deadline miss
    pthread_t tid;         //thread id
    struct timespec at;  	 //activation time corrente
    struct timespec dl;   	//dead line assoluta
    struct timespec now; 	 //tempo di fine task corrente
};
struct task_par tp[MAX_TASKS];

int task_create(void* (*task)(void*), int i, int period, int dline, int prio);

void time_add_ms(struct timespec* t, int ms);
void time_copy(struct timespec* td, struct timespec ts);
int time_cmp(struct timespec t1, struct timespec t2);

int get_period(int i);

void periodic_task_init(int i);
void dmiss_counter(int i);
void wait_for_period(int i);
void wait_for_task_end(int i);

int get_task_index(void* arg);

#endif 
