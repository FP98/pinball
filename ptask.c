#include "ptask.h"

#include <math.h>
#include <stdlib.h>




int task_create(void* (*task)(void*), int i, int period, int dline, int prio)  //funzione di creazione Task
{
    pthread_attr_t myatt;
    struct sched_param mypar;
    int tret;
    if (i >= MAX_TASKS) return -1;                                                                                        

    tp[i].arg = i;
    tp[i].period = period;
    tp[i].dline = dline;
    tp[i].prio = prio;
    tp[i].dmiss = 0;
   
    pthread_attr_init(&myatt);
    pthread_attr_setinheritsched(&myatt, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&myatt, SCHED_FIFO);
    mypar.sched_priority = tp[i].prio;
    pthread_attr_setschedparam(&myatt, &mypar);
    tret = pthread_create(&tp[i].tid, &myatt, task, (void*)(&tp[i]));

    return tret;
}

void time_add_ms(struct timespec* t, int ms)   // Funzione di addizione millisecondi 
{
    t->tv_sec += ms / 1000;
    t->tv_nsec += (ms % 1000) * 1000000;
    if (t->tv_nsec > 1000000000) {
        t->tv_nsec -= 1000000000;
        t->tv_sec += 1;
    }
}

void time_copy (struct timespec *td, struct timespec ts)  // copio il valore di ts(tempo sorgente) in td (tempo destinatario)
{
	td->tv_sec = ts.tv_sec;
	td->tv_nsec = ts.tv_nsec;
}

int time_cmp(struct timespec t1, struct timespec t2)  // compara due tempi di sistema t1 e t2, se t1>t2 mi da come output 1, se t1<t2 -1 se t1=t2 0
{
	if (t1.tv_sec > t2.tv_sec) return 1;
	if (t1.tv_sec < t2.tv_sec) return -1;
	if (t1.tv_nsec > t2.tv_nsec) return 1;
	if (t1.tv_nsec < t2.tv_nsec) return -1;
	return 0;
}

int get_period(int i)
{
	return tp[i].period;
}

void periodic_task_init(int i)
{
	clock_gettime(CLOCK_MONOTONIC, &(tp[i].at));
	time_copy(&(tp[i].dl), tp[i].at);
	time_add_ms(&(tp[i].at), tp[i].period);
	time_add_ms(&(tp[i].dl), tp[i].dline);
}

void dmiss_counter(int i)
{
	clock_gettime(CLOCK_MONOTONIC, &(tp[i].now));
	if (time_cmp(tp[i].now, tp[i].dl) > 0) tp[i].dmiss++;
}


void wait_for_period(int i)
{
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(tp[i].at), NULL);
	time_add_ms(&(tp[i].at), tp[i].period);
	time_add_ms(&(tp[i].dl), tp[i].dline);
}

void wait_for_task_end(int i)          //Funzione che aspetta terminazione del task
{
	pthread_join(tp[i].tid, NULL);
}

int get_task_index(void* arg)
{
    struct task_par *tpar;
    tpar = (struct task_par *)arg;
    return tpar->arg;
}
