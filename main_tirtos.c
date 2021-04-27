/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 * Simplified (c) 2019, Hogeschool Rotterdam
 */

/* POSIX Header files */
#include <pthread.h>

/* RTOS header files */
#include <ti/sysbios/BIOS.h>

/* Example/Board Header files */
#include "Board.h"

extern void *mainThread(void *args);

int main(void)
{
    /* Call board init functions */
    Board_initGeneral();

    /* Create and start mainThread */
    pthread_attr_t pAttrs;
    pthread_attr_init(&pAttrs);
    struct sched_param priParam;
    priParam.sched_priority = 1;
    int retc = pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);
    retc |= pthread_attr_setschedparam(&pAttrs, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs, 4096);
    pthread_t thread;
    retc |= pthread_create(&thread, &pAttrs, mainThread, NULL);
    if(retc != 0)
    {
        /* Failed to create mainThread */
        while(1);
    }

    BIOS_start();


    return 0;
}
