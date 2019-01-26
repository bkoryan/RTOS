//######################################################################################################################
//# Burak Koryan | bkoryan@unb.ca | January 24 2019
//# Lab notes for ECE 4333 - Robotics | Faculty of Engineering at University of New Brunswick,Fredericton,Canada.
//# Description: Laboratory Experiment 2 : Applications of Real Time Systems Concepts by Prof.Chris Diduch at UNB

//######################################################################################################################

/*
Import libraries:
-> mbed.h can be found at https://github.com/ARMmbed/mbed-os/blob/master/mbed.h
-> rtos.h can be found at https://github.com/ARMmbed/mbed-os/blob/master/rtos/rtos.h
*/
#include "mbed.h"
#include "rtos.h"

/*

*/
void WatchdogISR(void const *n);
void WatchdogThread(void const *argument);

// Processes and threads
int32_t SignalWatchdog;
osThreadId WatchdogId;
osTimerDef(Wdtimer, WatchdogISR); // Declare a watch dog timer
osThreadDef(WatchdogThread, osPriorityRealtime, 1024); // Declare WatchdogThread as a thread/process

// IO Port Configuration
DigitalOut led1(LED1);
Serial pc(USBTX, USBRX); // Pins (tx, rx) for PC serial channel

//*********************************************************************************************************
int main()
{
  char x;
// Start execution of the thread: WatchdogThread with ID, WatchdogId:
  WatchdogId = osThreadCreate(osThread(WatchdogThread), NULL);
// Start the watch dog timer and enable the watch dog interrupt
  osTimerId OneShot = osTimerCreate(osTimer(Wdtimer), osTimerOnce, (void *)0);
  pc.printf("\r\n Hello World - RTOS Template Program 1");
    do {
        if (pc.readable())
        {
        x=pc.getc();
        pc.putc(x); //Echo keyboard entry
        if(x=='r') led1=0; // Turn LED off.
          osTimerStart(OneShot, 2000); // Start or restart the watchdog timer interrupt and set to 2000ms.
        }
          Thread::wait(500); // Go to sleep for 500 ms
    }while(1);
}

void WatchdogThread(void const *argument)
{
    while (true) {
      osSignalWait(SignalWatchdog, osWaitForever); // Go to sleep until a signal, SignalWatchdog, is received
      led1 = 1;
    }
}

//*********************************************************************************************************
void WatchdogISR(void const *n)
{
 osSignalSet(WatchdogId,0x1); // Send signal to thread with ID, WatchdogId, i.e., WatchdogThread.
}
