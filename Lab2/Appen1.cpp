
/*
Burak Koryan | bkoryan@unb.ca | January 24 2019
Lab notes for ECE 4333 - Robotics | Faculty of Engineering at University of New Brunswick,Fredericton,Canada.
Description: Laboratory Experiment 2 : Applications of Real Time Systems Concepts by Prof.Chris Diduch at UNB.
Extra comments for learning purposes have been added by Burak.
*/

/*
Import libraries:
-> mbed.h can be found at https://github.com/ARMmbed/mbed-os/blob/master/mbed.h
-> rtos.h can be found at https://github.com/ARMmbed/mbed-os/blob/master/rtos/rtos.h
*/
#include "mbed.h"
#include "rtos.h"

/*
Function prototypes:
-> These function names have been given by Prof.Diduch with specific meanings.Both functions return nothing osTimerOnce
they are declared as void.Both functions also have input parameters as two constant pointers named n and argument
*/
void WatchdogISR(void const *n);
void WatchdogThread(void const *argument);

/*
Processes and threads:
-> These are defined processes and threads by Prof.Diduch.
##########
The SignalWatchdog variable is defined as 32-bit signed integer.
WatchdogId is defined as a variable for osThreadId.
more can be read on osThreadId at https://www.keil.com/pack/doc/cmsis/RTOS/html/cmsis__os_8h.html#adfeb153a84a81309e2d958268197617f
##########
osTimerDef : Attiributes of a timer.More can be read at https://www.keil.com/pack/doc/cmsis/RTOS/html/group__CMSIS__RTOS__TimerMgmt.html#ga1c720627e08d1cc1afcad44e799ed492
Wdtimer : name of the timer object
WatchdogISR : Name of the timer callback functions
##########
osThreadDef:
WatchdogThread : Given thread name
osPriorityRealtime : initial priority of the thread function.Priority: realtime (highest)
"1024" : This parameter defines the number of times that osThreadCreate can be called for the same osThreadDef.
More can be read about osThreadDef: https://www.keil.com/pack/doc/cmsis/RTOS/html/group__CMSIS__RTOS__ThreadMgmt.html#gaee93d929beb350f16e5cc7fa602e229f
##########
*/
int32_t SignalWatchdog;
osThreadId WatchdogId;
osTimerDef(Wdtimer, WatchdogISR);
osThreadDef(WatchdogThread, osPriorityRealtime, 1024);

/*
IO Port Configuration:
*/
DigitalOut led1(LED1);
Serial pc(USBTX, USBRX);

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
