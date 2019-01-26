
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
/*
    osThread : Access to the thread definition for the function osThreadCreate.
    osThreadCreate :
    more can be read at https://www.keil.com/pack/doc/cmsis/RTOS/html/group__CMSIS__RTOS__ThreadMgmt.html#gaf0c7c6b5e09f8be198312144b5c9e453
    osThreadCreate: osThread(WatchdogThread): thread definition referenced with osThread.
                    "NULL":pointer that is passed to the thread function as start argument.In this example,no pointer passed.
*/
  WatchdogId = osThreadCreate(osThread(WatchdogThread), NULL);

/*
osTimerId : structure for given timer id as oneshot.
osTimerCreate : "osTimer(Wdtimer)-> name of the timer object is Wdtimer"
                osTimerOnce : means one shot timer.for one-shot or osTimerPeriodic for periodic behavior.
                (void*)0 : argument to the timer call back function.
More can be read at https://www.keil.com/pack/doc/cmsis/RTOS/html/group__CMSIS__RTOS__TimerMgmt.html#ga1b8d670eaf964b2910fa06885e650678
*/
  osTimerId OneShot = osTimerCreate(osTimer(Wdtimer), osTimerOnce, (void *)0);

  pc.printf("\r\n Hello World - RTOS Template Program 1");

/*
pc.readable() : Determine if there is a character available to read.
pc.getc (): get character into char variable x
pc.putc() : print character in char variable x
if the character 'r' is entered,then the led1 is turned off and the timer restarted
osTimerStart: OneShot : Timer ID obtained by osTimerCreate
              "2000" : 2000 miliseconds time delay value
Thread::wait : go to sleep for 500 miliseconds.It can be set max 32-bit value

*/
do {
        if (pc.readable())
        {
        x=pc.getc();
        pc.putc(x);
        if(x=='r') led1=0;
          osTimerStart(OneShot, 2000);
        }
          Thread::wait(500);
    }while(1);
}

void WatchdogThread(void const *argument)
{
/*
    while(true) : infinite loop.Same as do{....}while(1);
    osSignalWait:
          SignalWatchdog :Integer value."wait until all specified signal flags set or 0 for any single signal flag"
          osWaitForever  : the function will wait for an infinite time until a specified signal is set.
    More can be read at https://www.keil.com/pack/doc/CMSIS/RTOS/html/group__CMSIS__RTOS__SignalMgmt.html#ga38860acda96df47da6923348d96fc4c9
    'led1 = 1' : turn on led1 LED
*/
    while (true) {
      osSignalWait(SignalWatchdog, osWaitForever); // Go to sleep until a signal, SignalWatchdog, is received
      led1 = 1;
    }
}

//*********************************************************************************************************
/*
Watchdog Interrupt service routine:
-> a constant variable pointer named n is passed in
WatchdogId : thread ID obtained by osThreadCreate or osThreadGetId.
"0x1": specifies the signal flags of the thread that should be set.It can be 32-bit integer
More can be read at : https://www.keil.com/pack/doc/CMSIS/RTOS/html/group__CMSIS__RTOS__SignalMgmt.html#ga3de2730654589d6c3559c4b9e2825553
*/

void WatchdogISR(void const *n)
{
 osSignalSet(WatchdogId,0x1); // Send signal to thread with ID
}
