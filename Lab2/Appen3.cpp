
/*
Burak Koryan | bkoryan@unb.ca | January 24 2019
Lab notes for ECE 4333 - Robotics | Faculty of Engineering at University of New Brunswick,Fredericton,Canada.
Description: Laboratory Experiment 2 : Applications of Real Time Systems Concepts by Prof.Chris Diduch at UNB.
Extra comments for learning purposes have been added by Burak.

Part 3 : Thread Triggered by an External Interrupt from Appendix 3
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

1) WatchdogISR : void function that returns nothing but takes in a constant pointer called "n"
2) WatchdogThread : void function that returns nothing but takes in a constant pointer called "argument"
3) ExtISR : void function that returns nothing and takes in no pointer or variables
4) ExtThread : void function that takes in a constant pointer named "argument"
5) PeriodicISR : void function that returns nothing and takes in no pointer or variables
6) PeriodicThread : void function that returns nothing but takes in a constant pointer named "argument"
*/

void WatchdogISR(void const *n);
void WatchdogThread(void const *argument);
void ExtISR(void);
void ExtThread(void const *argument);
void PeriodicISR(void);
void PeriodicThread(void const *argument);


/*
Processes and threads:
-> These are defined processes and threads by Prof.Diduch.
##########
The SignalWatchdog,SignalExt,SignalPeriodic variables are defined as 32-bit signed integer.
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
-> In this code,there is another thread called "PeriodicThread" with the same parameters of "WatchdogThread"
-> "ExtThread" thread has the same parameter "1024" but its initial priority is set as "osPriorityHigh"
##########
*/

int32_t SignalWatchdog, SignalExt, SignalPeriodic;
osThreadId WatchdogId, ExtId, PeriodicId;
osTimerDef(Wdtimer, WatchdogISR);
osThreadDef(WatchdogThread, osPriorityRealtime, 1024);            // Declare WatchdogThread as a thread/process
osThreadDef(ExtThread, osPriorityHigh, 1024);                     // Declare ExtThread as a thread/process
osThreadDef(PeriodicThread, osPriorityRealtime, 1024);            // Declare PeriodicThread as a thread/process

/*
IO Port Configuration:
-> 3 leds defined as digital out : LED1,LED2,LED3
-> serial communication is achieved using "serial PC connection"
-> Ticker : PeriodicInt -> Timer interrupt.The Ticker interface is used to setup a recurring interrupt to repeatedly call a function at a specified rate.
-> InterruptIn : The InterruptIn interface is used to trigger an event when a digital input pin changes.Interrupt named "bumper" that is assigned to pin8
*/

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
Serial pc(USBTX, USBRX);
InterruptIn Bumper(p8);                         // External interrupt pin declared as Bumper
Ticker PeriodicInt;                             // Declare a timer interrupt: PeriodicInt

/*
Declare global variables
*/

int Position;                                   // integer position variable


int main() {

    char x;                                     // x character variable

/*
    InterruptIn : The InterruptIn interface is used to trigger an event when a digital input pin changes.
    Bumper.rise : Attach a function to call when a rising edge occurs on the input.
    &ExtISR : External interrupt service routine function address
*/
    Bumper.rise(&ExtISR);                       // Attach the address of the interrupt handler to the rising edge of Bumper

/*
    osThread : Access to the thread definition for the function osThreadCreate.
    osThreadCreate :
    more can be read at https://www.keil.com/pack/doc/cmsis/RTOS/html/group__CMSIS__RTOS__ThreadMgmt.html#gaf0c7c6b5e09f8be198312144b5c9e453
    osThreadCreate: osThread(WatchdogThread): thread definition referenced with osThread.
                        "NULL":pointer that is passed to the thread function as start argument.In this example,no pointer passed.
    WatchdogId : thread ID for reference by other functions or NULL in case of error.
    PeriodicId : thread ID for reference by other functions or NULL in case of error.
*/
    WatchdogId = osThreadCreate(osThread(WatchdogThread), NULL);
    ExtId = osThreadCreate(osThread(ExtThread), NULL);
    PeriodicId = osThreadCreate(osThread(PeriodicThread), NULL);

    /*
    osTimerId : structure for given timer id as oneshot.
    osTimerCreate : "osTimer(Wdtimer)-> name of the timer object is Wdtimer"
                    osTimerOnce : means one shot timer.for one-shot or osTimerPeriodic for periodic behavior.
                    (void*)0 : argument to the timer call back function.
    More can be read at https://www.keil.com/pack/doc/cmsis/RTOS/html/group__CMSIS__RTOS__TimerMgmt.html#ga1b8d670eaf964b2910fa06885e650678
    */

    osTimerId OneShot = osTimerCreate(osTimer(Wdtimer), osTimerOnce, (void *)0);       // Start the watch dog timer and enable the watch dog interrupt
    pc.printf("\r\n Hello World - RTOS Template Program 3");

    /*
        Specify address of the PeriodicInt ISR as PiControllerISR, specify the interval
        in seconds between interrupts, and start interrupt generation.

        &PeriodicISR : callback (void) function
        .05 : float number 0.05 seconds interval

    */

    PeriodicInt.attach(&PeriodicISR, .05);

    x = 0;                    // character x = 0

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
       if (pc.readable()){
         x=pc.getc();
         if(x=='r') led1 = 0;                             // Turn LED off.
         osTimerStart(OneShot, 2000);                     // Start or restart the watchdog timer interrupt and set to 2000ms.
       }
       pc.printf("\r\n %c %5d", x, Position);             // print the character x as well as the position(integer variable)
       Thread::wait(500);                                 // Go to sleep for 500 ms
    }while(1);
}

/*
    WatchdogThread:
    while(true) : infinite loop.Same as do{....}while(1);
    osSignalWait:
          SignalWatchdog :Integer value."wait until all specified signal flags set or 0 for any single signal flag"
          osWaitForever  : the function will wait for an infinite time until a specified signal is set.
    More can be read at https://www.keil.com/pack/doc/CMSIS/RTOS/html/group__CMSIS__RTOS__SignalMgmt.html#ga38860acda96df47da6923348d96fc4c9
    'led1 = 1' : turn on led1 LED
*/

void WatchdogThread(void const *argument) {
while (true) {
 osSignalWait(SignalWatchdog, osWaitForever); // Go to sleep until a signal, SignalWatchdog, is received
 led1 = 1;
 }
}

/*
Watchdog Interrupt service routine:
-> a constant variable pointer named n is passed in
WatchdogId : thread ID obtained by osThreadCreate or osThreadGetId.
"0x1": specifies the signal flags of the thread that should be set.It can be 32-bit integer
More can be read at : https://www.keil.com/pack/doc/CMSIS/RTOS/html/group__CMSIS__RTOS__SignalMgmt.html#ga3de2730654589d6c3559c4b9e2825553
*/

void WatchdogISR(void const *n) {
 osSignalSet(WatchdogId,0x1);            // Send signal to thread with ID, WatchdogId, i.e., WatchdogThread
}

/*
    ExtThread:
    while(true) : infinite loop.Same as do{....}while(1);
    osSignalWait:
          SignalWatchdog :Integer value."wait until all specified signal flags set or 0 for any single signal flag"
          osWaitForever  : the function will wait for an infinite time until a specified signal is set.
    More can be read at https://www.keil.com/pack/doc/CMSIS/RTOS/html/group__CMSIS__RTOS__SignalMgmt.html#ga38860acda96df47da6923348d96fc4c9
    led2 = !led2; // led 2 is toggled
*/

void ExtThread(void const *argument) {
    while (true)
    {
       osSignalWait(SignalExt, osWaitForever); // Go to sleep until signal, SignalExtCollision, is received
       led2 = !led2;
     }
}

/*
    External Interrupt Service Routine:
    ExtId : thread ID obtained by osThreadCreate or osThreadGetId.
    "0x1": specifies the signal flags of the thread that should be set.It can be 32-bit integer
    More can be read at : https://www.keil.com/pack/doc/CMSIS/RTOS/html/group__CMSIS__RTOS__SignalMgmt.html#ga3de2730654589d6c3559c4b9e2825553

*/
void ExtISR(void) {
 osSignalSet(ExtId,0x1); // Send signal to the thread with ID, ExtId, i.e., ExtThread.
}

/*
    PeriodicThread :
    while(true) : infinite loop.Same as do{....}while(1);
    osSignalWait:
          SignalWatchdog :Integer value."wait until all specified signal flags set or 0 for any single signal flag"
          osWaitForever  : the function will wait for an infinite time until a specified signal is set.
    More can be read at https://www.keil.com/pack/doc/CMSIS/RTOS/html/group__CMSIS__RTOS__SignalMgmt.html#ga38860acda96df47da6923348d96fc4c9
    'led1 = 1' : turn on led1 LED
*/
void PeriodicThread(void const *argument) {
    while (true)
    {
       osSignalWait(SignalPeriodic, osWaitForever); // Go to sleep until signal, SignalPi, is received.
       led3 = !led3; // Alive status - led3 toggles each time PeriodicThread is signaled.
       Position = Position + 1;
    }
}
// ******** Period Timer Interrupt Handler ********
void PeriodicISR(void) {
 osSignalSet(PeriodicId,0x1); // Send signal to the thread with ID, PeriodicId,
 // i.e., PeriodicThread.
}
