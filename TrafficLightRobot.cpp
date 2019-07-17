/*
  Project Name : Autonomous Traffic Robot Car using RTOS
  Developed By : Eric Henderson and Burak Koryan at University of New Brunswick,Canada
  Date : April 2019
  Description : The source code below was written in C++ to use with LPC1768 MBED.The mbed
  in the project was used as a data processor and a microcontroller.Proximity and object detection data
  from a PixyCam was fed to MBED to control the autonomous robot car.The code below includes functions
  for multithread RTOS and the necessary configurations
*/
#include "mbed.h"
#include "rtos.h"
#include "Pixy.h"

#define SLEEP_TIME                  500 // (msec)
#define PRINT_AFTER_N_LOOPS         20
#define PIXY_MAX_SIGNATURE          7

// PIXYCAM COLORS:
// 1 - BLUE
// 2 - RED
// 3 - YELLOW
// 4 - GREEN

#define SLEEP_TIME                  500 // (msec)
#define PRINT_AFTER_N_LOOPS         20

Serial pc(USBTX, USBRX); // tx, rx
Serial bluetooth(p9, p10); // TX on pin 9 and RX on pin 10
SPI DE0(p5, p6, p7); // mosi, miso, sclk (BRK)

void PeriodicInnerLoopISR(void);
void PeriodicOuterLoopISR(void);
void WatchdogISR(void const *n);
void InnerLoopThread(void const *argument);
void OuterLoopThread(void const *argument);
void WatchdogThread(void const *argument);

// Processes and threads
//int32_t SignalInnerLoop, SignalOuterLoop;
osThreadId PeriodicInnerId;
osThreadDef(InnerLoopThread, osPriorityRealtime, 1024); // Declare InnerLoopThread as a thread/process
osThreadId PeriodicOuterId;
osThreadDef(OuterLoopThread, osPriorityHigh, 1024); // Declare OuterLoopThread as a thread/process
osThreadId WatchdogId;
osThreadDef(WatchdogThread, osPriorityHigh, 1024); // Declare WatchdogThread as thread/process

osTimerDef(Wdtimer, WatchdogISR);
osTimerId OneShot = osTimerCreate(osTimer(Wdtimer), osTimerOnce, (void *)0);

Ticker PeriodicInnerInt; // Declare a timer interrupt: PeriodicInnerInt
Ticker PeriodicOuterInt;
PwmOut PwmLeft(p23); // PWM output for the left motor
PwmOut PwmRight(p26);// PWM output for the right motor
DigitalOut IoReset(p15);    // Resets all spi prepherials on the DE0
DigitalOut directionLeft(p22),directionRight(p25); // motor direction pins
DigitalOut brakeLeft(p21),brakeRight(p24); // motor brake pins
DigitalOut led3(LED3), led2(LED2), led4(LED4);
DigitalOut SpiReset(p14); // DE0 SPI reset

int outerLoopFlag = 0;

float T = 1000; // PWM period (us)
float h = 0.005; // periodic interrupt period (s)
float PWMdutyCycle;

float L = 0.306; // axis length of the robot (used in kinematic calculations)
float vRight, vLeft, vLeft_m , vRight_m ; // motor velocities (setpoint and measured)

int ID, counter = 0;; // ID returned from DE0 SPI read
int16_t dPosition1,dPosition2; // change in position of the motors from last read (DE0)
unsigned short dTime1,dTime2,Time1,Time2; // change in time since last read (DE0)
float Position1,Position2; //
float rampVal1,rampVal2,dRamp;
float omegaL_m,omegaR_m, omegaL,omegaR; // angular velocity of motors (setpoint and measured)
float omega_m = 0; // overall robot angular velocity
float Position, Theta, PrevThetaError; // Pos and Angle of robot and error from last loop
float speedSetpoint,storedSpeedSetpoint,angularSetpoint,thetaSetpoint; // setpoints for control
float eLeft=0,eRight=0; // inner loop (individual motor) errors
float xiLeft=0, xiRight=0, xiAngle = 0, kp, ki, kd; // control gains and accumulators
float uLeft, uRight, outputLeft, outputRight; // PWM outputs
int instructionComplete = 1;
uint16_t x,y,width,height, blocksR;
float distToBlock = 0, distSetpoint = 0.20, headingErrorPrev = 0, xiPos, posError;
float accumulatedDist = 0, xiTheta = 0, dTheta = 0, stabilityFlag = 0, vRobot;

Mutex r_mutex; // declaration of mutex

SPI spiR(p11, p12, p13); // Create Pixy SPI port (mosi, miso, sclk)
PixySPI pixyR(&spiR, &bluetooth); // Select SPI interface for PixyCam

char sig[6]; // global cc signature string

// Uses code from TPixy.h, adapted from code written by swilkins8
void blockName(int idx){ // stores CC name in global char sig[6]
    int i, j;
    char d;
    bool flag;
    i++;
    for (i = 12, j = 0, flag = false; i >= 0; i -= 3) {
        d = (pixyR.blocks[idx].signature >> i) & 0x07;
        if (d > 0 && !flag) {
            flag = true;
        }
        if (flag) {
            sig[j++] = d + '0';
        }
    } // end for loop
    sig[j] = '\0';
} // end blockName()


void newcommand(float angle){
    bluetooth.printf("\n\rNew Command Observed: %f\n\r",angle);

    r_mutex.lock();
    storedSpeedSetpoint = speedSetpoint;
    instructionComplete = 0;
    angularSetpoint = 0;
    speedSetpoint = 0;
//    r_mutex.unlock();
    Thread::wait(5000); // stops robot and waits briefly before executing command
//    r_mutex.lock();
    thetaSetpoint = thetaSetpoint + angle; // adds new command to theta setpoint
    xiPos = 0;
    r_mutex.unlock();
    sig[0] = '\0';
} // end newcommand()

void controlAdjust(void){
    //float distToCenter = 999; // will only give new command if this is updated
    float angle;
    if(sig[0] == '1' and sig[1] == '2'){ // Blue and Red
//        distToCenter = centerSignature();
        angle = 90; // turns robot 90 degrees (left turn)
    }
    else if(sig[0] == '1' and sig[1] == '3'){ // Blue and Yellow
//        distToCenter = centerSignature();
        angle = -90; // turns robot -90 degrees (right turn)
    }
    else if(sig[0] == '1' and sig[1] == '4'){ // Blue and Green
//        distToCenter = centerSignature();
        angle = 180; // turns the robot 180 degrees (left turn around)
    }
    //if((distToCenter < 5 and distToBlock <= distSetpoint and instructionComplete == 1)){
    if((distToBlock <= distSetpoint and instructionComplete == 1)){
        newcommand(angle); // only executes command when the robot is close enough
                           // and when it is centered on the "traffic light"
                           // AND when the prev instruction is complete
    }

}// end controlAdjust()

void PixyCamRead(void)
{
    static int i = 0;
    int j;

    blocksR = pixyR.getBlocks(); // number of blocks
    //bluetooth.printf("getting blocks..\n\r");

    if (blocksR) {
        //bluetooth.printf("blocks here..\n\r");
        if (i % 50 == 0 ){
            //bluetooth.printf("\n\rDetected blocksR %d:\n\r", blocksR);
            //bluetooth.printf(buf);

            for (j = 0; j < blocksR; j++) {
                blockName(j); // gets the name of the currently observed sig
                //bluetooth.printf(" blockR %d: \n\r", j);
                //bluetooth.printf(buf);
                //pixyR.printBlock(pixyR.blocks[j]);
                if((sig[0] == '1') and (sig[1] == '2' or sig[1] == '3' or sig[1] == '4')){
                    r_mutex.lock();
                    x = pixyR.blocks[j].x;
                    y = pixyR.blocks[j].y;
                    width = pixyR.blocks[j].width;
                    height = pixyR.blocks[j].height;
                    distToBlock = ((((8.0/width)+(4.0/height))*257.2969)/2.0)/100.0; // using f from calibration (which is in cm)
                                                                                    // also converts to meters
                    accumulatedDist = 0;
                    r_mutex.unlock();
                    //bluetooth.printf("\n\r [%d,%d] position of CC %s is %f away\n\r",x,y,sig,distToBlock);
                    controlAdjust(); // adjusts controls based on observed sig
                    osTimerStart(OneShot, 2000); // Start or restart the watchdog timer interrupt and set to  2000ms.
                    led4 = 0;
                    break;
                }
            }// end for
        }// end if
    }// end if

} // end PixyCamRead()

int main(){
    brakeRight = 0;
    brakeLeft = 0;

    PwmLeft.period_us(T); // PWM period to 2.5 ms
    PwmLeft.pulsewidth_us(0); // Initialize to zero
    PwmRight.period_us(T); // PWM period to 2.5 ms
    PwmRight.pulsewidth_us(0); // Initialize to zero

    DE0.format(16,1);     // 16-bit data,mode: 1 (https://os.mbed.com/handbook/SPI) (BRK)
    DE0.frequency(1000000);  // SPI bit rate: 5 MHz (BRK)
    IoReset = 0;
    IoReset = 1;
    wait_us(5);
    IoReset = 0;
    SpiReset = 0;
    SpiReset = 1;
    wait_us(5);
    SpiReset = 0;

    ID = DE0.write(0x8002);
    bluetooth.printf("\n\rDevice ID: %x",ID);

    // Initialization of Globals
    Position1 = 0;Position2 = 0; Time1 = 0; Time2 = 0; //ii = 0;
    rampVal1 = 0; rampVal2 = 0; xiLeft = 0; xiRight = 0, xiPos = 0;

    Position = 0; Theta = 0; // Resetting position of robot

    omega_m = 0; speedSetpoint = 0; angularSetpoint = 0; thetaSetpoint = 0;
    PrevThetaError = 0;

    char newcontrol = 0;
    float tempkp, tempki, tempkd, tempdRamp;
    float tempSpeedSetpoint, tempAngularSetpoint;

    int debugFlag = 0;
    bluetooth.printf("\r\n\t\t\tECE 4333 Robotics Course Project - Traffic Light Robot");
    bluetooth.printf("\r\n\t\t\t\t Eric Henderson - Burak Koryan\n\r");
    bluetooth.printf("\r\nEnter desired mode of operation\r\n<0> for traffic or <1> for manual");
    bluetooth.printf("\r\n(for debugging via print statements, <d> and then as above)\n\r");
    while(true){
        if(bluetooth.readable()){
            outerLoopFlag = bluetooth.getc();
            //bluetooth.printf("%c\n\r",outerLoopFlag);
            if(outerLoopFlag == '1'){
                bluetooth.printf("\n\rPress any keys listed below to control the robot:\n\r");
                bluetooth.printf("\n\r\t\t\t(w)Forward\n\r \t(a)Turn left \t\t\t (d)Turn right \n\r \t\t\t(s)Reverse\n\r");
//                bluetooth.printf("... but a flag is implemented to take out outer loop control\n\r");
                break;
            }
            else if(outerLoopFlag == '0'){
                break;
            }
            else if(outerLoopFlag == 'd'){debugFlag = 1;bluetooth.printf("...debug mode active...\n\r");}
            else{bluetooth.printf("Invalid command, try again\n\r");}
        }
    }

    WatchdogId = osThreadCreate(osThread(WatchdogThread), NULL);

    PeriodicInnerId = osThreadCreate(osThread(InnerLoopThread), NULL);
    // Specify address of the PeriodicInnerInt ISR as PiControllerISR, specify the interval
    // in seconds between interrupts, and start interrupt generation:
    PeriodicInnerInt.attach(&PeriodicInnerLoopISR, h);

    // if in manual mode, don't enable the outer loop controller
    if(outerLoopFlag == '0'){
        PeriodicOuterId = osThreadCreate(osThread(OuterLoopThread), NULL);
        // Specify address of the PeriodicOuterInt ISR as PiControllerISR, specify the interval
        // in seconds between interrupts, and start interrupt generation:
        PeriodicOuterInt.attach(&PeriodicOuterLoopISR, h*10);
    }

    pixyR.init();


    bluetooth.printf("\r\nPress space for a new setpoint, enter for new controls\n\r");
    while(true){
        PixyCamRead(); // reading from the sensor (the PixyCam)
        if(bluetooth.readable()){
            newcontrol = bluetooth.getc();
            if(newcontrol == ' '){ // get new setpoints
                bluetooth.printf("\r\nEnter the desired forward velocity in m/s: ");
                bluetooth.scanf("%f", &tempSpeedSetpoint);
                bluetooth.printf("%f", tempSpeedSetpoint); // echo
                bluetooth.printf("\r\nEnter the desired angular velocity in rad/s: ");
                bluetooth.scanf("%f", &tempAngularSetpoint);
                bluetooth.printf("%f\n\r", tempAngularSetpoint); // echo

                // Mutex setting variables
                r_mutex.lock();
                speedSetpoint = tempSpeedSetpoint;
                angularSetpoint = tempAngularSetpoint;
                r_mutex.unlock();
            }
            if(newcontrol == '\r'){ // get new controls
                bluetooth.printf("\n\rProportional control: ");
                bluetooth.scanf("%f",&tempkp);
                bluetooth.printf("%f",tempkp);
                bluetooth.printf("\n\rIntegral control: ");
                bluetooth.scanf("%f",&tempki);
                bluetooth.printf("%f",tempki);
                bluetooth.printf("\n\rDerivative control: ");
                bluetooth.scanf("%f",&tempkd);
                bluetooth.printf("%f",tempkd);
                bluetooth.printf("\n\rRamping rate: ");
                bluetooth.scanf("%f",&tempdRamp);
                bluetooth.printf("%f\n\r",tempdRamp);
                r_mutex.lock();
                rampVal1 = 0;
                rampVal2 = 0;
                kp = tempkp;
                ki = tempki;
                kd = tempkd;
                dRamp = tempdRamp;
                r_mutex.unlock();
            }
            if(outerLoopFlag == '1'){ // only do this if in manual mode
                if(newcontrol == 'w'){
                    //bluetooth.printf("\n\r W Pressed : Speeding up\n\r");
                    r_mutex.lock();
                    speedSetpoint = speedSetpoint + 0.05;
                    osTimerStart(OneShot, 5000); // Start or restart the watchdog timer interrupt and set to  2000ms.
                    angularSetpoint = 0;
                    led4 = 0;
                    r_mutex.unlock();
                }
                if(newcontrol == 's'){
                    //bluetooth.printf("\n\r S Pressed : Slowing down\n\r");
                    r_mutex.lock();
                    speedSetpoint = speedSetpoint - 0.05;
                    osTimerStart(OneShot, 5000);
                    led4 = 0;
                    r_mutex.unlock();
                }
                if(newcontrol == 'a'){
                    //bluetooth.printf("\n\r A Pressed : Turning left\n\r");
                    r_mutex.lock();
                    angularSetpoint = angularSetpoint + 0.05;
                    osTimerStart(OneShot, 5000);
                    led4 = 0;
                    r_mutex.unlock();
                }
                if(newcontrol == 'd'){
                    //bluetooth.printf("\n\r D Pressed : Turning right\n\r");
                    r_mutex.lock();
                    angularSetpoint = angularSetpoint - 0.05;
                    osTimerStart(OneShot, 5000);
                    led4 = 0;
                    r_mutex.unlock();
                }
            }
        }// end new user input if()


        counter = counter + 1;
        if((counter >= 10) and (debugFlag == 1)){ // only print every 10th loop
            //r_mutex.lock();
            bluetooth.printf("%f, %f, ",omegaL,omegaR);
            bluetooth.printf("%f, %f, ",omegaL_m,omegaR_m);
            //bluetooth.printf("%f, %f  | ",eLeft,eRight);
            //bluetooth.printf("%f, %f  | ",eLeft,eRight);
            bluetooth.printf("%d, %f, ",x,distToBlock);
            //bluetooth.printf("%f, %f, %f, %f, %f\n",rampVal,Theta,e,output/T*100,xi);
            bluetooth.printf("%f, %f, ",thetaSetpoint, Theta);
            bluetooth.printf("%f, %f, ", speedSetpoint, angularSetpoint);
            bluetooth.printf("%f, %f    ",omega_m, angularSetpoint);
            bluetooth.printf("%f, %f\n\r",vLeft_m, vRight_m);
            //r_mutex.unlock();
            counter = 0;
            bluetooth.printf("\n\r");
        }// end print thing

        Thread::wait(25);

    }// end while(True)
} //end of main

void trafficLightMode(void){
    float ThetaError, dTerm;

    ThetaError = thetaSetpoint - Theta; // error in orientation
    dTerm = (ThetaError - PrevThetaError)/(h*10); // derivative term for controller

    if((abs(ThetaError)) < 5){stabilityFlag = stabilityFlag + 1;} // error is less than 5
    else{stabilityFlag = 0;}

    if(stabilityFlag >= 100){ // Instruction is only complete when 100 time steps
                            // read that the error is less than 5 degrees
        speedSetpoint = storedSpeedSetpoint;
        instructionComplete = 1;
        distToBlock = 0; // clearing previous so it doesn't keep moving
        x = 0;          // same as above
        osTimerStart(OneShot, 2000);
        bluetooth.printf(".....Finished Command.....\n\r");
    }

    r_mutex.lock();
    angularSetpoint = (ThetaError*(kp/1) + xiTheta*(ki/15) + dTerm*kd/100)/100.0; // new angular setpoint
    if(abs(angularSetpoint) > 2){angularSetpoint = (angularSetpoint/abs(angularSetpoint))*2;} // maintains direction
    else{xiTheta = xiTheta + ThetaError*(h*10);}
    r_mutex.unlock();

    PrevThetaError = ThetaError;
}

void approachMark(void){
    float headingError, dTermHeading;

    // Measuring the distance between the setpoint and the robot
    // (accumulatedDist away from the observed traffic light)
    posError = (distToBlock - 0.01) - accumulatedDist - distSetpoint;

    // Only track the movement in theta with the heading error if an actual
    // mark is observed- this prevents the robot from drifting off without seeing it
    if(blocksR){
        // calculating error in "heading" or the angle made with the distance to the object and the
        // x position in pixel space
        headingError = -atan2((x-160),distToBlock)*(180.0/3.1415); // 160 is the center x coordinate
        // ** in reviewing this code before submission, I now see that this angle would not be correct
        //    as the x-160 term is in pixels, instead of meters as the distToBlock is... changing this would
        //    improve the oscillation seen in the approach significantly
        dTermHeading = (headingError - headingErrorPrev)/(h*10);
        headingErrorPrev = headingError;
    }
    else{headingError = 0; dTermHeading = 0;}

    thetaSetpoint = thetaSetpoint + dTheta;

    r_mutex.lock();

    speedSetpoint = (posError*(kp/5) + (ki/15)*xiPos)/5.0;
    angularSetpoint = (headingError*kp + dTermHeading*kd/50)/200.0;

    if(abs(speedSetpoint) > 0.5){speedSetpoint = (speedSetpoint/abs(speedSetpoint))*0.5;} // maintains direction
    else{xiPos = xiPos + posError*(h*10);} // preventing windup
    if(abs(angularSetpoint) > 2){angularSetpoint = (angularSetpoint/abs(angularSetpoint))*2;} // maintains direction

    r_mutex.unlock();

    headingErrorPrev = headingError;
}

// ******** Outer Loop Periodic Timer Interrupt Thread ********
void OuterLoopThread(void const *argument){
    float omega_m_new;
    while(true){
        osSignalWait(0x1, osWaitForever); // Go to sleep until signal, SignalOuterLoop, is received.

        r_mutex.lock();
        omega_m_new = (vRight_m - vLeft_m)/L; // calcs for overall angular velocity of robot
        dTheta = (h*10)*((omega_m+omega_m_new)/2)*(180.0/3.1415);
        Theta = Theta + dTheta; // accumulates angular movement
        omega_m = omega_m_new;

        vRobot = (vRight_m + vLeft_m)/2; // calcs for overall forward velocity of robot
        accumulatedDist = accumulatedDist + vRobot*(h*10);
        r_mutex.unlock();

        // if an instruction has been set, do what it tells you
        if((instructionComplete == 0)){trafficLightMode();}
        // ... if not, approach a "traffic light"
        else{approachMark();}
        led2 = !led2;
    }
}

// ******** Inner Loop Periodic Timer Interrupt Thread ********
void InnerLoopThread(void const *argument) {
    while (true) {
        osSignalWait(0x1, osWaitForever); // Go to sleep until signal, SignalInnerLoop, is received.
        //led3= !led3; // Alive status - led3 toggles each time InnerLoopThread is signaled.

//        if(outerLoopFlag == '0'){outerLoop();} // only do

        // individual motor velocity calculations
        vLeft = -(speedSetpoint - angularSetpoint*L/2); // left motor is turned around
        vRight = speedSetpoint + angularSetpoint*L/2;

        // motor velocities to motor angular velocities
        omegaL = vLeft*(1.0/0.05)*(90.0/44.0)*18.75;
        omegaR = vRight*(1.0/0.05)*(90.0/44.0)*18.75;

        if(speedSetpoint == 0 and angularSetpoint == 0){brakeLeft = 1;brakeRight = 1;}
        else{brakeLeft = 0;brakeRight = 0;}

        // Reading Encoder Feedback
        SpiReset = 0;
        SpiReset = 1;
        wait_us(5);
        SpiReset = 0;

        ID = DE0.write(0x8004);

        dPosition1 = DE0.write(0xFF); // send dummy value and read position1
        dTime1 = DE0.write(0xFF);     // send dummy value and read time1
        dPosition2 = DE0.write(0xFF); // send dummy value and read position2
        dTime2 = DE0.write(0xFF);     // send dummy value and read time2

        // angular speed = (change in pos/ change in time) * Scaling
        omegaL_m = ((float(dPosition1)*2.0*3.1415)/64.0)/(float(dTime1)*10.24e-6);
        omegaR_m = ((float(dPosition2)*2.0*3.1415)/64.0)/(float(dTime2)*10.24e-6);


        // velocity of wheel = omega of motor scaled thru gears, to wheel and * radius
        vLeft_m = -(omegaL_m*(44.0/90.0)*0.05*(1.0/18.75)); // negative b/c motor is turned around
        vRight_m = omegaR_m*(44.0/90.0)*0.05*(1.0/18.75);

        // Ramping up
        rampVal1 = rampVal1 + dRamp;
        if(rampVal1 > omegaL){rampVal1 = omegaL;}
        rampVal2 = rampVal2 + dRamp;
        if(rampVal2 > omegaR){rampVal2 = omegaR;}


        // Setting PWM Output
        eLeft = rampVal1 - omegaL_m;   // speed error left wheel
        eRight = rampVal2 - omegaR_m; // speed error right wheel

        uLeft = kp*eLeft + ki*xiLeft;
        uRight = kp*eRight + ki*xiRight;

        outputLeft = abs(uLeft);
        outputRight = abs(uRight);

        if(outputLeft > 0.5*T){outputLeft = 0.5*T;} // 50% is the max duty cycle
        else{xiLeft = xiLeft + eLeft*h;}

        if(outputRight > 0.5*T){outputRight = 0.5*T;} // 50% is the max duty cycle
        else{xiRight = xiRight + eRight*h;}

        if(uLeft > 0){directionLeft = 0;}
        else{directionLeft = 1;}

        if(uRight > 0){directionRight = 0;}
        else{directionRight = 1;}

        // setting PWM outputs
        PwmLeft.pulsewidth_us(outputLeft);
        PwmRight.pulsewidth_us(outputRight);
        if(counter > 0 and counter <= 5){
            led3= !led3;
        }
    }// end while(True)
}// end InnerLoopThread()

void WatchdogThread(void const *argument){
    while(true) {
        osSignalWait(0x1, osWaitForever); // Go to sleep until a signal, SignalWatchdog, is received
        speedSetpoint = 0; // stops robot
        angularSetpoint = 0;
        led4 = 1; // led4 is activated when the watchdog timer times out
    }
}

// ******** Inner Loop Periodic Timer Interrupt Handler ********
void PeriodicInnerLoopISR(void) {
    osSignalSet(PeriodicInnerId,0x1); // Send signal to the thread with ID, PeriodicInnerId
}

// ******** Outer Loop Periodic Timer Interrupt Handler ********
void PeriodicOuterLoopISR(void) {
    osSignalSet(PeriodicOuterId,0x1); // Send signal to the thread with ID, PeriodicOuterId
}

// ******** Watchdog Interrupt Handler ********
void WatchdogISR(void const *n)
{
    osSignalSet(WatchdogId,0x1); // Send signal to thread with ID, WatchdogId, i.e., WatchdogThread
}
