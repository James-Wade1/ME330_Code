#include "xc.h"
#include "helpful_functions.h"
#include <math.h>
#pragma config FNOSC = FRCDIV //8MHz
#pragma config ICS = PGx3
#pragma config OSCIOFNC = OFF
#pragma config SOSCSRC = DIG // Pins 9 and 10
#define TURNING_TOP_SPEED 11000
#define TOP_SPEED 3050 //3250
#define RETURN_LINE_SPEED 4000
#define CANYON_TOP_SPEED 2500
#define TOP_POST_TASK_SPEED 2750
#define STEPPING_MODE .25
#define LEFT_W _LATB2
#define RIGHT_W _LATB9
#define FORWARD 0
#define BACKWARD 1
#define STEPS_PER_REV 200/STEPPING_MODE
#define FULL_TURN_REV 1.86
#define FULL_TURN_STEPS STEPS_PER_REV*FULL_TURN_REV
#define HALF_TURN_STEPS (FULL_TURN_STEPS/2 + 40) 
#define QUARTER_TURN_CORRECTION 35

#define BOOLEAN unsigned int
#define TRUE 1
#define FALSE 0

#define LEFT_QRD 0      // also found in helpful_functions
#define MIDDLE_QRD 1    // also found in helpful_functions
#define RIGHT_QRD 2     // also found in helpful_functions

#define LINE_THRESHOLD 1

#define NUM_TASKS_IMPLEMENTED 4

#define ONE_DEG 22
#define START_ANGLE 4000
#define END_ANGLE 1500
#define LASER_THRESHOLD 1400 //1900 is saturation max

int angle = START_ANGLE;

int rightSteps = 0;
int leftSteps = 0;
BOOLEAN goFaster = FALSE;
BOOLEAN isTurning = FALSE;

#define WHITE 1
#define BLACK 0
int color = WHITE;

/*PID CONTROL*/
float error = 0;
float P = 0;
float D = 0;
double Kp = 900; //900
double Kd = 1100; //1100
int KpFaster = 700;
int KdFaster = 900;
float correction = 0;
float lastError = 0;
double linePosition[3];

#define NUM_ERRORS 5

#define KS 80
#define KSFASTER 80
float previousErrors[NUM_ERRORS];
int nextIndex = 0;
double Ks = KS; // 80 works well, with Kp = 750 and Kd = 1000 and TOP_SPEED = 3500
double KsFaster = KSFASTER;
float errorSpeed = 0;
float sum = 0;

int currentSpeed = 0;
int leftAccelerationCounts = 0;
int rightAccelerationCounts = 0;
#define ACCELERATION_COUNTS 1

int pinFiredTime = 0;
BOOLEAN pinFired = FALSE;

/*STATES*/
enum direction {STOP, LEAVE_HOME, LINE_FOLLOW, PICK_BALL, CANYON, DROP_BALL, ENTER_HOME, LASER, SERVICE, TURNING} state;
enum {TURN, BACKUP, PICKUPBALL, RETURN} sub_state_pickup;
enum {TURNTOBIN, BACKUPBIN, DROPOFF, RETURNLINE} sub_state_dropoff;
enum {GO_FORWARD, FIRED} serviceStationSubState;
enum {NAVIGATING, EXITING} canyonSubState;

enum direction prevState;

BOOLEAN proxSensorTripped = FALSE;
BOOLEAN taskSensorTripped = FALSE;
BOOLEAN readingTask = FALSE;

/*QR CODE*/
BOOLEAN linesBeingRead = FALSE;
int numLinesRead = 0;
int back_to_line = 0;

int numTasksDone = 0;

void getAD();

void getError();

void getPID();

void correctSpeed(float error);

void goForward(long int speed);

void goBackward(long int speed);

void turnLeft90(long int speed);

void turnRight90(long int speed);

void turnRight180(long int speed);

void turnLeft45(long int speed);

void turnLeftUntilLine(long int speed);

void turnRightUntilLine(long int speed);

void qrRecoverLeft(int speed);

void turnLeftUntilLineRight(long int speed);

void backupUntilLine(long int speed);

void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void) {
    // triggers each PWM period
    _OC1IF = 0; // Clear interrupt flag
    
    int topSpeed = 0;
    if (state == CANYON) {
        topSpeed = CANYON_TOP_SPEED;
    }
    else {
        topSpeed = TOP_SPEED;
    }    
    if (isTurning == FALSE && (state == DROP_BALL || state == LEAVE_HOME || state == TURNING || (state == CANYON && canyonSubState == NAVIGATING))) {
        if (currentSpeed > topSpeed) {
            if (rightAccelerationCounts == ACCELERATION_COUNTS) {
                currentSpeed--;
                setRightStepperSpeed(currentSpeed);
                rightAccelerationCounts = 0;
            }
            else {
                rightAccelerationCounts++;
            } 
        }
        
    }
    rightSteps++;
}

void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void) {
    // triggers each PWM period
    _OC2IF = 0; // Clear interrupt flag
    int topSpeed = 0;
    if (state == CANYON) {
        topSpeed = CANYON_TOP_SPEED;
    }
    else {
        topSpeed = TOP_SPEED;
    }
    
    if (isTurning == FALSE && (state == DROP_BALL || state == LEAVE_HOME || state == TURNING || (state == CANYON && canyonSubState == NAVIGATING))) {
        if (currentSpeed > topSpeed) {
            if (leftAccelerationCounts == ACCELERATION_COUNTS) {
                currentSpeed--;
                setLeftStepperSpeed(currentSpeed);
                leftAccelerationCounts = 0;
            }
            else {
                leftAccelerationCounts++;
            }
        }
    }
    leftSteps++;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    _T1IF = 0; // Clear interrupt flag
    if (goFaster == FALSE && (numTasksDone == NUM_TASKS_IMPLEMENTED - 1) && pinFired == FALSE) {
        goFaster = TRUE;
    }
    if (state == LINE_FOLLOW || state == ENTER_HOME || state == SERVICE || (state == CANYON && canyonSubState == EXITING)) {
        getAD();
        getError();
        getPID();
        
        if (correction > 0) {
            if (goFaster == TRUE) {
                setRightStepperSpeed(TOP_POST_TASK_SPEED + correction + (int)KsFaster*errorSpeed);
                setLeftStepperSpeed(TOP_POST_TASK_SPEED - correction + (int)KsFaster*errorSpeed);
            }
            else {
                setRightStepperSpeed(TOP_SPEED + correction + (int)Ks*errorSpeed);
                setLeftStepperSpeed(TOP_SPEED - correction + (int)Ks*errorSpeed);
            }
        }
        /*
        else if (errorSpeed == 0) {
            setRightStepperSpeed(CANYON_TOP_SPEED);
            setLeftStepperSpeed(CANYON_TOP_SPEED);
        }
         * */
        else {
            if (goFaster == TRUE) {
                setRightStepperSpeed(TOP_POST_TASK_SPEED + correction + (int)Ks*errorSpeed);
                setLeftStepperSpeed(TOP_POST_TASK_SPEED - correction + (int)Ks*errorSpeed);
            }
            else {
                setRightStepperSpeed(TOP_SPEED + correction + (int)Ks*errorSpeed);
                setLeftStepperSpeed(TOP_SPEED - correction + (int)Ks*errorSpeed);
            }
        }
    }
}

int main(void) {
    _RCDIV = 0b000; //Postscalar of 1
    
    for (int i = 0; i < NUM_ERRORS; i++) {
        previousErrors[i] = 0;
    }
    
    goFaster = TRUE;
    
    setupPins();
    configAD();
    
    //Wheels
    _TRISB2 = 0; //LEFT_W, Left wheel direction
    _TRISB9 = 0; //RIGHT_W, Right wheel direction
    
    //Front IR sensor
    _TRISB8 = 1;
    
    //Side ultrasonic 
    _TRISB15 = 1;
    _ANSB15 = 1;
    
    //QRD Line followers
    _TRISB12 = 1;
    _TRISB13 = 1;
    _TRISA1 = 1;
    _ANSB12 = 1;
    _ANSB13 = 1;
    _ANSA1 = 1;
    
    //Service station
    _TRISA0 = 1;
    _ANSA0 = 1;
    
      //Laser on/off
    _TRISB7 = 0;
    _LATB7 = 0; 
    
    //QR Code sensor
    _TRISB14 = 1;
    _ANSB14 = 1;
    
    //QR Color Sensing
    _TRISA2 = 1;
    _ANSA2 = 1;
    
    //QR T Junction
    _TRISA3 = 1;
    _ANSA3 = 1;
    
    //Service Station Fire
    _TRISA4 = 0;
    _LATA4 = 0;
    
    //Laser Angle system
    _TRISB4 = 1;
    _ANSB4 = 1;
    
    setupOC1Interrupt(4);
    setupOC2Interrupt(5);
    setupOC1Right(); //Right wheel; OC1 is pin 14 (RA6), corresponds with RIGHT_W for direction
    setupOC2Left(); //Left wheel; OC2 is pin 4 (RB0), corresponds with LEFT_W for direction
    setupServo();
    setupTimer1();
    
    state = STOP;
    canyonSubState = NAVIGATING;
    turnServo(2500);
    
    haltRightWheel();
    haltLeftWheel();
    
    delay(50000);

    while(1) {
        switch(state) {
            case LEAVE_HOME:
                if (rightSteps >= 1700 && leftSteps >= 1700) {
                    isTurning = TRUE;
                    goForward(TOP_SPEED*1.5);
                }
                if (ADC1BUF14 < 3000) {
                    turnLeftUntilLine(TURNING_TOP_SPEED);
                    haltLeftWheel();
                    haltRightWheel();
                    delay(30000);
                    getAD();
                    getPID();
                    state = LINE_FOLLOW;
                    isTurning = FALSE;
                    goForward(TOP_SPEED);
                }
                break;
            case LINE_FOLLOW:
                if (ADC1BUF10 < 1200 && taskSensorTripped == FALSE) { //Change to 400
                    if (readingTask == FALSE) {
                        readingTask = TRUE;
                        rightSteps = 0;
                        leftSteps = 0;
                    }
                    numLinesRead++;
                    taskSensorTripped = TRUE;
                }
                else if (ADC1BUF10 > 2300 && taskSensorTripped == TRUE) { //Change to 2300
                    taskSensorTripped = FALSE;
                }
                
                if (readingTask == TRUE && rightSteps >= 475 && leftSteps >= 475) {
                    switch (numLinesRead) {
                        case 1: //Misread line continue
                            break;
                        case 2:
                            leftSteps = 0;
                            rightSteps = 0;
                            state = PICK_BALL;
                            sub_state_pickup = TURN;
                            break;
                        case 3:
                            leftSteps = 0;
                            rightSteps = 0;
                            state = DROP_BALL;
                            sub_state_dropoff = TURNTOBIN;
                            break;
                        case 4:
                            state = CANYON;
                            canyonSubState = NAVIGATING;
                            rightSteps = 0;
                            leftSteps = 0;
                            proxSensorTripped = FALSE;
                            goForward(CANYON_TOP_SPEED);
                    }
                    readingTask = FALSE;
                    numLinesRead = 0;
                    taskSensorTripped = FALSE;
                }
                
                if (numTasksDone >= NUM_TASKS_IMPLEMENTED && ADC1BUF14 < 3000) {
                    turnServo(3500);
                    state = TURNING;
                    turnLeft45(TURNING_TOP_SPEED);
                    turnLeftUntilLine(TURNING_TOP_SPEED);
                    haltLeftWheel();
                    haltRightWheel();
                    delay(70000);
                    getAD();
                    getPID();
                    state = ENTER_HOME;
                    goForward(CANYON_TOP_SPEED);
                }
                
                double voltage = ADC1BUF9 / 4095.0 * 3.3;
                if (ADC1BUF0 > 1500 && pinFired == FALSE && voltage < 2.4) {
                    state = SERVICE;
                    serviceStationSubState = GO_FORWARD;
                    rightSteps = 0;
                    leftSteps = 0;
                }
                
                break;
            case PICK_BALL:
                
                    if (sub_state_pickup == TURN){
                        if (leftSteps >= 0 && rightSteps >= 0){
                            turnServo(3500);
                            haltLeftWheel();
                            haltRightWheel();
                            delay(50000);
                            leftSteps = 0;
                            rightSteps = 0;
                            goBackward(TURNING_TOP_SPEED);
                            while(rightSteps < 130 && leftSteps < 130);
                            haltLeftWheel();
                            haltRightWheel();
                            delay(100000);
                            turnLeft90(TURNING_TOP_SPEED);
                            haltLeftWheel();
                            haltRightWheel();
                            delay(100000);
                            leftSteps = 0;
                            rightSteps = 0;
                            goBackward(RETURN_LINE_SPEED);
                            sub_state_pickup = BACKUP;
                        }
                    }
                    
                    else if (sub_state_pickup == BACKUP){ 
                        if (leftSteps >= 875 && rightSteps >= 875) {
                            goBackward(TOP_SPEED*4);
                        }
                        if (leftSteps >= 975 && rightSteps >= 975) { //rough number of steps is 965
                            haltRightWheel();
                            haltLeftWheel();
                            turnServo(2500);
                            sub_state_pickup = PICKUPBALL;
                        }
                    }
                    
                    else if (sub_state_pickup == PICKUPBALL){
                            delay((long int)450000);
                            leftSteps = 0;
                            rightSteps = 0;
                            isTurning = TRUE;
                            goForward(RETURN_LINE_SPEED);
                            sub_state_pickup = RETURN;
                    }
                    
                    else if (sub_state_pickup == RETURN){
                        if (ADC1BUF14 < 3000){
                            turnRightUntilLine(TURNING_TOP_SPEED);
                            haltLeftWheel();
                            haltRightWheel();
                            delay(50000);
                            getAD();
                            getPID();
                            turnServo(END_ANGLE+250);
                            isTurning = FALSE;
                            goForward(TOP_SPEED);
                            state = LINE_FOLLOW;
                            numTasksDone++;
                        }
                    }
                    
                                         
                break;
                
           case DROP_BALL:
                
                 if (sub_state_dropoff == TURNTOBIN){
                        if (leftSteps >= 200 && rightSteps >= 200){
                            haltLeftWheel();
                            haltRightWheel();
                            turnServo(2500);
                            delay(100000);
                            if (ADC1BUF13 < 3000) {
                                color = WHITE;
                            }
                            else {
                                color = BLACK;
                            }
                            
                            leftSteps = 0;
                            rightSteps = 0;
                            goBackward(TURNING_TOP_SPEED);
                            while(rightSteps < 200 && leftSteps < 200);
                            haltLeftWheel();
                            haltRightWheel();
                            delay(70000);
                            
                            if (color == BLACK){
                                turnLeft90(TURNING_TOP_SPEED);
                                back_to_line = 0;
                            }
                            else {
                                turnRight90(TURNING_TOP_SPEED);
                                back_to_line = 1;
                            }
                            haltLeftWheel();
                            haltRightWheel();
                            delay(70000);
                            leftSteps = 0;
                            rightSteps = 0;
                            goBackward(TOP_SPEED * 2);
                            sub_state_dropoff = BACKUPBIN;
                        }
                    }
                    
                    else if (sub_state_dropoff == BACKUPBIN){ 
                            if (leftSteps >= 275 && rightSteps >= 275) {
                                haltRightWheel();
                                haltLeftWheel();
                                sub_state_dropoff = DROPOFF;
                            }
                    }
                 
                    else if (sub_state_dropoff == DROPOFF){
                        turnServo(4800);
                        delay((long int)200000);
                        leftSteps = 0;
                        rightSteps = 0;
                        isTurning = TRUE;
                        goForward(TOP_SPEED * 2);
                        sub_state_dropoff = RETURNLINE;
                    }
                 
                    else if (sub_state_dropoff == RETURNLINE){
                        if (ADC1BUF14 < 3500){
                            if(back_to_line == 0){
                                turnRightUntilLine(TURNING_TOP_SPEED); 
                            }
                            else if(back_to_line == 1){
                                turnLeftUntilLine(TURNING_TOP_SPEED); 
                            }
                            haltLeftWheel();
                            haltRightWheel();
                            delay(50000);
                            getAD();
                            getPID();
                            turnServo(3000);
                            isTurning = FALSE;
                            goForward(TOP_SPEED);
                            state = LINE_FOLLOW;
                            numTasksDone++;
                        }
                    }
                        
                break;
                
            case CANYON:
                if (canyonSubState == NAVIGATING) {
                    if (_RB8 == 0) {
                            haltRightWheel();
                            haltLeftWheel();

                            delay(50000);
                            if (!(rightSteps > 400 && leftSteps > 400)) {
                                turnRight180(TURNING_TOP_SPEED);
                                haltLeftWheel();
                                haltRightWheel();
                                delay(10000);
                                leftSteps = 0;
                                rightSteps = 0;
                                goForward(CANYON_TOP_SPEED);
                            }
                            else {
                                double voltage = ADC1BUF9 / 4095.0 * 3.3;
                                if (voltage <= 1.3) {
                                    leftSteps = 0;
                                    rightSteps = 0;
                                    turnLeft90(TURNING_TOP_SPEED);
                                } 
                                else {
                                    leftSteps = 0;
                                    rightSteps = 0;
                                    turnRight90(TURNING_TOP_SPEED);
                                }
                                haltLeftWheel();
                                haltRightWheel();
                                delay(50000);
                                goForward(CANYON_TOP_SPEED);
                            }
                        }
                    else if (ADC1BUF14 < 3000){
                        haltLeftWheel();
                        haltRightWheel();
                        delay(50000);
                        double voltage = ADC1BUF9 / 4095.0 * 3.3;
                        if (voltage <= 1.3) {
                            leftSteps = 0;
                            rightSteps = 0;
                            turnLeftUntilLine(TURNING_TOP_SPEED);
                        }
                        else {
                            leftSteps = 0;
                            rightSteps = 0;
                            turnRightUntilLine(TURNING_TOP_SPEED);
                        }
                        haltLeftWheel();
                        haltRightWheel();
                        delay(50000);
                        getAD();
                        getPID();
                        canyonSubState = EXITING;
                        rightSteps = 0;
                        leftSteps = 0;
                        goForward(TOP_SPEED);
                    }
                }
                else if (canyonSubState == EXITING) {
                    if (rightSteps > 800 && leftSteps > 800) {
                        state = LINE_FOLLOW;
                        canyonSubState = NAVIGATING;
                        numTasksDone++;
                    }
                    else if (_RB8 == 0) {
                        haltRightWheel();
                        haltLeftWheel();
                        state = TURNING;
                        delay(50000);
                        leftSteps = 0;
                        rightSteps = 0;
                        turnRight90(TURNING_TOP_SPEED);
                        turnRightUntilLine(TURNING_TOP_SPEED/2);
                        haltLeftWheel();
                        haltRightWheel();
                        delay(50000);
                        getAD();
                        getPID();
                        state = LINE_FOLLOW;
                        goForward(TOP_SPEED);
                        canyonSubState = NAVIGATING;
                        numTasksDone++;
                    }
                }
                break;
            case ENTER_HOME:
                if (_RB8 == 0) {
                    leftSteps = 0;
                    rightSteps = 0;
                    while(rightSteps < 10 && leftSteps < 10);
                    haltLeftWheel();
                    haltRightWheel();
                    state = LASER;
                }
                break;
            case LASER:
                while(angle > END_ANGLE) {
                    if (ADC1BUF15 > LASER_THRESHOLD) {
                        angle -= 10*ONE_DEG;
                        turnServo(angle);
                        delay((long int)50000);
                        _LATB7 = 1;
                        while(1);
                    }
                    else {
                        angle -= 5*ONE_DEG;
                        turnServo(angle);
                        delay((long int)44444);
                    }
                }
                
                turnServo((START_ANGLE+END_ANGLE)/2);
                _LATB7 = 1;
                while(1); //make everything stop
                break;
            case SERVICE:
                switch(serviceStationSubState) {
                    case GO_FORWARD:
                        if (rightSteps >= 0 && leftSteps >= 0) { //110
                            state = TURNING;
                            haltLeftWheel();
                            haltRightWheel();
                            delay(20000);
                            _LATA4 = 1;
                            serviceStationSubState = FIRED;
                            pinFired = TRUE;
                            rightSteps = 0;
                            leftSteps = 0;
                            delay(50000);
                            state = SERVICE;
                        }
                        break;
                    case FIRED:
                        if (pinFired == TRUE && rightSteps >= 200) {
                            _LATA4 = 0;
                            state = LINE_FOLLOW;
                            numTasksDone++;
                        }
                        break;
                }
                break;
            case STOP:
                state = LEAVE_HOME;
                RIGHT_W = FORWARD;
                LEFT_W = FORWARD;
                setRightStepperSpeed(TOP_SPEED);
                setLeftStepperSpeed(TOP_SPEED );
                startRightWheel(&rightSteps);
                startLeftWheel(&leftSteps);
                break;
        }
    }
    
    return 0;
}


void getAD() {
    linePosition[LEFT_QRD] = (double)ADC1BUF1/4095.0 * 3.3;
    linePosition[MIDDLE_QRD] = (double)ADC1BUF11/4095.0 * 3.3;
    linePosition[RIGHT_QRD] = (double)ADC1BUF12/4095.0 * 3.3;
}

void getError() {
    if ((linePosition[LEFT_QRD] <= LINE_THRESHOLD) && (linePosition[MIDDLE_QRD] >= LINE_THRESHOLD) && (linePosition[RIGHT_QRD] >= LINE_THRESHOLD)) {
        error = -2;
    }
    else if ((linePosition[LEFT_QRD] <= LINE_THRESHOLD) && (linePosition[MIDDLE_QRD] <= LINE_THRESHOLD) && (linePosition[RIGHT_QRD] >= LINE_THRESHOLD)) {
        error = -1;
    }
    else if (linePosition[LEFT_QRD] >= LINE_THRESHOLD && linePosition[MIDDLE_QRD] <= LINE_THRESHOLD && linePosition[RIGHT_QRD] >= LINE_THRESHOLD) {
        error = 0;
    }
    else if (linePosition[LEFT_QRD] >= LINE_THRESHOLD && linePosition[MIDDLE_QRD] <= LINE_THRESHOLD && linePosition[RIGHT_QRD] <= LINE_THRESHOLD) {
        if (lastError <= -1  && numTasksDone < NUM_TASKS_IMPLEMENTED) {
            prevState = state;
            state = TURNING;
            qrRecoverLeft(TOP_SPEED*2);
            state = prevState;
            error = 0;
        }
        else {
            error = 1;
        }
    }
    else if (linePosition[LEFT_QRD] >= LINE_THRESHOLD && linePosition[MIDDLE_QRD] >= LINE_THRESHOLD && linePosition[RIGHT_QRD] <= LINE_THRESHOLD) {
        if (lastError <= -1 && numTasksDone < NUM_TASKS_IMPLEMENTED) {
            prevState = state;
            state = TURNING;
            qrRecoverLeft(TOP_SPEED*2);
            error = 0;
            state = prevState;
        }
        else {  
            error = 2;
        }
    }
    else if ((linePosition[LEFT_QRD] >= LINE_THRESHOLD && linePosition[MIDDLE_QRD] >= LINE_THRESHOLD && linePosition[RIGHT_QRD] >= LINE_THRESHOLD)) {
        if (lastError == 2) {
            error = 2.1;
        }
        else if (lastError == -2) {
            error = -2.1;
        }
    }
}

void getPID() {
    P = error;
    D = error - lastError;
    lastError = error;
    correctSpeed(error);
    if (goFaster == TRUE) {
        correction = (int)((KpFaster + (errorSpeed/(10.0)*110.0))*P + (KdFaster + (errorSpeed/(10.0)*150.0))*D);
    }
    else {
        correction = (int)((Kp + (errorSpeed/(10.0)*110.0))*P + (Kd + (errorSpeed/(10.0)*150.0))*D); //200 and 170
    }
}

void correctSpeed(float error) {
    sum = errorSpeed;
    sum -= previousErrors[nextIndex];
    previousErrors[nextIndex] = fabs(error);
    sum += fabs(error);
    
    if (nextIndex == (NUM_ERRORS - 1)) {
        nextIndex = 0;
    }
    else {
        nextIndex++;
    }
    
    errorSpeed = sum;
}

void goForward(long int speed) {
    RIGHT_W = FORWARD;
    LEFT_W = FORWARD;
    currentSpeed = speed;
    setRightStepperSpeed(speed);
    setLeftStepperSpeed(speed);
    startRightWheel(&rightSteps);
    startLeftWheel(&leftSteps); 
}

void goBackward(long int speed) {
    RIGHT_W = BACKWARD;
    LEFT_W = BACKWARD;
    currentSpeed = speed;
    setRightStepperSpeed(speed);
    setLeftStepperSpeed(speed); 
}

void turnLeft90(long int speed) {
    RIGHT_W = FORWARD;
    LEFT_W = BACKWARD;
    setRightStepperSpeed(speed);
    setLeftStepperSpeed(speed);
    startRightWheel(&rightSteps);
    startLeftWheel(&leftSteps); 
    
    isTurning = TRUE;
    if (state == CANYON) {
        while (leftSteps < (long int)(HALF_TURN_STEPS/2 + 45) && rightSteps < (long int) (HALF_TURN_STEPS/2 + 45));
    }
    else {
        while (leftSteps < (long int)(HALF_TURN_STEPS/2 + QUARTER_TURN_CORRECTION) && rightSteps < (long int) (HALF_TURN_STEPS/2 + QUARTER_TURN_CORRECTION));
    }
    isTurning = FALSE;
}

void turnRight90(long int speed) {
    RIGHT_W = BACKWARD;
    LEFT_W = FORWARD;
    setRightStepperSpeed(speed);
    setLeftStepperSpeed(speed);
    startRightWheel(&rightSteps);
    startLeftWheel(&leftSteps); 
    isTurning = TRUE;
    
    //currentSpeed = speed*2;
    
    //leftAccelerationCounts = 0;
    //rightAccelerationCounts = 0;
    
    while (leftSteps < (long int)(HALF_TURN_STEPS/2) && rightSteps < (long int) (HALF_TURN_STEPS/2));
    isTurning = FALSE;
    //leftAccelerationCounts = 0;
    //rightAccelerationCounts = 0;
}

void turnRight180(long int speed) {
    RIGHT_W = BACKWARD;
    LEFT_W = FORWARD;
    setRightStepperSpeed(speed);
    setLeftStepperSpeed(speed);
    startRightWheel(&rightSteps);
    startLeftWheel(&leftSteps); 
    isTurning = TRUE;
    
    while (leftSteps < (long int)(HALF_TURN_STEPS) && rightSteps < (long int) (HALF_TURN_STEPS));
    isTurning = FALSE;
}

void turnLeft45(long int speed) {
    RIGHT_W = FORWARD;
    LEFT_W = BACKWARD;
    setRightStepperSpeed(speed);
    setLeftStepperSpeed(speed);
    startRightWheel(&rightSteps);
    startLeftWheel(&leftSteps); 
    
    
    //currentSpeed = speed*2;
    //leftAccelerationCounts = 0;
    //rightAccelerationCounts = 0;
    isTurning = TRUE;
    while (leftSteps < (long int)(HALF_TURN_STEPS/4 + QUARTER_TURN_CORRECTION/2) && rightSteps < (long int) (HALF_TURN_STEPS/4 + QUARTER_TURN_CORRECTION/2));
    isTurning = FALSE;
    //leftAccelerationCounts = 0;
    //rightAccelerationCounts = 0;
}

void turnLeftUntilLine(long int speed) {
    RIGHT_W = FORWARD;
    LEFT_W = BACKWARD;
    
    setRightStepperSpeed(speed);
    setLeftStepperSpeed(speed);
    startRightWheel(&rightSteps);
    startLeftWheel(&leftSteps); 
    
    isTurning = TRUE;
    getAD();
    while(!(linePosition[LEFT_QRD] >= LINE_THRESHOLD && linePosition[MIDDLE_QRD] <= LINE_THRESHOLD && linePosition[RIGHT_QRD] >= LINE_THRESHOLD)) {
        getAD();
    }
    isTurning = FALSE;
}

void turnRightUntilLine(long int speed) {
    RIGHT_W = BACKWARD;
    LEFT_W = FORWARD;
    setRightStepperSpeed(speed);
    setLeftStepperSpeed(speed);
    startRightWheel(&rightSteps);
    startLeftWheel(&leftSteps); 
    
    isTurning = TRUE;
    getAD();
    while(!(linePosition[LEFT_QRD] >= LINE_THRESHOLD && linePosition[MIDDLE_QRD] <= LINE_THRESHOLD && linePosition[RIGHT_QRD] >= LINE_THRESHOLD)) {
        getAD();
    }
    isTurning = FALSE;
}

void qrRecoverLeft(int speed) {
    haltRightWheel();
    haltLeftWheel();
    delay(100000);
    
    goBackward(speed);
    delay(150000);
    
    haltRightWheel();
    haltLeftWheel();
    
    turnLeftUntilLineRight(TURNING_TOP_SPEED);
    
    haltRightWheel();
    haltLeftWheel();
    delay(120000);
    
    goForward(TOP_SPEED);
}

void turnLeftUntilLineRight(long int speed) {
    RIGHT_W = FORWARD;
    LEFT_W = BACKWARD;
    
    setRightStepperSpeed(speed);
    setLeftStepperSpeed(speed);
    startRightWheel(&rightSteps);
    startLeftWheel(&leftSteps); 
    
    isTurning = TRUE;
    getAD();
    while(!(linePosition[LEFT_QRD] >= LINE_THRESHOLD && linePosition[MIDDLE_QRD] <= LINE_THRESHOLD && linePosition[RIGHT_QRD] <= LINE_THRESHOLD)) {
        getAD();
    }
    isTurning = FALSE;
}

void backupUntilLine(long int speed) {
    RIGHT_W = BACKWARD;
    LEFT_W = BACKWARD;
    setRightStepperSpeed(speed);
    setLeftStepperSpeed(speed);
    startRightWheel(&rightSteps);
    startLeftWheel(&leftSteps); 
    
    isTurning = TRUE;
    while(ADC1BUF14 > 3000);
    isTurning = FALSE;
}