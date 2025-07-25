#include "helpful_functions.h"
#include "xc.h"

void setupPins() {
    TRISA = 0;
    TRISB = 0;
    ANSA = 0;
    ANSB = 0;
    LATA = 0;
    LATB = 0;
}

void setupOC1Right() {
    OC1CON1 = 0; // clear OC1CON1 register
    OC1CON2 = 0; // clear OC1CON2 register

    OC1CON1bits.OCTSEL = 0b111;
    OC1CON1bits.OCM = 0b110; // edge-aligned PWM signal. Set to 0 to disable
    OC1CON2bits.SYNCSEL = 0b11111;
    OC1CON2bits.OCTRIG = 0;
}

void setupOC2Left() {
    OC2CON1 = 0; // clear OC1CON1 register
    OC2CON2 = 0; // clear OC1CON2 register

    OC2CON1bits.OCTSEL = 0b111;
    OC2CON1bits.OCM = 0b110; // edge-aligned PWM signal. Set to 0 to disable
    OC2CON2bits.SYNCSEL = 0b11111;
    OC2CON2bits.OCTRIG = 0;
}

void setupServo(){
    OC3CON1 = 0; // clear OC1CON1 register
    OC3CON2 = 0; // clear OC1CON2 register

    OC3CON1bits.OCTSEL = 0b111;
    OC3CON1bits.OCM = 0b110; // edge-aligned PWM signal. Set to 0 to disable
    OC3CON2bits.SYNCSEL = 0b11111;
    OC3CON2bits.OCTRIG = 0;
    OC3RS = (unsigned int) 79999;
    turnServo(3500);
}

void turnServo(int period){
    OC3R = period;
}

void setupOC1Interrupt(int priority) {
    _OC1IP = priority; //Select the interrupt priority
    _OC1IF = 0; //clear flag
    _OC1IE = 1; //enable the OC2 interrupt
}

void setupOC2Interrupt(int priority) {
    _OC2IP = priority; //Select the interrupt priority
    _OC2IF = 0; //clear flag
    _OC2IE = 1; //enable the OC2 interrupt
}

void setupINT1Interrupt(int priority) {
    _INT1IP = priority;
    _INT1IF = 0;
    _INT1EP = 1;
    _INT1IE = 1;
}

void setRightStepperSpeed(int period) {
    OC1CON1bits.OCM = 0b110;
    OC1RS = period;
    OC1R = period/2;
}

void setLeftStepperSpeed(int period) {
    OC2CON1bits.OCM = 0b110;
    OC2RS = period;
    OC2R = period/2;
}

void delay(long int n) {
    long int k = 0;
    while(k < n) {
        k++;
    }
}

void haltRightWheel() {
    OC1CON1bits.OCM = 0;
}

void haltLeftWheel() {
    OC2CON1bits.OCM = 0;
}

void startRightWheel(int* rightSteps) {
    *rightSteps = 0;
    OC1CON1bits.OCM = 0b110;
}

void startLeftWheel(int* leftSteps) {
    *leftSteps = 0;
    OC2CON1bits.OCM = 0b110;
}

void rampUpRightWheel(long int desiredSpeed) {
    
}

void rampUpLeftWheel(long int desiredSpeed);

void configAD(void)
{
    _ADON = 0;    // AD1CON1<15> -- Turn off A/D during config
    
    // AD1CON1 register
    _ADSIDL = 0;  // AD1CON1<13> -- A/D continues in idle mode
    _MODE12 = 1;  // AD1CON1<10> -- 12-bit A/D operation
    _FORM = 0;    // AD1CON1<9:8> -- Unsigned integer output
    _SSRC = 7;    // AD1CON1<7:4> -- Auto conversion (internal counter)
    _ASAM = 1;    // AD1CON1<2> -- Auto sampling

    // AD1CON2 register
    _PVCFG = 0;   // AD1CON2<15:14> -- Use VDD as positive
                  // ref voltage
    _NVCFG = 0;   // AD1CON2<13> -- Use VSS as negative
                  // ref voltage
    _BUFREGEN = 1;// AD1CON2<11> -- Result appears in buffer
                  // location corresponding to channel
    _CSCNA = 1;   // AD1CON2<10> -- Scans inputs specified
                  // in AD1CSSx registers
    _SMPI = 8;	  // AD1CON2<6:2> -- Every 9th conversion sent (number of channels sampled -1)
                  // to buffer (if sampling 10 channels)
    _ALTS = 0;    // AD1CON2<0> -- Sample MUXA only

    _CN2PDE = 1; 
    
    // AD1CON3 register
    _ADRC = 0;    // AD1CON3<15> -- Use system clock
    _SAMC = 0;    // AD1CON3<12:8> -- Auto sample every A/D
                  // period TAD
    _ADCS = 0x3F; // AD1CON3<7:0> -- A/D period TAD = 64*TCY

    AD1CSSL = 0; // choose A2D channels you'd like to scan here.
    // AD1CSS registers
    _CSS12 = 1; //Line following (pin 15)
    _CSS11 = 1; //Line following (pin 16)
    _CSS1 = 1; // Line following (pin 3)
    _CSS10 = 1; //Task detection (pin 17)
    _CSS9 = 1; // Ultrasonic Sensor (pin 18)
    _CSS13 = 1; // Color sensing (pin7)
    _CSS14 = 1; // T Junction (pin 8)
    _CSS15 = 1;
    _CSS0 = 1; //Service station (pin 2)
    
    _ADON = 1;    // AD1CON1<15> -- Turn on A/D
}

void setupTimer1() {
    T1CONbits.TON = 1; // start timer
    T1CONbits.TCS = 0; // select internal clock
    T1CONbits.TCKPS = 0b00; // select prescale of 64
    
    _T1IP = 6; // Select interrupt priority (1-7, 7 being highest)
    _T1IF = 0; // Clear interrupt flag
    _T1IE = 1; // Enable interrupt
    PR1 = 10000; // set the period in clock ticks; 40000 is 10ms
    TMR1 = 0; // reset Timer1 to zero;
}

void setupINT0(int priority) {
    _INT2IP = priority; // Select interrupt priority (4 is default)
    _INT2IF = 0; // Clear interrupt flag
    _INT2EP = 1; // Set edge detect polarity to positive edge
    _INT2IE = 1; // Enable interrupt
}