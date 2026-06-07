#include <msp430.h>
#include <string.h>
// Daniel To, EELE 371, Final Demo, 11/30/25
// Motor Type A: 1/16  (513 steps/rev)



// ADC Comments---------------------------------------------------------------

// picked 30 lb for high warning start because thats normal limit of the press
// picked 45 lb for unsafe because thats about 90% of max (50 lb)
// gives user most of the range but still some safety before hitting the max

//--Load to Voltage Calculations ---------------------------------
// Sensor sensitivity = 40 mV/lb = 0.04 V per pound
//
// Safe limit (30 lb):
//    V30 = 30 lb * 0.04 V/lb = 1.20 V
//
// Unsafe limit (45 lb, ~90% of 50-lb max):
//    V45 = 45 lb * 0.04 V/lb = 1.80 V
//
// Maximum loading (50 lb):
//    V50 = 50 lb * 0.04 V/lb = 2.00 V
// END ADC Comments---------------------------------------------------------------


// Timer Calculations---------------------------------------------------------------

//   SMCLK is 1MHz. I used ID__4 and TBIDEX__1 to divide by 4.
//   Timer Clock = 250 kHz (1 count = 4 microseconds).
//
//--SW2 Configuration---------------------------------------------
//    DataSheet specifies 5V to stay under 25 RPM
//    Time for 1 Rev: 60 seconds / 25 RPM = 2.4 seconds.
//    Time per step: 2.4s / 513 steps = 4.68 ms.
//    Timer Counts: 4.68ms * 250 counts/ms = 1170 counts.
//
//--SW1 Configuration----------------------------------------------
//    Need to move 36 degrees or 1/10th of a circle
//    - Steps needed: 513 / 10 = 51.3 which rounded to 51 steps.
//    - Target Time: 50% of 2.4s = 1.2 seconds.
//    - Time per step: 1.2s / 51 steps = ~23.5 ms.
//    - Timer Counts: 23.5ms * 250 counts/ms = 5875 counts.

// End Timer Calculations---------------------------------------------------------------

//  Global Variables---------------------------------------------------------------

//--ADC and I2C Variables----------------------------------------
    int ADCValue=0;
    int ADC_Sum = 0;
    int ADC_Count = 0;

    int do_read=0;
    int Unsafe_Flag = 0;
    int Data_Cnt=0;
    char TxPacket[]= {0x03, 0x00, 0x00, 0x12, 0x01, 0x01, 0x12, 0x25}; // Set to 12:00:00 PM Mon 12/01/25
    char RxData[7];        //Create array of size 7 to hold the received data
    unsigned int saved_seconds; //View in Hex
    unsigned int saved_minutes;
    unsigned int saved_hours;
    unsigned int saved_day;
    unsigned int saved_month;
    unsigned int saved_date;
    unsigned int saved_year;
//--UART Variables-----------------------------------------
    int position;
    char Forward[] = "\r\n Motor advanced 1 step \r\n";
    char Reverse[] = "\r\n Motor reversed 1 rotation \r\n";
    int S1Flag = 0;
    int S2Flag = 0;
//--Motor Control Variables--------------------------------
    int Steps_Remaining = 0;
    int Direction = 0;       // 0 = Forward, 1 = Reverse
    int Step_Index = 0;
    int Desired_Steps = 51;
// Used the Double-Step Pattern from the Slides
// Sequence becomes: 0011, 0110, 1100, 1001
    const unsigned char Motor_Pattern[4] = { 0b0011, 0b0110, 0b1100, 0b1001};
//---------------------------------------------------------------
// End Global Variables
//---------------------------------------------------------------

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer


// INIT---------------------------------------------------------------
//--Configure LED1 P1.0------------------------------------------
    P1SEL0 &= 0x00;
    P1SEL1 &= 0x00;

    P1DIR |= BIT0;
    P1OUT &= ~BIT0;
//--Configure LED 2 P6.6------------------------------------------
    P6SEL0 &= 0x00;
    P6SEL1 &= 0x00;

    P6DIR |= BIT6;
    P6OUT &= ~BIT6;
//--Configure P5.0:2 as Outputs for LEDS--------------------------
    P5SEL0 &= 0x00;
    P5SEL1 &= 0x00;

    P5DIR |= BIT0|BIT1|BIT2;
    P5OUT &= ~(BIT0|BIT1|BIT2);
//--Configure P3.0:3 as Outputs for Motor-------------------------
    P3SEL0 &= 0x00;
    P3SEL1 &= 0x00;

    P3DIR |= BIT0|BIT1|BIT2|BIT3;
    P3OUT &= ~(BIT0|BIT1|BIT2|BIT3);
//--Configure SW1------------------------------------------------
    P4SEL0 &= ~BIT1;
    P4SEL1 &= ~BIT1;

    P4DIR &= ~BIT1;
    P4REN |= BIT1;      // Resistor enable
    P4OUT |= BIT1;      // Specify as a pull-up Resistor
//--Configure SW2-------------------------------------------------
    P2SEL0 &= ~BIT3;
    P2SEL1 &= ~BIT3;

    P2DIR &= ~BIT3;
    P2REN |= BIT3;
    P2OUT |= BIT3;
//--Configure Port for UART Tx---------------------------------------
    P4SEL1 &= ~BIT3;
    P4SEL0 |= BIT3;
//--Configure Ports for UART Rx--------------------------------------
    P4SEL1 &= ~BIT2;
    P4SEL0 |= BIT2;

    P1DIR |= BIT0;
    P1OUT &= ~BIT0;
//--Configure ADC Ports------------------------------------------
    P1SEL1 |= BIT4;     //Configure Pin 4 for ADC
    P1SEL0 |= BIT4;
//--Configure I2C Pins-------------------------------------------
    P1SEL1 &= ~BIT3;    // P1.3 = SCL
    P1SEL0 |=  BIT3;

    P1SEL1 &= ~BIT2;    // P1.2 = SDA
    P1SEL0 |=  BIT2;
//--Configure Timer TB0--------------------------------------------
    TB0CTL |= TBCLR;   // Clear Timer Count and Divider
    TB0CTL |= TBSSEL__SMCLK;        //Choose SMCLK for Timer
    TB0CTL |= MC__UP;                //Set mode to up
    TB0CTL |= ID__4;                 // Set Divider to 4
    TB0EX0 |= TBIDEX__1;             // Extra divider = 1
//--Configure ADC---------------------------------------------------
    ADCCTL0 &= ~ADCSHT;     //clear ADCSHT from def. of ADCSHT=01
    ADCCTL0 |= ADCSHT_2;    //Converstion cycles =16 (ADCSHT=10)
    ADCCTL0 |= ADCON;       //turn ADC on

    ADCCTL1 |= ADCSSEL_2;   //ADC Clock Source = SMCLK
    ADCCTL1 |= ADCSHP;      //sample singal source = sampling timer

    ADCCTL2 &= ~ADCRES;      //Clear ADCRES from def. of ADCRES=01
    ADCCTL2 |= ADCRES_2;     // Resolution = 12 bit

    ADCMCTL0 |= ADCINCH_4;    //ADC Input CHannel = A4
//--I2C Core Configuration---------------------------------------
    UCB0CTLW0 |= UCSWRST;           //Pause Communication

    UCB0CTLW0 |= UCSSEL__SMCLK;     //Choose SMCLK
    UCB0BRW = 10;                   //Divide BRCLK by 10 for SCL=100KHz

    UCB0CTLW0 |= UCMODE_3;          //I2C mode
    UCB0CTLW0 |= UCMST;             //Specify MSP430 as Master
    UCB0I2CSA = 0x0068;             //Set Slave address to 0x0068 for RTC

    UCB0CTLW1 |= UCASTP_2;          //Auto Stop when UCB0TBCNT is reached

    UCB0CTLW0 &= ~UCSWRST;          //Un-pause Communication
//--Pause Communication-------------------------------------------
    UCA1CTLW0 |= UCSWRST;
//--Configure eUSCI_A1-------------------------------------------
    UCA1CTLW0 |= UCSSEL__SMCLK;     //Choose SMCLK
    UCA1BRW = 17;
    UCA1MCTLW |= 0x4A00;            //Set for 1Mhz and 57600 bps.
//--Un-pause Communication----------------------------------------
    UCA1CTLW0 &= ~UCSWRST;

// Initialize Interrupts

//--I2C Interrupts-----------------------------------------------
    UCB0IE = 0;         // I2C IRQ
//--ADC Interrupts----------------------------------------------
    ADCIE |= ADCIE0;        //Enable ADC Conv Complete IRQ
//--SW1 Interrupts-----------------------------------------------
     P4IE |= BIT1;
     P4IFG &= ~BIT1;
     P4IES |= BIT1;
//--SW2 Interrupts------------------------------------------------
     P2IE |= BIT3;
     P2IFG &= ~BIT3;
     P2IES |= BIT3;
//--UART Interrupts-----------------------------------------------
     UCA1IE |= UCRXIE;
     UCA1IFG &= ~UCRXIFG;
//--Enable Global Interrupts-------------------------------------
     __enable_interrupt();
//--Exit Low power mode-------------------------------------------
     PM5CTL0 &= ~LOCKLPM5;

// Initialize RTC (One Time Write)-------------------------------

     Data_Cnt = 0;                       // Start at zero
     UCB0IE    |= UCTXIE0;               // enable TX ISR
     UCB0CTLW0 |= UCTR;                  // Set to TX Mode
     UCB0TBCNT  = sizeof(TxPacket);      // 7 bytes of Data
     UCB0CTLW0 |= UCTXSTT;               // Generate START
     while ((UCB0IFG & UCSTPIFG) == 0);  // wait for a STOP condition
     UCB0IFG &= ~UCSTPIFG;               // Clear Flag
     UCB0IE  &= ~UCTXIE0;                // disable TX ISR
//----------------------------------------------------------

     ADCCTL0 |= ADCENC | ADCSC;  // enable and start conversion
// Initialization End---------------------------------------------------------------


while(1){

// MAIN---------------------------------------------------------------
// ADC and I2C-----------------------------------------
    ADCCTL0 |= ADCENC | ADCSC;  // enable and start conversion

    UCB0CTLW0 |= UCSWRST;   // Put eUSCI in reset (Clears UCBBUSY)
    UCB0CTLW0 &= ~UCSWRST;  // Release from reset to start fresh

    if (do_read) {  // set by ADC ISR when load is unsafe
        do_read = 0; // Reset the flag

//---Transmit register address (0x03) -----------------------------------
    UCB0CTLW0 |= UCTR;    //Set to Tx Mode
    UCB0TBCNT  = 1;         //Set Auto Stop to 1 Byte
    UCB0CTLW0 |= UCTXSTT;      //Generate START Condition
    while ((UCB0IFG & UCTXIFG0)==0);  //Wait for Tx Buffer
    UCB0TXBUF = 0x03;               //Set Slave Address
    while ((UCB0IFG & UCSTPIFG) == 0);  //Wait for Stop
    UCB0IFG &= ~UCSTPIFG;       //Clear Flag

//--Receive 7 bytes of Data----------------------------------------------
    UCB0CTLW0 &= ~UCTR;     //Set to Rx Mode
    UCB0TBCNT  = 7;         //Set Auto Stop to 7 Bytes
    UCB0CTLW0 |= UCTXSTT;   //Generate START Condition
    int i = 0;
    while (i < 7) {         //Receive 7 Bytes of Data put into RxData
        while ((UCB0IFG & UCRXIFG0)==0);
        RxData[i++] = UCB0RXBUF;
    }
    while ((UCB0IFG & UCSTPIFG) == 0); //Wait for Stop
    UCB0IFG &= ~UCSTPIFG;              //Clear Flag

//--Save Time---------------------------------------------------------------
    saved_seconds = RxData[0];
    saved_minutes = RxData[1];
    saved_hours = RxData[2];
    saved_day = RxData[3];
    saved_date = RxData[4];
    saved_month = RxData[5];
    saved_year = RxData[6];

//--Transmit the Data to the Serial Terminal------------------------------
    char TimeLabel[] = "\r\n Unsafe at ";
    int j;
    i=0;

    while(i < strlen(TimeLabel)){
        UCA1TXBUF = TimeLabel[i];
        for (j = 0; j < 500; j++) { }
        i++;
    }

//--Print Time-stamp---------------------------------------------------------
    //----Hours RxData[2]----
    UCA1TXBUF = ((saved_hours & 0xF0) >> 4) + '0';
    for (j = 0; j < 500; j++) { }
    UCA1TXBUF = (saved_hours & 0x0F) + '0';
    for (j = 0; j < 500; j++) { }

    UCA1TXBUF = ':';
    for (j = 0; j < 500; j++) { }

    //----Minutes RxData[1]----
    UCA1TXBUF = ((saved_minutes & 0xF0) >> 4) + '0';
    for (j = 0; j < 500; j++) { }
    UCA1TXBUF = (saved_minutes & 0x0F) + '0';
    for (j = 0; j < 500; j++) { }

    UCA1TXBUF = ':';
    for (j = 0; j < 500; j++) { }

    //----Seconds RxData[0]----
    UCA1TXBUF = ((saved_seconds & 0xF0) >> 4) + '0';
    for (j = 0; j < 500; j++) { }
    UCA1TXBUF = (saved_seconds & 0x0F) + '0';
    for (j = 0; j < 500; j++) { }

    // Space between Time and Date
    UCA1TXBUF = ' ';
    for (j = 0; j < 500; j++) { }

    //----Month RxData[5]----
    UCA1TXBUF = ((saved_month & 0xF0) >> 4) + '0';
    for (j = 0; j < 500; j++) { }
    UCA1TXBUF = (saved_month & 0x0F) + '0';
    for (j = 0; j < 500; j++) { }

    UCA1TXBUF = '/';
    for (j = 0; j < 500; j++) { }

    //----Date RxData[4]----
    UCA1TXBUF = ((saved_date & 0xF0) >> 4) + '0';
    for (j = 0; j < 500; j++) { }
    UCA1TXBUF = (saved_date & 0x0F) + '0';
    for (j = 0; j < 500; j++) { }

    UCA1TXBUF = '/';
    for (j = 0; j < 500; j++) { }

    //----Year RxData[6]----
    // Hardcode "20" first to make it 2025
    UCA1TXBUF = '2';
    for (j = 0; j < 500; j++) { }
    UCA1TXBUF = '0';
    for (j = 0; j < 500; j++) { }

    // Print the year from RTC
    UCA1TXBUF = ((saved_year & 0xF0) >> 4) + '0';
    for (j = 0; j < 500; j++) { }
    UCA1TXBUF = (saved_year & 0x0F) + '0';
    for (j = 0; j < 500; j++) { }

    // New Line / Return
    UCA1TXBUF = '\n';
    for (j = 0; j < 100; j++) { }

    UCA1TXBUF = '\r';
    for (j = 0; j < 100; j++) { }

    }

// ADC and I2C end---------------------------------------------------------------

// Motor Controls---------------------------------------------------------------
    if(Unsafe_Flag == 0){ // Check flag so we don't move if Unsafe
//-- Check SW1 (Forward - Slow Precision) -----------------
    if( (P4IN & BIT1) == 0 ){

        while(Steps_Remaining > 0){
           // Wait for previous move to finish
        }

        // Setup Forward Move
        TB0CCR0 = 5875;         // Slow speed (23.5ms)
        Steps_Remaining = Desired_Steps;   // 36 degrees (1/10th rotation) default
        Direction = 0;          // Specify Direction to Forward

        TB0CCTL0 |= CCIE;       // Start the Timer Interrupt
        while((P4IN & BIT1) == 0); //To stop Repeat Movements
    }


//-- Check SW2 (Reverse - Fast Reset) -------------------------
    if( (P2IN & BIT3) == 0 ){

        while(Steps_Remaining > 0){
            // Wait for previous move to finish
        }

        // Setup Reverse Move
        TB0CCR0 = 1170;        // Max safe speed (4.68ms)
        Steps_Remaining = 513;  // Full 360 Rotation
        Direction = 1;          // Specify Direction to Reverse

        TB0CCTL0 |= CCIE;       // Start the Timer Interrupt
        while((P2IN & BIT3) == 0); //To stop Repeat Movements
        }
    }
// Motor Controls End---------------------------------------------------------------

    }

    return 0;
}

// End Main---------------------------------------------------------------

// ISR's---------------------------------------------------------------
//--SW1 Interrupt (Forward)--------------------------------------
#pragma vector = PORT4_VECTOR
__interrupt void ISR_PORT_4(void){
    int i;
      for (i = 0; i < 20000; i++) {
          // Delay Loop
      }

    position = 0;     // Reset position

    S1Flag = 1;       // Set flags based on switch pressed
    S2Flag = 0;

    UCA1IE  |= UCTXCPTIE;     // Enable TX complete interrupt
    UCA1IFG &= ~UCTXCPTIFG;   // Clear TX complete flag
    UCA1TXBUF = Forward[position++];  // Send first char

    P4IFG &= ~BIT1;       // Clear interrupt flag
}

//--SW2 Interrupt (Reverse)--------------------------------------
#pragma vector = PORT2_VECTOR
__interrupt void ISR_PORT_2(void){
    int i;
      for (i = 0; i < 20000; i++) {
          // Delay Loop
      }
    position = 0;     // Reset position

    S1Flag = 0;
    S2Flag = 1;       // Set flags based on switch pressed

    UCA1IE  |= UCTXCPTIE;     // Enable TX complete interrupt
    UCA1IFG &= ~UCTXCPTIFG;   // Clear TX complete flag
    UCA1TXBUF = Reverse[position++];  // Send first char

    P2IFG &= ~BIT3;       // Clear interrupt flag
}

//--ADC Interrupt---------------------------------------------------
#pragma vector = ADC_VECTOR
__interrupt void ADC_ISR(void){

// Thresholds:
// High = 30 lb  (1.20 V)
// UNSAFE  = 45 lb  (1.80 V, higher than 90% of 50 lb rating then unsafe)
// 1.2V is roughly 1489
// 1.8V is roughly 2234

    ADC_Sum += ADCMEM0;
    ADC_Count++;

    if (ADC_Count == 4) {
        ADCValue = ADC_Sum >> 2; // divide by 4

        // Reset counters for next batch
        ADC_Sum = 0;
        ADC_Count = 0;

    // Safe: 0–30 lbs
    if (ADCValue < 1475) {
        P1OUT &= ~BIT0;   // Red OFF
        P6OUT |=  BIT6;   // Green ON

        P5OUT &= ~(BIT1 | BIT2);
        P5OUT |= BIT0;
        Unsafe_Flag = 0;
    }
    // Warning: 30–45 lbs
    else if (ADCValue < 2245) {
        P1OUT &= ~BIT0;   // Red OFF
        P6OUT &= ~BIT6;   // Green OFF

        P5OUT &= ~(BIT0 | BIT2);
        P5OUT |= BIT1;
        if (ADCValue < 2200) {
             Unsafe_Flag = 0;
        }
    }
    // Unsafe: >45 lbs
    else {
        P1OUT |=  BIT0;   // Red ON
        P6OUT &= ~BIT6;   // Green OFF

        P5OUT &= ~(BIT0 | BIT1);
        P5OUT |= BIT2;
        if(Unsafe_Flag == 0){
            do_read = 1;      // grab time stamp in main()
            Unsafe_Flag=1;
            }
        }
    }
}

//--I2C ISR------------------------------------------------------
#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void){
    switch (UCB0IV) {
    case 0x18: //TXIFG0
        UCB0TXBUF = TxPacket[Data_Cnt++];    //Transmit Data
        if (Data_Cnt >= sizeof(TxPacket)){   //Stop Transmitting
            Data_Cnt = 0;                    //Reset Count if we go over size
        }

        break;

    default: //Other Flags Ignored
        break;
    }
}


//--eUSCI_A1 interrupt-------------------------------------------
#pragma vector = EUSCI_A1_VECTOR
__interrupt void ISR_eUSCI_A1(void){

   if (UCA1IFG & UCRXIFG) {
        char rx_char = UCA1RXBUF;

        switch(rx_char){
            case '1':
                Desired_Steps = 128;
            break; // 0.25 rotation

            case '2':
                Desired_Steps = 256;
            break; // 0.50 rotation

            case '3':
                Desired_Steps = 385;
            break; // 0.75 rotation

            case '4':
                Desired_Steps = 513;
            break; // Full Rotation

            case '0':
                Desired_Steps = 51;
            break;

            default:
                // No change
            break;
            }
        UCA1IFG &= ~UCRXIFG;        //Clear Receive Flag
        }

    if (UCA1IFG & UCTXCPTIFG) {

    if (S1Flag) {
        if (position < strlen(Forward)) { //Check if still within forward array
            UCA1TXBUF = Forward[position++];
        } else {
            S1Flag = 0;
            UCA1IE &= ~UCTXCPTIE;    // Done sending Forward
        }
    }

    if (S2Flag) {
        if (position < strlen(Reverse)) { //Check if still within reverse array
            UCA1TXBUF = Reverse[position++];
        } else {
            S2Flag = 0;
            UCA1IE &= ~UCTXCPTIE;    // Done sending Reverse
        }
    }

    UCA1IFG &= ~UCTXCPTIFG;         // Clear TX complete flag
    }
}

//--Timer TB0 ISR------------------------------------------
#pragma vector = TIMER0_B0_VECTOR
__interrupt void TB0_ISR(void){
    TB0CCTL0 &= ~CCIFG; // Clear interrupt flag

    if (Steps_Remaining > 0){
        // Output the pattern to P3.0 - P3.3
        P3OUT = Motor_Pattern[Step_Index];

        // Update Index for the next step
        if (Direction == 0){ // Forward Direction
            if(Step_Index == 3){
                Step_Index = 0;
            }
            else{
                Step_Index++;
            }
        }

        else{ // Reverse Direction
            if(Step_Index == 0){
                Step_Index = 3;
            }
            else{
                Step_Index--;
            }
        }

        Steps_Remaining--; // Count down
    }

    else {
        // Movement Complete
        TB0CCTL0 &= ~CCIE; // Turn off Timer Interrupt
    }
}


// End ISR---------------------------------------------------------------
