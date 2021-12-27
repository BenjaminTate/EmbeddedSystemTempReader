#include <msp430.h>
#include <math.h>
#include <stdlib.h>

#define CALADC12_12V_30C *((unsigned int *)0x1A1A) // Temperature Sensor Calibration-30 C
//See MSP430FR6989 datasheet page 131 for TLV table memory mapping
#define CALADC12_12V_85C *((unsigned int *)0x1A1C) // Temperature Sensor Calibration-85 C
#define BUT1 BIT1 // Button S1 at P1.1
#define BUT2 BIT2 // Button S2 at P1.2
#define ACLK 0x0100 // Timer_A ACLK source
#define TAIfg 0x0001 // Used to look at Time A Interrupt Flag
#define UART_CLK_SEL 0x0080 // Specifies accurate SMCLK clock for UART
#define BR0_FOR_9600 0x34 // Value required to use 9600 baud
#define BR1_FOR_9600 0x00 // Value required to use 9600 baud
#define CLK_MOD 0x4911 // Microcontroller will "clean-up" clock signal

// LCD Prototype
void Ini_LCD();

// Prototype for getting temp vals
int firstNum(int n);
int lastNum(int n);

// Queue prototypes
int empty(int head, int tail);
int full(int tail, int size);
int dequeue(int *q, int *head);
void enqueue(int *q, int *tail, int element);
void init(int *head, int *tail);

// UART Prototypes
void select_clock_signals(void);
void assign_pins_to_uart(void);
void use_9600_baud(void);

// Temperature variabless
unsigned int temp;
volatile float tempDegC;
volatile float tempDegF;
volatile int fCNum, lCNum, fFNum, lFNum, choice = 0;

// Queue Variables
const int SIZE = 10;
int head, tail, element;
int queue[SIZE];

// Number array for LCD
const unsigned char lcd_num[10] = {
    0xFC, // 0
    0x60, // 1
    0xDB, // 2
    0xF3, // 3
    0x67, // 4
    0xB7, // 5
    0xBF, // 6
    0xE4, // 7
    0xFF, // 8
    0xF7, // 9
};

int main(void)
 {
    WDTCTL = WDTPW + WDTHOLD; // Stop WDT
    PM5CTL0 &= ~LOCKLPM5;
    // LCD initialization
    Ini_LCD();

    // Initialization for queue
    init(&head, &tail);

    // UART Functions
    select_clock_signals();
    assign_pins_to_uart();
    use_9600_baud();
    // Initialize the shared reference module
    // By default, REFMSTR=1 => REFCTL is used to configure the internal reference

    UCA1IE = UCTXCPTIE; // Interrupt when TX bit is finished
    UCA1IE = UCRXIE;

    TA0CCR0 = 40000; // Count from 0 to 40,000
    TA0CTL = ACLK | MC_1; // Use ACLK, for UP
    TA0CCTL0 = CCIE; // Enable interrupt for Timer_0

    P1DIR &= ~(BUT1 | BUT2); // Direct pin as input
    P1REN |= (BUT1 | BUT2); /// Enable built-in resistor
    P1OUT |= (BUT1 | BUT2); // Set resistor as pull-up
    P1IE |= (BUT1 | BUT2); // Enable interrupt for buttons
    P1IFG &= ~(BUT1 | BUT2);
    P1IES |= (BUT1 | BUT2); // High to low transition

    while(REFCTL0 & REFGENBUSY); // If ref generator busy, WAIT
    REFCTL0 |= REFVSEL_0 + REFON; // Enable internal 1.2V reference
    /* Initialize ADC12_A */
    ADC12CTL0 &= ~ADC12ENC; // Disable ADC12 so that you could modify the values in registers
    ADC12CTL0 = ADC12SHT0_8 + ADC12ON; // Set sample time
    ADC12CTL1 = ADC12SHP; // Enable sample timer
    ADC12CTL3 = ADC12TCMAP; // Enable internal temperature sensor
    ADC12MCTL0 = ADC12VRSEL_1 + ADC12INCH_30; // ADC input ch A30 => temp sense
    ADC12IER0 = 0x001; // ADC_IFG upon conv result-ADCMEMO
    while(!(REFCTL0 & REFGENRDY)); // Wait for reference generator
    // to settle
    ADC12CTL0 |= ADC12ENC;
    while(1)
    {
        ADC12CTL0 |= ADC12SC; // Sampling and conversion start
        __bis_SR_register(LPM4_bits | GIE); // LPM4 with interrupts enabled
        __no_operation();
        // Temperature in Celsius. See the Device Descriptor Table section in the
        // System Resets, Interrupts, and Operating Modes, System Control Module
        // chapter in the device user's guide for background information on the
        // used formula.
        tempDegC = (float)(((long)temp - CALADC12_12V_30C) * (85 - 30)) /
        (CALADC12_12V_85C - CALADC12_12V_30C) + 30.0f;
        // Temperature in Fahrenheit Tf = (9/5)*Tc + 32
        tempDegF = tempDegC * 9.0f / 5.0f + 32.0f;

        // Separate variables for F and C
        fCNum = firstNum(tempDegC);
        lCNum = lastNum(tempDegC);
        fFNum = firstNum(tempDegF);
        lFNum = lastNum(tempDegF);

        // Button 1 and 2 Interrupt statements
        if(choice == 0) {
            // Fahreinheit
            LCDM8 = 0x8F; // F - fahrenheit
            LCDM15 = lcd_num[lFNum];
            LCDM19 = lcd_num[fFNum];
        } else if(choice == 1) {
            // Celsius
            LCDM8 = 0x9C; // C - celsius
            LCDM15 = lcd_num[lCNum];
            LCDM19 = lcd_num[fCNum];
        }

        LCDCCTL0 |= LCDON; //Turn LCD on
        __no_operation(); // SET BREAKPOINT HERE
    }
}
#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
    if(ADC12IV & ADC12IV_ADC12IFG0)
    {
    // Vector 12: ADC12MEM0 Interrupt
    temp = ADC12MEM0; // Move results, IFG is cleared
    __bic_SR_register_on_exit(LPM4_bits); // Exit active CPU
    }
}
#pragma vector=PORT1_VECTOR
__interrupt void Button_1 (void)
{
    // Fahrenheit
    if((BUT1 & P1IN) == 0) // Is P1 button pushed?
    {
        choice = 0;
    }
    // Celsius
    if((BUT2 & P1IN) == 0) // Is P2 button pushed?
    {
        choice = 1;
    }
    P1IFG &= ~(BUT1 | BUT2);
}
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_ISR(void)
{
    // Queue operations for timer interrupt
    // Adds new value to queue every second
    // If queue is full, removes first value
    // and adds in the next in the rear
    if(full(tail, SIZE) == 0) {
        enqueue(queue, &tail, tempDegC);
    } else if(full(tail, SIZE) == 1) {
        dequeue(queue, &head);
        enqueue(queue, &tail, tempDegC);
    }
}
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI__A1(void)
{
    if(UCA1IFG & UCRXIFG) // Received a new message?
    {
        // Message has to be '0'
        if(UCA1RXBUF == '0')
        {
            // Calculate Average of queue array
            // over past 10 seconds of values
            int i, avg = 0;
            for(i = 0; i < 10; i++) {
                avg += queue[i];
            }
            avg = avg / 10;
            UCA1TXBUF = avg;
            __delay_cycles(20000);
        }
    }
    UCA1IFG = UCA1IFG & (~UCTXCPTIFG);
}
void Ini_LCD()
{
    PJSEL0 = BIT4 | BIT5; // For LFXT
    // Initialize LCD segments 0 - 21; 26 - 43
    LCDCPCTL0 = 0xFFFF;
    LCDCPCTL1 = 0xFC3F;
    LCDCPCTL2 = 0x0FFF;
    // Configure LFXT 32kHz crystal
    CSCTL0_H = CSKEY >> 8; // Unlock CS registers
    CSCTL4 &= ~LFXTOFF; // Enable LFXT
    do
    {
        CSCTL5 &= ~LFXTOFFG; // Clear LFXT fault flag
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG); // Test oscillator fault flag
    CSCTL0_H = 0; // Lock CS registers
    // Initialize LCD_C
    // ACLK, Divider = 1, Pre-divider = 16; 4-pin MUX
    LCDCCTL0 = LCDDIV__1 | LCDPRE__16 | LCD4MUX | LCDLP;
    // VLCD generated internally,
    // V2-V4 generated internally, v5 to ground
    // Set VLCD voltage to 2.60v
    // Enable charge pump and select internal reference for it
    LCDCVCTL = VLCD_1 | VLCDREF_0 | LCDCPEN;
    LCDCCPCTL = LCDCPCLKSYNC; // Clock synchronization enabled
    LCDCMEMCTL = LCDCLRM; // Clear LCD memory
    return;
}
// Select Clock Signals
void select_clock_signals(void)
{
    CSCTL0 = 0xA500; // "Password" to access clock calibration registers
    CSCTL1 = 0x0046; // Specifies frequency of main clock
    CSCTL2 = 0x0133; // Assigns additional clock signals
    CSCTL3 = 0x0000; // Use clocks at intended frequency, do not slow them down
}
// Used to give UART control of approriate pins
void assign_pins_to_uart(void)
{
    P3SEL1 = 0x00; // 0000 0000
    P3SEL0 = BIT4 | BIT5; // 0000 1100
}
// Specify UART baud rate
void use_9600_baud(void)
{
    UCA1CTLW0 = UCSWRST; // Put UART into software reset
    UCA1CTLW0 = UCA1CTLW0 | UART_CLK_SEL; // Specifies clock source for UART
    UCA1BR0 = BR0_FOR_9600; // Specifies bit rate (baud) of 9600
    UCA1BR1 = BR1_FOR_9600; // Specifies bit rate (baud) of 9600
    UCA1MCTLW = CLK_MOD; // "Cleans" clock signal
    UCA1CTLW0 = UCA1CTLW0 & (~UCSWRST); // Takes UART out of software reset
}
int firstNum(int n)
{
    int digits = (int)log10(n);
    n = (int)(n / pow(10, digits));
    return n;
}
int lastNum(int n)
{
    return (n % 10);
}
void init(int *head, int *tail)
{
    *head = *tail = 0;
}
void enqueue(int *q,int *tail, int element)
{
    q[(*tail)++] = element;
}
int dequeue(int *q,int *head)
{
    return q[(*head)++];
}
int full(int tail,const int size)
{
    return tail == size;
}
