//#include "msp430g2211.h"
//#include "stdint.h"
//unsigned int buffer[33];
//unsigned char i=0;
//void main(void)
//{
//  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
//  if (CALBC1_1MHZ ==0xFF || CALDCO_1MHZ == 0xFF)
//  {
//    while(1);                               // If calibration constants erased
//                                            // do not load, trap CPU!!
//  }
//  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO to 1MHz
//  DCOCTL = CALDCO_1MHZ;
//
//
//  BCSCTL1 &= ~XTS;								// LFXTCLK 0:Low Freq.
//  BCSCTL1 |= DIVA_3;
//  BCSCTL3 &= ~(LFXT1S0 + LFXT1S1);				// Watch crystal mode
//  BCSCTL3 |= XCAP_3;                              // XIN/XOUT Cap : 12.5 pF */
//
//  TACTL = TASSEL_2 + MC_2;                  // SMCLK, contmode
//  TACCTL0 = CM_1+CCIS_1+CAP+CCIE;
//  _BIS_SR(LPM0_bits + GIE);                 // Enter LPM0 w/ interrupt
//}
//
//// Timer A0 interrupt service routine
//#pragma vector=TIMERA0_VECTOR
//__interrupt void TIMERA0 (void)
//{
//	static uint16_t Compare, Oldcapture = 0;
//	Compare = TACCR0;                       // Get current captured SMCLK
//	Compare = Compare - Oldcapture;         // SMCLK difference
//    Oldcapture = TACCR0;
//    buffer[i++] = Compare;
//    if (i>32)
//      __no_operation();                       // PLACE BREAKPOINT HERE
//                                            // VIEW 'BUFFER' VARIABLE
//
//}


//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
#include "msp430g2211.h"
#include "stdint.h"

//------------------------------------------------------------------------------
// Hardware-related definitions
//------------------------------------------------------------------------------
#define UART_TXD     BIT2
#define F_in         BIT0
//------------------------------------------------------------------------------
// Conditions for 9600 Baud SW UART, SMCLK = 1MHz
//------------------------------------------------------------------------------
#define UART_TBIT           (1000000 / 9600)

//------------------------------------------------------------------------------
// Global variables used for full-duplex UART communication
//------------------------------------------------------------------------------
uint16_t txData; // UART internal variable for TX
int8_t freqstr[11];
volatile uint32_t freq;

//------------------------------------------------------------------------------
// Function prototypes
//------------------------------------------------------------------------------
void TimerA_UART_init(void);
void CPU_init(void);
void TimerA_UART_tx(uint8_t byte);
void TimerA_UART_print(int8_t *string);
void ltos(uint32_t val, int8_t *str);
void measureFrequency(void);

//------------------------------------------------------------------------------
// main()
//------------------------------------------------------------------------------

void main(void) {
	CPU_init();
	_enable_interrupts();

	while (1) {

		measureFrequency();
		//freq=123456;
		ltos(freq, freqstr);

		TimerA_UART_init(); // Setup Timer_A as UART TX
		TimerA_UART_print("\r\nfreq=");
		TimerA_UART_print(freqstr);
		while (TACCTL1 & CCIE); // Ensure last char got TX'd

	}
}
void measureFrequency(void) {
	//setup TimerA  clock signal TACLK input
	//setup TimerA CC
//    The capture inputs CCIxA and CCIxB are connected to
//	external pins or internal signals and are selected with the CCISx bits. The CMx bits select the capture
//	edge of the input signal as rising, falling, or both. A capture occurs on the selected edge of the input
//	signal. If a capture occurs:
//	• The timer value is copied into the TACCRx register
//	• The interrupt flag CCIFG is set

	freq = 0;


	BCSCTL1 |= RSEL0 + RSEL1 + RSEL2 + RSEL3;
	DCOCTL |= 0xE0;
	BCSCTL1 |= DIVA_3;
	TACTL = TASSEL_2 + MC_2; // SMCLK, cont-mode, clear
	TACCTL0 = CM_1 + CCIS_1 + CAP + CCIE; // CAP, ACLK

	//TACTL = TASSEL_2 + MC_2;    //for debug
	//TACTL = TASSEL_0 + MC_2;  // Timer_A clock source select: 00 = TACLK,
	// Mode control: 10 = Continuous mode: the timer counts up to 0FFFFh.
	//TACCTL0 = CM0 + CCIS0 + CAP + CCIE;  //Capture on rising edge,
	// Capture/compare input select: 01 CCIxB: ACLK (internal)
	// Capture mode: 1
	// Capture/compare interrupt enable.
	volatile uint16_t i;

	for (i = 0; i <= 512; i++)
		LPM0; // Count frequency for 32768/(8*64*64) = 1 seconds
	TACCTL0 &= ~CCIE; // Disable further TACCTL0 interrupts
}

//------------------------------------------------------------------------------
// TIMER0_A0_VECTOR Frequency measurement
//------------------------------------------------------------------------------
#define NCAPTURES 8
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void) {
	static uint16_t Captures = 0; // Number of captures accumulated
	static uint16_t StartValue; // Time at start of sampling
	switch (Captures) {
	case 0: // Starting new sequence of captures
		StartValue = TACCR0; // Starting time
		Captures = NCAPTURES; // Initialize down counter
		break;
	case 1: // Final capture of sequence
		Captures = 0; // Finished
		freq += TACCR0 - StartValue; // Display result
		LPM0_EXIT;

		break;
	default: // Sequence of captures continues
		--Captures;
		break;
	}
}

//------------------------------------------------------------------------------
// Function configures CPU
//------------------------------------------------------------------------------
void CPU_init(void) {
	WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer

	// Set DCOCLK to 1MHz
	if (CALBC1_1MHZ == 0xFF || CALDCO_1MHZ == 0xFF) {
		_bis_SR_register(LPM4_bits);
	}

	BCSCTL1 = CALBC1_1MHZ;
	DCOCTL = CALDCO_1MHZ;

	BCSCTL1 &= ~XTS; // LFXTCLK 0:Low Freq.
	BCSCTL1 |= DIVA_3;
	BCSCTL3 &= ~(LFXT1S0 + LFXT1S1); // Watch crystal mode
	BCSCTL3 |= XCAP_3; // XIN/XOUT Cap : 12.5 pF */

	P1OUT = 0x00; // Initialize all GPIO
	P1SEL = UART_TXD; // Timer function for TXD pins
	P1DIR = 0xFF; // Set pins to output
	P1DIR &= ~F_in; // except F_in
	P1REN |= F_in;
	P1SEL |= F_in;
//  P2OUT = 0x00;
//  P2SEL = 0xC0;
//  P2DIR = 0xFF;
}
//------------------------------------------------------------------------------
// Function configures Timer_A for full-duplex UART operation
//------------------------------------------------------------------------------
void TimerA_UART_init(void) {

	BCSCTL1 = CALBC1_1MHZ;
	DCOCTL = CALDCO_1MHZ;

	TACCTL1 = OUT; // Set TXD Idle as Mark = '1'
	TACTL = TASSEL_2 + MC_2; // SMCLK, start in continuous mode
}
//------------------------------------------------------------------------------
// Outputs one byte using the Timer_A UART
//------------------------------------------------------------------------------
void TimerA_UART_tx(unsigned char byte) {
	while (TACCTL1 & CCIE)
		; // Ensure last char got TX'd
	TACCR1 = TAR; // Current state of TA counter
	TACCR1 += UART_TBIT; // One bit time till first bit
	TACCTL1 = OUTMOD0 + CCIE; // Set TXD on EQU0, Int
	txData = byte; // Load global variable
	txData |= 0x100; // Add mark stop bit to TXData
	txData <<= 1; // Add space start bit
}

//------------------------------------------------------------------------------
// Prints a string over using the Timer_A UART
//------------------------------------------------------------------------------
void TimerA_UART_print(int8_t *string) {
	while (*string) {
		TimerA_UART_tx(*string++);
	}
}
//------------------------------------------------------------------------------
// Timer_A UART - Transmit Interrupt Handler
//------------------------------------------------------------------------------
#pragma vector = TIMERA1_VECTOR
__interrupt void Timer_A1_ISR(void) {
	static unsigned char txBitCnt = 10;
	switch (__even_in_range(TAIV, TAIV_TAIFG)) {
	case TAIV_TACCR1:
		TACCR1 += UART_TBIT; // Add Offset to CCRx
		if (txBitCnt == 0) { // All bits TXed?
			TACCTL1 &= ~CCIE; // All bits TXed, disable interrupt
			txBitCnt = 10; // Re-load bit counter
		} else {
			if (txData & 0x01) {
				TACCTL1 &= ~OUTMOD2; // TX Mark '1'
			} else {
				TACCTL1 |= OUTMOD2; // TX Space '0'
			}
			txData >>= 1;
			txBitCnt--;
		}
		break;
	}
}
//------------------------------------------------------------------------------
// ltos - Long to String
//------------------------------------------------------------------------------
inline void ltos(uint32_t val, int8_t *str) {
	uint32_t temploc = 0;
	uint32_t digit = 0;
	uint32_t strloc = 0;
	int8_t tempstr[10]; //32-bit number can be at most 10 ASCII digits;

	do {
		digit = val % 10;
		tempstr[temploc++] = digit + '0';
		val /= 10;
	} while (val > 0);
	// reverse the digits back into the output string
	while (temploc > 0)
		str[strloc++] = tempstr[--temploc];
	str[strloc] = 0;
}



////******************************************************************************
////   main.c MSP430G2553 Frequency counter with UART output
////
////   Description: Measures DCO, ACLK and P1.0 frequency up to 16MHz
////   measurements are sent over UART at 9600 baud
////
////                MSP430G2553
////             -----------------
////         /|\|              XIN|-
////          | |                 |	32.768kHz
////          --|RST          XOUT|-
////            |                 |
////            |                 |external clock input
////            |     P1.0/TA0CLK |<-----------
////            |                 |
////            |     P1.2/UCA0TXD|------------>
////            |                 | 9600 - 8N1
////            |     |
//
//
///*
// *
// * main.c
// */
//#include <stdint.h>
//#include <msp430g2211.h>
//
//volatile uint32_t mfreq;
//
//inline void MCUinit(){
//
//			WDTCTL = WDTPW + WDTHOLD;      //stop WDT
//
//			// If cal constants erased, trap CPU!!
//			// we will need that 1MHZ calibration
//			if (CALBC1_1MHZ ==0xFF || CALDCO_1MHZ == 0xFF){
//	          _bis_SR_register(LPM4_bits);
//			}
//
//			//setup ports
//
//			P1SEL |= BIT0;                              // Use P1.0 as TimerA input
//			P1SEL2 &= ~BIT0;                            //
//			P1DIR &= ~BIT0;                             //
//			P1OUT &= ~BIT0;                             // Enable pull down resistor to reduce stray counts
//			P1REN |= BIT0;                              //
//
//			//todo P2DIR = 0xFF;?!?!?!?!??!?!?!!???
//			P2DIR = 0xFF;                              // All P2.x outputs
//			P2OUT = 0;                                 // All P2.x reset
//
//			//todo ACLK #define DIVA1                  (0x20)         /* ACLK Divider 1 */
//			BCSCTL1 &= ~XTS;
//			BCSCTL3 |= LFXT1S_0 + XCAP_3;   // clock system setup, 12.5pF cap- setting for 32768Hz crystal
//			//SugarAddict: 	BCSCTL1 &= ~XTS;								// External source is LF
//			//[17:12]	SugarAddict: 	BCSCTL3 &= ~(LFXT1S0 + LFXT1S1);				// Watch crystal mode
//			//[17:12]	SugarAddict: 	BCSCTL3 |= XCAP0 + XCAP1;
//		}
//
//
////base 10 long to ASCII conversion method, works for unsigned long, can cast integers to it as well
//inline void ltos(unsigned long val, uint8_t *str)
//{
//  long temploc = 0;
//  long digit = 0;
//  long strloc = 0;
//  char tempstr[10]; 							//32-bit number can be at most 10 ASCII digits;
//
//  do{
//	digit = val % 10;
//    tempstr[temploc++] = digit + '0';
//    val /= 10;
//  } while (val > 0);
//  	  	  	  	  	  	  	  	  	  	  	  	// reverse the digits back into the output string
//  while(temploc>0)
//  str[strloc++] = tempstr[--temploc];
//  str[strloc]=0;
//}
//
//inline void UARTsend(uint8_t *buffer){}
//inline void LCDinit(){}
//inline void LCDsend(uint8_t *buffer){
//	//send buffer
//}
//uint32_t measuref(void){
//	//setup DCO clock overclock
//	//configure TimerA
//	CCTL0 = CCIE;
//	TACTL = TASSEL_0;
//	_bis_SR_register(LPM3_bits + GIE);
//	return mfreq;
//}
//
//
//#define NCAPTURES 4096
//#pragma vector=TIMER0_A0_VECTOR
//__interrupt void Timer_A (void)
//{
//	static uint16_t Captures = 0; // Number of captures accumulated
//	static uint16_t StartTime; // Time at start of sampling
//	if (TAIV == TAIV_CCIFG2) { // CCIFG2 vector (4)
//	switch (Captures) {
//	case 0: // Starting new sequence of captures
//	StartTime = TACCR2; // Starting time
//	Captures = NCAPTURES; // Initialize down counter
//	break;
//	case 1: // Final capture of sequence
//	DisplayUint (TACCR2 - StartTime); // Display result
//	Captures = 0; // Finished
//	TACCTL2_bit.CCIE = 0; // Disable further interrupts
//	break;
//	default: // Sequence of captures continues
//	--Captures;
//	break;
//	}
//	}
//
//
//}
//
//
//
//void main(void) {
//	MCUinit();
//	//LCDinit();
//
//	uint32_t freq;
//	uint8_t buffer[12];
//
//	dragons:
//	  freq=measuref();
//	  ltos(freq,buffer);
//	  //setup DCO to 1MHz
//	  UARTsend(*buffer);
//	  LCDsend(*buffer);
//	goto dragons;
//}
//
//
