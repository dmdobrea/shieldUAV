#include <msp430.h>				
#include <stdlib.h>


unsigned char lastByte = 0x0;
int readNoQuestion = 0;
unsigned short int PWM1, PWM2, LED1, LED2, LED3;

void main(void)
{
	WDTCTL = WDTPW | WDTHOLD;		// stop watchdog timer

	  if (CALBC1_1MHZ==0xFF)                 // If calibration constant erased
	      {
	      while(1);                               // do not load, trap CPU!!
	      }
	  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
	  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
	  DCOCTL = CALDCO_1MHZ;


    // ------ PWM-uri
    P2DIR  |=  0x04 |  0x10;                    // P2.2 and P2.4 and (P2.1, P2.3, P2.5 as LEDs) output
    P2SEL  |=  0x04 |  0x10;                    // P2.2 and P2.4 options select
    P2SEL2 &= !0x04 & !0x10;

    TA1CCTL1 |= OUTMOD_7;                       // CCR1 toggle/set
    TA1CCTL2 |= OUTMOD_7;                       // CCR1 toggle/set

    TA1CCR0  = 20000;                           // PWM Period/2
    TA1CCR1  = 2000;                            // CCR1 PWM duty cycle
    TA1CCR2  = 1000;                            // CCR1 PWM duty cycle
    TA1CTL = TASSEL_2 + MC_1;                   // MC_0 - stop, MC_1 - up, MC_2 - cont, MC_3 - up/down

    // ------ Serial port config
    P1SEL  |= BIT1 | BIT2;                      // P1.1 = RXD, P1.2=TXD
    P1SEL2 |= BIT1 | BIT2;                     // P1.4 = SMCLK, others GPIO

    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 8;                              // 1MHz 115200
    UCA0BR1 = 0;                              // 1MHz 115200
    UCA0MCTL = UCBRS2 + UCBRS0;               // Modulation UCBRSx = 5
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt

    // ------ LED pins config as output
    P2DIR |= BIT1 | BIT3 | BIT5;


	// ------ P1.6 and P1.7 pins config as input for buttons
	P1IE |=  BIT6;                            // P1.6 interrupt enabled
    P1IE |=  BIT7;                            // P1.7 interrupt enabled
	P1IES |= BIT6;                            // P1.6 Hi/lo edge
    P1IES |= BIT7;                            // P1.7 Hi/lo edge
	P1REN |= BIT6;                            // Enable Pull Up on SW2 (P1.6)
    P1REN |= BIT7;                            // Enable Pull Up on SW2 (P1.7)
	P1IFG &= ~BIT6;                           // P1.6 IFG cleared
    P1IFG &= ~BIT7;                           // P1.7 IFG cleared

    __bis_SR_register(GIE);


    unsigned int i = 0;                       //used for timing in the while(1) loop
	while(1)
	{
	    if(i % 100000 == 0)
	    {
            switch(LED1)
            {
                case 0x0: //off
                    P2OUT &= ~BIT1;      //0xFD
                    break;
                case 0x01: //on
                    P2OUT |= BIT1;
                    break;
                case 0x02: //blink
                    P2OUT ^= BIT1;
                    break;
            }

            switch(LED2)
            {
                case 0x0: //off
                    P2OUT &= ~BIT3;      //0xF7
                    break;
                case 0x01: //on
                    P2OUT |= BIT3;
                    break;
                case 0x02: //blink
                    P2OUT ^= BIT3;
                    break;
            }

            switch(LED3)
            {
                case 0x0: //off
                    P2OUT &= ~BIT5;      //0xDF
                    break;
                case 0x01: //on
                    P2OUT |= BIT5;
                    break;
                case 0x02: //blink
                    P2OUT ^= BIT5;
                    break;
            }
	    }

        i++;
        if(i == 4294900000)
            i = 0;
	}
}

// Echo back RXed character, confirm TX buffer is ready first
#pragma vector=USCIAB0RX_VECTOR

__interrupt void USCI0RX_ISR(void)
{
    unsigned char recCh;

    recCh = UCA0RXBUF;

    if (readNoQuestion == 0)
    {
        if(lastByte == 0x55 && recCh == 0xAA)
        {
            lastByte = 0x0;
            readNoQuestion = 5;
        }
        else
        {
            lastByte = recCh;
        }
    }
    else
    {
        switch(readNoQuestion)
        {
        case 5:
            PWM1 = ((short unsigned int)recCh) << 8;
            break;
        case 4:
            PWM1 |= recCh;
            break;
        case 3:
            PWM2 = (((short unsigned int)recCh) << 8);
            break;
        case 2:
            PWM2 |= recCh;
            break;
        case 1:
            LED1 = (recCh & 0xC0)  >> 6;
            LED2 = (recCh & 0x30)  >> 4;
            LED3 = (recCh & 0x0C)  >> 2;
            TA1CCR1  = PWM1;
            TA1CCR2  = PWM2;
            break;
        }
        readNoQuestion--;
    }
  /*
  if (i%2 == 0)
      {
      i++;
      b = UCA0RXBUF;
      unsigned short int cc = (((short unsigned int) a) << 8) + b;
      TA1CCR2 = cc;
      }
  else
      {
      i++;
      a = UCA0RXBUF;
      }
*/

  //UCA0TXBUF = UCA0RXBUF;                    // TX -> RXed character
}

/*interrupt for P1.X used for
 *      -P1.6 button
 *      -P1.7 button
 */
#pragma vector=PORT1_VECTOR

__interrupt void Port_1(void)
{
    /*
    P2OUT ^= BIT1;
    //P1IFG &= ~BIT6;
    //P1IFG &= ~BIT7;
    return;
    */
    while (!(IFG2&UCA0TXIFG));
    UCA0TXBUF = 0x55;
    while (!(IFG2&UCA0TXIFG));
    UCA0TXBUF = 0xAA;
    //check if the interrupt was caused by the P1.6 button
    unsigned short int toSend = 0;
    if(P1IFG & BIT6)
    {
        P1IFG &= ~BIT6;
        toSend |= 0xF0;
    }
    //check if the interrupt was caused by the P1.7 button
    if(P1IFG & BIT7)
    {
        P1IFG &= ~BIT7;
        toSend |= 0xF;
    }
    while (!(IFG2&UCA0TXIFG));
    UCA0TXBUF = toSend;
}
