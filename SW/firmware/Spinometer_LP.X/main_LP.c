
// PIC16LF1459 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config CPUDIV = CLKDIV6 // CPU System Clock Selection Bit (CPU system clock divided by 6)
#pragma config USBLSCLK = 48MHz // USB Low SPeed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
#pragma config PLLMULT = 3x     // PLL Multipler Selection Bit (3x Output Frequency Selected)
#pragma config PLLEN = DISABLED // PLL Enable Bit (3x or 4x PLL Disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <string.h>


void initSPI();
void transferSPI(unsigned char address,
				 unsigned char* data,
				 unsigned char length, char write);

static int spin = 0; // current spin
static int maxSpin = 0; // max spin since reset

void initGyro();
void readAgularRate(unsigned short angRate[3]);

void shutdown();

typedef unsigned long timems;
volatile timems millis = 0;
const unsigned short timerReloadValue = 0xFFFF - 125;
void initMilisTimer()
{
	// configure timer to generate an interrupt each millisecond
	T1CON = 0x30; // Clock source is Fosc/4, 1:8 prescale --> 1500kHz
	
	// load timer
	TMR1H = (unsigned char)(timerReloadValue >> 8);
	TMR1L = (unsigned char)(timerReloadValue & 0xFF);

	// clear timer interrupt flag
	PIR1bits.TMR1IF = 0;

	// enable interrupt
	PIE1bits.TMR1IE = 1;

	// start timer
	T1CONbits.TMR1ON = 1;

}

inline void millisISR()
{
	// Timer1 interrupt flag set?
	if (PIR1bits.TMR1IF)
	{
		// reload timer
		TMR1H = (unsigned char)(timerReloadValue >> 8);
		TMR1L = (unsigned char)(timerReloadValue & 0xFF);

		// increment milliseconds
		millis++;
		// clear timer interrupt flag
		PIR1bits.TMR1IF = 0;
	}
}

#define abs(x) (x<0?-x:x)

int main(void)
{
	OSCCONbits.IRCF = 0xD;

	ANSELC = 0;
	TRISC = 0x01;
	PORTC = 0X00;

	TRISBbits.TRISB7 = 0;
	PORTBbits.RB7 = 1;
	
	//InitializeUSART();
	initMilisTimer();
	INTCONbits.GIE = 1;
	INTCONbits.PEIE = 1;
	initGyro();
	while(millis < 100);
	
	for (;;)
	{
        
        static int spin = 0; // current spin
        static int maxSpin = 0; // max spin since reset
        
        static timems lastAngRateUpdate = 0;
        if(millis - lastAngRateUpdate > 100) // every 100ms
        {
            lastAngRateUpdate = millis;
            
            unsigned char angRate[8] = {0};

			readAgularRate((unsigned short*)(angRate));

			spin = *(short*)(&angRate[4]) / 98; // /16 -> dps, /98 -> rpm, /5898 -> rps
            
            if(abs(spin) > abs(maxSpin))
            {
                maxSpin = spin;
                
                unsigned char rps = (abs(maxSpin) + 30) / 60; // add half a rps for rounding
                
                PORTC = 0xff >> (6 - rps);
                
                PORTBbits.RB7 = (rps > 5) ? 1:0;
            } 
        }

        if(millis > 120000 )
            shutdown();//shutdown
	}
}

void interrupt isr()
{
	millisISR();
}

#define CS LATBbits.LATB5
#define CS_TRIS TRISBbits.TRISB5

#define SCL LATBbits.LATB6
#define SCL_TRIS TRISBbits.TRISB6

#define SDO LATCbits.LATC7
#define SDO_TRIS TRISCbits.TRISC7

#define SDI PORTBbits.RB4
#define SDI_TRIS TRISBbits.TRISB4

void initSPI()
{
	ANSELB=0;

	// Chip select
	CS_TRIS = 0;
	CS = 1;
	
	// clock
	SCL_TRIS = 0;

	// data out
	SDO_TRIS = 0;
	
	// data in
	SDI_TRIS = 1;

	    // SSPSTAT Register
    SSP1STATbits.CKE = 0;//1;       // Transmit occurs on transition from active to idle clock state
    SSP1STATbits.SMP = 1;       // Input data sampled at end of data output time


    // SSP1CON1 Register
    SSPCON1bits.SSP1M0 = 0;     // SPI Master Mode - 0000 Clock = Fosc/4
    SSPCON1bits.SSP1M1 = 1;     // SPI Master Mode - 0001 Clock = Fosc/16
    SSPCON1bits.SSP1M2 = 0;     // SPI Master Mode - 0010 Clock = Fosc/64 <- selected
    SSPCON1bits.SSP1M3 = 0;     // SPI Master Mode - 0011 Clock = TMR2/2
    SSPCON1bits.CKP   = 1;     // Idle clock state is high
    SSPCON1bits.SSPEN = 1;     // Enable S-Port & Config PINS

}

void transferSPI(unsigned char address,
				 unsigned char* data,
				 unsigned char length, char write)
{
	CS = 0;
	unsigned char temp;            // received bytes
	temp = SSPBUF;                  // clear BF

	SSPBUF = address;
    while(!SSP1STATbits.BF);
	temp = SSPBUF;

	for(int i = 0; i < length; i++)
	{
		if(write)
		{
			SSPBUF = data[i];
			while(!SSP1STATbits.BF);
			temp = SSPBUF;
		}
		else
		{
			
			SSPBUF = 0;
			while(!SSP1STATbits.BF);
			data[i] = SSPBUF;
		}
	}

	CS = 1;
}

#define WHO_AM_I 0x0f
#define CTRL1 0x20
#define CTRL4 0x23
#define OUT_X_L 0x28

void initGyro()
{
	initSPI();

	unsigned char data = 0x09; // enable z axis only
	transferSPI(CTRL1, &data, 1, 1);
	
	data = 0x20; // FS range: 0x00 = 200dps, 0x10 = 500dps, 0x20 = 2000dps
	transferSPI(CTRL4, &data, 1, 1);
}

void readAgularRate(unsigned short angRate[3])
{ 
	// read 6 bytes with auto increment
	transferSPI(OUT_X_L+0x80+0x40,(unsigned char*)(angRate), 6, 0);

}

void shutdown()
{
    // stop timer
	T1CONbits.TMR1ON = 0;
    
    // power down gyro
	unsigned char data = 0x00;
	transferSPI(CTRL1, &data, 1, 1);
    
    // switch off leds
    PORTB = 0;
    PORTC = 0;
    
    // interrupt on negedge oft RA3 (MCLR) for wake up from sleep
    ANSELA = 0;
    IOCANbits.IOCAN3 = 1;
    IOCAF = 0;
    IOCBF = 0;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 0; // to avoid branching to ISR after wake up
    INTCONbits.IOCIF = 0;
    INTCONbits.IOCIE = 1;
  
    asm("SLEEP"); // till faling edge on RA3
    
    INTCONbits.IOCIF = 0;
    INTCONbits.IOCIE = 0;
    
    maxSpin = 0;
    spin = 0;
    
    initGyro();
    
    initMilisTimer();
}