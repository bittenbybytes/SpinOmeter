/*
    example minimal CDC serial port adapter using PIC16F1454 microcontroller

    this is specific to the PIC16F1454; TX is on pin RC4 and RX on pin RC5

    based on M-Stack by Alan Ott, Signal 11 Software

    culled from USB CDC-ACM Demo (by Alan Ott, Signal 11 Software)
    and ANSI C12.18 optical interface (by Peter Lawrence)

    Copyright (C) 2014,2015 Peter Lawrence

    Permission is hereby granted, free of charge, to any person obtaining a 
    copy of this software and associated documentation files (the "Software"), 
    to deal in the Software without restriction, including without limitation 
    the rights to use, copy, modify, merge, publish, distribute, sublicense, 
    and/or sell copies of the Software, and to permit persons to whom the 
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in 
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.
*/

#include "usb.h"
#include <xc.h>
#include <string.h>
#include "usb_config.h"
#include "usb_ch9.h"
#include "usb_cdc.h"

/* local data buffers for CDC functionality */
static uint8_t PC2PIC_Buffer[EP_2_LEN];
static uint8_t PIC2PC_Buffer[EP_2_LEN];

/* variables to track positions and occupancy in local CDC buffers */
uint8_t PIC2PC_pending_count;
size_t  PC2PIC_buffer_occupancy;
uint8_t PC2PIC_read_index;


void initSPI();
void transferSPI(unsigned char address,
				 unsigned char* data,
				 unsigned char length, bool write);

static int spin = 0; // current spin
static int maxSpin = 0; // max spin since reset

void initGyro();
void readAgularRate(unsigned short angRate[3]);

void shutdown();

typedef unsigned long timems;
volatile timems millis = 0;
const unsigned short timerReloadValue = 0xFFFF - 1500; 
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
	uint8_t pc2pic_data_ready;
	uint8_t *in_buf;
	const uint8_t *out_buf;
	int i;

	pc2pic_data_ready = 0;

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

    TRISBbits.TRISB7 = 0;
	PORTBbits.RB7 = 1;
    
	PIC2PC_pending_count = 0;
	PC2PIC_buffer_occupancy = 0;

/* Configure interrupts, per architecture */
#ifdef USB_USE_INTERRUPTS
	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;
#endif

	usb_init();

    /*char c = 0xff;
    transferSPI(0x0F+0x80, &c, 1, 0);
    PIC2PC_Buffer[PIC2PC_pending_count] = c;
    ++PIC2PC_pending_count;*/
    
	for (;;)
	{
#ifndef USB_USE_INTERRUPTS
		usb_service();
#endif
        
//        static timems lastBlink = 0;
//        if(millis - lastBlink > 1000) // every 1000ms
//        {
//            lastBlink = millis;
//            PORTBbits.RB7 = !PORTBbits.RB7;
//        }
        
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

        if(millis > 120000 && !usb_is_configured())
            shutdown();//shutdown
        
		/* if USB isn't configured, there is no point in proceeding further */
		if (!usb_is_configured())
			continue;
        
		static unsigned int i = 0;
		if((i++ % 0x800) == 0 && PIC2PC_pending_count == 0)
		{

			unsigned char angRate[8] = {0};

			const char str[] = "Spin: ";
			for (int x = 0; str[x] != 0; x++)
			{
				PIC2PC_Buffer[PIC2PC_pending_count] = str[x];
				++PIC2PC_pending_count;
			}
            
            itoa(angRate, spin, 10);
            
			for (int x = 0; angRate[x] != 0; x++)
			{
				PIC2PC_Buffer[PIC2PC_pending_count] = angRate[x];
				++PIC2PC_pending_count;
			}

            const char str2[] = " rpm, max: ";
			for (int x = 0; str2[x] != 0; x++)
			{
				PIC2PC_Buffer[PIC2PC_pending_count] = str2[x];
				++PIC2PC_pending_count;
			}
            
            itoa(angRate, maxSpin, 10);
            
			for (int x = 0; angRate[x] != 0; x++)
			{
				PIC2PC_Buffer[PIC2PC_pending_count] = angRate[x];
				++PIC2PC_pending_count;
			}
            
			const char str3[] = " rpm (negative -> right flick)\n";
			for (int x = 0; str3[x] != 0; x++)
			{
				PIC2PC_Buffer[PIC2PC_pending_count] = str3[x];
				++PIC2PC_pending_count;
			}
		}

		/* if our PC2PIC buffer is not empty *AND* the USART can accept another byte, transmit another byte */
		if (pc2pic_data_ready)
		{
			/* Check for commands */
            switch(PC2PIC_Buffer[PC2PIC_read_index])
            {
                case 'r':
                    maxSpin = 0;
                    break;
                    
                case 'm':
                    maxSpin = 32767/98; // for limit checking
                    break;
                        
                default:
                    break;
            }
                        
			/* update the read index */
	    		++PC2PIC_read_index;
			/* if the read_index has reached the occupancy value, signal that the buffer is empty again */
	    		if (PC2PIC_read_index == PC2PIC_buffer_occupancy)
	    			pc2pic_data_ready = 0;
		}

		/* proceed further only if the PC can accept more data */
		if (usb_in_endpoint_halted(2) || usb_in_endpoint_busy(2))
			continue;

		/* if we've reached here, the USB stack can accept more; if we have PIC2PC data to send, we hand it over */
		if (PIC2PC_pending_count > 0)
		{
			in_buf = usb_get_in_buffer(2);
			memcpy(in_buf, PIC2PC_Buffer, PIC2PC_pending_count);
			usb_send_in_buffer(2, PIC2PC_pending_count);
			PIC2PC_pending_count = 0;
		}

		/* if our PC2PIC buffer is NOT empty, we won't go further (where we would try to accept more from the PC) */
		if (pc2pic_data_ready)
			continue;

		/* if we pass this test, we are committed to make the usb_arm_out_endpoint() call */
		if (!usb_out_endpoint_has_data(2))
			continue;

		/* ask USB stack for more PC2PIC data */
		PC2PIC_buffer_occupancy = usb_get_out_buffer(2, &out_buf);

		/* if there was any, put it in the buffer and update the state variables */
		if(PC2PIC_buffer_occupancy > 0)
		{
			memcpy(PC2PIC_Buffer, out_buf, PC2PIC_buffer_occupancy);
			/* signal that the buffer is not empty */
			pc2pic_data_ready = 1;
			/* rewind read index to the beginning of the buffer */
			PC2PIC_read_index = 0;
		}

		usb_arm_out_endpoint(2);
	}
}

void interrupt isr()
{
#ifdef USB_USE_INTERRUPTS
    usb_service();
#endif
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
				 unsigned char length, bool write)
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
