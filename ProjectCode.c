/********************************************************************************
 Written by: Vijeth Hebbar, IIT Bombay  
 AVR Studio Version 4.0

 Date: 10st October 2018
 
 Application example: Robot communication with Ras-Pi 3B+ uver USB RS232 Serial

 
 Serial Port used: UART2

 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA3
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B.

Connection Details:  	
 						
  Motion control:		L-1---->PA0;		L-2---->PA1;
   						R-1---->PA2;		R-2---->PA3;
   						PL3 (OC5A) ----> Logic 1; 	PL4 (OC5B) ----> Logic 1; 


  Serial Communication:	PORTH 0 --> RXD2 UART 2 receive for USB - RS232 communication
						PORTH 1 --> TXD2 UART 2 transmit for USB - RS232 communication
	

 In this code, we are using 100ms timer interrupt to keep track of present encoder counts of robot. 
 After every 100ms timer ISR will invoke. Left and right encoders are configured first.

Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0  (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)

 2. Difference between the codes for RS232 serial, USB and wireless communication is only in the serial port number.
 	Rest of the things are the same. 

 3. For USB communication check the Jumper 1 position on the ATMEGA2560 microcontroller adaptor board
 
 4. Auxiliary power can supply current up to 1 Ampere while Battery can supply current up to 
 	2 Ampere. When both motors of the robot changes direction suddenly without stopping, 
	it produces large current surge. When robot is powered by Auxiliary power which can supply
	only 1 Ampere of current, sudden direction change in both the motors will cause current 
	surge which can reset the microcontroller because of sudden fall in voltage. 
	It is a good practice to stop the motors for at least 0.5seconds before changing 
	the direction. This will also increase the useable time of the fully charged battery.
	the life of the motor.

 **********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#include <math.h>
#include  <stdlib.h>
#define pi 3.1415926535897932384626433832795
unsigned char FBR_Flag=0;   					//Keeps track of whether a wheel is moving forward of back. 1 for forward.
unsigned char FBL_Flag=0;
long int ShaftCountLeft = 0;														//to keep track of left position encoder 
long int ShaftCountLeftPrev = 0;													//to keep track of left position encoder
long int ShaftCountRight = 0;														//to keep track of right position encoder
long int ShaftCountRightPrev = 0;													//to keep track of right position encoder

unsigned char data;																//to store received data from UDR1
unsigned char incomingByte;	
int packet_cnt=0,packet_len=4;
char d[4]; 
void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;															//Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18;															//PL3 and PL4 pins are for velocity control using PWM.
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	 DDRE  = DDRE & 0xEF;															//Set the direction of the PORTE 4 pin as input
	 PORTE = PORTE | 0x10;															//Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	 DDRE  = DDRE & 0xDF;															//Set the direction of the PORTE 4 pin as input
	 PORTE = PORTE | 0x20;															//Enable internal pull-up for PORTE 4 pin
}

//Function to initialize ports
void port_init()
{
	motion_pin_config();
	left_encoder_pin_config();														//left encoder pin config
    right_encoder_pin_config();													//right encoder pin config	

}

// Function to enable Interrupt 4
void left_position_encoder_interrupt_init (void) 
{
	 cli();																			// Clears the global interrupt
	 EICRB = EICRB | 0x02;															// INT4 is set to trigger with falling edge
	 EIMSK = EIMSK | 0x10;															// Enable Interrupt INT4 for left position encoder
	 sei();																			// Enables the global interrupt 
}

// Function to enable Interrupt 5
void right_position_encoder_interrupt_init (void) 
{
	 cli();																			// Clears the global interrupt
	 EICRB = EICRB | 0x08;															// INT5 is set to trigger with falling edge
	 EIMSK = EIMSK | 0x20;															// Enable Interrupt INT5 for right position encoder
	 sei();																			// Enables the global interrupt 
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 																						For Overriding normal port functionality to OCRnA outputs.
				  																		{WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}


//Function To Initialize UART2
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart2_init(void)
{
 UCSR2B = 0x00;																	//disable while setting baud rate
 UCSR2A = 0x00;
 UCSR2C = 0x06;
 UBRR2L = 0x5F;																	//set baud rate lo
 UBRR2H = 0x00;																	//set baud rate hi
 UCSR2B = 0x98;
}


ISR(USART2_RX_vect)																// ISR for receive complete interrupt
{	/*********************************************************************************
	We will take four bytes of data at a time ,t,he first is an escape character 0x7E.
	This is followed by a direction character 8,6,2,4,5 for F,R,L,B,Stop respectively.
	Final two bytes indicating the PWM input to each motor.
	*********************************************************************************/
	incomingByte = UDR2; 
	d[packet_cnt]=incomingByte;
	//UDR2=incomingByte;
	packet_cnt++;
	if (d[0]!='A')
	packet_cnt=0; 
	//UDR2=packet_cnt;
	if( packet_cnt>=packet_len && d[0]=='A')
	{
		packet_cnt=0;	
		velocity((int)d[2],(int)d[3]);													  
		if(d[1] == 0x38)														//ASCII value of 8
		{
			PORTA=0x06;															//forward
			//Both wheels move forward
			FBL_Flag=1;
			FBR_Flag=1;
		}

		if(d[1] == 0x32)														//ASCII value of 2
		{
			PORTA=0x09;															//back
			//Both wheels move back.
			FBL_Flag=0;
			FBR_Flag=0;
		}

		if(d[1] == 0x34)														//ASCII value of 4
		{
			PORTA=0x05;															//left
			//Right wheel should move with forward and LEft backward for perfect left turn.
			FBL_Flag=0;
			FBR_Flag=1;
		}

		if(d[1] == 0x36)														//ASCII value of 6
		{
			PORTA=0x0A;															//right
			//Left wheel should move with forward and right backward for perfect right turn..
			FBL_Flag=1;
			FBR_Flag=0;
		}

		if(d[1] == 0x35)														//ASCII value of 5
		{
			PORTA=0x00;															//stop
		}
	}

}
//Interrupt which runs every 100 ms. We use this to send data of the current encoder position.

void timer4_init(void)
{
 TCCR4B = 0x00; //stop
 TCNT4H = 0xF7; //247 //Counter higher 8 bit value
 TCNT4L = 0x00; //0+247*256 = 63232 to reach 65535 we need 2303 counts.
 OCR4AH = 0x00; //Output Compair Register (OCR)- Not used Since TIMSK=0x01 we only use the overflow counter.
 OCR4AL = 0x00; //Output Compair Register (OCR)- Not used
 OCR4BH = 0x00; //Output Compair Register (OCR)- Not used
 OCR4BL = 0x00; //Output Compair Register (OCR)- Not used
 OCR4CH = 0x00; //Output Compair Register (OCR)- Not used
 OCR4CL = 0x00; //Output Compair Register (OCR)- Not used
 ICR4H  = 0x00; //Input Capture Register (ICR)- Not used
 ICR4L  = 0x00; //Input Capture Register (ICR)- Not used
 TCCR4A = 0x00; //This ensures normal mode as WGM0,1 are both 0. In addition WGM2 in in TCCR4B (in the 4th bit from right) is also 0
 TCCR4C = 0x00; //Just set to zero for now.
 TCCR4B = 0x04; //start Timer 0x04. Prescaler is 64 for CS0:2 set to 011. 14745600/64=230400  
}


//Function To Initialize all The Devices
void init_devices()
{
 cli();																			//Clears the global interrupts
 port_init();																	//Initializes all the ports
 uart2_init();
 timer5_init();
 timer4_init();
 TIMSK4 = 0x01;    //Enables the overflow interrupt.
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 																				//Initailize UART1 for serial communiaction
 sei();																			//Enables the global interrupts
}

//If this doesn't work then an interrupt can also be used. Refer page 106 of Software Manual
void USART_Transmit( unsigned char data )										
{
/* Wait for empty transmit buffer*/
while( !( UCSR2A & (1<<UDRE2)) )
;
/* Put data into buffer, sends the data*/
UDR2 = data;
}


ISR(TIMER4_OVF_vect)
{
char chksum;
TCCR4B = 0x00;			//Stops clock.
/*Apparently gives 0.2s gap or 5Hz. Should have been 20Hz */
//TCNT4H = 0xD2; 		//210 
//TCNT4L = 0xFF; 		//255+210*256 = 54015 to reach 65535 we need 11520 counts.

/*Apparently gives 0.04s gap 25Hz. Should have been 100Hz*/
TCNT4H = 0xF7; 			//247 
TCNT4L = 0x00; 			//0+247*256 = 63232 to reach 65535 we need 2303 counts.

/*Apparently gives 0.01s gap. But random stuff creep in so we will keep 25Hz*/
//TCNT4H = 0xFD; 			//253 
//TCNT4L = 0xBF; 			//191+253*256 = 64959 to reach 65535 we need 577 counts.

TCCR4B =  0x04;			// Restarts clock with 64 prescaler

//Here we just need to send the encoder positions. 

USART_Transmit('A');					//'A' denotes the start of the sequence of data to be sent.
USART_Transmit(ShaftCountRight/256);	//Sending the 4 bytes of encoder data.
USART_Transmit(ShaftCountRight%256);
USART_Transmit(ShaftCountLeft/256);
USART_Transmit(ShaftCountLeft%256);

chksum=ShaftCountRight/256+ShaftCountRight%256+ShaftCountLeft/256+ShaftCountLeft%256; 
USART_Transmit(chksum);					//Send the calculated checksum for comparison and accuracy check.
}

/************************************************************************************************* 
The interrupt above will run start at 54015 and go upto 65535 which is 11520 counts.
Further the overflow interrupt on going from 65535 to 0. So 11521 counts before interrupts. 
counts of system clock 11521*64 = 737364. 0.05000569 is the time between steps.

To get a time step of 0.01 secs we need to get a count of 2304 or 1/5 of 11520. 65535-2304 = 63231.
We add one for the step to 0. 63232 should be the start coount. In hex this is 
**************************************************************************************************/


//ISR for right position encoder
ISR(INT5_vect)  
{
	if(FBR_Flag == 1)
	ShaftCountRight++;																//increment right shaft position count for forward motion
	else
	ShaftCountRight--;
}

//ISR for left position encoder
ISR(INT4_vect)
{
	if(FBL_Flag == 1)
	ShaftCountLeft++;																//increment left shaft position count for forward motion
	else
	ShaftCountLeft--;																//decrement left shaft position count for backward motion			
}

//Main Function
int main(void)
{
	init_devices();
	while(1)
	{	
		//USART_Transmit(23);
		//unsigned long measurement=256;
		//USART_Transmit(measurement/256); //Encoder measurements are 2-bytes long and require 2 bytes to be sent. 
		//USART_Transmit(measurement%256);		
	}
}




