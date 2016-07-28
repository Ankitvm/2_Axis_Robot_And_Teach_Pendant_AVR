/*****************************************************************************************
Code:			Code for Operating 2-axis Robot Assembly with a Teach Pendant and Motion 
				Smoothening Algorithm
Author:			Ankit Manerikar
Date Created:	11/20/2013
Last Update:	12/15/2013
Description:	The following code allows a two axis robot assembly with a gripper to be 
				taught a PTP motion and repeat the same with curvilinear motion using parabolic 
				approximation.
				The Robot is operated by 3 servos - M0 for Elbow, M1 for Shoulder, M2 for Gripper
				Each motor is controlled by switches on the teach pendant.
				Pin Connections:
				
				PB.0 - M0 PWM
				PB.1 - M1 PWM
				PB.2 - M2 PWM
				
				PA.0 - M0 UP
				PA.1 - M0 DOWN
				PA.2 - M1 UP
				PA.3 - M1 DOWN
				PA.4 - M2 UP
				PA.5 - M2 DOWN
				
				PA.7 - MODE SELECT 
					   PA.7 = 0: Teach Mode  - Learns and Saves a Set of PTP motion
					   PA.7 = 1: Repeat Mode - Repeats a saved motion
*****************************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "servo4_avr.h"

unsigned int mot1[50],mot2[50], mot3[50];		// Array to store point values
unsigned int m0,m1,m2;

unsigned int stepcnt, stepnum;					
unsigned int step = 0;

unsigned int speed = 20;						// Value controlling speed of operation

unsigned char status;							// indicates Teach or Repeat Mode
unsigned char tx0, tx1, tx2;

/*Uart Initialization Function **********************************************************/
void uartinit()
{
	UCSRA=0x00;				//USART Control and Status Registers
	UCSRB=0x18;
	UCSRC=0x86;
	
	UBRRH=0x00;				//USART Baud Rate Registers
	UBRRL=0x06;
	_delay_ms(100);
}

/*****************************************************************************************/
void uartcharput(unsigned char uchar)
{
	UDR=uchar;						// Put Data into UART Buffer
	while((UCSRA&0x40)==0x00);		// Wait till transmission is complete
	_delay_ms(1000);				 
}
/*****************************************************************************************/

int main()
 {
 
 //Initializations
 servo_init();						
 uartinit();
 
 //Soft Home Position for the robot
 unsigned int k0 = 90;
 unsigned int k1 = 120;
 unsigned int k2 = 150;
 
 DDRA = 0x00;

  while(1)
   {

/** Teach Mode **************************************************************************/

	if((PINA&0x40)==0x40) 
	{  
		status = 'T';		//Set status to Teach Mode
		tx0 = k0;
		tx1 = k1;
		tx2 = k2;
	
/****************SWITCHES FOR CONTROLLING MOTOR 1*****************************************/
	
	 if((PINA&0x01)==0x01) // increase angle if PA.0 pressed
		{
		
			if (k0==190)	{k0 = 190;}
			else			{k0++;	_delay_ms(5);}
			setmotorval(k0,0);
		}
	 else					{k0=k0;	setmotorval(k0,0);}

	 if((PINA&0x02)==0x02) // decrease angle if PA.1 pressed
		{
			if (k0== 50)	{k0 = 50;}
			else			{k0--;	_delay_ms(5);}
			setmotorval(k0,0);
		}
	 else					{k0=k0;	setmotorval(k0,0);}

	/****************SWITCHES FOR CONTROLLING MOTOR 2*************************************/
	
	 if((PINA&0x04)==0x04) // increase angle if PA.2 pressed
		{	
			if (k1==190)	{k1 = 190;}
			else			{k1++;	_delay_ms(5);}
			setmotorval(k1,1);
		}
  	 else					{k1=k1;	setmotorval(k1,1);}
		
	 if((PINA&0x08)==0x08) // decrease angle if PA.3 pressed
		{
		if (k1== 50)		{k1 = 50;}
		else				{k1--; _delay_ms(5);}
		setmotorval(k1,1);
		}
	 else					{k1=k1;setmotorval(k1,1);}
	
	/****************SWITCHES FOR CONTROLLING MOTOR 3*************************************/
	
	if((PINA&0x10)==0x10) // decrease angle if PA.4 pressed
		{
			if (k2==190)	{k2 = 190;}
			else			{k2++;	_delay_ms(5);}
			setmotorval(k2,2);
		}
 	 else					{k2=k2;	setmotorval(k2,2);}	
		
	 if((PINA&0x20)==0x20) // decrease angle if PA.5 pressed
		{
		if (k2== 50)		{k2 = 50;}
		else				{k2--; _delay_ms(5);}
		setmotorval(k2,2);
		}
	 else					{k2=k2;setmotorval(k2,2);}	
		
	/*******************************************************************************************/

	setmotorval(k0,0);		// Retain values once changed				
	setmotorval(k1,1);
	setmotorval(k2,2);
	
/*********************REMEMBER STEP*************************************************************/	
	
	if((PINA&0x80)==0x80)		// if Save button is pressed
		{
		status = 0xfE; 
		mot1[step] = k0;
		mot2[step] = k1;
		mot3[step] = k2;
				
		stepcnt = step;
		step++;
		while((PINA&0x80)==0x80);
		}

/*******************************************************************************************/	
}

else
{	
	status = 'R';				// Change status to Teach Mode
	tx0 = m0;
	tx1 = m1;
	tx2 = m2;	

	if(stepcnt>0)
	{
	for(stepnum = 0; stepnum <= stepcnt; stepnum++)		// Repeat Saved Steps
		{
/************************For Speed Control*************************************************/

			if((PINA&0x01)==0x01){speed = speed + 5;}		// Increase Speed
			
			if((PINA&0x02)==0x02)							// Decrease Speed
			{
				if(speed == 0)	{speed = 0;}
				else		    {speed = speed - 5;}
			}
			
/******************************************************************************************/		

			m0 = mot1[stepnum];
			m1 = mot2[stepnum];
			m2 = mot3[stepnum];	
			
			if(stepnum != stepcnt)							
			{
			
/* Here a parabolic approximation to the PTP trajectory is generated:
   As the robot moves from one saved point to the next, the speed of the robot is maintained 
   constant so the displacement is parabolic in nature giving a smooth transition.
*/
			while((m0!=mot1[stepnum+1]) || (m1!=mot2[stepnum+1]) || (m2!=mot3[stepnum+1]))
				{
					if(m0 < mot1[stepnum+1]) 			{m0++;}
					else if(m0 > mot1[stepnum+1]) 		{m0--;}
					else if(m0 == mot1[stepnum+1]) 	{m0 = m0;}	
					setmotorval(m0,0);
				
					if(m1 < mot2[stepnum+1]) 			{m1++;}	
					else if(m1 > mot2[stepnum+1]) 		{m1--;}	
					else if(m1 == mot2[stepnum+1]) 	{m1 = m1;}
					setmotorval(m1,1);
					
					if(m2 < mot3[stepnum+1]) 			{m2++;}			
					else if(m2 > mot3[stepnum+1]) 		{m2--;}	
					else if(m2 == mot3[stepnum+1]) 	{m2 = m2;}
					setmotorval(m2,2);					
					_delay_ms(speed);
				}
					
			}
			else if(stepcnt==stepnum)								// for last step in the sequence
			{	
				uartcharput('7');
				_delay_ms(10);
				
			while((m0!=mot1[0]) || (m1!=mot2[0]) || (m2!=mot3[0]))	 
				{
				if(m0 < mot1[0]) 				{m0++;}				// for motor 0
				else if(m0 > mot1[0]) 			{m0--;}
				else if(m0 == mot1[0]) 		{m0 = m0;}	
				setmotorval(m0,0);
			
				if(m1 < mot2[0]) 				{m1++;}				// for motor 1
				else if(m1 > mot2[0]) 			{m1--;}	
				else if(m1 == mot2[0]) 		{m1 = m1;}
				setmotorval(m1,1);
				
				if(m2 < mot3[0]) 				{m2++;}				// for motor 2
				else if(m2 > mot3[0]) 			{m2--;}	
				else if(m2 == mot3[0]) 		{m2 = m2;}
				setmotorval(m2,2);
				_delay_ms(speed);
				}
			}
			
		setmotorval(m0,0);						//assign motor values
		setmotorval(m1,1);
		setmotorval(m2,2);
		_delay_ms(speed);
		
		}
	}
	else
	{	setmotorval(90,0);
		setmotorval(120,1);
		setmotorval(150,2);	
		_delay_ms(1000);
	}
}

//Transmit Motor Values and Status to display on MATLAB
		uartcharput(status);				
		uartcharput(k0);
		uartcharput(k1);
		uartcharput(k2);
   }
 }
/**********************************End***************************************************************/ 
 
