/******************************************************************************************* 
Code:			Header file for controlling four servos with a single timer
Author:   		Ankit Manerikar
Created: 		09/12/2012
Last Modified:	09/20/2012
*******************************************************************************************/
#ifndef _SERVO4_AVR_H_
#define _SERVO4_AVR_H_

int TimeMultiplex;   
unsigned char val0h,val0l,val1h,val1l,val2h,val2l,val3h,val3l,x,y;
unsigned char tempval;

/******************************************************************************************/
void servo_init()
{

TCCR1A=0x03;			//Timer Control Status Registers
TCCR1B=0x1A;

TCNT1H=0x00;			// Timer Value
TCNT1L=0x00;

OCR1AH=0x02;			// OCRA register for setting PWM frequency
OCR1AL=0xB9;

OCR1BH=0x02;			// OCRB register for setting PWM duty cycle
OCR1BL=0x0A;

TIMSK=0x18;  			
DDRB=0x0f;

sei(); 
}

/****************************************************************************************/
ISR (TIMER1_COMPA_vect) // Sets the frequency to 5 ms (20 ms / 4)
{
PORTB=0X00;
TCNT1H=0x00;
TCNT1L=0x00;
}

/*****************************************************************************************/
ISR (TIMER1_COMPB_vect) // Sets PWM duty cycle for each multiplexed section to as per motor value 
{
switch(TimeMultiplex)
{
case 0:
 OCR1BH=val0h;     
 OCR1BL=val0l;     
 PORTB=0X01;
 TimeMultiplex=1;
 break ;  
 
case 1:
 OCR1BH=val1h;       
 OCR1BL=val1l;      
 PORTB=0X02;
 TimeMultiplex=2;
 break ; 
 
case 2:
 OCR1BH=val2h;              
 OCR1BL=val2l;                
 PORTB=0X04;
 TimeMultiplex=3;
 break ;
 
case 3:
 OCR1BH=val3h;   
 OCR1BL=val3l;      
 PORTB=0X08;
 TimeMultiplex=0;
 break ; 
 } 
}
/*****************************************************************************************/
void setmotorval(float value,int motorno)
{
        unsigned int value;
        va=351+value*1.27;	   		// Conversion from float angle value to int
		if(motorno==2)             	// Feed motor angle value as motor selected
        {
                val1l=value;
                tempval=value>>8;
                val1h=tempval;
        }
        if(motorno==3)            
        {
                val2l=value;
                tempval=value>>8;
                val2h=tempval;
        }
        if(motorno==0)       
        {
                val3l=value;
                tempval=value>>8;
                val3h=tempval;
        }
        if(motorno==1)     
        {
                val0l=va;
                tempval=value>>8;
                val0h=tempval;
        }
}

/*******************************************************************************************/
#endif 
