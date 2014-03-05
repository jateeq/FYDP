/*
	Authors: Jawad Ateeq
*/

#include <Servo.h>


/* Constants */
const int MIN_FLAG_DISP = 10;	//Min dist flag has to move in either direction to move servo
const int MIN_SERVO_DISP = 20;	//Servo will move this many degrees every time actuated
const int MIN_SERVO_POS = 50;	//degrees
const int MAX_SERVO_POS = 100;	//degrees
const int LOOP_DELAY = 100;	//ms
const int NUM_IR_READINGS = 5;	//this many readings taken and averaged to get overall reading
const int BAUD_RATE = 9600; 	//Arduino Baud Rate
const float IR_RES = 5.0/1023; 	//Resolution of IR sensor
const int IR_PIN_1 = 0;			//IR 1 connected to analog pin 1
const int IR_PIN_2 = 1;			//IR 2 connected to analog pin 2
const int SERVO_PIN_1 = 9;		//Servo 1 connected to digital pin 9
const int SERVO_PIN_2 = 10;		//Servo 2 connected to digital pin 10

/* House Keeping */
Servo servo_obj_1;				//Servo 1 
Servo servo_obj_2;				//Servo 2
int dir;						//Direction flag is moving in
int IR_val_1[ NUM_IR_READINGS ] = { -1 }; 
int IR_val_2[ NUM_IR_READINGS ] = { -1 };
int IR_val_1_prev;
int IR_val_2_prev;
int servo_pos_1; 
int servo_pos_2;
int servo_pos_1_prev;
int servo_pos_2_prev;
int force_1;
int force_2;
String serialMsg;


/* Function Definitions */
void print1IRval( int val );
float strToFloat( String string);
int strToInt(String string);
boolean getForce( String string, int * num1 );
int average ( int values[] );

void setup() 
{ 
	/* attach servos to the servo objects */
	servo_obj_1.attach( SERVO_PIN_1 );  
	servo_obj_2.attach( SERVO_PIN_2 );

	Serial.begin( BAUD_RATE );		
	 
	/* Initialize values */
	servo_pos_1 = MIN_SERVO_POS;
	servo_pos_2 = MIN_SERVO_POS;
	dir = -1;
	force_1 = -1;
	force_2 = -1;
} 


void loop() 
{
	//Record previous IR vals so change in flag position can be found
	//IR_val_1_prev = IR_val_1[0];
	//IR_val_2_prev = IR_val_2[0];

	/* Read a sequence of IR readings, average to filter out some noise */
	for ( int i = 0 ; i < NUM_IR_READINGS ; i++ )
	{
		IR_val_1[ i ] = analogRead( IR_PIN_1 );
		IR_val_2[ i ] = analogRead( IR_PIN_2 );
		delay( 50 );
	}

	IR_val_1[ 0 ] = average( IR_val_1 );
	
	/*
	for ( int i = 0 ; i < NUM_IR_READINGS ; i++ )
	{
		print1IRval(IR_val_1[ i ]);
	}
	Serial.println();
	Serial.println();			
	*/
	
	print1IRval(IR_val_1[ 0 ]);
	
	//print1IRval( IR_val_1 * IR_RES );	//send the IR value in voltage to the remote robot
										//this is used to figure out the finger position
									
	/* 	Find out direction of flag movement 
		Note that the sensor value decreases the farther it is from the 
		sensor
	*/
	/*if ( ( IR_val_1 - IR_val_1_prev ) > MIN_FLAG_DISP )
	{
		//flag is moving towards IR sensor (finger moving up)		
		dir = 0;
	}
	else if ( ( IR_val_1 - IR_val_1_prev ) < ( -MIN_FLAG_DISP ) )
	{
		//flag is moving away from IR sensor (finger moving down)
		dir = 1;
	}
	else
	{
		dir = -1;
	}*/
	
	/* get the force being applied to the remote robot */
	serialMsg = "";
	while ( Serial.available() > 0 )
	{
		char inChar = Serial.read();
		serialMsg += inChar;
	}
	
	//reset force from last time
	force_1 = -1; 
	
	//if (getForce( serialMsg, &force_1 ))
	//{
		/* move servo if no object detected */
		//if ( force_1  == 1 )
		//{
			//hold previous position
		//}
		//else if ( force_1 == 0 )
		//{
		/*
			if (dir == 1)
			{
				servo_pos_1 += MIN_SERVO_DISP;
			}
			else if (dir == 0)
			{
				servo_pos_1 -= MIN_SERVO_DISP;
			}
			
			if ( servo_pos_1 < MIN_SERVO_POS )
			{
				servo_pos_1 = MIN_SERVO_POS;
			}
			else if ( servo_pos_1 > MAX_SERVO_POS )
			{
				servo_pos_1 = MAX_SERVO_POS;
			}
			*/
			// scale it to use it with the servo (value between 0 and 180)
			//servo_pos_1 = map(IR_val_1, 0, 1023, 0, 179);
			
			// sets the servo position according to the scaled value 
			servo_obj_1.write(servo_pos_1); 
		//}
	//}   
	//print1IRval(servo_pos_1);
        
	delay( LOOP_DELAY ); // waits for the servo to get there
}

void print1IRval( int val )
{
	Serial.print('i');
	Serial.print(val, DEC);
	Serial.println('e');
}

/*Converts a string to float. Can't use atof directly since it expects a char 
array as parameter, so need to convert strign to char array first*/
float strToFloat( String string)
{
	char floatbuf[32]; // make this at least big enough for the whole string
	string.toCharArray(floatbuf, sizeof(floatbuf));
	return atof(floatbuf);
}

int strToInt(String string)
{
	char floatbuf[32]; // make this at least big enough for the whole string
	string.toCharArray(floatbuf, sizeof(floatbuf));
	return atoi(floatbuf);
}

boolean getForce( String string, int * num1 )
{
	int index1 = -1;
	int index2 = -1;
	int index3 = -1;
	int temp = 0;

	while( temp < string.length() )
	{
		if (string[temp] == 'T')
			index1 = temp;
		else if (string[temp] == '/' )
			index2 = temp;
		else if (string[temp] == ';' )
			index3 = temp;
		temp++;
	}

	if (index1 < index2 && index1 != -1 && index2 != -1)
	{		
		*num1 = (string[index1+1]-'0');
		return true;
	} 
	else 
	{
		return false;
	}
}

int average( int values[] )
{
	int sum = 0;
	int num_values = sizeof( values ) / sizeof( int );
	for( int i = 0 ; i < num_values ; i++ )
	{
		sum += values[ i ];
	}
	
	return sum / num_values;
}