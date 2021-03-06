/*
	Authors: Jawad Ateeq
*/

#include <Servo.h>

#define REMOTE 1
#define UPDATE_WITH_IR 2


/* Constants */
const int MIN_FLAG_DISP_1 = 2;	//Min dist flag has to move in either direction to move servo
const int MIN_SERVO_DISP_1 = 7;	//Servo will move this many degrees every time actuated
const int MIN_SERVO_POS_1 = 70;	//degrees
const int MAX_SERVO_POS_1 = 110;	//degrees

const int MIN_FLAG_DISP_2 = 4;	//Min dist flag has to move in either direction to move servo
const int MIN_SERVO_DISP_2 = 10;	//Servo will move this many degrees every time actuated
const int MIN_SERVO_POS_2 = 90;	//degrees
const int MAX_SERVO_POS_2 = 130;	//degrees

const int NUM_IR_READINGS = 10;	//this many readings taken and averaged to get overall reading
const int BAUD_RATE = 9600; 	//Arduino Baud Rate
const float IR_RES = 5.0/1023; 	//Resolution of IR sensor
const int IR_PIN_1 = 0;			//IR 1 connected to analog pin 1
const int IR_PIN_2 = 1;			//IR 2 connected to analog pin 2
const int SERVO_PIN_1 = 9;		//Servo 1 connected to digital pin 9
const int SERVO_PIN_2 = 10;		//Servo 2 connected to digital pin 10
const int LOOP_DELAY = 100;		//ms
const int NUM_DECIMAL_PLACES = 3;

#ifdef UPDATE_WITH_IR
const int MIN_FLAG_POS_1 = 490;
const int MAX_FLAG_POS_1 = 420;
const int MIN_FLAG_POS_2 = 430;
const int MAX_FLAG_POS_2 = 400;
#endif

/* House Keeping */
Servo servo_obj_1;				//Servo 1 
Servo servo_obj_2;				//Servo 2
int dir_1;						//Direction flag of sensor 1 is moving in
int dir_2;						//Direction flag of sensor 2 is moving in
int IR_val_1_cur;
int IR_val_2_cur;
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
boolean getForce( String string, int * num1, int * num2 );
int average ( int values[] );
void FindFingerDir( int cur, int prev, int * dir );
void ReadFingerPos( int * IR_val, int IR_val_prev, int IR_pin );
int updateServo( int servo_pos, int force, int direction, int min_servo_disp, int min_servo_pos, int max_servo_pos );
#ifdef UPDATE_WITH_IR
int updateServoWithIR( int IR_reading, int min_flag_pos, int max_flag_pos, int min_servo_pos, int max_servo_pos );
#endif

void setup() 
{ 
	/* attach servos to the servo objects */
	servo_obj_1.attach( SERVO_PIN_1 );  
	servo_obj_2.attach( SERVO_PIN_2 );

	Serial.begin( BAUD_RATE );		
	 
	/* Initialize values */
	servo_pos_1 = MIN_SERVO_POS_1;
	servo_pos_2 = MIN_SERVO_POS_2;
	dir_1 = -1;
	dir_2 = -1;
	force_1 = -1;
	force_2 = -1;
	IR_val_1_prev = 0;
	IR_val_2_prev = 0;
	
	servo_obj_1.write( servo_pos_1 );
	servo_obj_2.write( servo_pos_2 );
} 

void loop() 
{			
	ReadFingerPos( &IR_val_1_cur, IR_val_1_prev, IR_PIN_1 );
	ReadFingerPos( &IR_val_2_cur, IR_val_2_prev, IR_PIN_2 );
	
#ifdef REMOTE
	/* Send the IR value in voltage to the remote robot this is used to figure 
	out the finger position */
	sendFingerPos2Remote( ( float ) IR_val_1_cur * IR_RES, ( float ) IR_val_2_cur * IR_RES );
#endif

//#ifndef UPDATE_WITH_IR
	/* Find out direction of flag movement - Note that the sensor value decreases 
	the farther it is from the sensor */
	FindFingerDir( IR_val_1_cur, IR_val_1_prev, &dir_1, MIN_FLAG_DISP_1 );	
	FindFingerDir( IR_val_2_cur, IR_val_2_prev, &dir_2, MIN_FLAG_DISP_2 );
//#endif

	/* get the force being applied to the remote robot */
	serialMsg = "";
	while ( Serial.available() > 0 )
	{
		char inChar = Serial.read();
		serialMsg += inChar;
	}
	
	/* Update the servos based on new IR reading */
#ifdef REMOTE
	force_1 = -1;
	force_2 = -1;
	if ( getForce( serialMsg, &force_1, &force_2 ) )
	{
	#ifdef UPDATE_WITH_IR
		if( !force_1 )
		{
			servo_pos_1 = updateServoWithIR( IR_val_1_cur, MIN_FLAG_POS_1, MAX_FLAG_POS_1, MIN_SERVO_POS_1, MAX_SERVO_POS_1 );
		}
		if( !force_2 )
		{
			servo_pos_2 = updateServoWithIR( IR_val_2_cur, MIN_FLAG_POS_2, MAX_FLAG_POS_2, MIN_SERVO_POS_2, MAX_SERVO_POS_2 );
		}
	#else
		servo_pos_1 = updateServo( servo_pos_1, force_1, dir_1, MIN_SERVO_DISP_1, MIN_SERVO_POS_1, MAX_SERVO_POS_1 );
		servo_pos_2 = updateServo( servo_pos_2, force_2, dir_2, MIN_SERVO_DISP_2, MIN_SERVO_POS_2, MAX_SERVO_POS_2 );
	#endif
	}
#else
	#ifdef UPDATE_WITH_IR	
	servo_pos_1 = updateServoWithIR( IR_val_1_cur, MIN_FLAG_POS_1, MAX_FLAG_POS_1, MIN_SERVO_POS_1, MAX_SERVO_POS_1 );
	servo_pos_2 = updateServoWithIR( IR_val_2_cur, MIN_FLAG_POS_2, MAX_FLAG_POS_2, MIN_SERVO_POS_2, MAX_SERVO_POS_2 );
	#else
	servo_pos_1 = updateServo( servo_pos_1, force_1, dir_1, MIN_SERVO_DISP_1, MIN_SERVO_POS_1, MAX_SERVO_POS_1 );
	servo_pos_2 = updateServo( servo_pos_2, force_2, dir_2, MIN_SERVO_DISP_2, MIN_SERVO_POS_2, MAX_SERVO_POS_2 );
	#endif
	Serial.print( IR_val_1_cur );
	Serial.print('/');
	Serial.println( servo_pos_1 );
#endif

	//Move the servos to the new position
	servo_obj_1.write( servo_pos_1 );
	servo_obj_2.write( servo_pos_2 );
	
	//Record previous IR vals so change in flag position can be found
	IR_val_1_prev = IR_val_1_cur;
	IR_val_2_prev = IR_val_2_cur;

#ifndef UPDATE_WITH_IR
	// Delay added for optimal performance
	delay( LOOP_DELAY );
#endif
}

void ReadFingerPos( int * IR_val, int IR_val_prev, int IR_pin )
{
	/* Read a sequence of IR readings, average to filter out some noise */
	int temp = 0;
	for ( int i = 0 ; i < NUM_IR_READINGS ; i++ )
	{
		temp += analogRead( IR_pin );
	}
	temp = temp / NUM_IR_READINGS;
	*IR_val =  smooth( temp, 0.7, IR_val_prev );
}

#ifdef UPDATE_WITH_IR

int updateServoWithIR( int IR_reading, int min_flag_pos, int max_flag_pos, int min_servo_pos, int max_servo_pos )
{
	int servo_pos;
	servo_pos = map( IR_reading, min_flag_pos, max_flag_pos, min_servo_pos, max_servo_pos );
	servo_pos = constrain( servo_pos, min_servo_pos, max_servo_pos ); //map does not constrain range bounds
	//Serial.println( servo_pos );
	return servo_pos;
}

#endif

int updateServo( int servo_pos, int force, int direction, int min_servo_disp, int min_servo_pos, int max_servo_pos )
{  	
	/* move servo if no object detected */
	if ( force  == 1 )
	{
		//hold previous position
	}
	else if ( force == 0 )
	{		
		if ( direction == 1 )
		{
			servo_pos += min_servo_disp;
		}
		else if ( direction == 0 )
		{
				servo_pos -= min_servo_disp;
		}	
	}		
	servo_pos = constrain( servo_pos, min_servo_pos, max_servo_pos );
	return servo_pos;
}

void FindFingerDir( int cur, int prev, int * dir, int min_flag_disp )
{
	if ( ( cur - prev ) > min_flag_disp )
	{
		//flag is moving towards IR sensor (finger moving up)		
		*dir = 0;
		
	}
	else if ( ( cur - prev ) < ( - min_flag_disp ) )
	{
		//flag is moving away from IR sensor (finger moving down)
		*dir = 1;
	}
	else
	{
		*dir = -1;
	}
	//Serial.println(*dir);
}

void print1IRval( int val )
{
	Serial.print('i');
	Serial.print(val, DEC);
	Serial.println('e');
}

void sendFingerPos2Remote( float pos_1, float pos_2 )
{
	Serial.print( 'i' );
	Serial.print( pos_1, NUM_DECIMAL_PLACES );
	Serial.print( '/' );
	Serial.print( pos_2,NUM_DECIMAL_PLACES );
	Serial.println( 'e' );
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

boolean getForce( String string, int * num1, int * num2 )
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
		*num1 = ( string[ index1 + 1 ] - '0' );
		*num2 = ( string[ index2 + 1 ] - '0' );
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

int smooth(int data, float filterVal, float smoothedVal){


  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = ( data * ( 1 - filterVal ) ) + ( smoothedVal  *  filterVal );

  return ( int )smoothedVal;
}
