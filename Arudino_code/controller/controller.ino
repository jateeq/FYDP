/*
	Authors: Jawad Ateeq
*/

#include <Servo.h>

//#define REMOTE 1

/* Constants */
const int MIN_FLAG_DISP_1 = 3;	//Min dist flag has to move in either direction to move servo
const int MIN_SERVO_DISP_1 = 7;	//Servo will move this many degrees every time actuated
const int MIN_SERVO_POS_1 = 70;	//degrees
const int MAX_SERVO_POS_1 = 130;	//degrees
const int MIN_SERVO_DISP_2 = 7;	//Servo will move this many degrees every time actuated
const int MIN_SERVO_POS_2 = 30;	//degrees
const int MAX_SERVO_POS_2 = 70;	//degrees
const int NUM_IR_READINGS = 5;	//this many readings taken and averaged to get overall reading
const int BAUD_RATE = 9600; 	//Arduino Baud Rate
const float IR_RES = 5.0/1023; 	//Resolution of IR sensor
const int IR_PIN_1 = 0;			//IR 1 connected to analog pin 1
const int IR_PIN_2 = 1;			//IR 2 connected to analog pin 2
const int SERVO_PIN_1 = 9;		//Servo 1 connected to digital pin 9
const int SERVO_PIN_2 = 10;		//Servo 2 connected to digital pin 10
const int LOOP_DELAY = 100;		//ms

/* House Keeping */
Servo servo_obj_1;				//Servo 1 
Servo servo_obj_2;				//Servo 2
int dir_1;						//Direction flag of sensor 1 is moving in
int dir_2;						//Direction flag of sensor 2 is moving in
int IR_val_1[ NUM_IR_READINGS ] = { -1 }; 
int IR_val_2[ NUM_IR_READINGS ] = { -1 };
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

void setup() 
{ 
	/* attach servos to the servo objects */
	servo_obj_1.attach( SERVO_PIN_1 );  
	servo_obj_2.attach( SERVO_PIN_2 );

	Serial.begin( BAUD_RATE );		
	 
	/* Initialize values */
	servo_pos_1 = MIN_SERVO_POS_1;
	servo_pos_2 = MIN_SERVO_POS_1;
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
	/* Read a sequence of IR readings, average to filter out some noise */
	for ( int i = 0 ; i < NUM_IR_READINGS ; i++ )
	{
		IR_val_1[ i ] = analogRead( IR_PIN_1 );
		IR_val_2[ i ] = analogRead( IR_PIN_2 );
	}
	IR_val_1[ 0 ] = average( IR_val_1 );
	IR_val_2[ 0 ] = average( IR_val_2 );
	IR_val_1_cur =  smooth( IR_val_1[ 0 ], 0.7, IR_val_1_prev );
	IR_val_2_cur =  smooth( IR_val_2[ 0 ], 0.7, IR_val_2_prev );

#ifdef REMOTE
	/* Send the IR value in voltage to the remote robot this is used to figure 
	out the finger position */
	sendFingerPos2Remote( ( float ) IR_val_1_cur * IR_RES, ( float ) IR_val_2_cur * IR_RES );
#else
	//print1IRval( IR_val_2_cur );
	Serial.print( IR_val_1_prev );
	Serial.print('/');
	Serial.print( IR_val_2_prev );
	Serial.print(' ');
	Serial.print( IR_val_1_cur );
	Serial.print('/');
	Serial.println( IR_val_2_cur );
#endif

	/* Find out direction of flag movement - Note that the sensor value decreases 
	the farther it is from the sensor */
	FindFingerDir( IR_val_1_cur, IR_val_1_prev, &dir_1, MIN_FLAG_DISP_1 );	
	FindFingerDir( IR_val_2_cur, IR_val_2_prev, &dir_2, MIN_FLAG_DISP_1 );
	
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
	if (getForce( serialMsg, &force_1, &force_2	))
	{
		updateServo( &servo_pos_1, &servo_obj_1, force_1, dir_1, MIN_SERVO_DISP_1, MIN_SERVO_POS_1, MAX_SERVO_POS_1 );		
		updateServo( &servo_pos_2, &servo_obj_2, force_2, dir_2, MIN_SERVO_DISP_2, MIN_SERVO_POS_2, MAX_SERVO_POS_2 );
	}
#else
	force_1 = 0;
	force_2 = 0;
	updateServo( &servo_pos_1, &servo_obj_1, force_1, dir_1, MIN_SERVO_DISP_1, MIN_SERVO_POS_1, MAX_SERVO_POS_1 );
	updateServo( &servo_pos_2, &servo_obj_2, force_2, dir_2, MIN_SERVO_DISP_2, MIN_SERVO_POS_2, MAX_SERVO_POS_2 );	
#endif	 	
	
	//Record previous IR vals so change in flag position can be found
	IR_val_1_prev = IR_val_1_cur;
	IR_val_2_prev = IR_val_2_cur;
	
	// Delay added for optimal performance
	delay( LOOP_DELAY );
}


void updateServo( int * servo_pos_handle, Servo * servo_obj, int force, int direction, int min_servo_disp, int min_servo_pos,
					int max_servo_pos )
{  	
	int servo_pos = *servo_pos_handle;
	/* move servo if no object detected */
	if ( force  == 1 )
	{
		//hold previous position
	}
	else if ( force == 0 )
	{		
		if ( direction == 1 )
		{
			if( servo_pos < max_servo_pos )
			{
				servo_pos += min_servo_disp;
			}
		}
		else if ( direction == 0 )
		{
			if( servo_pos > min_servo_pos )
			{
				servo_pos -= min_servo_disp;
			}
		}
		
		// sets the servo position according to the scaled value
		*servo_pos_handle = servo_pos;
		(*servo_obj).write(servo_pos); 
	}
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
	Serial.print( pos_1, DEC );
	Serial.print( '/' );
	Serial.print( pos_2, DEC );
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