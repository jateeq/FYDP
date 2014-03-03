/*
Authors: Jawad Ateeq, Jake Park
*/
adding his stuff
#include <Servo.h>

/* Constants */

//Min dist flag has to move in either direction to move servo
const int MIN_FLAG_DISP = 10;
const int MIN_SERVO_DISP = 20;
const int MIN_SERVO_POS = 50;	//degrees
const int MAX_SERVO_POS = 100;	//degrees
const int LOOP_DELAY = 50;		//ms

/* create servo object to control a servo */
Servo servo_obj_1;  
Servo servo_obj_2;

int baud_rate = 9600;

/*Initialize IR pins and values. There are 2 IR sensors being used - 
sensor 1 relates to the index finger, sensor */
int IR_pin_1 = 0; 
int IR_pin_2 = 1; 
int IR_val_1 = 0; 
int IR_val_2 = 0;
int IR_val_1_prev = 0;
int IR_val_2_prev = 0;
float IR_res = 5.0 / 1023;

//Direction flag is moving in
int dir = -1;

/*Initialize servo pins and values*/
int servo_pin_1 = 9;
int servo_pin_2 = 10;
int servo_pos_1 = 0; 
int servo_pos_2 = 0;
int servo_pos_1_prev = 0;
int servo_pos_2_prev = 0;
int force_1 = 0;
int force_2 = 0;
String serialMsg;

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
		// String string1 = string.substring( index1+1, index2-1 );
		//*num1 = strToInt( string1 );
		*num1 = (string[index1+1]-'0');
		return true;
	} 
	else 
	{
		return false;
	}
}

void setup() 
{ 
	/* attach servos to the servo objects */
	servo_obj_1.attach(servo_pin_1);  
	servo_obj_2.attach(servo_pin_2);

	Serial.begin(baud_rate);
	
	//starting position of servos is 
	servo_pos_1 = MIN_SERVO_POS;
} 

//int x = 20;

void loop() 
{
	//Record previous IR vals so change in flag position can be found
	IR_val_1_prev = IR_val_1;
	IR_val_2_prev = IR_val_2;

	/* read the IR sensor and convert to voltage*/
	IR_val_1 = analogRead(IR_pin_1);
	IR_val_2 = analogRead(IR_pin_2);

	//send the IR value in voltage to the remote robot
	//this is used to figure out the finger position
	//print1IRval(IR_val_1*IR_res);
	print1IRval(IR_val_1);
	
	/* Find out direction of flag movement 
		Note that the sensor value decreases the farther it is from the 
		sensor*/
	if ( ( IR_val_1 - IR_val_1_prev ) > MIN_FLAG_DISP )
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
	}
	
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