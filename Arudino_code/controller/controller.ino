// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h>

/* create servo object to control a servo */
Servo servo_obj_1;  
Servo servo_obj_2;

int baud_rate = 9600;

/*Initialize IR pins and values. There are 2 IR sensors being used - sensor 1 relates to the index finger, sensor */
int IR_pin_1 = 0; 
int IR_pin_2 = 1; 
float IR_val_1 = 0; //[V]
float IR_val_2 = 0; //[V]
float IR_res = 5.0 / 1023;

/*Initialize servo pins and values*/
int servo_pin_1 = 9;
int servo_pin_2 = 10;
int servo_val_1 = 0; 
int servo_val_2 = 0;
float force_1 = 0;
float force_2 = 0;
String serialMsg;

void print1IRval( float val )
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

void getForce( String string, float * num1 )
{
	int index1 = 0;
	int index2 = 0;
	int index3 = 0;
	int temp = 0;

	while( temp < string.length() )
	{
		if (string[temp] == 't')
			index1 = temp;
		else if (string[temp] == '/' )
			index2 = temp;
		else if (string[temp] == ';' )
			index3 = temp;
		temp++;
	}

	String string1 = string.substring( index1+1, index2-1 );
	*num1 = strToFloat( string1 );
}

void setup() 
{ 
	/* attach servos to the servo objects */
	servo_obj_1.attach(servo_pin_1);  
	servo_obj_2.attach(servo_pin_2);

	Serial.begin(baud_rate);
} 
 
void loop() 
{
	/* read the IR sensor and convert to voltage*/
	IR_val_1 = analogRead(IR_pin_1) * IR_res;
	IR_val_2 = analogRead(IR_pin_2) * IR_res;	
	//print1IRval(IR_val_1);

	/* get the force being applied to the remote robot */
	serialMsg = "";
	while ( Serial.available() > 0 )
	{
		char inChar = Serial.read();
		serialMsg += inChar;
	}

	getForce( serialMsg, &force_1 );
	/* move servo */
	servo_val_1= map(IR_val_1, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
	Serial.println(force_1);
	//servo_obj_1.write(servo_val_1);                  // sets the servo position according to the scaled value 
	delay(15);                           // waits for the servo to get there
}



