// ENG1000 Group 6 MERE Prototype rover firmware MK1
// This firmware will run on an ARDUINO UNO R3
// and serve as a translation layer between BTserial commands and
// arduino functions
// The aim of this firmware is to be adaptable and able to change
// the functionality of the rover without reflashing the ARDUINO

// lines139-167 are an adaptaion of the included magsensor_Example.ino 
// supplied with Adafruit_HMC5883_U.h library Written by Kevin Townsend for Adafruit Industries

// code written by Edward Ambrogio z5361373

#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#define MAX_ANGLE 180
#define MIN_ANGLE 0
#define SERVOMIN  600 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2400 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define FREQUENCY 50 // Analog servos run at ~50 Hz updates

#define DECLINATION 0.22

#define SERVOARM 4
#define SERVOPAYLOAD 3
#define SERVOBUCKETL 2
#define SERVOBUCKETR 1

#define ARM 'A'
#define PAYLOAD 'a'
#define BUCKET 'B'

#define MOTORRR 'b'
#define MOTORAA 'C'
#define MOTORLL 'c'

#define GETCOMPASS 'D'
#define GETUWB 4

#define TRUE 1
#define FALSE 0

#define MOTORLPWM 5
#define MOTORLPIN 6
#define MOTORLINV 7
#define MOTORRINV 8
#define MOTORRPIN 9
#define MOTORRPWM 10

SoftwareSerial BTserial(2, 3); // RX | TX
// Connect the HC-06 TX to the Arduino RX on pin 2. 
// Connect the HC-06 RX to the Arduino TX on pin 3 through a voltage divider.

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(); //initalise servo board
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); //intalise compass

int compassOnline;
int servosOnline;
int buffer;
char cmd;
String str_sub_cmd;
int sub_cmd;

void setup() { //Initialize Serial Monitor
    Serial.begin(9600);
    BTserial.begin(9600); //Initialize BTserial Serial Port
    servo.begin();
    servo.setPWMFreq(FREQUENCY);

    //setup motor driver
    pinMode(MOTORLPIN, OUTPUT);
    pinMode(MOTORLINV, OUTPUT);
    pinMode(MOTORRPIN, OUTPUT);
    pinMode(MOTORRINV, OUTPUT);
    //maybe 2 more pins for PWM
    
    pinMode(13, OUTPUT);

    if(mag.begin()) {
        sensor_t sensor;
        mag.getSensor(&sensor);
        compassOnline = TRUE;
    } else {
        compassOnline = FALSE;
    }
}

void loop() {
    bool done = false;
    /* 
    Packet Style
    char : decimal (as characters so string varies in size
    e.g A123
    CMD = A (ASCII character)
    str_sub_cmd = "123" (ASCII string)
    sub_cmd = 123 (integer number)
    this allows us input the subcmd straight into the program :)

    https://forum.arduino.cc/index.php?topic=396450.0
    found this thread a long while after coding this its just here as a refrence atm

    */
    while(!BTserial.available());
    buffer = BTserial.read();
    if (buffer == 122) {
        
        done = true;
    } else if (isDigit(buffer)) {   
        digitalWrite(13, HIGH); 
        str_sub_cmd += (char)buffer;
    
    } else {
        digitalWrite(13, HIGH);
        cmd = (char)buffer;
    }
    
    
    //debug(cmd,sub_cmd,str_sub_cmd);
    if (done) {
        digitalWrite(13, LOW);
        sub_cmd = str_sub_cmd.toInt();
        debug(cmd, sub_cmd, str_sub_cmd);
        if (cmd == ARM) {
            move(SERVOARM, sub_cmd);
        } else if (cmd == PAYLOAD) {
            move(SERVOPAYLOAD, sub_cmd);
        } else if (cmd == BUCKET) {
            move(SERVOBUCKETL, sub_cmd);
            move(SERVOBUCKETR, sub_cmd);
        } else if (cmd == MOTORAA) {
            //Serial.println("noises");
            motorAA(sub_cmd);
        } else if (cmd == MOTORLL) {
            digitalWrite(MOTORLPIN, LOW);
            digitalWrite(MOTORLINV, HIGH);
            digitalWrite(MOTORRPIN, HIGH);
            digitalWrite(MOTORRINV, LOW);
        } else if (cmd == MOTORRR) {
            digitalWrite(MOTORRPIN, LOW);
            digitalWrite(MOTORRINV, HIGH);
            digitalWrite(MOTORLPIN, HIGH);
            digitalWrite(MOTORLINV, LOW);
        } else if (cmd == GETCOMPASS) {
            BTserial.println(int(get_bearing(sub_cmd)));
        }
        sub_cmd = 0;
        str_sub_cmd = "";
    }
}

// debug printout.
void debug(char cmd, int sub_cmd, String str_sub_cmd) {
    Serial.print("cmd: ");
    Serial.println(cmd);
    Serial.print("sub_cmd: ");
    Serial.println(sub_cmd);
    Serial.print("String sub_cmd: ");
    Serial.println(str_sub_cmd);
}

//runs all motors
void motorAA(int speed) {
    if (speed > 256) { // rover go forward
        digitalWrite(MOTORLPIN, LOW);
        digitalWrite(MOTORLINV, HIGH);
        digitalWrite(MOTORRPIN, LOW);
        digitalWrite(MOTORRINV, HIGH);
        analogWrite(MOTORLPWM, decode_for_speed(speed));
        analogWrite(MOTORRPWM, 255);

    } else if (speed < 256) { // Rover go back
        digitalWrite(MOTORLPIN, HIGH);
        digitalWrite(MOTORLINV, LOW);
        digitalWrite(MOTORRPIN, HIGH);
        digitalWrite(MOTORRINV, LOW);
        
       
    } else { // Rover no go
        digitalWrite(MOTORLPIN, LOW);
        digitalWrite(MOTORLINV, LOW);
        digitalWrite(MOTORRPIN, LOW);
        digitalWrite(MOTORRINV, LOW);
        
    }
}

int decode_for_speed(int rawspeed) {
    int speed = 0;
    if (rawspeed > 256) {
        speed = rawspeed - 256;
        // rover go forward
    } else if (rawspeed < 256) {
        speed = abs(rawspeed - 256);
        // Rover go back
    } else {
        // Rover no go
    }
    return speed;
}


// gets comapssbearings with a sample amount between 0-100.
int get_bearing(int samples) {
    float bearings[100];
    float heading;
    if (compassOnline) {
        for (int i = 0; i < samples; i++) {
            // get a new sensor event
            sensors_event_t event; 
            mag.getEvent(&event);
            // write heading to array
            bearings[i] = atan2(event.magnetic.y, event.magnetic.x);
            bearings[i] += DECLINATION;
        }
        heading = averagef(samples, bearings); //average reading
            
        // Correct for when signs are reversed.
        if (heading < 0) {
            heading += 2*PI;
        }
        // Check for wrap due to addition of declination.
        if(heading > 2*PI) {
            heading -= 2*PI;
        }
    } else {
        for (int i = 0; i < samples; i++) {
            bearings[i] = 361;
        }
    }
    
    // Convert radians to degrees for readability.
    return (heading * 180/M_PI);
}

// returns the average of a float array.
float averagef(int length, float arr[100]) {
    float sum;
    float average;
    
    for(int i = 0; i < length; i++) {
        sum = sum + arr[i];
    }

    average = (sum / length);
    return(average);
}

// drives motors at a certain speed.
void drive(char cmd, int speed) {
}

// moves a servo to a certain degree.
void move(int servonum, int angle) {
    int pulse_wide, pulse_width;

    pulse_wide = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  
    // Control Motor
    servo.setPWM(servonum, 0, pulse_width);
}