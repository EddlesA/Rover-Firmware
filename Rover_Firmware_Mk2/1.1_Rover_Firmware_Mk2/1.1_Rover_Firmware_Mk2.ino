// ENG1000 Group 6 MERE Prototype rover firmware MK1
// This firmware will run on an ARDUINO UNO R3
// and serve as a translation layer between bluetooth commands and
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

#define MOTORLPWM 10
#define MOTORLPIN 9
#define MOTORLINV 8
#define MOTORRPIN 7
#define MOTORRINV 6
#define MOTORRPWM 5

#define NULLCHAR (char)0
#define ARM (char)1
#define PAYLOAD (char)2
#define BUCKET (char)3

#define MOTORRR (char)4
#define MOTORAA (char)5
#define MOTORLL (char)6

#define COMPASS (char)7
#define CORDINATE (char)8
#define MUSIC (char)9

#define STARTMARKER 254
#define ENDMARKER 255
#define SPECIALBYTE 253
#define MAXMESSAGE 16

SoftwareSerial bluetooth(2,3);

byte bytesRecvd = 0;
byte dataSentNum = 0; // the transmitted value of the number of bytes in the package i.e. the 2nd byte received
byte dataRecvCount = 0;

byte dataRecvd[MAXMESSAGE]; 
byte dataSend[MAXMESSAGE];  
byte tempBuffer[MAXMESSAGE];

byte dataSendCount = 0; // the number of 'real' bytes to be sent to the PC
byte dataTotalSend = 0; // the number of bytes to send to PC taking account of encoded bytes

boolean inProgress = false;
boolean startFound = false;
boolean allReceived = false;

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(); //initalise servo board
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); //intalise compass

boolean compassOnline;
boolean servosOnline;

char cmd;
int subcmd;


void setup() { //Initialize Serial Monitor
    byte bin;
    bluetooth.begin(9600); //Initialize Bluetooth Serial Port
    pinMode(13, OUTPUT);
    //wait untill a device connects and sends a byte
    while (!bluetooth.available());
    bin = bluetooth.read();
    bluetooth.println("==========================================");
    bluetooth.println("                   WIRING");
    bluetooth.println("==========================================");
    bluetooth.println("Motor Driver");
    bluetooth.println("Arduino: 5 6 7 8 9 10");
    bluetooth.println("LN298A:  A 4 3 2 1 B ");

    bluetooth.println("Bluetooth Adaptor");


    //this goes to the device that connects to the rover
    bluetooth.println("==========================================");
    bluetooth.println("                   INFO");
    bluetooth.println("==========================================");
    bluetooth.println("Name: Rover");
    bluetooth.println("Firmware: Rover_Firmware_Mk2");
    bluetooth.println("Revision: 1.1");
    bluetooth.println("Flashed: 3/04/2021");
    bluetooth.println("Rover location: Not on Mars");

    bluetooth.println("Rover is hungry");

    //servo.begin();
    //servo.setPWMFreq(FREQUENCY);
    blinkLED(2);
    if(mag.begin()) {
        sensor_t sensor;
        mag.getSensor(&sensor);
        compassOnline = true;
    } else {
        compassOnline = false;
    }
    bluetooth.println(get_bearing(10));
    bluetooth.println(get_bearing(10));
    bluetooth.println(get_bearing(10));
    // This is the command that say's we're ready to begin
    bluetooth.println("Waiting...");
}

void loop() {
    byte buffer;

    if(bluetooth.available() > 0) {
		byte buffer = bluetooth.read();
		if (buffer == STARTMARKER) {
            digitalWrite(13, HIGH);
			bytesRecvd = 0; 
			inProgress = true;
            allReceived = false;
		}
			
		if(inProgress) {
			tempBuffer[bytesRecvd] = buffer;
			bytesRecvd ++;
		}

		if (buffer == ENDMARKER) {
            digitalWrite(13, LOW);
			inProgress = false;
			allReceived = true;
			dataSentNum = tempBuffer[1];
	
			decodeHighBytes();
            cmd = char(dataRecvd[0]);
            subcmd = BitShiftCombine(dataRecvd[1], dataRecvd[2]);
		}
	}
    
    if (cmd == ARM) {
        //move(SERVOARM, sub_cmd);
    } else if (cmd == PAYLOAD) {
        //move(SERVOPAYLOAD, sub_cmd);
    } else if (cmd == BUCKET) {
        //move(SERVOBUCKETL, sub_cmd);
        //move(SERVOBUCKETR, sub_cmd);
    } else if (cmd == MOTORAA) {
        motorAA(subcmd);
    } else if (cmd == MOTORLL) {
        motorLL(subcmd);
    } else if (cmd == MOTORRR) {
        motorRR(subcmd);
    }

}
//runs left motors
void motorLL(int speed) {
    if (speed > 256) { // rover go forward
        digitalWrite(MOTORLPIN, LOW);
        digitalWrite(MOTORLINV, HIGH);
        analogWrite(MOTORLPWM, decode_for_speed(speed));
    } else if (speed < 256) { // Rover go back
        digitalWrite(MOTORLPIN, HIGH);
        digitalWrite(MOTORLINV, LOW);
        analogWrite(MOTORLPWM, decode_for_speed(speed));
    } else { // Rover no go
        digitalWrite(MOTORLPIN, LOW);
        digitalWrite(MOTORLINV, LOW);
    }
}
//runs right motors
void motorRR(int speed) {
    if (speed > 256) { // rover go forward
        digitalWrite(MOTORRPIN, LOW);
        digitalWrite(MOTORRINV, HIGH);
        analogWrite(MOTORRPWM, decode_for_speed(speed));
    } else if (speed < 256) { // Rover go back
        digitalWrite(MOTORRPIN, HIGH);
        digitalWrite(MOTORRINV, LOW);
        analogWrite(MOTORRPWM, decode_for_speed(speed));
    } else { // Rover no go
        digitalWrite(MOTORRPIN, LOW);
        digitalWrite(MOTORRINV, LOW);
    }
}

//runs all motors
void motorAA(int speed) {
    if (speed > 256) { // rover go forward
        digitalWrite(MOTORLPIN, LOW);
        digitalWrite(MOTORLINV, HIGH);
        digitalWrite(MOTORRPIN, LOW);
        digitalWrite(MOTORRINV, HIGH);
        analogWrite(MOTORLPWM, decode_for_speed(speed));
        analogWrite(MOTORRPWM, decode_for_speed(speed));

    } else if (speed < 256) { // Rover go back
        digitalWrite(MOTORLPIN, HIGH);
        digitalWrite(MOTORLINV, LOW);
        digitalWrite(MOTORRPIN, HIGH);
        digitalWrite(MOTORRINV, LOW);
        analogWrite(MOTORLPWM, decode_for_speed(speed));
        analogWrite(MOTORRPWM, decode_for_speed(speed));
        
       
    } else { // Rover no go
        digitalWrite(MOTORLPIN, LOW);
        digitalWrite(MOTORLINV, LOW);
        digitalWrite(MOTORRPIN, LOW);
        digitalWrite(MOTORRINV, LOW);
        
    }
}

//blinks LED num of times
void blinkLED(byte numBlinks) {
    for (byte n = 0; n < numBlinks; n ++) {
      digitalWrite(13, HIGH);
      delay(200);
      digitalWrite(13, LOW);
      delay(200);
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

void decodeHighBytes() {

    //  copies to dataRecvd[] only the data bytes i.e. excluding the marker bytes and the count byte
    //  and converts any bytes of 253 etc into the intended numbers
    //  Note that bytesRecvd is the total of all the bytes including the markers
    dataRecvCount = 0;
    for (int n = 2; n < bytesRecvd - 1 ; n++) { // 2 skips the start marker and the count byte, -1 omits the end marker
        byte buffer = tempBuffer[n];
        if (buffer == SPECIALBYTE) {
            n++;
            buffer = buffer + tempBuffer[n];
        }
        dataRecvd[dataRecvCount] = buffer;
        dataRecvCount ++;
    }
}

//====================

void SEND_it() { // haha funny function name

      // expects to find data in dataSend[]
      //   uses encodeHighBytes() to copy data to tempBuffer
      //   sends data to PC from tempBuffer
    encodeHighBytes();

    bluetooth.write(STARTMARKER);
    bluetooth.write(dataSendCount);
    bluetooth.write(tempBuffer, dataTotalSend);
    bluetooth.write(ENDMARKER);
}

//============================

void encodeHighBytes() {
  // Copies to temBuffer[] all of the data in dataSend[]
  //  and converts any bytes of 253 or more into a pair of bytes, 253 0, 253 1 or 253 2 as appropriate
  dataTotalSend = 0;
  for (byte n = 0; n < dataSendCount; n++) {
    if (dataSend[n] >= SPECIALBYTE) {
      tempBuffer[dataTotalSend] = SPECIALBYTE;
      dataTotalSend++;
      tempBuffer[dataTotalSend] = dataSend[n] - SPECIALBYTE;
    }
    else {
      tempBuffer[dataTotalSend] = dataSend[n];
    }
    dataTotalSend++;
  }
}

/* debug printout.
void debug(char cmd, int sub_cmd, String str_sub_cmd) {
    Serial.print("cmd: ");
    Serial.println(cmd);
    Serial.print("sub_cmd: ");
    Serial.println(sub_cmd);
    Serial.print("String sub_cmd: ");
    Serial.println(str_sub_cmd);
}
*/

// gets comapssbearings with a sample amount between 0-100.
uint16_t get_bearing(int samples) {
    float bearings[100];
    float heading;
    if (compassOnline) {
        for (int i = 0; i < samples; i++) {
            // get a new sensor event
            sensors_event_t event; 
            mag.getEvent(&event);
            // write heading to array
            bearings[i] = atan2(event.magnetic.x, event.magnetic.z);
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

// moves a servo to a certain degree.
void move(int servonum, int angle) {
    int pulse_wide, pulse_width;

    pulse_wide = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  
    // Control Motor
    servo.setPWM(servonum, 0, pulse_width);
}

int BitShiftCombine(byte x_high, byte x_low) {
  int combined;
  combined = x_high;              // send x_high to rightmost 8 bits
  combined = combined<<8;         // shift x_high over to leftmost 8 bits
  combined |= x_low;              // logical OR keeps x_high intact in combined and fills in 
  return combined;                // rightmost 8 bits
}

void Send_bearing(){
    int bearing = get_bearing(10);
    buildcommand(COMPASS, bearing);
    SEND_it();
}

void buildcommand(byte cmd, uint16_t subcmd) {
    byte bytes[2];
    dataSend[0] = cmd;
    dataSend[1] = *((uint8_t*)&(subcmd)+1); // high byte 
    dataSend[2] = *((uint8_t*)&(subcmd)+0); // low byte 
}

void send_XY() {

}
