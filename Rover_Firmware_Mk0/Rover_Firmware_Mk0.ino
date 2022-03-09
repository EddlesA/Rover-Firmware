// ENG1000 Group 6 MERE Prototype rover firmware MK0
// This firmware will run on an ARDUINO UNO R3
// and serve as a translation layer between bluetooth commands and
// arduino functions
// The aim of this firmware is to be adaptable and able to change
// the functionality of the rover without reflashing the ARDUINO

#include <Servo.h>
#include <SoftwareSerial.h>
#define PACKETSIZE 3

#define SERVOBR 'A'
#define SERVOBL 'a'

#define SERVOFR 'B'
#define SERVOFL 'b'


#define SERVOBB 'C'
#define SERVOFF 'c'

#define SERVOAA 'D'

#define SERVOLL 'E'
#define SERVORR 'e'



SoftwareSerial bluetooth(2,3);


Servo servoBL;
Servo servoFL;
Servo servoBR;
Servo servoFR;


void setup() { //Initialize Serial Monitor
    Serial.begin(9600);
    bluetooth.begin(9600); //Initialize Bluetooth Serial Port
    servoBL.attach(6);
    servoFL.attach(9);
    servoBR.attach(10);
    servoFR.attach(11);
}

void loop() {
    int buffer;
    char cmd;
    String str_sub_cmd = "";
    int sub_cmd;
    /* 
    Packet Style
    char : decimal (as characters so string varies in size
    e.g A123
    CMD = A (ASCII character)
    str_sub_cmd = "123" (ASCII string)
    sub_cmd = 123 (integer number)
    this allows us input the subcmd straight into the program :)
    */

    while(!bluetooth.available());
    while (buffer != EOF) {
        buffer = bluetooth.read();
        if (buffer == EOF) {
        } else if (isDigit(buffer)) {    
            str_sub_cmd += (char)buffer;
        
        } else {
            cmd = (char)buffer;
        
        }
        
    }

    sub_cmd = str_sub_cmd.toInt();
    
    debug(cmd, sub_cmd, str_sub_cmd);

    
    if (cmd == SERVOBL) {
        servoBL.write(sub_cmd);
    } else if (cmd == SERVOFL) {
        servoFL.write(sub_cmd);
    } else if (cmd == SERVOBR) {
        servoBR.write(sub_cmd);
    } else if (cmd == SERVOFR) {
        servoFR.write(sub_cmd);
    } else if (cmd == SERVOAA) {
        servoAA_write(sub_cmd);
    } else if (cmd == SERVOBB) {
        servoBB_write(sub_cmd);
    } else if (cmd == SERVOFF) {
        servoFF_write(sub_cmd);
    } 

    sub_cmd = 0;
}

void debug(char cmd, int sub_cmd, String str_sub_cmd) {
    Serial.print("cmd: ");
    Serial.println(cmd);
    Serial.print("sub_cmd: ");
    Serial.println(sub_cmd);
    Serial.print("String sub_cmd: ");
    Serial.println(str_sub_cmd);
}
void servoAA_write(int speed) {
    servoBL.write(speed);
    servoBR.write(speed);
    
    speed = abs(speed - 180);

    servoFL.write(speed);
    servoFR.write(speed);
}
void servoBB_write(int speed) {
    servoBL.write(speed);
    servoBR.write(speed);
}

void servoFF_write(int speed) {
    servoFL.write(speed);
    servoFR.write(speed);
}