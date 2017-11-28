#include <Arduino.h>
#include <SoftwareSerial.h>
#include "PID.h"
#include "Encoder.h"
#include "Commands.h"
#include "Movement.h"
#include "EarthToMars.h"
#include "Ultrasound.h"

#define MAX_MESSAGE_LENGTH 50
#define MAX_RESPONSE_LENGTH 20

ENCODER R_ENC;
ENCODER L_ENC;

String inputMessage = "";
boolean inputMessageComplete = false;

void setup()
{
	// start serial port at 9600 bps:
	Serial.begin(115200);
	// reserve 200 bytes for the inputMessage:
	inputMessage.reserve(MAX_MESSAGE_LENGTH);
	setupETM();
  
	R_ENC.init(RIGHT_ENCODER_A,RIGHT_ENCODER_B,R_EncHandler);
	L_ENC.init(LEFT_ENCODER_A,LEFT_ENCODER_B,L_EncHandler);
	initMotors();
	initPID();
}

void loop()
{
  serialRecieve();
  checkRFID();
  
 if(get_movement()){
   calcPID();
   }
  
  if(!inputMessageComplete)
    return;
    
  //a message has been recieved
  //convert String to char array for parsing
  char inputMessageCharArray[MAX_MESSAGE_LENGTH];
  inputMessage.toCharArray(inputMessageCharArray, MAX_MESSAGE_LENGTH);
  
  //declare variables and parsing
  int command;
  int pin, value;
  sscanf(inputMessageCharArray,"%i,%i,%i",&command, &pin, &value);
  if(!checkPinNumber(&pin))
  { 
    inputMessage="";
    inputMessageComplete=false;
    return;
  }

  //call the method corresponding to the command
  String answer = "";
  answer.reserve(MAX_RESPONSE_LENGTH);
  switch (command)
  {
  case READ_RFID:
       answer += getRFID();
       break;
  case READ_DIGITAL:
       answer =+ readDigital(pin) ? "1" : "0";
       break; 
  case READ_ANALOG:
       char analogValue[5];
       sprintf(analogValue,"%i",readAnalog(pin));
       answer =+ analogValue;
       break; 
  case WRITE_DIGITAL:
       writeDigital(pin,value);
       break; 
  case WRITE_PWM:
       writePWM(pin,value);
       break; 
  case READ_ULTRASOUND:
       char distanceValue[5];
       sprintf(distanceValue,"%i",readUltrasound(pin));
       answer =+ distanceValue;
       break;
  case POSITION:
	  movePos(value,pin);
	  break;
  case ROTATE:
	  moveRotate(value,pin);
	  break;
  case READ_RIGHT_DISTANCE:
	  char rightDistance[5];
	  sprintf(rightDistance,"%i",getDistanceRight());
	  answer =+ rightDistance;
	  break;
  case READ_LEFT_DISTANCE:
	  char leftDistance[5];
	  sprintf(leftDistance,"%i",getDistanceLeft());
	  answer =+ leftDistance;
	  break;
  case AT_POSITION:
	  char arrived[1];
	  sprintf(arrived,"%i",get_at_position());
	  answer =+ arrived;
	  break;
  default:
       break;
  }
  Serial.print(answer+'\n');
 
  inputMessageComplete = false;
  inputMessage = "";
}

void serialRecieve() {
  while (Serial.available()) 
  {
    char inChar = (char)Serial.read(); 
    inputMessage += inChar;
    if (inChar == '\n')
      inputMessageComplete = true;
  }
}

bool checkPinNumber(int *pin)
{
  int analogPins[6][2] = { {A0,500}
                         ,{A1,501}
                         ,{A2,502}
                         ,{A3,503}
                         ,{A4,504}
                         ,{A5,505}
                        };
  if(*pin<0) 
    return false;
  else if(*pin>13 && *pin<500)
    return false;
  else if(*pin > 505)
    return false;
  
  for(int i=0;i<6;i++)
  {
	if(*pin == analogPins[i][1])
	*pin = analogPins[i][0];
}
return true;
}
