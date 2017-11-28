#ifndef COMMANDS_H
#define COMMANDS_H

typedef enum
  {
    READ_DIGITAL
  , READ_ANALOG
  , WRITE_DIGITAL
  , WRITE_PWM
  , READ_ULTRASOUND
  , READ_LEFT_ENCODER
  , READ_RIGHT_ENCODER
  , WRITE_NEO_PIXEL
  , WRITE_NEO_PIXELS
  , READ_RFID
  , POSITION
  , ROTATE
  , READ_RIGHT_DISTANCE
  , READ_LEFT_DISTANCE
  , AT_POSITION
  } Commands;

int readDigital(int pin)
{
  pinMode(pin, INPUT);
  return digitalRead(pin);
};

int readAnalog(int pin)
{
  pinMode(pin, INPUT);
  return analogRead(pin);
};

void writeDigital(int pin,int value)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin,value==1.0);
};

void writePWM(int pin,int value)
{
  pinMode(pin, OUTPUT);
  analogWrite(pin,value);
};

#endif
