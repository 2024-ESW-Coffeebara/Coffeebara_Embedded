#include "Arduino.h"
#include "Waterpump.h"

Waterpump::Waterpump(int En_pin, int Motor_pin1, int Motor_pin2){
    this->En_pin = En_pin;
    this->Motor_pin1 = Motor_pin1;
    this->Motor_pin2 = Motor_pin2;

    pinMode(this->En_pin, OUTPUT);
    pinMode(this->Motor_pin1, OUTPUT);
    pinMode(this->Motor_pin2, OUTPUT);
}

void Waterpump::Ready(){
    analogWrite(this->En_pin, 200);

    digitalWrite(this->Motor_pin1, LOW);
    digitalWrite(this->Motor_pin2, HIGH);
}

void Waterpump::On(){
    analogWrite(this->En_pin, 255);

    digitalWrite(this->Motor_pin1, LOW);
    digitalWrite(this->Motor_pin2, HIGH);
}

void Waterpump::Off(){
    digitalWrite(this->Motor_pin1, LOW);
    digitalWrite(this->Motor_pin2, LOW);
}