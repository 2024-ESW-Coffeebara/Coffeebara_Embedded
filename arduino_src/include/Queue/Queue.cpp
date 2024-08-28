#include "Arduino.h"
#include "Queue.h"

Queue::Queue(int Speed, int En_pin, int Motor_pin1, int Motor_pin2){
    this->Speed = Speed;
    this->En_pin = En_pin;
    this->Motor_pin1 = Motor_pin1;
    this->Motor_pin2 = Motor_pin2;

    pinMode(this->En_pin, OUTPUT);
    pinMode(this->Motor_pin1, OUTPUT);
    pinMode(this->Motor_pin2, OUTPUT);
}

void Queue::Cw(){
    analogWrite(this->En_pin, this->Speed);
    delay(100);
    digitalWrite(this->Motor_pin1, LOW);
    digitalWrite(this->Motor_pin2, HIGH);
    
    analogWrite(this->En_pin, this->Speed - 20);
}

void Queue::Ccw(){
    analogWrite(this->En_pin, this->Speed);
    delay(100);
    digitalWrite(this->Motor_pin1, HIGH);
    digitalWrite(this->Motor_pin2, LOW);

    analogWrite(this->En_pin, this->Speed - 20);
}

void Queue::Off(){
    analogWrite(this->En_pin, 0);
    digitalWrite(this->Motor_pin1, LOW);
    digitalWrite(this->Motor_pin2, LOW);
}