#include<StepperMulti.h>
#include<Servo.h>
#include<TimerOne.h>

#define GLITCH 1000

/* state machine set */
#define READY_FOR_CUP 0
#define DOWN 1
#define HOLD 2
#define READY_TO_GO 3
#define GO   4
#define BACK 5
#define STOP 6
#define FUNC 7

/* constant set */
#define FORWARD 0
#define BACKWARD 1

/* servo, stepper motor set */
Servo finger_servo;
Servo wrist_servo;

StepperMulti STEPPER_1(200, 23, 27, 25, 29);
StepperMulti STEPPER_2(200, 33, 37, 35, 39);
StepperMulti STEPPER_ELEVATOR(200, 8, 9, 10, 11);

/* interrupt, servo pin set */
const int interruptPin_1 = 20;
const int interruptPin_2 = 21;

const int finger_servoPin = 14;
const int wrist_servoPin = 15;

/* default variable set */
volatile int current_state = READY_FOR_CUP;
volatile int direct = FORWARD;
volatile int cup_size = 0;
int getByte = 0;

unsigned long start_time = 0, end_time = 0, last_end_time = 0;

int finger_default = 120;
int wrist_default = 120;
int StepsPerRevolution = 200;

void step_clear(){
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);

    digitalWrite(23, LOW);
    digitalWrite(25, LOW);
    digitalWrite(27, LOW);
    digitalWrite(29, LOW);

    digitalWrite(33, LOW);
    digitalWrite(35, LOW);
    digitalWrite(37, LOW);
    digitalWrite(39, LOW);
}

void default_set(){
    
    delay(100);
    wrist_servo.write(wrist_default);
    finger_servo.write(finger_default);
    delay(1000);

    for(int i = 0; i < StepsPerRevolution; i++){
        STEPPER_1.setStep(1);
        STEPPER_2.setStep(1);
        delayMicroseconds(2000);
    }

    step_clear();

    current_state = READY_FOR_CUP;
}

void go_forward(){
    while(current_state == GO){
        STEPPER_1.setStep(1);
        STEPPER_2.setStep(-1);
        delayMicroseconds(2000);
    }
}

void go_backward(){
    while(current_state == BACK){
        STEPPER_1.setStep(-1);
        STEPPER_2.setStep(1);
        delayMicroseconds(2000);
    }
}

void elevator_down(){
    for(int i = 0; i < cup_size; i++){
        STEPPER_ELEVATOR.setStep(-1);
        delayMicroseconds(5000);
    }

    delay(100);
    current_state = HOLD;
}

void elevator_up(){
    /* for hold cup, elevator down for 1 revolution */
    for(int i = 0; i < cup_size + StepsPerRevolution; i++){
        STEPPER_ELEVATOR.setStep(1);
        delayMicroseconds(5000);
    }

    delay(100);
}

void hold_cup(){
    delay(100);
    /* finger into */
    for(int i = 0; i < StepsPerRevolution; i++){
        STEPPER_1.setStep(-1);
        STEPPER_2.setStep(-1);
        delayMicroseconds(2000);
    }
    delay(500);

    /* for finger */
    finger_servo.write(finger_default - 60);
    delay(1000);

    /* for elevator set, elevator down 1 revolution */
    for(int i = 0; i < StepsPerRevolution; i++){
        STEPPER_ELEVATOR.setStep(-1);
        delayMicroseconds(5000);
    }
    delay(500);
    
    current_state = READY_TO_GO;
}

void ready_to_forward(){
    delay(100);
    /* for elevator up, arm have to go forward little */
    for(int i = 0; i < StepsPerRevolution * 3; i++){
        STEPPER_1.setStep(1);
        STEPPER_2.setStep(-1);
        delayMicroseconds(2000);
    }

    delay(500);
    /* then, elevator up */
    elevator_up();
    delay(100);
    current_state = GO;
}

void test_func(){
    delay(100);
    wrist_servo.write(wrist_default - 100);
    delay(3000);
    wrist_servo.write(wrist_default);
    delay(1000);
    current_state = BACK;
}

void state_change(){
    end_time = millis();

    if(end_time - start_time > GLITCH && last_end_time - end_time > 20000000){
        last_end_time = end_time;
        step_clear();

        if(current_state == GO){
            direct = BACKWARD;
        }else if(current_state == BACK){
            Serial.println(current_state);
            direct = FORWARD;
        }

        current_state = STOP;
    }

    start_time = end_time;
}

void setup(){
    Serial.begin(9600);

    STEPPER_1.setSpeed(200);
    STEPPER_2.setSpeed(200);
    STEPPER_ELEVATOR.setSpeed(150);

    wrist_servo.attach(wrist_servoPin);
    finger_servo.attach(finger_servoPin);

    //default_set();

    attachInterrupt(digitalPinToInterrupt(interruptPin_1), state_change, RISING);
    attachInterrupt(digitalPinToInterrupt(interruptPin_2), state_change, RISING);
}

void loop(){
    switch(current_state){
        case READY_FOR_CUP:
            step_clear();

            cup_size = 0;
            getByte = '0';
            if(Serial.available()){
                while(getByte == '0'){
                    getByte = Serial.read();
                    delay(500);
                    if(getByte == '1'){
                        cup_size = 8 * StepsPerRevolution + StepsPerRevolution / 2;
                        current_state = DOWN;
                    }else if(getByte == '2'){
                        cup_size = 9 * StepsPerRevolution ;
                        current_state = DOWN;
                    }else if(getByte == '3'){
                        cup_size = 600;
                        current_state = DOWN;
                    }else if(getByte < '0' || getByte > '3'){
                        delay(500);
                        Serial.println("Wrong Type");
                        getByte = '0';
                        break;
                    }
                }
            }
            break;

        case DOWN:
            elevator_down();
            break;

        case HOLD:
            hold_cup();
            break;

        case READY_TO_GO:
            ready_to_forward();
            break;

        case GO:
            go_forward();
            break;

        case BACK:
            go_backward();
            break;

        case STOP:
            if(direct == FORWARD){ // BACK to GO
                for(int i = 0; i < StepsPerRevolution / 4; i++){
                    STEPPER_1.setStep(1);
                    STEPPER_2.setStep(-1);
                    delayMicroseconds(2000);
                }
                delay(500);
                default_set();

            }else{ // GO to BACK
                for(int i = 0; i < StepsPerRevolution / 4; i++){
                    STEPPER_1.setStep(-1);
                    STEPPER_2.setStep(1);
                    delayMicroseconds(2000);
                }
                delay(500);
                current_state = FUNC;
            }
            break;

        case FUNC:
            test_func();
            break;
    }
}
