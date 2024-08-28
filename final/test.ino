// 임의의 로드셀 무게 추가 -> python으로 전송
#include<Constants.h>
#include<StepperMulti.h>
#include<Servo.h>
#include<TimerOne.h>
#include<CUP.h>
#include<Waterpump.h>
#include<Queue.h>
#include<Ultrasonic.h>
#include<HX711.h>

Waterpump waterpump(WATERPUMP_EN, WATERPUMP_PIN_1, WATERPUMP_PIN_2);

StepperMulti VERTICAL_STEPPER(SPR, 25, 29, 27, 31);
StepperMulti HORIZONTAL_STEPPER(SPR, 24, 30, 28, 26);

Queue queue(QUEUE_SPEED, QUEUE_MOTOR_EN, QUEUE_MOTOR_PIN_1, QUEUE_MOTOR_PIN_2);

CUP CurrentCUP(0, 0, 0);
CUP NextCUP(0, 0, 0);

Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);

HX711 LoadCellScale;

Servo HORIZONTAL_STEPPER_wrist;
Servo HORIZONTAL_STEPPER_finger;
Servo VERTICAL_STEPPER_finger;

Servo remove_lid_servo;

Servo small_cup_servo;
Servo regular_cup_servo;
Servo large_cup_servo;

volatile int reset_horizontal_direct;

volatile float current_horizontal_pos;
volatile float current_vertical_pos;

volatile float next_horizontal_pos;
volatile float next_vertical_pos;

volatile int horizontal_pos_cup;

volatile int cup_size, cup_size_entrance, holder_exist;
volatile int cup_cnt[5];

volatile int main_current_state = RESET;
volatile int queue_current_state = QUEUE_STOP;

volatile int main_last_state;
volatile int queue_last_state;

volatile int current_cup_size = 0;
volatile int current_entrance_size = 0;
volatile int current_holder_exist = 0;

volatile int next_cup_size = 0;
volatile int next_entrance_size = 0;
volatile int next_holder_exist = 0;

volatile int getByte;

volatile int end_time = 0, start_time = 0;
volatile int reset_flag = 0;
volatile int no_interrupt = 0;

volatile int all_stop = 0;
volatile byte tx_rx_data = 0;

void step_clear(){
    digitalWrite(24, LOW);
    digitalWrite(26, LOW);
    digitalWrite(28, LOW);
    digitalWrite(30, LOW);

    digitalWrite(25, LOW);
    digitalWrite(27, LOW);
    digitalWrite(29, LOW);
    digitalWrite(31, LOW);
    
    digitalWrite(40, LOW);
    digitalWrite(42, LOW);
    digitalWrite(44, LOW);
    digitalWrite(46, LOW);
}

int  cm2step(float cm){
    return (SPR / CmPerCycle * cm);
}

void cleaning_cup(){
    if(cup_cnt[1] >= small_cnt_threshold || cup_cnt[2] >= regular_cnt_threshold || cup_cnt[3] >= large_cnt_threshold){
        if(cup_cnt[1] >= small_cnt_threshold){
          small_cup_servo.write(cleaning_cup_angle);
          cup_cnt[1] = 0;
        }
        if(cup_cnt[2] >= regular_cnt_threshold){
          regular_cup_servo.write(cleaning_cup_angle);
          cup_cnt[2] = 0;
        }
        if(cup_cnt[3] >= large_cnt_threshold){
          large_cup_servo.write(cleaning_cup_angle);
          cup_cnt[3] = 0;
        }
        delay(1500);
    }

    small_cup_servo.write(default_cup_angle);
    regular_cup_servo.write(default_cup_angle);
    large_cup_servo.write(default_cup_angle);
}

void wash_post_process(){
    HORIZONTAL_STEPPER_wrist.write(HORIZONTAL_STEPPER_wrist_wash - 30);
    delay(300);
    HORIZONTAL_STEPPER_wrist.write(HORIZONTAL_STEPPER_wrist_wash);
    delay(300);
    HORIZONTAL_STEPPER_wrist.write(HORIZONTAL_STEPPER_wrist_wash - 30);
    delay(300);
    HORIZONTAL_STEPPER_wrist.write(HORIZONTAL_STEPPER_wrist_wash);
    delay(300);
    HORIZONTAL_STEPPER_wrist.write(HORIZONTAL_STEPPER_wrist_wash - 30);
    delay(300);
    HORIZONTAL_STEPPER_wrist.write(HORIZONTAL_STEPPER_wrist_wash);
    delay(300);
    HORIZONTAL_STEPPER_wrist.write(HORIZONTAL_STEPPER_wrist_wash - 30);
    delay(300);
    HORIZONTAL_STEPPER_wrist.write(HORIZONTAL_STEPPER_wrist_wash);
}

void HORIZONTAL_STEPPER_move(){
    int steps = abs(cm2step(current_horizontal_pos - next_horizontal_pos));

    if(current_horizontal_pos > next_horizontal_pos){
        for(int i = 0; i < steps; i++){
            HORIZONTAL_STEPPER.setStep(1);
            delayMicroseconds(HORIZONTAL_STEPPER_DELAY);
        }
    }else{
        for(int i = 0; i < steps; i++){
            HORIZONTAL_STEPPER.setStep(-1);
            delayMicroseconds(HORIZONTAL_STEPPER_DELAY);
        }
    }

    current_horizontal_pos = next_horizontal_pos;
}

void VERTICAL_STEPPER_move(){
    int steps = abs(cm2step(current_vertical_pos - next_vertical_pos));

    if(next_vertical_pos == vertical_pos_default){
        steps = steps + 50;
    }

    if(current_vertical_pos > next_vertical_pos){ // up
        for(int i = 0; i < steps; i++){
            VERTICAL_STEPPER.setStep(1);
            delayMicroseconds(VERTICAL_STEPPER_DELAY);
        }
    }else{ // down
        for(int i = 0; i < steps; i++){ 
            VERTICAL_STEPPER.setStep(-1);
            delayMicroseconds(VERTICAL_STEPPER_DELAY);
        }
    }

    current_vertical_pos = next_vertical_pos;
}

void VERTICAL_STEPPER_finger_move(int src, int dst){
    if(src < dst){
        for(int i = src; i < dst; i++){
            VERTICAL_STEPPER_finger.write(i);
            delay(10);
        }
    }else{
        for(int i = src; i > dst; i--){
            VERTICAL_STEPPER_finger.write(i);
            delay(10);
        }
    }
}

void remove_lid_servo_in(){
    remove_lid_servo.write(REMOVE_LID_ANGLE[current_entrance_size]);
}

void remove_lid_servo_reset(){
    remove_lid_servo.write(REMOVE_LID_ANGLE[0]);
}

void servo_attach(){
    HORIZONTAL_STEPPER_wrist.attach(HORIZONTAL_STEPPER_wrist_pin);
    HORIZONTAL_STEPPER_finger.attach(HORIZONTAL_STEPPER_finger_pin);
    VERTICAL_STEPPER_finger.attach(VERTICAL_STEPPER_finger_pin);

    remove_lid_servo.attach(remove_lid_servo_pin);
    small_cup_servo.attach(small_cup_servo_pin);
    regular_cup_servo.attach(regular_cup_servo_pin);
    large_cup_servo.attach(large_cup_servo_pin);
}

void servo_detach(){
    HORIZONTAL_STEPPER_wrist.detach();
    HORIZONTAL_STEPPER_finger.detach();

    remove_lid_servo.detach();
    small_cup_servo.detach();
    regular_cup_servo.detach();
    large_cup_servo.detach();
}


void queue_positioning(int speed){
    no_interrupt = 0;
    analogWrite(QUEUE_MOTOR_EN, speed + 20);
    digitalWrite(QUEUE_MOTOR_PIN_1, LOW);
    digitalWrite(QUEUE_MOTOR_PIN_2, HIGH);
    
    no_interrupt = 0;
    delay(150);
//    if(main_current_state == RESET_SERVO) delay(200);
//    else if(queue_current_state == QUEUE_STOP) delay(150);
            
    no_interrupt = 0;
    while(!no_interrupt){
        analogWrite(QUEUE_MOTOR_EN, speed - 20);
        digitalWrite(QUEUE_MOTOR_PIN_1, HIGH);
        digitalWrite(QUEUE_MOTOR_PIN_2, LOW);
    }

    queue.Off();
    delay(300);
    no_interrupt = !no_interrupt;
}

void horizontal_step_hold(){
    digitalWrite(24, HIGH);
    digitalWrite(26, LOW);
    digitalWrite(28, LOW);
    digitalWrite(30, HIGH);
}

void vertical_step_hold(){
    digitalWrite(25, HIGH);
    digitalWrite(27, LOW);
    digitalWrite(29, LOW);
    digitalWrite(31, HIGH);
}

void reset_state_change(){
    end_time = millis();
    step_clear();
    if(end_time - start_time > GLITCH){
        

        if(main_current_state == RESET_HORIZONTAL_INTERRUPT){
            main_current_state = RESET_HORIZONTAL_DEFAULT;
        }else if(main_current_state == RESET_VERTICAL_DEFAULT){
            main_current_state = STARTING;
        }else if(main_current_state == HORIZONTAL_HOLD_CUP){
            main_current_state = HORIZONTAL_WASTE_CUP;
        }
    }
    start_time = end_time;
}

void reset_interrupt(){
    end_time = millis();
    step_clear();
    if(end_time - start_time > GLITCH){
        
        if(main_current_state != ONLY_WAITING){
            reset_flag = 1;
            main_current_state = ONLY_WAITING;
        }else{
            main_current_state = RESET;
        }
    }
    start_time = end_time;
}

void queue_state_change(){
    end_time = millis();
    queue.Off();
    delayMicroseconds(1000);

    if(end_time - start_time > GLITCH && !no_interrupt && (queue_current_state == QUEUE_STOP || main_current_state == RESET_SERVO)) no_interrupt = !no_interrupt; 

    if(main_current_state == RESET_QUEUE) main_current_state = RESET_SERVO;
    else{
        if(queue_current_state == QUEUE_GO) queue_current_state = QUEUE_STOP;
        else if(queue_current_state == QUEUE_STOP) queue_current_state = QUEUE_GO;
    }
    start_time = end_time;
}

void queue_cup_init(){
    if(no_interrupt) return;

    end_time = millis();
    if(end_time - start_time > GLITCH){
        queue_current_state = QUEUE_GO;
    }
    start_time = end_time;
}

void setup(){
    Serial.begin(9600);

    HORIZONTAL_STEPPER.setSpeed(HORIZONTAL_STEPPER_SPEED);
    VERTICAL_STEPPER.setSpeed(VERTICAL_STEPPER_SPEED);
    
    servo_attach();

    queue_current_state = QUEUE_READY_CUP;

    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);

    LoadCellScale.begin(LOADCELL_DATA_PIN, LOADCELL_CLK_PIN);
    LoadCellScale.set_scale(18.575);              
    LoadCellScale.tare(20);

    attachInterrupt(digitalPinToInterrupt(RESET_PIN), reset_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(HORIZONTAL_STEPPER_reset_pin), reset_state_change, RISING);
    attachInterrupt(digitalPinToInterrupt(VERTICAL_STEPPER_reset_pin), reset_state_change, RISING);
    attachInterrupt(digitalPinToInterrupt(QUEUE_INTERRUPT_PIN), queue_state_change, RISING);
    attachInterrupt(digitalPinToInterrupt(QUEUE_INITIALIZE_PIN), queue_cup_init, RISING);
}

void loop(){
    tx_rx_data = 0x00;
    tx_rx_data = main_current_state << 3;

    Serial.println(tx_rx_data);

    step_clear();

//    if(main_current_state != STARTING && main_current_state == main_last_state) main_current_state = RESET;
    
    main_last_state = main_current_state;

    switch(main_current_state){
        case ONLY_WAITING:
            while(main_current_state == ONLY_WAITING){}
        case RESET:
            reset_flag = 0;
            for(int i = 0; i < 4; i++) cup_cnt[i] = 0;

            main_current_state = RESET_QUEUE;
            break;

        case RESET_QUEUE:
            while(main_current_state == RESET_QUEUE){
                analogWrite(QUEUE_MOTOR_EN, 80);
                digitalWrite(QUEUE_MOTOR_PIN_1, LOW);
                digitalWrite(QUEUE_MOTOR_PIN_2, HIGH);
            }
            break;

        case RESET_SERVO:
        
            delay(50);

            queue_positioning(QUEUE_SPEED);
            
            analogWrite(QUEUE_MOTOR_EN, 80);
            digitalWrite(QUEUE_MOTOR_PIN_1, LOW);
            digitalWrite(QUEUE_MOTOR_PIN_2, HIGH);
            delay(100);

            queue.Off();
            
            delay(1500);
        
            queue_current_state = QUEUE_READY_CUP;
            cleaning_cup();
            remove_lid_servo_reset();
            HORIZONTAL_STEPPER_wrist.write(HORIZONTAL_STEPPER_wrist_default);
            HORIZONTAL_STEPPER_finger.write(HORIZONTAL_STEPPER_finger_default);
            VERTICAL_STEPPER_finger_move(VERTICAL_STEPPER_finger_minimum, VERTICAL_STEPPER_finger_default);

            delay(3000);

            servo_detach();

            main_current_state = RESET_HORIZONTAL_INTERRUPT;
            break;

        case RESET_HORIZONTAL_INTERRUPT:
            current_horizontal_pos = horizontal_pos_reset;
            while(main_current_state == RESET_HORIZONTAL_INTERRUPT){
                HORIZONTAL_STEPPER.setStep(1);
            }

            break;

        case RESET_HORIZONTAL_DEFAULT:
            next_horizontal_pos = horizontal_pos_default;
            HORIZONTAL_STEPPER_move();

            main_current_state = RESET_VERTICAL_DEFAULT;
            break;

        case RESET_VERTICAL_DEFAULT:
            for(int i = 0; i < 100; i++){
                VERTICAL_STEPPER.setStep(-1);
                delayMicroseconds(VERTICAL_STEPPER_DELAY);
            }

            current_vertical_pos = vertical_pos_default;
            while(main_current_state == RESET_VERTICAL_DEFAULT){
                VERTICAL_STEPPER.setStep(1);
            }
            break;

        case STARTING:
            
            switch(queue_current_state){
                case QUEUE_GO:
                    vertical_step_hold();
                    CurrentCUP.modifyCupInfo(NextCUP.getCupSize(), NextCUP.getEntranceSize(), NextCUP.getExistHolder());

                    current_cup_size = CurrentCUP.getCupSize();
                    current_entrance_size = CurrentCUP.getEntranceSize();
                    current_holder_exist = CurrentCUP.getExistHolder();
                    
                    if(!reset_flag){
                        if(current_cup_size){
                            servo_attach();
                            VERTICAL_STEPPER_finger_move(VERTICAL_STEPPER_finger_default, VERTICAL_STEPPER_finger_ready_cup[current_entrance_size]);
                        }                        

                        while(queue_current_state == QUEUE_GO && !reset_flag){
                            digitalWrite(QUEUE_MOTOR_PIN_1, LOW);
                            digitalWrite(QUEUE_MOTOR_PIN_2, HIGH);

                            if(current_cup_size == 0x03){
                                for(int i = QUEUE_SPEED + 50; queue_current_state == QUEUE_GO && i > QUEUE_SPEED; i -= 10){
                                  analogWrite(QUEUE_MOTOR_EN, i);
                                  delay(100);
                                }
                            }else{
                                for(int i = QUEUE_SPEED + 20; queue_current_state == QUEUE_GO && i > QUEUE_SPEED; i -= 10){
                                  analogWrite(QUEUE_MOTOR_EN, i);
                                  delay(100);
                                }
                            }
                        }
                    }
                    break;

                case QUEUE_STOP:
                    vertical_step_hold();
                    delay(200);
                    
                    if(current_cup_size == 0x03){
                        queue_positioning(QUEUE_SPEED + 10);
                    }else queue_positioning(QUEUE_SPEED);

                    analogWrite(QUEUE_MOTOR_EN, QUEUE_SPEED - 20);
                    digitalWrite(QUEUE_MOTOR_PIN_1, LOW);
                    digitalWrite(QUEUE_MOTOR_PIN_2, HIGH);
                    delay(100);

                    queue.Off();
                    
                    delay(500);
                    // jetson tx2에 ready signal 보냄
                    tx_rx_data = 1 << 7;
                    Serial.println(tx_rx_data);

                    delay(500);

                    // jetson tx2에서 컵 정보 받아냄
                    // 일단 0 ~ 7로 하여 총 6개의 컵 ( 홀더 포함 ) 기준임
                    // 0을 입력받는 경우는 컵이 아닌 경우.. python에서 처리해서 넘겨줄것
                    tx_rx_data = 0xFF;

                    while(tx_rx_data == 0xFF && !reset_flag){
                        if(Serial.available() > 0){
                            tx_rx_data = Serial.read();

                            if(tx_rx_data % 8){
                                tx_rx_data = 0xFF;
                            }else{
                                tx_rx_data = tx_rx_data >> 3;
                                byte tmp = tx_rx_data & 0x03;
                                next_entrance_size = tmp;

                                // tmp : entrance 크기에 따른 size
                                // 01 : small, 10 : regular, 11 : large
                                tx_rx_data = tx_rx_data >> 2;
                                tmp = tx_rx_data & 0x03;
                                next_cup_size = tmp;

                                tx_rx_data = tx_rx_data >> 2;
                                tmp = tx_rx_data & 0x01;
                                next_holder_exist = tmp;
                                
                                NextCUP.modifyCupInfo(next_cup_size, next_entrance_size, next_holder_exist);
                            }

                            delay(50);
                        }
                    }

                    if(!reset_flag){
                        if(current_cup_size){
                            cup_cnt[current_cup_size]++;

                            if(next_cup_size){
                                queue_current_state = QUEUE_GO;
                            }else{
                                queue_current_state = QUEUE_READY_CUP;
                            }

                            main_current_state = VERTICAL_HOLD_CUP;
                            break;
                        }else{
                            // 현재 처리중인 컵이 없다면 python 입장에서도 response를 기다리면안됨
                            // 즉, current가 없다면 python으로 0x1000_0000을 전달하여 READY로 돌아가도록 해야함
                            tx_rx_data = 1 << 7;
                            Serial.println(tx_rx_data);

                            if(next_cup_size){
                                queue_current_state = QUEUE_GO;
                            }else{
                                queue_current_state = QUEUE_READY_CUP;
                            }

                            break;
                        }
                    }

                    break;

                case QUEUE_READY_CUP:
                    long duration = 0;
                    int distance = 999;
                    
                    while(!reset_flag && distance > 7 && queue_current_state == QUEUE_READY_CUP){
                        digitalWrite(TRIG_PIN, LOW);
                        delayMicroseconds(2);
                        digitalWrite(TRIG_PIN, HIGH);
                        delayMicroseconds(10);
                        digitalWrite(TRIG_PIN, LOW);

                        duration = pulseIn(ECHO_PIN, HIGH);

                        distance = duration * 0.034 / 2;
                        
                        delay(100);
                    }
                    delay(1500);
                    queue_current_state = QUEUE_GO;
                    break;
            }
            break;
            
        
        case VERTICAL_HOLD_CUP:
            delay(300);
            vertical_step_hold();
            
            remove_lid_servo_in();

            VERTICAL_STEPPER_finger_move(VERTICAL_STEPPER_finger_ready_cup[current_entrance_size], VERTICAL_STEPPER_finger_remove_lid[current_entrance_size]);
            delay(500);

            if(!reset_flag){
                main_current_state = VERTICAL_REMOVE_LID_HOLDER;
            }
            break;

        case VERTICAL_REMOVE_LID_HOLDER:
            next_vertical_pos = vertical_pos_remove_lid;
            VERTICAL_STEPPER_move();
            delay(500);
            
            remove_lid_servo_reset();
            delay(500);

            next_vertical_pos = vertical_pos_holder;
            VERTICAL_STEPPER_move();

            if(!reset_flag){
                if(current_holder_exist){
                    main_current_state = HORIZONTAL_HOLD_HOLDER;
                }else{
                    main_current_state = HORIZONTAL_HOLD_CUP;
                }
            }
            break;

        case HORIZONTAL_HOLD_HOLDER:
            horizontal_step_hold();
            vertical_step_hold();
            
            VERTICAL_STEPPER_finger_move(VERTICAL_STEPPER_finger_remove_lid[current_entrance_size], VERTICAL_STEPPER_finger_ready_cup[current_entrance_size]);
            delay(500);
            HORIZONTAL_STEPPER_finger.write(HORIZONTAL_STEPPER_finger_holder);
            delay(500);

            next_vertical_pos = vertical_pos_remove_holder;
            VERTICAL_STEPPER_move();

            if(!reset_flag){
                main_current_state = HORIZONTAL_PLACE_HOLDER;
            }
            break;

        case HORIZONTAL_PLACE_HOLDER:
            HORIZONTAL_STEPPER_finger.write(HORIZONTAL_STEPPER_finger_default);
            delay(500);

            if(!reset_flag){
                main_current_state = HORIZONTAL_HOLD_CUP;
            }
            break;

        case HORIZONTAL_HOLD_CUP:
            next_vertical_pos = vertical_pos_cup;
            VERTICAL_STEPPER_move();

            HORIZONTAL_STEPPER_finger.write(HORIZONTAL_STEPPER_finger_cup);
            delay(500);
            VERTICAL_STEPPER_finger_move(VERTICAL_STEPPER_finger_ready_cup[current_entrance_size], VERTICAL_STEPPER_finger_default);
            delay(1000);
            
            HORIZONTAL_STEPPER_finger.write(HORIZONTAL_STEPPER_finger_ready_cup[current_cup_size]);
            delay(500);
            HORIZONTAL_STEPPER_finger.write(HORIZONTAL_STEPPER_finger_cup);
            delay(500);

            HORIZONTAL_STEPPER_finger.write(HORIZONTAL_STEPPER_finger_default);
            delay(1500);
            current_vertical_pos = vertical_pos_default;
            while(main_current_state == HORIZONTAL_HOLD_CUP && !reset_flag){
                VERTICAL_STEPPER.setStep(1);
            }
            break;

        case HORIZONTAL_WASTE_CUP:
            main_current_state = STARTING;
            break;

            
            next_horizontal_pos = horizontal_pos_waste;
            HORIZONTAL_STEPPER_move();

            HORIZONTAL_STEPPER_wrist.write(HORIZONTAL_STEPPER_wrist_waste);
            delay(1000);

            if(!reset_flag){
                main_current_state = HORIZONTAL_WASH_CUP;
            }
            break;

        case HORIZONTAL_WASH_CUP:
            HORIZONTAL_STEPPER_wrist.write(HORIZONTAL_STEPPER_wrist_wash);

            next_horizontal_pos = horizontal_pos_wash;
            HORIZONTAL_STEPPER_move();

            waterpump.On();

            delay(2000);
            waterpump.Off();
            delay(300);

            next_horizontal_pos = horizontal_pos_waste;
            HORIZONTAL_STEPPER_move();

            wash_post_process();

            if(!reset_flag){
                main_current_state = HORIZONTAL_LOAD_CUP;
            }
            break;

        case HORIZONTAL_LOAD_CUP:
        
            LoadCellScale.power_up();
            delay(300);
            int waste_weight = (LoadCellScale.get_units() / 1000);
            LoadCellScale.power_down();

            HORIZONTAL_STEPPER_wrist.write(HORIZONTAL_STEPPER_wrist_load);

            horizontal_pos_cup = horizontal_pos_cup_size[current_cup_size];
            next_horizontal_pos = horizontal_pos_cup;
            HORIZONTAL_STEPPER_move();

            HORIZONTAL_STEPPER_finger.write(HORIZONTAL_STEPPER_finger_cup + 20);
            delay(1500);

            HORIZONTAL_STEPPER_finger.write(HORIZONTAL_STEPPER_finger_default);
            delay(1000);

            next_horizontal_pos = horizontal_pos_default;
            HORIZONTAL_STEPPER_move();

            HORIZONTAL_STEPPER_wrist.write(HORIZONTAL_STEPPER_wrist_default);
            delay(500);

            cleaning_cup();

            delay(200);

            Serial.println(waste_weight);

            if(!reset_flag){
                servo_detach();
                main_current_state = STARTING;
            }
            break;
    }
}