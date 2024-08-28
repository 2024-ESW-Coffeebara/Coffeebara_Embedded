#ifndef CONSTANTS_H
#define CONSTANTS_H

#define GLITCH                      1

#define QUEUE_GO                    0
#define QUEUE_STOP                  1
#define QUEUE_READY_CUP             2

#define ONLY_WAITING                0
#define RESET                       1
#define RESET_QUEUE                 2   
#define RESET_SERVO                 3
#define RESET_HORIZONTAL_INTERRUPT  4   
#define RESET_HORIZONTAL_DEFAULT    5 
#define RESET_VERTICAL_DEFAULT      6
#define STARTING                    7
#define VERTICAL_HOLD_CUP           8
#define VERTICAL_REMOVE_LID_HOLDER  9
#define HORIZONTAL_HOLD_HOLDER      10
#define HORIZONTAL_PLACE_HOLDER     11
#define HORIZONTAL_HOLD_CUP         12
#define HORIZONTAL_WASTE_CUP        13
#define HORIZONTAL_WASH_CUP         14
#define HORIZONTAL_LOAD_CUP         15

const int QUEUE_SPEED = 110;

const int LOADCELL_DATA_PIN = 35;
const int LOADCELL_CLK_PIN  = 36;

const int RESET_PIN            = 2;
const int QUEUE_INITIALIZE_PIN = 3;

const int WATERPUMP_EN                     = 4;
const int WATERPUMP_PIN_1                  = 5;
const int WATERPUMP_PIN_2                  = 6;

const int QUEUE_MOTOR_EN = 7;
const int QUEUE_MOTOR_PIN_1 = 8;
const int QUEUE_MOTOR_PIN_2 = 9;

const int HORIZONTAL_STEPPER_wrist_pin     = 22;
const int HORIZONTAL_STEPPER_finger_pin    = 23;
const int VERTICAL_STEPPER_finger_pin      = 18;

const int QUEUE_STEPPER_pin = 2;
// const int regular_large_pin                = 10;
/////////////////////////
// ------- 1 -- 2 -- 3 //
/////////////////////////
const int small_cup_servo_pin               = 13;
const int regular_cup_servo_pin             = 12;
const int large_cup_servo_pin               = 11;
const int remove_lid_servo_pin              = 17;

const int VERTICAL_STEPPER_reset_pin        = 19;
const int HORIZONTAL_STEPPER_reset_pin      = 20;
const int QUEUE_INTERRUPT_PIN               = 21;

const int ECHO_PIN = 45;
const int TRIG_PIN = 47;
// const int HORIZONTAL_STEPPER_reset_pin      = 20;

// vertical : 19, 

const int VERTICAL_STEPPER_SPEED            = 250;
const int HORIZONTAL_STEPPER_SPEED          = 300;
const int QUEUE_STEPPER_SPEED               = 10;
const int SPR                               = 200;

const int VERTICAL_STEPPER_DELAY            = ((60L * 1000L * 1000 / SPR / VERTICAL_STEPPER_SPEED) + 50);
const int HORIZONTAL_STEPPER_DELAY          = ((60L * 1000L * 1000 / SPR / HORIZONTAL_STEPPER_SPEED) + 50);
const int QUEUE_STEPPER_DELAY               = ((60L * 1000L * 1000) / SPR / QUEUE_STEPPER_SPEED) + 200;

const float pi = 3.14;
const float CmPerCycle = 4.71;

/* position definition */
const float horizontal_pos_default          = 41.5;
const float horizontal_pos_reset            = 1.5;
const float horizontal_pos_wash             = 13;
const float horizontal_pos_waste            = 35;

const float vertical_pos_default            = 0;
const float vertical_pos_remove_lid         = 2;
const float vertical_pos_remove_holder      = 3;
const float vertical_pos_holder             = 12;
const float vertical_pos_cup                = 12;

/* horizontal servo motor angle definition */
const int HORIZONTAL_STEPPER_finger_default = 115;
const int HORIZONTAL_STEPPER_wrist_default  = 130;
// const int HORIZONTAL_STEPPER_wrist_load     = 5;
const int HORIZONTAL_STEPPER_wrist_load     = 5;

const int HORIZONTAL_STEPPER_finger_holder  = 55;
const int HORIZONTAL_STEPPER_finger_cup     = 50;
const int HORIZONTAL_STEPPER_wrist_waste    = 5;
const int HORIZONTAL_STEPPER_wrist_wash     = 40;

/* vertical servo motor angle definition */
const int VERTICAL_STEPPER_finger_minimum   = 40;
const int VERTICAL_STEPPER_finger_default   = 155;
 
/* cup cnt threshold */
const int small_cnt_threshold               = 99;
const int regular_cnt_threshold             = 99;
const int large_cnt_threshold               = 99;

const int cleaning_cup_angle                = 180;
const int default_cup_angle                 = 0;

/* actuator delay for removing lid */   
const int REMOVE_LID_ANGLE[7]               = {170, 75, 100, 125, 80, 100, 125};

// small : 스벅, 레스티오 (홀더 없을 때)
// regular : 1847, xx, 편의점 (홀더 있어도 ㄱㅊ)
// large : xx, 빽다방 (홀더 여부는 모름)

// vertical finger, actuator 전부 홀더 크기랑 entrance 따라 달라져야함
// 아래같이 small, regular, large 나누는건 조금 자제할 것 

// const float horizontal_pos_cup_size[7]      = {0, 61.5, 80.5, 96, 61.5, 80.5, 96};

const float horizontal_pos_cup_size[7]      = {0, 96, 80.5, 62.2, 96, 80.5, 62.2};

/* finger angle */
// const int HORIZONTAL_STEPPER_finger_cup[9] = {50, 50, 50, 50, 50, 50, 50, 50, 50};
// const int HORIZONTAL_STEPPER_finger_ready_cup[7] = {0, 60.5, 62.5, 66, 60.5, 62.5, 66};
const int HORIZONTAL_STEPPER_finger_ready_cup[7] = {0, 59.5, 62.5, 66, 59.5, 62.5, 66};
const int VERTICAL_STEPPER_finger_remove_lid[7]  = {0, 40, 55, 85, 40, 55, 85};
const int VERTICAL_STEPPER_finger_cup[7]         = {0, 60, 70, 85, 60, 70, 85};
const int VERTICAL_STEPPER_finger_ready_cup[7]   = {0, 75, 95, 120, 75, 95, 120};

/* queue constants */


#endif

// 입구 크기로 ready_cup, 서보는 좋은데 거의 small이랑 regular는 비슷함..
// 나중에 쓰레기 버릴 때는 인식된 컵 크기로 나눠서 할것