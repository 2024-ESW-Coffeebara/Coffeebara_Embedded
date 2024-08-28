#ifndef QUEUE_H
#define QUEUE_H

class Queue{
    public:
        Queue(int Speed, int En_pin, int Motor_pin1, int Motor_pin2);

        void Cw();
        void Ccw();
        void Off();

    private:
        int Speed;
        int En_pin;
        int Motor_pin1;
        int Motor_pin2;
};

#endif