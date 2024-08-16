#ifndef WATERPUMP_H
#define WATERPUMP_H

class Waterpump{
    public:
        Waterpump(int En_pin, int Motor_pin1, int Motor_pin2);

        void Ready();
        void On();
        void Off();

    private:
        int En_pin;
        int Motor_pin1;
        int Motor_pin2;
};

#endif
