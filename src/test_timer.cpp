#include "Arduino.h"

TimerHandle_t my_timer = NULL;
class Foo{
    public:
        void Create_Timer();
        void do_somthing(){
            Serial.print("    Hello====  ");
            Serial.println(x);
        }
    private:
        int x = 12345;
};


void timer_callback_is_not_isr(TimerHandle_t x_timer){
    Foo* f1 = (Foo*) pvTimerGetTimerID(x_timer);
    Serial.println((int) f1);
    f1->do_somthing();
}

void Foo::Create_Timer(){
    my_timer = xTimerCreate("test", 2000, pdFALSE, (void*) this, timer_callback_is_not_isr);
    xTimerStart(my_timer, 0);
}

Foo f;

void setup(){
    Serial.begin(115200);
    f.Create_Timer();
    Serial.print("real addr of object   ");
    Serial.println((int) &f);
    f.do_somthing();
}

void loop(){}