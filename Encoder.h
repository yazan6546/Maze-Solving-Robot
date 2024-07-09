#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB);
    void begin();
    long getCount() const;
    void resetCount();
    void handleInterrupt1_inst();
    void handleInterrupt2_inst();
    long oldPosition;
    long oldPosition_position;
    unsigned long lastTime;
    unsigned long lastTime_position;

private:
    volatile long encoderCount;
    uint8_t pinA;
    uint8_t pinB;
    uint8_t interruptA;
    uint8_t interruptB;
};

// Global pointers to Encoder instances
extern Encoder *encoderInstanceA;
extern Encoder *encoderInstanceB;
void handleInterrupt_channe1A();
void handleInterrupt_channe2A();
void handleInterrupt_channe1B();
void handleInterrupt_channe2B();
// Interrupt handler functions

#endif // ENCODER_H