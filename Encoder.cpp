#include "Encoder.h"

// Global pointers initialization
Encoder *encoderInstanceA = nullptr;
Encoder *encoderInstanceB = nullptr;

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : pinA(pinA), pinB(pinB), encoderCount(0) {
    // Determine the interrupt numbers for the pins
    this->interruptA = digitalPinToInterrupt(pinA);

    // Assign global pointers based on pin assignments
    if (pinA == 35 && pinB == 32) {
        encoderInstanceA = this;
    } else if (pinA == 17 && pinB == 16) {
        encoderInstanceB = this;
    }
}


void Encoder::begin() {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);

    this->oldPosition = 0;
    this->lastTime = millis();

    // Attach interrupts using global pointers and ISR functions
    if (encoderInstanceA == this) {
        attachInterrupt(interruptA, handleInterrupt_channe1A, RISING);
    } else if (encoderInstanceB == this) {
        attachInterrupt(interruptA, handleInterrupt_channe1B, RISING);
    }
}

long Encoder::getCount() const {
    return encoderCount;
}

void Encoder::resetCount() {
    encoderCount = 0;
}

// Actual ISR code for pin A
void handleInterrupt_channe1A() {
    if (encoderInstanceA != nullptr) {
        encoderInstanceA->handleInterrupt1_inst();
    }
}


// Actual ISR code for pin B
void handleInterrupt_channe1B() {
    if (encoderInstanceB != nullptr) {
        encoderInstanceB->handleInterrupt2_inst();
    }
}


// Non-static member function for handling pin A interrupt
void Encoder::handleInterrupt1_inst() {
    // Handle encoder count for pin A
    if (digitalRead(this->pinB)) {
        this->encoderCount++;
    } else {
        this->encoderCount--;
    }
}

// Non-static member function for handling pin B interrupt
void Encoder::handleInterrupt2_inst() {
    // Handle encoder count for pin B
    if (digitalRead(this->pinB)) {
        this->encoderCount--;
    } else if (digitalRead(this->pinA)) {
        this->encoderCount++;
    }
}