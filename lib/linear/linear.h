#ifndef LINEAR_H
#define LINEAR_H

#include <Arduino.h>
#include "DRV8825.h"

// Hardware configuration
constexpr int MOTOR_STEPS = 200;
constexpr int MOTOR_RPM = 30;
constexpr int PIN_DIR = 39;
constexpr int PIN_STEP = 40;
constexpr int PIN_SLEEP_RESET = 37;
constexpr int PIN_ENDSTOP = 11;
// Configurações da correia e polia para cálculo de passos por mm
constexpr int STEPS_PER_REV = 200;
constexpr int PULLEY_TEETH = 16;
constexpr float BELT_PITCH = 2.0f;

constexpr float MM_PER_STEP = (PULLEY_TEETH * BELT_PITCH) / STEPS_PER_REV;

enum Estado {
        PARADO,
        HOMING,
        INDO,
        VOLTANDO
    };
enum Comando {
        PLAY,
        STOP,
        BYPASS
};

class LinearCar {
public:
    LinearCar();
    void setup();
    void step_loop();
    void setTargetStep(long steps);
    void setHomingZeroStep(long value);
    void setCommand(Comando value);
    void setBypassControl(Estado value);
    void requestHoming();
    void setSpeed(float rpm);
    long getSteps() const;
    long getRelativeSteps() const;
    float getRelativePosition() const;
    float getAbsolutePosition() const;

private:
    DRV8825 stepper;
    mutable portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

    Comando comando = STOP;
    Estado estado = PARADO;
    Estado controle = PARADO;
    long steps = 0;
    long homing_zero_step = 0;

    long stepDelta = 0;
    const long limitePassos = 1750;

    void handleHoming();
    void updateStateFromCommand();
    void runMovement();
    bool isEndstopTriggered() const;

};

extern LinearCar linearCar;

#endif