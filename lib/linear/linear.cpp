#include "linear.h"

namespace {
float g_motorRpm = static_cast<float>(MOTOR_RPM);
}

LinearCar::LinearCar()
    : stepper(MOTOR_STEPS, PIN_DIR, PIN_STEP)
{}

void LinearCar::setup()
{
    stepper.begin(g_motorRpm);
    stepper.enable();
    stepper.setMicrostep(1);

    pinMode(PIN_SLEEP_RESET, OUTPUT);
    digitalWrite(PIN_SLEEP_RESET, HIGH);
    pinMode(PIN_ENDSTOP, INPUT_PULLUP);
}

void LinearCar::step_loop()
{
    Estado estadoLocal;
    taskENTER_CRITICAL(&stateMux);
    estadoLocal = estado;
    taskEXIT_CRITICAL(&stateMux);

    if (estadoLocal == HOMING) {
        handleHoming();
        return;
    }

    updateStateFromCommand();
    runMovement();
    stepper.nextAction();
}

void LinearCar::handleHoming()
{
    taskENTER_CRITICAL(&stateMux);
    bool stopRequested = (comando == STOP);
    taskEXIT_CRITICAL(&stateMux);

    if (stopRequested) {
        taskENTER_CRITICAL(&stateMux);
        estado = PARADO;
        taskEXIT_CRITICAL(&stateMux);
        stepper.startMove(0);
        return;
    }

    if (isEndstopTriggered()) {
        long zero;
        taskENTER_CRITICAL(&stateMux);
        estado = PARADO;
        zero = homing_zero_step;
        steps = zero;
        taskEXIT_CRITICAL(&stateMux);

        stepper.startMove(-1) ;
        Serial.print("Homing completo. Passos definidos para: ");
        Serial.println(zero);
        return;
    }

    stepper.startMove(1);
    stepper.nextAction();
}

void LinearCar::updateStateFromCommand()
{
    taskENTER_CRITICAL(&stateMux);
    if (comando == PLAY) {
        if (steps >= (homing_zero_step - 50)) {
            estado = INDO;
        }
        if (steps <= (homing_zero_step - limitePassos)) {
            estado = VOLTANDO;
        }
        taskEXIT_CRITICAL(&stateMux);
        return;
    }

    if (comando == BYPASS) {
        estado = controle;
        taskEXIT_CRITICAL(&stateMux);
        return;
    }

    if (comando == STOP) {
        estado = PARADO;
    }
    taskEXIT_CRITICAL(&stateMux);
}

void LinearCar::runMovement()
{
    Estado estadoLocal;
    taskENTER_CRITICAL(&stateMux);
    estadoLocal = estado;
    taskEXIT_CRITICAL(&stateMux);

    if (estadoLocal == INDO) {
        stepper.startMove(-1);
        taskENTER_CRITICAL(&stateMux);
        steps--;
        taskEXIT_CRITICAL(&stateMux);
    } else if (estadoLocal == VOLTANDO) {
        stepper.startMove(1);
        taskENTER_CRITICAL(&stateMux);
        steps++;
        taskEXIT_CRITICAL(&stateMux);
    } else {
        stepper.startMove(0);
    }
}

bool LinearCar::isEndstopTriggered() const
{
    Serial.println("Endstop state: " + String(digitalRead(PIN_ENDSTOP)));
    return !digitalRead(PIN_ENDSTOP);
}

void LinearCar::setTargetStep(long targetSteps)
{
    stepper.startMove(targetSteps - stepDelta);
}

void LinearCar::setHomingZeroStep(long value)
{
    taskENTER_CRITICAL(&stateMux);
    homing_zero_step = value;
    taskEXIT_CRITICAL(&stateMux);
}

void LinearCar::setCommand(Comando value)
{
    taskENTER_CRITICAL(&stateMux);
    comando = value;
    taskEXIT_CRITICAL(&stateMux);
}

void LinearCar::setBypassControl(Estado value)
{
    taskENTER_CRITICAL(&stateMux);
    controle = value;
    comando = BYPASS;
    taskEXIT_CRITICAL(&stateMux);
}

void LinearCar::requestHoming()
{
    taskENTER_CRITICAL(&stateMux);
    estado = HOMING;
    comando = BYPASS;
    controle = PARADO;
    taskEXIT_CRITICAL(&stateMux);
}

void LinearCar::setSpeed(float rpm)
{
    if (rpm < 1.0f) {
        rpm = 1.0f;
    }
    g_motorRpm = rpm;
    stepper.begin(g_motorRpm);
}

long LinearCar::getSteps() const
{
    taskENTER_CRITICAL(&stateMux);
    long current = steps;
    taskEXIT_CRITICAL(&stateMux);
    return current;
}

long LinearCar::getRelativeSteps() const
{
    taskENTER_CRITICAL(&stateMux);
    long relative = steps - homing_zero_step;
    taskEXIT_CRITICAL(&stateMux);
    return relative;
}

float LinearCar::getRelativePosition() const
{
    return static_cast<float>(getRelativeSteps()) * MM_PER_STEP;
}

float LinearCar::getAbsolutePosition() const
{
    return static_cast<float>(getSteps()) * MM_PER_STEP;
}