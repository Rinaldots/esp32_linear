
#include "linear.h"

LinearCar::LinearCar()
    : stepper(MOTOR_STEPS, PIN_DIR, PIN_STEP)
{}

void LinearCar::setup()
{  
  stepper.begin(MOTOR_RPM);
  stepper.enable();
  stepper.setMicrostep(1);
  stepper.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, 1000, 1000);
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
  // Permite abortar homing com STOP vindo do BLE.
  taskENTER_CRITICAL(&stateMux);
  if (comando == STOP) {
    estado = PARADO;
    taskEXIT_CRITICAL(&stateMux);
    stepper.startMove(0);
    return;
  }
  taskEXIT_CRITICAL(&stateMux);

  if (isEndstopTriggered()) {
    long zeroStep;
    taskENTER_CRITICAL(&stateMux);
    estado = PARADO;
    zeroStep = homing_zero_step;
    steps = zeroStep;
    taskEXIT_CRITICAL(&stateMux);

    stepper.startMove(1);
    Serial.print("Homing completo. Passos definidos para: ");
    Serial.println(zeroStep);
    return;
  }

  stepper.startMove(-1);
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
  taskENTER_CRITICAL(&stateMux);
  Estado estadoLocal = estado;
  taskEXIT_CRITICAL(&stateMux);

  if (estadoLocal == INDO) {
    stepper.startMove(1);
    taskENTER_CRITICAL(&stateMux);
    steps++;
    taskEXIT_CRITICAL(&stateMux);
  } else if (estadoLocal == VOLTANDO) {
    stepper.startMove(-1);
    taskENTER_CRITICAL(&stateMux);
    steps--;
    taskEXIT_CRITICAL(&stateMux);
  } else if (estadoLocal == PARADO) {
    stepper.startMove(0);
  }
}

bool LinearCar::isEndstopTriggered() const
{
  return !digitalRead(PIN_ENDSTOP);
}

void LinearCar::setTargetStep(long steps)
{
  stepper.startMove(steps-stepDelta);
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

void LinearCar::setSpeed(float rpm)
{
  stepper.setRPM(rpm);
}

void LinearCar::requestHoming()
{
  taskENTER_CRITICAL(&stateMux);
  estado = HOMING;
  taskEXIT_CRITICAL(&stateMux);
}

long LinearCar::getSteps() const
{
  taskENTER_CRITICAL(&stateMux);
  long currentSteps = steps;
  taskEXIT_CRITICAL(&stateMux);
  return currentSteps;
}


long LinearCar::getRelativeSteps() const
{
  taskENTER_CRITICAL(&stateMux);
  long relativeSteps = steps - homing_zero_step;
  taskEXIT_CRITICAL(&stateMux);
  return relativeSteps;
}

float LinearCar::getRelativePosition() const
{
  taskENTER_CRITICAL(&stateMux);
  long relativeSteps = steps - homing_zero_step;
  float relativePositionMM = relativeSteps * MM_PER_STEP;
  taskEXIT_CRITICAL(&stateMux);
  return relativePositionMM;
}

float LinearCar::getAbsolutePosition() const
{
  taskENTER_CRITICAL(&stateMux);
  float absolutePositionMM = steps * MM_PER_STEP;
  taskEXIT_CRITICAL(&stateMux);
  return absolutePositionMM;
}
