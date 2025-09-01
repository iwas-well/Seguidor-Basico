// -- Implementação da classe Motor --
// -- Plataforma: Arduino --
// -- Autor: Allan Cedric --

#include "DC-Motor-Arduino.h"

DC_Motor::DC_Motor(uint8_t pwmPin, uint8_t clockwise, uint8_t counterClockwise)
{
    _pwmPin = pwmPin;
    _clockwise = clockwise;
    _counterClockwise = counterClockwise;
    pinMode(_pwmPin, OUTPUT);           // Seta o pino PWM como OUTPUT
    pinMode(_clockwise, OUTPUT);        // Seta o pino clockwise como OUTPUT
    pinMode(_counterClockwise, OUTPUT); // Seta o pino counterclockwise como OUTPUT
    DC_Motor::stop();                   // Motor inicializa com velocidade nula
}

void DC_Motor::setSpeed(uint8_t speed)
{
    _speed = speed;
    analogWrite(_pwmPin, _speed);
}

uint8_t DC_Motor::getSpeed()
{
    return _speed;
}

void DC_Motor::forward(uint8_t speed)
{
    digitalWrite(_clockwise, HIGH);
    digitalWrite(_counterClockwise, LOW);
    DC_Motor::setSpeed(speed);
}

void DC_Motor::backwards(uint8_t speed)
{
    digitalWrite(_clockwise, LOW);
    digitalWrite(_counterClockwise, HIGH);
    DC_Motor::setSpeed(speed);
}

void DC_Motor::shortBrake()
{
    digitalWrite(_clockwise, HIGH);
    digitalWrite(_counterClockwise, HIGH);
    DC_Motor::setSpeed(0);
}

void DC_Motor::stop()
{
    digitalWrite(_clockwise, LOW);
    digitalWrite(_counterClockwise, LOW);
    DC_Motor::setSpeed(0);
}
