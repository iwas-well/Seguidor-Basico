// -- Classe que molda o comportamento de motores DC simples --
// -- Plataforma: Arduino --
// -- Autor: Allan Cedric --

#ifndef __DC_MOTOR_ARDUINO_H__
#define __DC_MOTOR_ARDUINO_H__

// -- Bibliotecas --
#include <Arduino.h>

// -- Classe DC_Motor --
class DC_Motor {

private:
    uint8_t _pwmPin; // Pino PWM do motor p/ controle de velocidade
    uint8_t _clockwise, _counterClockwise; // Pinos para setar o sentido do motor
    uint8_t _speed; // Velocidade do motor

public:
    /*!
        @brief  Construtor

        @param  pwmPin              Pino PWM
        @param  clockwise           Pino para o sentido horário
        @param  counterClockwise    Pino para o sentido anti-horário
    */
    DC_Motor(uint8_t pwmPin, uint8_t clockwise, uint8_t counterClockwise);

    /*!
        @brief  Seta uma nova velocidade para o motor

        @param  speed   Velocidade
    */
    void setSpeed(uint8_t speed);

    /*!
        @brief  Retorna a velocidade atual do motor

        @return Velocidade atual
    */
    uint8_t getSpeed();

    /*!
        @brief  Faz com que o motor vá para frente

        @param  speed   Velocidade
    */
    void forward(uint8_t speed);

    /*!
        @brief  Faz com que o motor vá para trás

        @param  speed   Velocidade
    */
    void backwards(uint8_t speed);

    /*!
        @brief  Parada brusca do motor
    */
    void shortBrake();

    /*!
        @brief  Desacelera o motor
    */
    void stop();
};

#endif
