#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include "pid.h"
#include "libEposCmd.so"

#define MOTOR_CAN_ID  0x01  // Endereço CAN do motor
#define ENCODER_CAN_ID 0x02  // Endereço CAN do encoder


int r = 0.1;

float measurement;
float setpoint;
float control_action;

void setup()
{
    Serial.begin(9600);

    //Configurar o tipo de motor e parâmetros do motor
    VCS_SetMotorType(VCS_GetMotorHandle(MOTOR_CAN_ID));
    VCS_SetDcMotorParameterEx(VCS_GetMotorHandle(MOTOR_CAN_ID));

    // Configurar o tipo de sensor e parâmetros do encoder
    VCS_SetSensorType(VCS_GetSensorHandle(ENCODER_CAN_ID));
    VCS_SetIncEncoderParameter(VCS_GetSensorHandle(ENCODER_CAN_ID));

    // Iniciar o controlador PID, se necessário
    PIDController controller;
    PIDController_Init(&controller);

}

int main(int argc, char *argv[])
{

    setup();

    while (1)
    { 
        // Leitura do potenciômetro
        int potValue = analogRead(potPin);
        uiContador = map(potValue, 0, 1023, 0, 9);

        // Ajuste direto da variável de referência (removendo lógica de botões)
        int vref = 5;  // Valor desejado (substitua com seu valor real)
        setpoint = vref * r;

        // Os parâmetros das funções VCS_ReadCANFrame e VCS_GetVelocityls precisam ser definidos ainda
        unsigned int KeyHandle = 123;  
        unsigned int CobID = 456;
        unsigned int Length = 7;     
        unsigned int NodeId = 1; 
        unsigned int pVelocityls[2]; 
        unsigned int pErrorCode = 0; 
        unsigned int Timeout = 1000; 
        unsigned int ErrorCode = 0;

        measurement = VCS_ReadCANFrame(KeyHandle, CobID, Length, VCS_GetVelocityls(KeyHandle, NodeId, pVelocityls, pErrorCode), Timeout, ErrorCode);

        PIDController controller;
        PIDController_Init(&controller);

        measurement = Sensor_update();
        control_action = PIDController_Update(&controller, setpoint, measurement);
    }
    VCS_CloseDevice(1);
    return 0;
}