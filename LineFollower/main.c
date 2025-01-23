#include <stdio.h>
#include "msp.h"
#include "CortexM.h"
#include "Clock.h"
#include "Motor.h"
#include "BumpInt.h"
#include "FSMController.h"
#include "Reflectance.h"
#include "SensorInt.h"


const uint8_t READ_DELAY = 1;
uint8_t CollisionData, CollisionFlag;  // mailbox
uint8_t SensorInput = 0;
uint8_t SensorInput_F = 0; // 0 -> no reading
uint8_t ReflectCount = 0;
int LEDCount = 0;
uint8_t ledState = 0;

void SysTick_Handler(void){

    if (ReflectCount % 10 == 0) {
        Reflectance_Start();
    } else if (ReflectCount % 10 == READ_DELAY) {
        SensorInput_F = 0;                  // Set Semaphore
        SensorInput = Reflectance_End();    // Write to SensorInput
        SensorInput_F = 1;                  // Reset Semaphore
    }

//    if (LEDCount >= 1000){
//        if (ledState == 0) {
//            Red_LED_Off();
//            Blue_LED_On();
//            ledState = 1;
//        } else {
//            Blue_LED_Off();
//            Red_LED_On();
//            ledState = 0;
//        }
//        LEDCount = 0; // 타이머 리셋
//    }

    ReflectCount ++;
//    LEDCount++;
}

uint8_t AverageSensor (uint8_t data) {
    uint8_t data_mut = data;
    uint32_t result = 0;
    uint32_t bits = 0;
    uint32_t i;

    for(i = 0; i < 8; i++) {
        result += (data_mut & 1) * (i + 1);
        bits += data_mut & 1;
        data_mut =  data_mut >> 1;
    }

    if (bits == 0) {
        return 0;
    } else {
        return (uint8_t) (result / bits) % 8;
    }
}


uint8_t ReadSensorData(void){

    // Waiting for SysTick_Handler to write
    //while (SensorInput_F);

    return AverageSensor(SensorInput);
}

uint8_t ReadSensorBit(void){

    // Waiting for SysTick_Handler to write
    //while (SensorInput_F);

    return SensorInput;
}


//Accessor Function for FSM Controller to stop operating if collision has occured.
uint8_t getColFlag(){
    return CollisionFlag;
}


void HandleCollision(uint8_t bumpSensor){
   CollisionData = bumpSensor;
   CollisionFlag = 1;
   Motor_Stop();
}


/**
 * main.c
 */
void main(void){
    Clock_Init48MHz();
    SW_Init();        // 스위치 초기화
    LED_Init();        // LED 초기화
    SysTick_Init(48000, 2); // Every 1ms, priority 2
    Reflectance_Init();
    EnableInterrupts();
    Motor_Init();
    BumpInt_Init(&HandleCollision); //Pass is not necessary in here since we call the function directly
    start_fsm();
}





