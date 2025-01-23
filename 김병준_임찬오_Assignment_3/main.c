#include "msp.h"
#include "Clock.h"
#include <stdio.h>
#include <stdint.h>

#define WHEEL_RADIUS 7.0         // cm, 바퀴 반지름
#define WHEEL_BASE 14.0          // cm, 바퀴 간 거리


#define P2 ((DIO_PORT_Even_Interruptable_Type*) (DIO_BASE + 0x0000))

void pwm_init34(uint16_t period, uint16_t duty3, uint16_t duty4) {
    // CCR0 period
    TIMER_A0->CCR[0] = period;

    // divide by 1
    TIMER_A0->EX0 = 0x0000;

    // toggle/reset
    TIMER_A0->CCTL[3] = 0x0040;
    TIMER_A0->CCR[3] = duty3;
    TIMER_A0->CCTL[4] = 0x0040;
    TIMER_A0->CCR[4] = duty4;

    // 0x200 -> SMCLK
    // 0b1100 0000 -> input divider /8
    // 0b0011 0000 -> up/down mode
    TIMER_A0->CTL = 0x02F0;

    // set alternative
    P2->DIR |= 0xC0;
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;
}

void motor_init(void) {
    // Configure nSLPR & nSLPL as GPIO, make them output, and set output LOW
    P3->SEL0 &= ~0xC0;   // 1) Clear bits 6 and 7 in SEL0 register to configure P3.6 and P3.7 as GPIO
    P3->SEL1 &= ~0xC0;   // 2) Clear bits 6 and 7 in SEL1 register to configure P3.6 and P3.7 as GPIO
    P3->DIR |= 0xC0;     // 3) Set bits 6 and 7 in DIR register to make P3.6 and P3.7 as output
    P3->OUT &= ~0xC0;    // 4) Clear bits 6 and 7 in OUT register to set P3.6 and P3.7 output to LOW

    // Configure DIRR & DIRL as GPIO, make them output, and set output LOW
    P5->SEL0 &= ~0x30;   // 1) Clear bits 4 and 5 in SEL0 register to configure P5.4 and P5.5 as GPIO
    P5->SEL1 &= ~0x30;   // 2) Clear bits 4 and 5 in SEL1 register to configure P5.4 and P5.5 as GPIO
    P5->DIR |= 0x30;     // 3) Set bits 4 and 5 in DIR register to make P5.4 and P5.5 as output
    P5->OUT &= ~0x30;    // 4) Clear bits 4 and 5 in OUT register to set P5.4 and P5.5 output to LOW

    // Configure PWMR & PWML as GPIO, make them output, and set output LOW
    P2->SEL0 &= ~0xC0;   // 1) Clear bits 6 and 7 in SEL0 register to configure P2.6 and P2.7 as GPIO
    P2->SEL1 &= ~0xC0;   // 2) Clear bits 6 and 7 in SEL1 register to configure P2.6 and P2.7 as GPIO
    P2->DIR |= 0xC0;     // 3) Set bits 6 and 7 in DIR register to make P2.6 and P2.7 as output
    P2->OUT &= ~0xC0;    // 4) Clear bits 6 and 7 in OUT register to set P2.6 and P2.7 output to LOW

    // Initialize PWM with a period and duty cycle of zero
    pwm_init34(7500, 0, 0);
}


void move(uint16_t leftDuty, uint16_t rightDuty) {
    P3->OUT |= 0xC0;               // Enable motors by setting P3.6 and P3.7 HIGH
    TIMER_A0->CCR[3] = leftDuty;   // Set PWM duty cycle for left motor
    TIMER_A0->CCR[4] = rightDuty;  // Set PWM duty cycle for right motor
}

// Function to set left motor direction to forward
void left_forward() {
    P5->OUT &= ~0x10;              // Clear P5.4 to set left motor direction forward
}

// Function to set left motor direction to backward
void left_backward() {
    P5->OUT |= 0x10;               // Set P5.4 to set left motor direction backward
}

// Function to set right motor direction to forward
void right_forward() {
    P5->OUT &= ~0x20;              // Clear P5.5 to set right motor direction forward
}

// Function to set right motor direction to backward
void right_backward() {
    P5->OUT |= 0x20;               // Set P5.5 to set right motor direction backward
}


void ir_init(){

    // 0,2,4,6 IR Emitter
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08;  //GPIO
    P5->DIR |= 0x08;    //OUTPUT
    P5->OUT &= ~0x08;   //turn off 4 even IR LEDs

    // 1,3,5,7 IR Emitter
    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04;  //GPIO
    P9->DIR |= 0x04;    //OUTPUT
    P9->OUT &= ~0x04;   //turn off 4 odd IR LEDs

    // 0~7 IR Sensor
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;  //GPIO
    P7->DIR &= ~0xFF;   //INPUT

}

void led_init() {

    // Set P2 as GPIO
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;

    // Input or Output
    // Current type is output
    P2->DIR |= 0x07;

    // Turn off LED
    P2->OUT &= ~0x07;
}

void switch_init() {
    // Setup switch as GPIO
    P1->SEL0 &= ~0x12;
    P1->SEL1 &= ~0x12;

    // Setup switch as input
    P1->DIR &= ~0x12;

    // Enable pull-up resistor
    P1->REN |= 0x12;

    // Now pull-up
    P1->OUT |= 0x12;

}

void turn_off_led() {

    // Turn off LED
    P2->OUT &= ~0x07;
}

void turn_on_led(int color) {

    // Turn on LED
    turn_off_led();
    P2->OUT |= color;
}

void systick_init(void) {
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL = 0x00000005;
}

void systick_wait1ms() {
    SysTick->LOAD = 48000 - 1; // Load 1 ms delay for 48 MHz clock
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000) == 0) {};
}

void systick_wait1s() {
    int i;
    int count = 1000;

    for (i = 0; i < count; i++) {
        systick_wait1ms();
    }
}

void timer_A3_capture_init() {

    P10->SEL0 |= 0x30;
    P10->SEL1 &= ~0x30;
    P10->DIR &= ~0x30;

    TIMER_A3->CTL &= ~0x0030;
    TIMER_A3->CTL = 0x0200;

    TIMER_A3->CCTL[0] = 0x4910;
    TIMER_A3->CCTL[1] = 0x4910;
    TIMER_A3->EX0 &= ~0x0007;

    NVIC->IP[3] = (NVIC->IP[3] & 0x0000FFFF) | 0x40400000;
    NVIC->ISER[0] = 0x0000C000;
    TIMER_A3->CTL |= 0x0024;

}

uint16_t first_left;
uint16_t first_right;

uint16_t period_left;
uint16_t period_right;

void TA3_0_IRQHandler(void) {
    TIMER_A3->CCTL[0] &= ~0x0001;
    period_right = TIMER_A3->CCR[0] - first_right;
    first_right = TIMER_A3->CCR[0];
}


uint32_t left_count;
void TA3_N_IRQHandler(void){
    TIMER_A3->CCTL[1] &= ~0x0001;
    left_count++;
}

void rotate(int degree) {

    int pulse = (WHEEL_BASE / WHEEL_RADIUS) * degree;

    left_backward();      // 왼쪽 모터 후진
    right_forward();      // 오른쪽 모터 전진
    left_count = 0;       // 펄스 카운트 초기화

    // 모터를 동작시켜 기기를 회전
    move(1000, 1000);

    // 목표 펄스에 도달할 때까지 기다림
    while (1){
        if(left_count > pulse)
            move(0,0);

    }

}

void main(void)
{

    Clock_Init48MHz();
    led_init();
    switch_init();
    systick_init();
    ir_init();
    motor_init();

    turn_off_led();

    timer_A3_capture_init();
    rotate(30);

}
