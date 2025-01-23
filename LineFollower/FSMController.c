#include <stdint.h>
#include <stdio.h>
#include "msp.h"
#include "CortexM.h"

/* LED, 스위치 초기설정 */

#define RED_LED BIT0     // P2.0
#define GREEN_LED BIT1   // P2.1
#define BLUE_LED BIT2    // P2.2

// 스위치 핀 설정
#define LEFT_SWITCH BIT4  // P1.4
#define RIGHT_SWITCH BIT1 // P1.1


void SW_Init(void) {
    // 스위치 설정 (P1.1, P1.4)
    P1->SEL0 &= ~(LEFT_SWITCH | RIGHT_SWITCH);
    P1->SEL1 &= ~(LEFT_SWITCH | RIGHT_SWITCH);
    P1->DIR &= ~(LEFT_SWITCH | RIGHT_SWITCH);   // 입력
    P1->REN |= (LEFT_SWITCH | RIGHT_SWITCH);    // 풀업/다운 활성화
    P1->OUT |= (LEFT_SWITCH | RIGHT_SWITCH);    // 풀업 설정
}

void LED_Init(void) {
    // LED 설정 (P2.0, P2.1, P2.2)
    P2->SEL0 &= ~(RED_LED | GREEN_LED | BLUE_LED);
    P2->SEL1 &= ~(RED_LED | GREEN_LED | BLUE_LED);
    P2->DIR |= (RED_LED | GREEN_LED | BLUE_LED); // 출력
    P2->OUT &= ~(RED_LED | GREEN_LED | BLUE_LED); // 초기화 (LED 꺼짐)
}


void Red_LED_On() {
    P2->OUT |= RED_LED;
}
void Red_LED_Off() {
    P2->OUT &= ~RED_LED;
}
void Green_LED_On() {
    P2->OUT |= GREEN_LED;
}
void Green_LED_Off() {
    P2->OUT &= ~GREEN_LED;
}
void Blue_LED_On() {
    P2->OUT |= BLUE_LED;
}
void Blue_LED_Off() {
    P2->OUT &= ~BLUE_LED;
}

uint8_t LeftButton_Pressed() {
    return (P1->IN & LEFT_SWITCH) == 0; // 눌리면 0 반환
}

uint8_t RightButton_Pressed() {
    return (P1->IN & RIGHT_SWITCH) == 0; // 눌리면 0 반환
}

// Linked data structure
struct State {
  int16_t leftDuty;              // Duty Cycle %
  int16_t rightDuty;             // [-100 to 100]
  uint32_t delay;                // Delay in ms
  const struct State *next[8];   // 3-bit input -> 8 next states
};
typedef const struct State State_t;

// Abbreviations:
// OL -> On Line
// L1 -> Off Left 1 (smallest deviation from line)
// NL -> No Line
// LLC -> Left Lost Check (Lost but know we are left of line)
// LC1 -> Lost Check 1 (Very lost)
// LC2 -> Lost Check 2
// FL -> Fully Lost

// State order in &fsm
#define OL &fsm[0]
#define L1 &fsm[1]
#define L2 &fsm[2]
#define L3 &fsm[3]
#define R1 &fsm[4]
#define R2 &fsm[5]
#define R3 &fsm[6]
#define LLC &fsm[7]
#define RLC &fsm[8]
#define LC1 &fsm[9]
#define LC2 &fsm[10]
#define LF &fsm[12]
#define FL &fsm[11]
#define RLLC &fsm[13]
#define RRLC &fsm[14]

// Standard time between states
#define dtGoodGood 40
#define dtGood 25 //was 30
#define dt 15
#define dtSlightLost 400
#define dtLost 700

// Speed PWM definitions (0, 14998)
#define speed 0.5
#define MAX 14998*speed  // 100%
#define HIGH 13000*speed // 80%
#define MED 10000*speed   // 50%
#define NMED -10000*speed // -50% (Reverse Medium)
#define LOW 5000*speed   // 20%


State_t fsm[15]={
  {MAX, MAX,  dtGoodGood, {LC1, L3, L2, L1, OL, R1, R2, R3}},  // On Line
  {MAX, HIGH, dtGood, {LLC, L3, L2, L1, OL, R1, R2, R3}},  // Left 1
  {MAX, MED,  dt, {LLC, L3, L2, L1, OL, R1, R2, R3}},  // Left 2
  {MAX, LOW,  dt, {LLC, L3, L2, L1, OL, R1, R2, R3}},  // Left 3
  {HIGH,MAX,  dtGood, {RLC, L3, L2, L1, OL, R1, R2, R3}},  // Right 1
  {MED, MAX,  dt, {RLC, L3, L2, L1, OL, R1, R2, R3}},  // Right 2
  {LOW, MAX,  dt, {RLC, L3, L2, L1, OL, R1, R2, R3}},  // Right 3
  {MED, NMED, dtSlightLost, {RLLC, L3, L2, L1, OL, R1, R2, R3}},  // Left Lost Check
  {NMED, MED, dtSlightLost, {RRLC, L3, L2, L1, OL, R1, R2, R3}},  // Right Lost Check
  {MED, NMED, dtLost, {LC2, L3, L2, L1, OL, R1, R2, R3}},  // Lost Check 1
  {NMED, MED, dtLost, {LF,  L3, L2, L1, OL, R1, R2, R3}},  // Lost Check 2
  {0, 0,    1000, {FL,  FL, FL, FL, FL, FL, FL, FL}},   // Fully Lost, Stop
  {MED, MED, 650, {FL, L3, L2, L1, OL, R1, R2, R3}}, // Lost forward check //was 700
  {NMED, MED, dtSlightLost, {LC1, L3, L2, L1, OL, R1, R2, R3}},  // Left Lost Check REV
  {MED, NMED, dtSlightLost, {LC1, L3, L2, L1, OL, R1, R2, R3}}  // Right Lost Check REV
};

// Motor Translation Function
// Motor cannot accept negative inputs, only PWM (0 to 14,998)
void call_motor(int16_t leftDuty, int16_t rightDuty){

    uint8_t leftForward = leftDuty >= 0;
    uint8_t rightForward = rightDuty >= 0;

    // Forward
    if (leftForward && rightForward) {
        Motor_Forward((uint16_t) leftDuty, (uint16_t) rightDuty);

    // Left
    } else if (!leftForward && rightForward) {
        Motor_Left((uint16_t) (-1 * leftDuty), (uint16_t) rightDuty);

    // Right
    } else if (leftForward && !rightForward) {
        Motor_Right((uint16_t) leftDuty, (uint16_t) (-1 * rightDuty));

    // Backward
    } else {
        Motor_Backward((uint16_t) (-1 * leftDuty), (uint16_t) (-1 * rightDuty));
    }
}


State_t *state;

// Starts fsm and will loop continuously
void start_fsm(){

    /* 버튼 단계 설정 */

    typedef enum {
        TurnOn,
        HalfInput,
        StandBy,
        Running,
        Finish
    } AppState;

    AppState appState = TurnOn;

    // Assume initially on line
    // 변수 초기화
    int x_coor = 0;
    int y_coor = 0;
    int current_x = 0;
    int current_y = 0;
    int currentState = 0; // 0: 출발 1: x 좌표 도달해서 좌회전 후 2: y 좌표 도달해서 우회전 후 3: x좌표 5 도달해서 좌회전 후 4: Finish Line에 도달한 후
    int rot_delay = 500; // 90도 회전할 수 있는 딜레이
    int straight_delay = 500; // 직진 얼마나 하는지 설정

    state = OL;

    while(1){

        switch (appState) {

            case TurnOn:
                printf("\nappState = TurnOn\n");

                // LED 동작: 모든 LED 끔
                Red_LED_Off();
                Green_LED_Off();
                Blue_LED_Off();

                // 상태 전환: 우측 버튼 누르면 HalfInput으로 이동
                if (RightButton_Pressed()) {
                    appState = HalfInput;
                    Clock_Delay1ms(200); // 버튼 바운스 방지
                }
                break;

            case HalfInput:
                printf("\nappState = HalfInput\n");

                // LED 동작: 빨간 LED 켬
                Red_LED_On();

                // 상태 전환: 왼쪽 버튼 N번 누르고 오른쪽 버튼
                if (LeftButton_Pressed()) {
                    x_coor++;
                    printf("\nx_coor: %d\n",x_coor);
                    Clock_Delay1ms(200); // 버튼 바운스 방지
                }
                if (RightButton_Pressed()) {
                    appState = StandBy;
                    Clock_Delay1ms(200);
                }
                break;

            case StandBy:
                printf("\nappState = Standby\n");

                // LED 동작: 빨간 LED 끄고 녹색 LED 켬
                Red_LED_Off();
                Green_LED_On();

                // 상태 전환: 왼쪽 버튼 M번 누르고 오른쪽 버튼
                if (LeftButton_Pressed()) {
                    y_coor++;
                    printf("\ny_coor: %d\n",y_coor);
                    Clock_Delay1ms(200);
                }
                if (RightButton_Pressed()) {
                    appState = Running;
                    printf("\nappState = Running\n");
                    printf("\nRunning... coordinate:(%d, %d)\n",x_coor, y_coor);
                    Clock_Delay1ms(500);
                    Green_LED_Off();
                }
                break;

            case Running: // 기기 작동 시작

                call_motor(state->leftDuty, state->rightDuty);  // Send state output to motor
                Clock_Delay1ms(state->delay);                   // wait

                uint8_t test = ReadSensorBit();
                printf("Sensor Readings (binary): ");
                int i;
                for (i = 7; i >= 0; i--) {  // MSB부터 출력
                    printf("%d", (test >> i) & 1);
                }
                printf("\n");

                switch(currentState) {
                    case 0: //전진 후 우회전 후 전진
                        printf("\ncurrentState = %d\n",currentState);
                        call_motor(HIGH,HIGH); // 전진
                        Clock_Delay1ms(straight_delay);

                        call_motor(HIGH,-HIGH); // 우회전
                        Clock_Delay1ms(rot_delay);

                        call_motor(HIGH,HIGH); // 전진
                        Clock_Delay1ms(straight_delay);

                        currentState = 1;
                        printf("\ncurrentState = %d\n",currentState);
                        break;

                    case 1: // x좌표 증가시키기

                        if((test & 0b11110000) == 0b11110000  || (test & 0b11110000) == 0b11100000 || (test & 0b11110000) == 0b01110000){ // check x_coor
                            current_x++;
                            printf("\ncurrent_x: %d\n",current_x);
                            if(x_coor == current_x){ // x좌표가 목표 좌표 도달
                                call_motor(-HIGH,HIGH); // 좌회전
                                Clock_Delay1ms(rot_delay);
                                call_motor(HIGH,HIGH); // 직진
                                Clock_Delay1ms(straight_delay);
                                currentState = 2;
                                printf("\ncurrentState = %d\n",currentState);
                                break;
                            }
                        }
                        break;

                    case 2: // y좌표 증가시키기

                        if((test & 0b10000001) == 0b10000001  || (test & 0b01000010) == 0b01000010 || (test & 0b10000010) == 0b10000010 || (test & 0b01000001) == 0b01000001){ // check if +
                            printf("\n\n + Zone!!!\n\n");
                            current_y++;
                            printf("\ncurrent_y: %d\n",current_y);

                            if(y_coor == current_y){ // y좌표가 목표 좌표 도달
                                call_motor(HIGH,-HIGH); // 우회전
                                Clock_Delay1ms(rot_delay);
                                call_motor(HIGH,HIGH); // 직진
                                Clock_Delay1ms(straight_delay);
                                currentState = 3;
                                printf("\ncurrentState = %d\n",currentState);
                                break;
                            }

                        }
                        break;

                    case 3: // x좌표 5까지 증가시키기

                        if((test & 0b10000001) == 0b10000001  || (test & 0b01000010) == 0b01000010 || (test & 0b10000010) == 0b10000010 || (test & 0b01000001) == 0b01000001){ // check +
                            current_x++;
                            printf("\ncurrent_x: %d\n",current_x);
                            if(current_x == 5){ // x좌표 5에 도달
                                call_motor(-HIGH,HIGH); // 좌회전
                                Clock_Delay1ms(rot_delay);
                                call_motor(HIGH,HIGH); // 직진
                                Clock_Delay1ms(straight_delay);
                                currentState = 4;
                                printf("\ncurrentState = %d\n",currentState);
                                break;
                            }
                        }
                        break;

                    case 4:
                        if((test & 0b11111111) == 0b11111111  || (test & 0b11111111) == 0b01111110 || (test & 0b11111111) == 0b01111111 || (test & 0b11111111) == 0b11111110){ // check finish line
                            Motor_Stop();
                            printf("\ncurrentState = done.\n",currentState);
                            appState = Finish;
                            printf("\nappState = Finish\n");
                            break;
                        }
                        break;
                }
                state = state->next[ReadSensorData()];          // next depends on input and state
                break;

            case Finish:
                Motor_Stop();
                // LED 동작: 흰색 LED (빨강+파랑+녹색) 깜빡임
                P2->OUT |= (RED_LED | GREEN_LED | BLUE_LED);
                Clock_Delay1ms(1000);
                P2->OUT &= ~(RED_LED | GREEN_LED | BLUE_LED);
                Clock_Delay1ms(1000);
                break;
        }
    }
}

