#ifndef FINAL_H
#define FINAL_H

#include <xc.h>

#pragma config OSC = IRCIO, WDTEN = OFF 

#define _XTAL_FREQ 8000000
#define TRUE 1
#define FALSE 0
#define INPUT 1
#define OUTPUT 0

#define LCD_RS LATAbits.LATA6
#define LCD_E LATCbits.LATC0

#define LCD_DP_C1 LATCbits.LATC1
#define LCD_DP_C2 LATCbits.LATC2
#define LCD_DP_D0 LATDbits.LATD0
#define LCD_DP_D1 LATDbits.LATD1

#define ResetCursor SendLCD(0b00000010,0)
#define ClearDisplay SendLCD(0b0000000001,0)

char buf[40];
char a, direction;

char IR_Left;
char IR_Right;
char LCDline_1[16];
char LCDline_2[16];
int leftie;
int rightie;
char reverse_direction[];
char Recorded_moves[];
char inverted[];
char duration;
char go;

//Newly added
char flag;
char moves[2];
char place = 0;
// Till here

unsigned char RFIDread = 0;

unsigned char ReceivedData[10];
unsigned char CheckSum[2];
unsigned char m;
unsigned char n;
unsigned char result;


struct DC_motor {
    char power;     //motor power, out of 100

    char direction; //motor direction, forward(1), reverse(0)

    unsigned char *dutyLowByte;  //PWM duty low byte address

    unsigned char *dutyHighByte; //PWM duty high byte address

    char dir_pin;  // pin that controls direction on PORTB

    int PWMperiod; //base period of PWM cycle

};


struct DC_motor motorL, motorR;

// Function Prototypes
void E_TOG(void);
void LCDout(unsigned char number);
void SendLCD(unsigned char Byte, char type);
void LCD_Init(void);
void SetLine (char line);
void LCD_String(char *string);

long initPWM_DCmotor(void);
void setMotorPWM(struct DC_motor *m);
void motor_stop(struct DC_motor *motorL, struct DC_motor *motorR);
void motor_forward(struct DC_motor *motorL, struct DC_motor *motorR, int pow);
void motor_reverse(struct DC_motor *motorL, struct DC_motor *motorR, int pow);
void turn_right(struct DC_motor *motorL, struct DC_motor *motorR, int pow);
void turn_left(struct DC_motor *motorL, struct DC_motor *motorR, int pow);
//void switch_func(char a);
void delay_s(char seconds);
char getCharSerial(void);
void initEUSART(void);
void sendUSART(char *string);
void initInterrupts(void);
void save_time(char duration,char* Recorded_time);
void __interrupt(high_priority) Interrupt_RFID();
void readRFID(void);
unsigned char AsciiToHex(unsigned char ToConvert);
unsigned char CompCheckSum(void);
void __interrupt(low_priority) interrupt_IR();
char Direction(char IR_Left, char IR_Right,int RangeLow, int RangeUp);
void Move_Car(char direction);

//void update_moves(char place,char move);
//void reset_moves(void);

#endif /* XC_HEADER_TEMPLATE_H */
