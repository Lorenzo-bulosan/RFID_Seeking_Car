#include "Final_header.h"


void E_TOG(void){
    LCD_E = 1;
    __delay_us(5);
    LCD_E = 0;
    __delay_us(5);
}

//function to send four bits to the LCD
void LCDout(unsigned char number){
    LCD_DP_D1 = ((0b1000 & number)>>3);
    LCD_DP_D0 = ((0b0100 & number)>>2);
    LCD_DP_C2 = ((0b0010 & number)>>1);
    LCD_DP_C1 = (0b0001 & number);
    __delay_ms(15);
    E_TOG();
    __delay_us(5); // 5us delay
    
}

//function to send data/commands over a 4bit interface
void SendLCD(unsigned char Byte, char type){
    // how do we treat a character as a number here automatically?
     unsigned char Lbits = (0b00001111 & Byte);
     unsigned char Hbits = ((0b11110000 & Byte)>>4);
     // set RS pin whether it is a Command (0) or Data/Char (1)
     // using type as the argument
     LCD_RS = type;
     LCDout(Hbits);
     __delay_us(10); 
     LCDout(Lbits);
     __delay_us(10);
}

// Function to initialize specific LCD
void LCD_Init(void){
    // set LCD pins as output (TRIS registers)
    TRISAbits.RA6 = 0;
    TRISCbits.RC0 = 0;

    TRISCbits.RC1 = 0;
    TRISCbits.RC2 = 0;
    TRISDbits.RD0 = 0;
    TRISDbits.RD1 = 0;
    // set initial LAT output values (they start up in a random state)
    LATC = 0;
    LATD = 0;
    LATA = 0;
    // Initialisation sequence code - see the data sheet
    __delay_ms(15);
    LCDout(0b0011);
    __delay_ms(5);
    LCDout(0b0011);
    __delay_us(200);
    LCDout(0b0011);
    __delay_us(50);
    LCDout(0b0010);
    __delay_us(50);
   
    SendLCD(0b00101000,0); //function set, if use one line then it's blurred text
    __delay_us(50);
    SendLCD(0b00001000,0); //display off
    __delay_us(50);
    SendLCD(0b00000001,0); //clear display
    __delay_us(50);
    SendLCD(0b00000110,0); //set entry mode
    __delay_us(50);
    //SendLCD(0b00011000,0); // shift, single shift if outside the while loop
    //__delay_us(50);
    SendLCD(0b00001100,0); //display on
    __delay_us(50);
    
}

//function to put cursor to start of line
void SetLine (char line){
    //Send 0x80 to set line to 1 (0x00 ddram address)
    //Send 0xC0 to set line to 2 (0x40 ddram address)
    if(line==1){SendLCD(0x80, 0);}
    if(line==2){SendLCD(0xC0, 0);}
     __delay_us(50); // 50us delay
}

// Function to send string to LCD
void LCD_String(char *string){
    while(*string != 0){
     //Send out the current byte pointed to
     // and increment the pointer
    SendLCD(*string++,1);
     }
}

// Function to initialize motors
long initPWM_DCmotor(){

    /*Initialising for DC motor*/

    // change to 1:1 prescaler. pic datasheet pg178
    PTCON0 = 0b00000000; // 1:1 prescaler, free running mode
    PTCON1 = 0b10000000; // enable PWM timer,  PWM time base is on

    /* This is for DC motor. we enable PWM1 + PWM3 as per pic datasheet pg179-pg180*/
    PWMCON0 = 0b01101111; // PWM1 PWM3 enabled, all independent mode 
    PWMCON1 = 0x00;       // special features, all 0 (default)

    /*****Obtaining PTPER and PDC0 and PDC1 for DC motor*******/  
    //calculate using formula and put in binary
    long PTPER = 0b0000000011000111; //= 199, for period of 1/10khz = 0.1ms , @1:1 prescaler, 8Mhz
    PTPERL = PTPER & 0b0000000011111111; // Base PWM period low byte
    PTPERH = PTPER >> 8;    // Base PWM period high byte
    return PTPER;
}
 
// Function to set motor PWM
void setMotorPWM(struct DC_motor *m)
{
    int PWMduty; //tmp variable to store PWM duty cycle  
    // Forward
    //direction channel is high - motor power/speed defined by off time of duty cycle
    //so if power == 100 have PWMduty == 0 and motor working full power
    if (m->direction){
        PWMduty=m->PWMperiod - ((int)(m->power)*(m->PWMperiod))/100; // low time increases with power
    }
    // Reverse

    //direction channel is low - motor power/speed defined by on time of duty cycle
    else {
        PWMduty=((int)(m->power)*(m->PWMperiod))/100; // high time increases with power
    }
    //this sets the duty cycle bits
    PWMduty = (PWMduty << 2);                   // two LSBs are reserved for other things
    *(m->dutyLowByte) = PWMduty & 0xFF;         //set low duty cycle byte
    *(m->dutyHighByte) = (PWMduty >> 8) & 0x3F; //set high duty cycle byte
    //this gives upper 6 bits from a byte for PDC0H
    
    // if direction is high

    if (m->direction)
    {
        LATB=LATB | (1<<(m->dir_pin));  
        //LATBbits.LATB2 = 0;// m->direction;// set dir_pin bit in LATB to high
    }
    // if direction is low
    else
    {
        LATB=LATB & (~(1<<(m->dir_pin))); // set dir_pin bit in LATB to low
        //LATBbits.LATB2 = 1;// m->direction;
    }

}

// Function to stop the car
void motor_stop(struct DC_motor *motorL, struct DC_motor *motorR)
{
    //this assumes that motor powers are always equal
    for (motorL->power;(motorL->power) > 0;(motorL->power)--){
        motorR->power = motorL->power;
        setMotorPWM(motorL); // here not use &
        setMotorPWM(motorR);
        __delay_ms(5);
    }
}

// function to go forward at user given power
void motor_forward(struct DC_motor *motorL, struct DC_motor *motorR, int pow)
{  
    //first, stop the motors from going (backward) by reducing speed, then
    //change direction and increase power
   
    //if direction is 0, then reverse but first stopping
    for (motorL->power;(motorL->power) > 0;(motorL->power)--){
            motorR->power = motorL->power;
            setMotorPWM(motorL); // here not use &
            setMotorPWM(motorR);
            __delay_ms(5);
        }
    
    motorL->direction = 1; //change direction for the left motor
    motorR->direction = 1; //change direction for the right motor
    setMotorPWM(motorL); // here not use &
    setMotorPWM(motorR);
    __delay_ms(5);

    for (motorR->power;(motorR->power) < pow;(motorR->power)++){
        motorL->power = motorR->power;
        setMotorPWM(motorL); // here not use &
        setMotorPWM(motorR);
        __delay_ms(5);
    }
}

void motor_reverse(struct DC_motor *motorL, struct DC_motor *motorR, int pow)
{
    //first, stop the motors from going (forward) by reducing speed, then
    //change direction and increase power
    for (motorL->power;(motorL->power) > 0;(motorL->power)--){
            motorR->power = motorL->power;
            setMotorPWM(motorL); // here not use &
            setMotorPWM(motorR);
            __delay_ms(5);
        }
    
    motorL->direction = 0; //change direction for the left motor
    motorR->direction = 0; //change direction for the right motor
    setMotorPWM(motorL); // here not use &
    setMotorPWM(motorR);
    __delay_ms(5);

    for (motorR->power;(motorR->power) < pow;(motorR->power)++){
        motorL->power = motorR->power;
        setMotorPWM(motorL); // here not use &
        setMotorPWM(motorR);
        __delay_ms(5);
    }
}

void turn_left(struct DC_motor *motorL, struct DC_motor *motorR, int pow)
{
    for (motorL->power;(motorL->power) > 0;(motorL->power)--){
            motorR->power = motorL->power;
            setMotorPWM(motorL); // here not use &
            setMotorPWM(motorR);
            __delay_ms(5);
        }
    
    motorL->direction = 0; //change direction for the left motor
    motorR->direction = 1; //change direction for the right motor
    setMotorPWM(motorL); // here not use &
    setMotorPWM(motorR);
    __delay_ms(5);

    for (motorR->power;(motorR->power) < pow;(motorR->power)++){
        motorL->power = motorR->power;
        setMotorPWM(motorL); // here not use &
        setMotorPWM(motorR);
        __delay_ms(5);
    }
}

void turn_right(struct DC_motor *motorL, struct DC_motor *motorR, int pow)
{
    for (motorL->power;(motorL->power) > 0;(motorL->power)--){
            motorR->power = motorL->power;
            setMotorPWM(motorL); // here not use &
            setMotorPWM(motorR);
            __delay_ms(5);
        }
    
    motorL->direction = 1; //change direction for the left motor
    motorR->direction = 0; //change direction for the right motor
    setMotorPWM(motorL); // here not use &
    setMotorPWM(motorR);
    __delay_ms(5);

    for (motorR->power;(motorR->power) < pow;(motorR->power)++){
        motorL->power = motorR->power;
        setMotorPWM(motorL); // here not use &
        setMotorPWM(motorR);
        __delay_ms(5);
    }
}

void delay_s(char seconds){
    unsigned char k;
    unsigned char loops;
    loops = seconds * 20;//1000/50;
    //50ms * 20 == 1000
    for(k=0;k<loops;k++){
        __delay_ms(50);
    }
}

// Function to obtain characters over USART interface
char getCharSerial(void){
    while (!PIR1bits.RCIF); //wait for the data to arrive
    //LCD_String("bla");
    return RCREG; 
}

// Function to initialize USART 
void initEUSART(){
    //calc via [SPBRG:SPBRGH] = (Fosc/4)/Baud Rate
    SPBRG = 207;//207;//. dont set this to be a function because it takes up 
    //all the memory, 80-85%
    //pic data sheet p219 - 220. low byte
    // that would be the value if there is no prescaler
    //103==19200
    SPBRGH=0; // high byte
    BAUDCONbits.BRG16=1; //set baud rate register to 16 bit mode, p220
    // this means we're using 16 bits to store Baud rate
    TXSTAbits.BRGH=1; //high baud rate select bit, p218, 1 = high speed
    RCSTAbits.CREN=1; //continuous receive mode, p219
    RCSTAbits.SPEN=1; //enable serial port, other settings default, p219
    TXSTAbits.TXEN=1; //enable transmitter, other settings default, p218
    
    //TXSTAbits.SYNC = 0; //Set to asynchronous
}

// Function to send string over USART
void sendUSART(char *string){
    while(*string != 0){             
        while(!PIR1bits.TXIF);
        TXREG = *string++;     
    }
}

// Function to initialise interrupts on INT0, INT1 and INT2
void initInterrupts(){
    // This function is to set interrupts INT1 and INT2
    //To enable different priority modes
    RCONbits.IPEN = 1; 
    // Set global interrupts
    INTCONbits.GIEL = 1; //if comment this then motors work
    INTCONbits.GIEH = 1;
//
//  priotity selection
    INTCON3bits.INT2IP=1;   //High
    INTCON3bits.INT1IP=0;   //low
    
    //Enable INT1 and INT2
    INTCON3bits.INT1IE = 1;
    INTCON3bits.INT2IE = 1;
    
//    INTCONbits.INT0IE = 1; 
   
    // From Datasheet Pg 99
//    INTCON2bits.INTEDG0 = 0;   //Interrupt INT0 on Falling Edge
    INTCON2bits.INTEDG1 = 0;   //Interrupt INT1 on Falling Edge
    INTCON2bits.INTEDG2 = 0;   //Interrupt INT2 on Falling Edge

    //Flags for INT1 and INT2 are in INTCON3
}

// Function to save duration of a movement
void save_time(char duration,char* Recorded_time){
    char length = sizeof(Recorded_time)/sizeof(Recorded_time[0]);
    Recorded_time[length+1]=duration;
}

// Function to convert Ascii to Hex
unsigned char AsciiToHex(unsigned char ToConvert)
{
    //want to convert string characters into actual hex numbers
    // we get characters because USART is set up to receive characters
    unsigned char converted;
    // Ascii 48 to 57 are numbers 0-9,
    //so we convert them to actual numbers in hex
    if((ToConvert >= '0') && (ToConvert <= '9')){
        converted = ToConvert - '0';
    }
    // now we want to convert letter sent in Ascii to
    // corresponding letters in hex
    // Ascii 65 to 70 are letters A-F
    else if((ToConvert >= 'A') && (ToConvert <= 'F')){
        converted = 10 + ToConvert - 'A';
    }
    return converted;
}

// Function to compute checksum
unsigned char CompCheckSum(void)
{
    unsigned char ComputedCS = 0;
    unsigned char i;
    unsigned char TempConv[10];
    unsigned int Final[5];


    for(i=0;i<10;i++){
        // here we store received data converted to hex
        // each elements of an array is a single hex number 0-F
        TempConv[i] = AsciiToHex(ReceivedData[i]);
    }

    for(i=0;i<5;i++){
        // we bit shift the higher nibble to the left to get a full byte
        // or a hex with two digits, which is a single cell in Final array
        Final[i] =  TempConv[(i*2)+1] | TempConv[(i*2)] << 4;
        // now XOR all the final
        ComputedCS ^= Final[i];
    }

    return ComputedCS;
}

// Function to read RFID
void readRFID(void){
//    LCD_String("before1");
    unsigned char Char;
    unsigned char i;
//    LCD_String("1");
//    ResetCursor;
//    __delay_ms(2);
    Char = getCharSerial();
//    LCD_String("2");
//    ResetCursor;
//    __delay_ms(2);
    while(Char != 0x03){
//        LCD_String("in1");
//        ResetCursor;
//        __delay_ms(2);
        if (Char == 0x02){
//            LCD_String("3");
//            ResetCursor;
//            __delay_ms(2);
            for(i=0;i<10;i++){
                Char = getCharSerial();
                ReceivedData[i] = Char;
//                LCD_String("4");
            }
            for(i=0;i<2;i++){
                Char = getCharSerial();
                CheckSum[i] = Char;
            }
        }
        Char = getCharSerial();
    }
}

// Interrupt routine for RFID
void __interrupt(high_priority) Interrupt_RFID()
{ 
    //Check flag at INT1
    if (INTCON3bits.INT2IF)
    {
        //read values and compute checksum from it, then compare to the 
        //cehcksum read
        RFIDread = 1;
        readRFID();
        m = AsciiToHex(CheckSum[0]);
        n = AsciiToHex(CheckSum[1]);
        result = ((m<<4)|n);
        //clear the interrupt flag
        INTCON3bits.INT2IF = 0;
    }
}

// Interrupt routine for IR sensors
void __interrupt(low_priority) interrupt_IR()
{ 
    //Check flag at INT1
    if (INTCON3bits.INT1IF)
    {    
       flag =1;    // signifies theres signal
       leftie = 0; // restart counter
       
       while (PORTCbits.RC3 == 0){
            leftie++;
            } 
       
       // Check if IR emitter is in front
       if (11000<leftie && leftie<15000){
           go = 1;
       }
       
       // IR is not in front
       else {
           go= 0;
       }
        
    }
    INTCON3bits.INT1IF = 0;
      
}


//void Move_Car(char direction){
//    
//       
//    if (direction=='w'){
//        ClearDisplay;
//        motor_forward(&motorL,&motorR,50);
//        LCD_String("going forward");
//
//    }
//    
//    else if (direction=='s'){
//
//            ClearDisplay;
//            motor_reverse(&motorL,&motorR,50);
//            LCD_String("going back");
//
//    }
//    
//    else if (direction=='a'){
//           
//            ClearDisplay;
//            turn_left(&motorL,&motorR,50);
//            LCD_String("turning left");
//
//    }
//    
//    else if (direction=='d'){
//            ClearDisplay;
//            turn_right(&motorL,&motorR,50);
//            LCD_String("turning right");
//
//    }
//    
//    else if (direction=='z'){
//            ClearDisplay;
//            motor_stop(&motorL,&motorR);
//            LCD_String("stopping");
//    }
//     
//}


// Newly added
//void update_moves(char place,char move){
//    if(place==0){
//        moves[0] = move;       
//    }
//    else if(place==1){
//        moves[1] = move;      
//    }
//}
//
//void reset_moves(void){
//    moves[0] = 2;
//    moves[1] = 2;
//    //place = 0; // Original has this
//}