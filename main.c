
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "Final_header.h"

long PWMcycle;
unsigned char i=0;
unsigned char j;
unsigned char k;

// array to keep track of special condition
void update_moves(char place,char move){
    if(place==0){
        moves[0] = move;       
    }
    else if(place==1){
        moves[1] = move;      
    }
}

// Resets moves in the array keeping track of special condition
void reset_moves(void){
    moves[0] = 2;
    moves[1] = 2;
    //place = 0; // Original has this
}

// Stores moves dynamically in an array
void save_moves(char direction,char* Recorded_moves){
    Recorded_moves[i++]=direction;
}

// Computes inverse of a list of moves
void backtrack(char* list) {
    
    char info_length = sizeof(list)/sizeof(list[0]);
    
         // as first one is f and second is duration
    for (j=0;j<=info_length;j++){
        if (list[j] == 'w'){
            reverse_direction[j] = 's';
        }
        else if (list[j] == 'a'){
            reverse_direction[j]= 'd';
        }
        else if (list[j]=='d'){
            reverse_direction[j] = 'a';
        }

        }
   
}

// function to make the car move in desired direction
//void Move_Car(char direction){
//    
//       
//    if (direction=='w'){
////        ClearDisplay;
//        motor_forward(&motorL,&motorR,50);
////        LCD_String("going forward");
//
//    }
//    
//    else{// (direction== 's'){
//
////            ClearDisplay;
//            motor_reverse(&motorL,&motorR,50);
////            LCD_String("going back");
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
     
//}


// Main function
void main(void){ 
    // For motor
    TRISBbits.TRISB0 = 0; 
    TRISBbits.TRISB1 = 0;
    TRISBbits.TRISB2 = 0;
    TRISBbits.TRISB3 = 0;   
    
    TRISCbits.RC5 = 1 ;   // INT2
    TRISCbits.RC3 = 1 ;   // INT0
    TRISCbits.RC4 = 1 ;   // INT1
    
    // For USART 
    TRISC = TRISC | 0b11000000; //set data direction registers
    // Oscillator
    OSCCON = 0b11110010;     // internal oscillator, 8MHz
    while(!OSCCONbits.IOFS); //Wait for OSC to become stable
   

 // initialising PWM registers and returning PWMcycle also known as PTPER
    PWMcycle = initPWM_DCmotor(); //PTPER, PWMperiod
    
    struct DC_motor motorL, motorR;
    
    //Starting values of both motors
    // assigning values to structure variable left motor
    motorL.dir_pin=0;          //pin RB0/PWM0
    motorL.PWMperiod=PWMcycle; //store PWMperiod for motor
    motorL.power=0;          //out of 100, controlled with pin RB1/PWM1
    motorL.dutyLowByte=(unsigned char *)(&PDC0L);  //this is for left motor
    motorL.dutyHighByte=(unsigned char *)(&PDC0H);
    motorL.direction=1; //set default motor direction <-ONLY CHANGE THIS 0/1

    //assigning values to structure variable right motor
    motorR.dir_pin = 2;        // RB2/PWM2
    motorR.PWMperiod=PWMcycle; //store PWMperiod for motor
    motorR.power=0;          // out of 100, controlled with pin RB3/PWM3
    motorR.dutyLowByte=(unsigned char *)(&PDC1L); // these for the right motor
    motorR.dutyHighByte=(unsigned char *)(&PDC1H);
    motorR.direction=1; //set default motor direction <-ONLY CHANGE THIS 0/1
    
    //Initialise Interrupts
    initInterrupts();
    
    //Initialise the LCD and USART
    LCD_Init(); 
    
    initEUSART(); 
    
    // Initializing variables
    leftie = 0;
    rightie = 0;
   
    moves[0] = 2;
    moves[1] = 2;
   
    flag =0;
    
    ResetCursor;
    
    
    while(!RFIDread){
        
        // If there's signal and in between ranges
        if (go==1 && flag ==1){
            //LCD_String("1");  
            motor_forward(&motorL,&motorR,60); // move forward
            save_moves('w',Recorded_moves);    // record move
            update_moves(0,1);                 // update move
            __delay_ms(250);
            flag = 0;                          // reset flag
        } 
        
        // If theres signal but not in range
        else if(go==0 && flag ==1){
            
            // if car passed the beacon turn left
            if(moves[0]==1 && moves[1]==0){
                //LCD_String("2"); 
                turn_left(&motorL,&motorR,60);
                save_moves('a',Recorded_moves);
                reset_moves();
            }
           // keep turning right
            else{
                turn_right(&motorL,&motorR,45);
                save_moves('d',Recorded_moves);
                update_moves(1,0);
            }
        } // end else if
      
        // If there is no signal keep turning until found
        else{ 
            //LCD_String("3"); 
            turn_right(&motorL,&motorR,45);
            save_moves('d',Recorded_moves);
            update_moves(1,0);
        }// end else
        
    }// end while
    
    // When RFID read 
    motor_stop(&motorL,&motorR);
    LCD_String(ReceivedData);   //Display RFID
    
    unsigned char computedCS = CompCheckSum();
    if (computedCS == result){
        LCD_String("True");
    }
    else{
        LCD_String("nah");
    }
    
    __delay_ms(5000);
    
    // Show Backtrack movement
    ResetCursor;
    ClearDisplay;
    
    LCD_String(Recorded_moves);
    backtrack(Recorded_moves);
    SetLine(2);
    LCD_String(reverse_direction);
    while(1);
} //end of main
    

    // &&&
//    reverse_direction = {'w'};
//    char lengthk = sizeof(reverse_direction)/sizeof(reverse_direction[0]);
//    for (k=0;k<=lengthk;k++){
//        if (reverse_direction[k] == 'w'){
//            motor_forward(&motorL,&motorR,60);
//        }
//        else if (reverse_direction[k] == 's'){
//            motor_reverse(&motorL,&motorR,60);
//        }
//        else if (reverse_direction[k]=='a'){
//            turn_left(&motorL,&motorR,45);
//        }
//        else if (reverse_direction[k]=='d'){
//            turn_right(&motorL,&motorR,60);
//        }
//    }
    // &&&


//    
//    //Move_Car(reverse_direction);
//    //Move_Car(1);
//    
//    //while(*reverse_direction++){
//    //    Move_Car(*reverse_direction++);
//    //}
//        
//    while(1);
    //LCD_String("bla");
    //readRFID();



