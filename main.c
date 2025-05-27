#include <stdio.h>
#include <xc.h>

#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)


#define _XTAL_FREQ     8000000
#define Baud_rate 9600
#define LDR_THRESHOLD 65 // value of adc to turn on and off front lights 

void delay_ms(unsigned int);
void delay_us(unsigned int x);
void Initialize_UART(void); // initalize UART for bluetooth module
void Forward(); // moving function
void Backward(); // moving function
void Right(); // moving function
void Left(); // moving function
void Stop(); // moving function
void MOVE(); // respond to the char recived and move 
char UART_get_char(); // read char from bluetooth module 
unsigned int adc();//analog digital converter 
unsigned int Read_Ultrasonic1(void); //read distance from ultrasonic 1
unsigned int Read_Ultrasonic2(void); //read distance from ultrasonic 2
void PWM_Init();//for servo 
void PWM_Init_H();
void Set_PWM_Duty(unsigned int duty_us); // for servo motor
void SetPWM2DutyCycle(unsigned int duty_percent); // for h-bridge
void FlagUp();
void FlagDown();
void init_interrupt();
void __interrupt()myISR(void);
//***********************************************************
//var
char get_value; // store value recived from bluetooth
unsigned int adc_val;
unsigned int uv1;
unsigned int uv2;
char no_backward;
char no_forward;
char status = 's';

void main(void) {
    Initialize_UART();
    PWM_Init_H();
    Stop();
    /*
    h-bridge -> RC[5,4,3,0] (RC1 PWM)
    LDR -> RA0
    LED -> RA1
    Ultrasonic(1) -> RB4,RB5
    Ultrasonic(2) -> RB6,RB7
    Servo -> RC2
    RX -> RC7
    TX -> RC6
    Flame Sensor -> RD1
    Buzzer -> RD0
    Push button -> RB0 (interrupt)
    */
    TRISA = 0b00000001; //RA0 is input (ADC) LDR , front LED's
    TRISB = 0b10100001;
    TRISC = 0b10000000;
    TRISD = 0b00000010;

    FlagDown();
    init_interrupt();
    SetPWM2DutyCycle(40);
    while (1) {
        if (PORTD & (1 << 1)) { // check flame 
            PORTD &= ~(1 << 0); // RD0 = 0 buzzer off
        } else {
            PORTD |= (1 << 0); // RD0 = 1 buzzer on
        }
        adc_val = adc();
        if (adc_val > LDR_THRESHOLD) {
            PORTA |= (1 << 1); // Set RA1 Front light on 
        } else {
            PORTA &= ~(1 << 1); // Clear RA1 Front light off
        }

        uv2 = Read_Ultrasonic1();
        uv1 = Read_Ultrasonic2();
        if (uv1 < 7 && uv1 > 0) {
            if (status == 'f') {
                Stop();
            }
            FlagUp();
            no_forward = 1;
        } else {
            FlagDown();
            no_forward = 0;
        }
        if (uv2 < 7 && uv2 > 0) {
            if (status == 'b') {
                Stop();
            }
            no_backward = 1;
        } else {
            no_backward = 0;
        }
        MOVE();
    }
}
void __interrupt() myISR(void) {
    INTCON &= ~(1 << 7);           // Clear GIE (bit 7)
    
    if (INTCON & (1 << 1)) {       // If INTF (bit 1) is set
        Stop();
        INTCON &= ~(1 << 1);       // Clear INTF
    }

    INTCON |= (1 << 7);            // Set GIE (bit 7)
}

void init_interrupt() {
    INTCON |= (1 << 4);            // Set INTE (bit 4)
    INTCON |= (1 << 7);            // Set GIE (bit 7)
}


void delay_ms(unsigned int x) {
    unsigned int i, j;
    for (i = 0; i < x; i++) {
        delay_us(1000);
    }
}
void delay_us(unsigned int x) {
    unsigned int i;
    for (i = 0; i < x; i++) {
        asm("NOP");
        asm("NOP");
    }
}
void PWM_Init() {
    // Set RC2 as output
    PR2 = 249; // Period register for 50 Hz (20 ms)
    T2CON = 0b00000111; // Timer2 ON, Prescaler 1:16
    CCP1CON = 0b00001100; // CCP1 in PWM mode
    CCPR1L = 0; // Initial duty cycle (0%)
}

void Set_PWM_Duty(unsigned int duty_us) {
    // Calculate the PWM duty cycle from the pulse width (in microseconds)
    unsigned int duty = (duty_us * (_XTAL_FREQ / 4)) / (16 * (PR2 + 1) * 1000);
    
    // Set the upper 8 bits of the duty cycle in CCPR1L
    CCPR1L = duty >> 2; // Upper 8 bits
    
    // Clear CCP1X and CCP1Y bits in CCP1CON to prepare for the lower 2 bits
    CCP1CON &= 0xCF; // 0xCF = 11001111, clears bits 5 and 4
    
    // Set CCP1X (bit 5) and CCP1Y (bit 4) based on the lower 2 bits of the duty
    CCP1CON |= ((duty & 0x03) << 4);
    CCP1CONbits.CCP1Y |= ((duty & 0x03) << 4);
    
}

unsigned int Read_Ultrasonic1(void) {
    unsigned int timeout = 300;
    unsigned int time = 0;

    PORTB |= (1 << 4);           // RB4 = 1
    delay_us(10);
    PORTB &= ~(1 << 4);          // RB4 = 0

    while (!(PORTB & (1 << 5)) && timeout--) delay_us(1);  // Wait for RB5 high (echo start)
    if (timeout == 0) return -1;

    timeout = 30000;
    while ((PORTB & (1 << 5)) && timeout--) {  // Wait while RB5 is high
        delay_us(1);
        time++;
    }
    if (timeout == 0) return -1;

    return time / 5.8;
}

unsigned int Read_Ultrasonic2(void) {
    unsigned int timeout = 300;
    unsigned int time = 0;

    PORTB |= (1 << 6);           // RB6 = 1
    delay_us(10);
    PORTB &= ~(1 << 6);          // RB6 = 0

    while (!(PORTB & (1 << 7)) && timeout--) delay_us(1);  // Wait for RB7 high
    if (timeout == 0) return -1;

    timeout = 30000;
    while ((PORTB & (1 << 7)) && timeout--) {  // Wait while RB7 is high
        delay_us(1);
        time++;
    }
    if (timeout == 0) return -1;

    return time / 5.8;
}

void Initialize_UART(void) {
    SPBRG = ((_XTAL_FREQ / 16) / Baud_rate) - 1;

    TXSTA |= (1 << 2);          // BRGH = 1 High-speed baud rate is selected 
    TXSTA &= ~(1 << 4);         // SYNC = 0 Asynchronous mode is selected
    TXSTA |= (1 << 5);          // TXEN = 1 Transmitter is enabled
    TXSTA &= ~(1 << 6);         // TX9 = 0  8-bit transmission is used

    RCSTA |= (1 << 7);          // SPEN = 1 Serial port is enabled
    RCSTA |= (1 << 4);          // CREN = 1 Continuous receive mode is enabled
    RCSTA &= ~(1 << 6);         // RX9 = 0  8-bit reception is used 
}



void PWM_Init_H() {
    // Set RC1 as output (PWM2)

    // Configure CCP2 module
    CCP2CON = 0b00001100; // PWM mode

    //     Set PWM frequency: PR2 = [(Fosc / (PWM_freq * 4 * TMR2_Prescale)) - 1]
    // For 5kHz PWM, PR2 = [(8000000 / (5000 * 4 * 16)) - 1] = ~24
    PR2 = 199; // Use this for ~1kHz PWM with prescaler 16 (adjust as needed)

    T2CON = 0b00000111; // Timer2 on, prescaler 1:16

    TMR2 = 0; // Clear Timer2
}

void SetPWM2DutyCycle(unsigned int duty_percent) {
    unsigned int duty;

    // Convert percentage to 10-bit duty (max 1023)
    duty = ((unsigned long) duty_percent * 1023) / 100;

    CCPR2L = duty >> 2; // Upper 8 bits
    CCP1CON |= ((duty & 0x03) << 4);
}
void Forward() {
    PORTC |= (1 << 0);   // RC0 = 1
    PORTC &= ~(1 << 3);  // RC3 = 0
    PORTC |= (1 << 4);   // RC4 = 1
    PORTC &= ~(1 << 5);  // RC5 = 0
}

void Backward() {
    PORTC &= ~(1 << 0);  // RC0 = 0
    PORTC |= (1 << 3);   // RC3 = 1
    PORTC &= ~(1 << 4);  // RC4 = 0
    PORTC |= (1 << 5);   // RC5 = 1
}

void Right() {
    PORTC |= (1 << 0);   // RC0 = 1
    PORTC &= ~(1 << 3);  // RC3 = 0
    PORTC &= ~(1 << 4);  // RC4 = 0
    PORTC |= (1 << 5);   // RC5 = 1
}

void Left() {
    PORTC &= ~(1 << 0);  // RC0 = 0
    PORTC |= (1 << 3);   // RC3 = 1
    PORTC |= (1 << 4);   // RC4 = 1
    PORTC &= ~(1 << 5);  // RC5 = 0
}

void Stop() {
    PORTC &= ~(1 << 0);  // RC0 = 0
    PORTC &= ~(1 << 3);  // RC3 = 0
    PORTC &= ~(1 << 4);  // RC4 = 0
    PORTC &= ~(1 << 5);  // RC5 = 0
}


void MOVE() {
    get_value = UART_get_char();
    if (get_value == 'F' && no_forward == 0) {
        status = 'f';
        Forward();
    }
    if (get_value == 'B' && no_backward == 0) {
        status = 'b';
        Backward();
    }
    if (get_value == 'R') {
        status = 'r';
        Right();
    }
    if (get_value == 'L') {
        status = 'l';
        Left();
    }
    if (get_value == 'S') {
        status = 's';
        Stop();
    }
    if (get_value == 'M') {
        FlagUp();
    }
    if (get_value == 'm') {
        FlagDown();
    }
    if (get_value == 'N') {
        SetPWM2DutyCycle(40);
    }
    if (get_value == 'n') {
        SetPWM2DutyCycle(95);
    }
}
char UART_get_char() {
    // If Overrun Error (OERR) is set
    if (RCSTA & (1 << 1)) {
        RCSTA &= ~(1 << 4);  // CREN = 0 (Disable continuous receive)
        RCSTA |= (1 << 4);   // CREN = 1 (Enable it again)
    }

    // If no data is available (RCIF = 0)
    if (!(PIR1 & (1 << 5))) {
        return 0xFF;
    }

    return RCREG;
}


unsigned int adc() {
    unsigned int adcval;
    ADCON1 = 0xc0; //right justified
    ADCON0 = 0x85; //adc on, fosc/64
    while (GO_nDONE); //wait until conversion is finished
    adcval = ((ADRESH << 8) | (ADRESL)); //store the result
    adcval = (adcval / 3) - 1;
    return adcval;
}

void FlagUp() {
    PWM_Init();
    delay_ms(30);
    Set_PWM_Duty(319);
    delay_ms(30);
    PWM_Init_H();
}

void FlagDown() {
    PWM_Init();
    delay_ms(30);
    Set_PWM_Duty(60);
    delay_ms(30);
    PWM_Init_H();
}

// ***********************************************************************************************************
