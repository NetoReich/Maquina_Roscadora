/*
 * File:   MAIN.c
 * Author: Johann Haeussler
 *
 * Created on 1 de septiembre de 2025, 05:19 PM
 */

// PIC16F887 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1
// soft_uart_tx_16f887.c  (XC8 - PIC16F887)
// Soft UART TX por bit-banging con Timer1
// Configuración: Fosc interno = 8 MHz, Baud = 9600
// Pin TX = RD5
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Librerías
#include <xc.h>
#include <stdint.h>
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Bits de configuración
#pragma config FOSC = INTRC_NOCLKOUT
#pragma config WDTE = ON
#pragma config PWRTE = OFF
#pragma config MCLRE = ON
#pragma config CP = OFF
#pragma config CPD = OFF
#pragma config BOREN = OFF
#pragma config IESO = OFF
#pragma config FCMEN = OFF
#pragma config LVP = OFF
#pragma config BOR4V = BOR40V
#pragma config WRT = OFF
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Definiciones
#define _XTAL_FREQ          8000000UL           // 8 MHz (INTOSC)
#define SOFTUART_BAUD       9600UL              // Baud Soft UART
#define SOFT_UART_TX_TRIS   TRISDbits.TRISD5
#define SOFT_UART_TX        PORTDbits.RD5
#define WDT_LED_TRIS        TRISEbits.TRISE0
#define WDT_LED             PORTEbits.RE0
#define SOFTUART_T1_PRELOAD 65395               // 65536−141=65395 Valor obtenido de forma experimental
#define LED_RA7_TRIS        TRISAbits.TRISA7
#define LED_RA7             PORTAbits.RA7
#define LED_RA6_TRIS        TRISAbits.TRISA6
#define LED_RA6             PORTAbits.RA6
#define LED_RD0_TRIS        TRISDbits.TRISD0
#define LED_RD0             PORTDbits.RD0
#define DIn_RD1_TRIS        TRISDbits.TRISD1
#define DIn_RD1             PORTDbits.RD1
#define DIn_RD2_TRIS        TRISDbits.TRISD2
#define DIn_RD2             PORTDbits.RD2
#define EN_Out_RD3_TRIS    TRISDbits.TRISD3
#define EN_Out_RD3         PORTDbits.RD3
#define EN_Out_RD4_TRIS    TRISDbits.TRISD4
#define EN_Out_RD4         PORTDbits.RD4
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Funciones de SoftUART para canal de Debug en RD5
void SoftUART_Timer1_Init(void){
    T1CONbits.T1CKPS = 0b00;                // Prescaler 1:1
    T1CONbits.TMR1CS = 0;                   // Reloj interno Fosc/4
    T1CONbits.T1OSCEN = 0;                  // Osc T1 off
    T1CONbits.TMR1ON = 0;                   // Timer1 apagado
    PIR1bits.TMR1IF = 0;                    // Limpia flag
}
void SoftUART_Init(void){                   // Configurar oscilador interno a 8 MHz
    OSCCONbits.IRCF = 0b111;                // IRCF=111 → 8 MHz
    OSCCONbits.SCS  = 0b10;                 // Reloj interno
    SOFT_UART_TX_TRIS = 0;                  // Pin TX como salida
    SOFT_UART_TX = 1;                       // Reposo UART = nivel alto
}
void SoftUART_T1BitDelay(void){
    PIR1bits.TMR1IF = 0;
    TMR1H = (uint8_t)(SOFTUART_T1_PRELOAD >> 8);
    TMR1L = (uint8_t)(SOFTUART_T1_PRELOAD & 0xFF);
    T1CONbits.TMR1ON = 1;
    while (!PIR1bits.TMR1IF);               // Espera 1 bit-time
    T1CONbits.TMR1ON = 0;
}
void SoftUART_WriteChar(char c){
    SOFT_UART_TX = 0;                       // Start bit
    SoftUART_T1BitDelay();
    for (uint8_t i = 0; i < 8; i++) {       // 8 bits (LSB primero)
        SOFT_UART_TX = (c >> i) & 0x01;
        SoftUART_T1BitDelay();
    }
    SOFT_UART_TX = 1;                       // Stop bit
    SoftUART_T1BitDelay();
}
void Send_Debug(const char *s){
    while (*s) {
        SoftUART_WriteChar(*s++);
    }
    SoftUART_WriteChar('\r');
    SoftUART_WriteChar('\n');
}
void Send_DebugNum(unsigned int val){
    char buf[6];                            // hasta "65535" + '\0'
    char *p = &buf[5];
    *p = '\0';
    do {                                    // Convierte el número a string decimal
        *(--p) = '0' + (val % 10);
        val /= 10;
    } while(val > 0);
    Send_Debug(p);                          // Envía con salto de línea
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Funciones para PWM en CCP1 y CCP2
void PWM_Init_1kHz(void){
    // Timer2: 1 kHz @ Fosc=8 MHz, prescaler 1:16 → PR2=124
    PR2 = 124;
    T2CONbits.T2CKPS = 0b11;     // 1:16
    T2CONbits.TMR2ON = 1;

    // Modo PWM en CCP1 y CCP2 (salidas físicas RC2 y RC1)
    CCP1CON = 0b00001100;        // CCP1 PWM
    CCP2CON = 0b00001100;        // CCP2 PWM

    // Dejar pines inactivos al inicio (tri–state)
    TRISCbits.TRISC2 = 1;        // RC2 (CCP1) inactivo
    TRISCbits.TRISC1 = 1;        // RC1 (CCP2) inactivo

    // Duty inicial = 0%
    CCPR1L = 0; 
    CCP1CONbits.DC1B0 = 0; 
    CCP1CONbits.DC1B1 = 0;
    CCPR2L = 0; 
    CCP2CONbits.DC2B0 = 0; 
    CCP2CONbits.DC2B1 = 0;
}
static inline void PWM_SetDuty_CCP1_10bit(unsigned int d){
    unsigned int maxd = (PR2 << 2) + 3;                         // 4*(PR2+1)-1
    if (d > maxd) {
        d = maxd;
    }
    CCP1CONbits.DC1B0 =  (d     & 0x01);
    CCP1CONbits.DC1B1 = ((d>>1) & 0x01);
    CCPR1L = (unsigned char)(d >> 2);
}

static inline void PWM_SetDuty_CCP2_10bit(unsigned int d){
    unsigned int maxd = (PR2 << 2) + 3;                         // 4*(PR2+1)-1
    if (d > maxd){
        d = maxd;
    } 
    CCP2CONbits.DC2B0 =  (d     & 0x01);
    CCP2CONbits.DC2B1 = ((d>>1) & 0x01);
    CCPR2L = (unsigned char)(d >> 2);
}
static inline void PWM_SetDuty_CCP1_pct(unsigned char duty_pct){    // Helper para visualizar duty cycle con porsentaje
    unsigned int maxd = (PR2 << 2) + 3;                             // 0..499 (con PR2=124)
    unsigned int d = ((unsigned long)maxd * duty_pct) / 100;
    PWM_SetDuty_CCP1_10bit(d);
}
static inline void PWM_SetDuty_CCP2_pct(unsigned char duty_pct){
    unsigned int maxd = (PR2 << 2) + 3;
    unsigned int d = ((unsigned long)maxd * duty_pct) / 100;
    PWM_SetDuty_CCP2_10bit(d);
}
void PWM_Enable(unsigned char channel, unsigned char enable){       // Activar / Desactivar salida por canal
    if(channel == 1){                                               // CCP1 → RC2
        TRISCbits.TRISC2 = enable ? 0 : 1;                          // 0=activa, 1=inactiva (hi-Z)
    }
    else if(channel == 2){                                          // CCP2 → RC1
        TRISCbits.TRISC1 = enable ? 0 : 1;
    }
}
static inline void PWM_SetPulseFreq_Hz(unsigned int f_hz){          // Con prescaler fijo 1:16 (ya configurado): f_PWM = Fosc / (4*16*(PR2+1)) = _XTAL_FREQ / (64*(PR2+1))
    if (f_hz == 0) return;                                          // Evitar división por cero
    unsigned long pr = (_XTAL_FREQ / (64UL * (unsigned long)f_hz)) - 1UL;
    if (pr > 255UL) pr = 255UL;                                     // Límite de PR2
    PR2 = (unsigned char)pr;                                        // Actualiza período del PWM (afecta ambos CCP)
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Verificar Reset por WDT
unsigned char WasResetByWDT(void){
    if ((STATUS & 0x10) == 0) {                       // Bit4=TO; 0 => WDT
        return 1;
    } 
    else {
        return 0;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Funciones de iniciación
void setup(void){
    SoftUART_Init();
    SoftUART_Timer1_Init();
    // Entradas
    DIn_RD1_TRIS = 1;
    DIn_RD2_TRIS = 1;
    // Salidas
    LED_RA7_TRIS = 0;
    LED_RA7 = 0;
    LED_RA6_TRIS = 0;
    LED_RA6 = 0;
    LED_RD0_TRIS = 0;
    LED_RD0 = 0;
    WDT_LED_TRIS = 0;
    WDT_LED = 0;
    EN_Out_RD3_TRIS = 0;
    EN_Out_RD3 = 0;
    EN_Out_RD4_TRIS = 0;
    EN_Out_RD4 = 0;
    
    PWM_Init_1kHz();
    Send_Debug("INIT");
}
void InitBlink(void){
    LED_RD0 = 1;
    __delay_ms(250);
    LED_RD0 = 0;
    __delay_ms(250);
    LED_RD0 = 1;
    __delay_ms(250);
    LED_RD0 = 0;
    __delay_ms(250);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Función principal del sistema
void main(void){
    setup();
    if (WasResetByWDT()){
        WDT_LED = 1;
        Send_Debug("WDT_RST");
        __delay_ms(500);
        WDT_LED = 0;
    }
    InitBlink();
    Send_Debug("INIT");
    LED_RD0 = 1;                // Indicador de sistema encendido
    EN_Out_RD3 = 1;             // Habiltado en LOW
    EN_Out_RD4 = 1;
    PWM_SetPulseFreq_Hz(2000);  // 2kHz
    PWM_SetDuty_CCP1_pct(50);   // 50%
    PWM_SetDuty_CCP2_pct(50);   // 50%
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Bucle principal del sistema
    while (1) {
        if ((DIn_RD1) && (!DIn_RD2)){
            EN_Out_RD3 = 1;
            EN_Out_RD4 = 0;
            LED_RA7 = 1;
            LED_RA6 = 0;
            PWM_Enable(1, 1);
            PWM_Enable(2, 0);
        }
        else if ((DIn_RD2) && (!DIn_RD1)){
            EN_Out_RD3 = 0;
            EN_Out_RD4 = 1;
            LED_RA6 = 1;
            LED_RA7 = 0;
            PWM_Enable(1, 0);
            PWM_Enable(2, 1);
        }
        else if ((DIn_RD1) && (DIn_RD2)){
            EN_Out_RD3 = 0;
            EN_Out_RD4 = 0;
            LED_RA6 = 1;
            LED_RA7 = 1;
            PWM_Enable(1, 1);
            PWM_Enable(2, 1);
        }
        else {
            EN_Out_RD3 = 1;
            EN_Out_RD4 = 1;
            LED_RA6 = 0;
            LED_RA7 = 0;
            PWM_Enable(1, 0);
            PWM_Enable(2, 0);
        }
        CLRWDT();
    }
}
