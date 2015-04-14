/*
 * File:   mainVCRT.c
 * Author: fcc89
 *
 * Created on April 11, 2015, 9:28 AM
 */


#include "xc.h"
#include <stdio.h>
// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Mode (Internal Fast RC (FRC))
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start-up device with user-selected oscillator source)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

#define    FCY    40000000UL

#include <libpic30.h>
#include <math.h>
#include "config.h"

//variables de control 
//control hobby servo
float resHobby=0.164;

//lecturas de distancia
int sensor1=65535;
int sensor2=65535;
int sensor3=65535;
int sensor4=65535;
int sensor5=65535;

//variables para control pid de posicion
double dt=0.00050;
double dt_O=0.0050;

double E_1=0;   //parte integral del controlador pid(suma de errores)
double e_1=0;   //error instantaneo
double e_dot_1=0;//error diferencial
double e_old_1=0;//error de la anterior medición
double KP_1=40.00;//constante proporcional
double KI_1=1.0;//constante integral
double KD_1=0.04;//constante derivativa

double E_2=0;   //parte integral del controlador pid(suma de errores)
double e_2=0;   //error instantaneo
double e_dot_2=0;//error diferencial
double e_old_2=0;//error de la anterior medición
double KP_2=80.00;//constante proporcional
double KI_2=2.0;//constante integral
double KD_2=0.2;//constante derivativa


double acum_1;
double acum_2;
unsigned int p_rev_1=2300; //valor máximo alcanzable en movimiento continuo antes de reiniciar los valores  para mantener precisión
unsigned int p_rev_2=2300;
double pos_act_1=0.0;
double pos_act_2=0.0;

double pos_mcont_1=0; //posicion deseada usada para movimiento continuo
double pos_mcont_2=0;

double KI_t_1=0.0000;//constante integral integral multiplicada por el tiempo
double KD_t_1=0.0000;//constante derivativa dividida entre el tiempo
double KI_t_2=0.0000;//constante integral integral multiplicada por el tiempo
double KD_t_2=0.0000;//constante derivativa dividida entre el tiempo



double pos_des_1=0.0;    //posicion deseada instantanea para el controlador pid
double pos_obj_1=0.0;    //posicion final de destino
double etapa1_1=0.0;     //desplazamiento en la primera mitad del recorrido
double r_vel_cte_1=0.0;    //registro de # pulsos a velocidad constante
double dstot_1=0.0;      //desplazamiento total en pulsos

double pos_des_2=0.0;    //posicion deseada instantanea para el controlador pid
double pos_obj_2=0.0;    //posicion final de destino
double etapa1_2=0.0;     //desplazamiento en la primera mitad del recorrido
double r_vel_cte_2=0.0;    //registro de # pulsos a velocidad constante
double dstot_2=0.0;      //desplazamiento total en pulsos





char p1sat=0;     //variable para comprobar saturación del pwm 1
char p2sat=0;     //variable para comprobar saturación del pwm 2

///////////////////////////////////////////////////////////////////////////////
float vL=0;    //velocidad rueda izquierda en pulsos/dt en este caso dt = 500 ns
float vR=0;    //velocidad rueda derecha en pulsos/dt

float L=0.19;   //distancia entre ruedas en metros
float D=0.0775;

float V=0.00; //velocidad del robot en m/s
float W=0; //velocidad angular del robot

double dl=0;
double dr=0;
double dc=0;

double x=0;
double y=0;
double o=0;

double x_trgt=0;
double y_trgt=0;

double pos_last_1=0;
double pos_last_2=0;


double E_O=0; 
double e_o=0;
double pos_des_o=0;
double pos_act_o=0;
double e_old_o=0;
double e_dot_o=0;
double KP_O=1.0;//constante proporcional
double KI_O=0.8;//constante integral
double KD_O=0.00;//constante derivativa
double KI_t_O=0.0000;//constante integral integral multiplicada por el tiempo
double KD_t_O=0.0000;//constante derivativa dividida entre el tiempo

//pruebas

float dutym1=0;
float dutym2=0;

//FUNCIONES

void putch(unsigned char val) {
    while (!U1STAbits.TRMT);
    U1TXREG = val;
}
void set_Hservo1(float a){
    if(a<-90)
        a=-90;
    if(a>90)
        a=90;
    OC1RS=(unsigned int)(a/resHobby+1100);
    
}
void set_Hservo2(float a){
    if(a<-90)
        a=-90;
    if(a>90)
        a=90;
    OC2RS=(unsigned int)(a/resHobby+1100);
}

void set_duty_1(double d){//permite al pid trabajar con valores negativos con el
                          //pwm y ademas notificar si el sistema se encuentra
                          //saturado
    if(d>830||d<-830){
        if(d>830)
           d=830;
        if(d<-830)
           d=-830;
        p1sat=1; //actuador saturado
    }
    else p1sat=0;

    P1DC1=(unsigned int)(d+2000);
}
void set_duty_2(double d){//permite al pid trabajar con valores negativos con el
                          //pwm y ademas notificar si el sistema se encuentra
                          //saturado
    if(d>1000||d<-1000){
        if(d>1000)
           d= 1000;
        if(d<-1000)
           d=-1000;
        p2sat=1; //actuador saturado
    }
    else p2sat=0;

    P1DC2=(unsigned int)(d+2000);
}
void update_pos(){//actualiza la posicion actual en base al registro del modulo
                  //de encoder y el movimiento total realizado por el equipo
    pos_act_1=(double)(POS1CNT+acum_1);
    pos_act_2=(double)(POS2CNT+acum_2);
}


void mov_prof(){
    if(!p1sat)
        pos_des_1+=vL;
    if(!p2sat)
        pos_des_2+=vR;
}
void calc_vel(){
    vR=121.89181*V+11.5805*W;
    vL=121.89181*V-11.5805*W;
}

//INTERRUPCIONES
//RECEPCIÓN SERIAL
//UART1
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    char a=U1RXREG;
    //putch(a);
    if(a=='A'){
        x_trgt=1.5;
        y_trgt=1;
        V=0.05;
    }
    if(a=='B'){
        x_trgt=2;
        y_trgt=2;
        V=0.05;
    }

    if(a=='C')
        W=0;
    if(a=='D')
        V=0;
    calc_vel();
    IFS0bits.U1RXIF = 0;
}
//UART2

char cr2=0; 
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void)
{
    unsigned char a=U2RXREG;
    switch(cr2){
        case 0:{
            if(a==13)
                cr2++;
            break;
        }
        case 1:{
            *((unsigned char *)&sensor1+1)=a;
            cr2++;
            break;
        }
        case 2:{
            *((unsigned char *)&sensor1)=a;
            cr2++;
            break;
        }
        case 3:{
            *((unsigned char *)&sensor2+1)=a;
            cr2++;
            break;
        }
        case 4:{
            *((unsigned char *)&sensor2)=a;
            cr2++;
            break;
        }
        case 5:{
            *((unsigned char *)&sensor3+1)=a;
            cr2++;
            break;
        }
        case 6:{
            *((unsigned char *)&sensor3)=a;
            cr2++;
            break;
        }
        case 7:{
            *((unsigned char *)&sensor4+1)=a;
            cr2++;
            break;
        }
        case 8:{
            *((unsigned char *)&sensor4)=a;
            cr2++;
            break;
        }
        case 9:{
            *((unsigned char *)&sensor5+1)=a;
            cr2++;
            break;
        }
        case 10:{
            *((unsigned char *)&sensor5)=a;
            cr2++;
            break;
        }
        case 11:{
            //ver si usar valor final pero puede servir como salvaguarda
            //llamar a funcion cosa procesadora, de momento solo actualizamos
            cr2=0;
            break;
            
        }
    }
    
    
    
    
    IFS1bits.U2RXIF = 0;
}

//TIMER1
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void){//Salta cada 500 us
    //LOAD_LED=1;
    update_pos();   //se actualiza la posicion actual
    mov_prof();     //generacion de velocidad constante
    //motor 1
    e_1= pos_des_1-pos_act_1;
    e_dot_1=e_1-e_old_1;    //calculo de cambio en el error
    e_old_1=e_1;
    set_duty_1((double)(E_1*KI_t_1+e_1*KP_1+e_dot_1*KD_t_1));//calculo de los terminos del
                                                 //controlador pid
    if(!p1sat)        //acumulacion de error solo si el sistema no esta saturado
        E_1=E_1+e_1;
    
    //motor 2
    e_2= pos_des_2-pos_act_2;

    e_dot_2=e_2-e_old_2;    //calculo de cambio en el error
    e_old_2=e_2;

    set_duty_2((double)(E_2*KI_t_2+e_2*KP_2+e_dot_2*KD_t_2));//calculo de los terminos del
                                                 //controlador pid
    if(!p2sat)        //acumulacion de error solo si el sistema no esta saturado
        E_2=E_2+e_2;
    //LOAD_LED=0;
    IFS0bits.T1IF=0;
}
//TIMER4

void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void){//Salta cada 5ms
    update_pos();
    double deltaTick_1=pos_act_1-pos_last_1;
    double deltaTick_2=pos_act_2-pos_last_2;
    
    pos_last_1=pos_act_1;
    pos_last_2=pos_act_2;
    
    
    double DL=deltaTick_1*0.000105858;
    double DR=deltaTick_2*0.000105858;
    double DC=(DL+DR)/2.000;
    
    x=x+DC*cos(o);
    y=y+DC*sin(o);
    o=o+(DR-DL)/L;
    o=atan2(sin(o),cos(o));
    
    
    pos_des_o=atan((y_trgt-y)/(x_trgt-x));
        
    e_o= pos_des_o-o;
    e_dot_o=e_o-e_old_o;    //calculo de cambio en el error
    e_old_o=e_o;
    
    if(e_o<0&&e_o>-0.034){
        W=0;
    }else if(e_o>0&&e_o<0.034){
        W=0;
    }else{
        W=(double)(E_O*KI_t_O+e_o*KP_O+e_dot_o*KD_t_O);//calculo de los terminos del
                                                 //controlador pid
    }
    
    
    calc_vel();
    if(!p1sat)        //acumulacion de error solo si el sistema no esta saturado
        E_O=E_O+e_o;
    
    double difx=x_trgt-x;
    double dify=y_trgt-y;
    
    if(difx<0){
        difx*=-1;
    }
    if(dify<0){
        dify*=-1;
    }
    if(difx<0.10&&dify<0.10){
        V=0;
    }
    
    
    
    IFS1bits.T4IF = 0;
}

//desborde QEI1 utilizado para posicion absoluta del motor 1
void __attribute__((__interrupt__, no_auto_psv)) _QEI1Interrupt(void){
    unsigned int s=QEI1CONbits.UPDN;
    if(s==1){
        acum_1+=((double)p_rev_1);
    }
    if(s==0){
        acum_1-=((double)p_rev_1);
    }

    IFS3bits.QEI1IF=0;
}
//desborde QEI2 utilizado para posicion absoluta del motor 2
void __attribute__((__interrupt__, no_auto_psv)) _QEI2Interrupt(void){
    unsigned int s=QEI2CONbits.UPDN;
    if(s==1){
        acum_2+=((double)p_rev_2);
    }
    if(s==0){
        acum_2-=((double)p_rev_2);
    }

    IFS4bits.QEI2IF=0;
}


int main(void) {
    KI_t_1=dt*KI_1;
    KD_t_1=KD_1/dt;
    KI_t_2=dt*KI_2;
    KD_t_2=KD_2/dt;

    KI_t_O=dt_O*KI_O;
    KD_t_O=KD_O/dt_O;
    
    configBoard();

    
//   printf("$$$");
//   __delay_ms(1000);
//   printf("set wlan phrase 77775522\n\r");
//   __delay_ms(1000);
//   printf("set wlan ssid KMCastro\n\r");
//   __delay_ms(1000);
//   printf("save\n\r");
//   __delay_ms(1000);
//   printf("reboot\n\r");
//   __delay_ms(1000);
   
    set_duty_1(0);
    set_duty_2(0);
    

    *((unsigned char *)&sensor1+1)=0b00001001;
    *((unsigned char *)&sensor1)=0b00110011;
    
    while(1){
        update_pos();
        printf("X : %3.0f\nY : %3.0f\nO : %3.0f\n\n ",x,y,o);
        //printf("Sensor 1 :%3i\nSensor 2 :%3i\nSensor 3 :%3i\nSensor 4 :%3i\nSensor 5 :%3i\n\n ",sensor1,sensor2,sensor3,sensor4,sensor5);
        __delay_ms(2000);
    }
    
    return 0;
}