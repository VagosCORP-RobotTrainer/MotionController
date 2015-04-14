#include "config.h"
#include "xc.h"
#include <libpic30.h>

void configBoard(){
    ////////////////////////////////////////////////////////////////////////////
    //////////////////////////CONFIGURACION RELOJ///////////////////////////////
    //para 20 MHz
    //se obtendra una frecuencia 80 MHz para tener 40 mips

    PLLFBD=22;
    CLKDIVbits.PLLPOST=0;               //PostScaler a 0
    CLKDIVbits.PLLPRE=1;                //Prescaler a 1


    //cambio de reloj del interno al oscilador primario
    // Initiate Clock Switch to Primary Oscillator with PLL (NOSC = 0b011)
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(OSCCON | 0x01);
    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b011);
    // Wait for PLL to lock
    while(OSCCONbits.LOCK!=1) {};
    
    
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////CONFIGURACION DE PINES REMAPEABLES//////////////////////
    //desbloqueo de registros
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));

      //Puerto Serial --> 1

    RPINR18bits.U1RXR=11; // pin de recepcion de datos del puerto serial 1 será RP11-->B11
    RPINR18bits.U1CTSR=0b11111;//control de flujo del puerto serial 1 directo a vss

    RPOR5bits.RP10R=0b00011; //pin de transmision del puerto serial 1 conectada a rp10-->B10

    //puerto serial --> 2
    
    RPINR19bits.U2RXR=6; // pin de recepcion de datos del puerto serial 1 será RP7
    RPINR19bits.U2CTSR=0b11111;//control de flujo del puerto serial 1 directo a vss

//Cuadrature Encoder --> 1
    RPINR14bits.QEA1R=2;   //fase a del encoder a rp0-->b2
    RPINR14bits.QEB1R=3;   //fase b del encoder a rp1-->b3
    RPINR15bits.INDX1R=0b11111;//index de encoder 1 directo a vss
//    RPOR3bits.RP7R=0b11010;// sentido de giro del encoder a pin b7
//Cuadrature Encoder --> 2
    RPINR16bits.QEA2R=5;   //fase a del encoder a rp5
    RPINR16bits.QEB2R=7;   //fase b del encoder a rp7
    RPINR17bits.INDX2R=0b11111;//index de encoder 1 directo a vss
//captura
//    RPINR7bits.IC1R=7;     //captura 1 escucha a pin b7
    
//Output compare
    RPOR0bits.RP0R=0b10011; //output compare 2 a pin b0
    RPOR0bits.RP1R=0b10010; //output compare 1 a pin b1
    
    

//bloqueo de registros
    __builtin_write_OSCCONL(OSCCON | (1<<6));
    
    /////////////////////////////////
    //cONFIGURACION DE PUERTOS
    TRIS_SERVO1=0; //pin asignado a servo 1 como salida
    TRIS_SERVO2=0;
    

    /////////////////////////////////////////////////////////////////////////////
   /////////////////////////CONFIGURACION DE MODULOPWM//////////////////////////

    P1TCONbits.PTSIDL=1; //basedetiempo se detiene con el procesador
    P1TCONbits.PTOPS=0b00;  //postsacaler a 1
    P1TCONbits.PTCKPS=0b00; //prescale a 1
    P1TCONbits.PTMOD=0b00;  //timebase in freeRuning
    P1TPER=1999;//valor para registro de comparación Fcy a 40MHz y se obtiene 20KHz para el pwm con valor de 1999
                 //para 50 Hz el valor es 12499


    PWM1CON1bits.PMOD1=0; //pwm par 1 en modo complementario
    PWM1CON1bits.PMOD2=0; //pwm par 2 en modo complementario

    PWM1CON1bits.PEN1L=1; //pin configurado como salida pwm
    PWM1CON1bits.PEN1H=1; //pin configurado como salida pwm
    PWM1CON1bits.PEN2L=1; //pin configurado como salida pwm
    PWM1CON1bits.PEN2H=1; //pin configurado como salida pwm

    PWM1CON2bits.SEVOPS=0; //postscaler de evento de pwm2 a 1
    PWM1CON2bits.IUE=1;  //actualizacion del pwm está relacionada con la base de tiempo del oscilador
    PWM1CON2bits.OSYNC=1; //las sobrecargas en las salidas se dan en base a la pase dde tiempo del pwm
    PWM1CON2bits.UDIS=0; //actualizaciones deshabilitadas


    P1DC2=2000;
    P1DC1=2000;  // a 50 Hz 25000 es 100% del duty teniendose 1250 unidades por milisegundo
                 // a 20 kHz el maximo es 4000

    P1TCONbits.PTEN=1; //generacion de pulso habilitada
    
    
    
    
    ////////////////////////////////
    ///CONFIGURACION pwm para hobby servos
    //T2
    T2CONbits.TON=0;//timer deshabilitado
    T2CONbits.TSIDL=0;//operacion continua con procesador en idle
    T2CONbits.TGATE=0; //modo compueta deshabilitado
    T2CONbits.TCKPS=0b10; //prescaler a 64
    T2CONbits.T32=0; //modo 32 bits deshabilitado
    T2CONbits.TCS=0; //fuente de reloj interna
    PR2=12499;
    //outputcompare 1-4
    //1
    OC1CONbits.OCM=0b000;
    OC1CONbits.OCSIDL=0;//operacion continua en idle
    OC1CONbits.OCTSEL=0; //timer2 es la fuente de reloj
    OC1RS=938;
    OC1R=938;
    OC1CONbits.OCM=0b110; //pwm sin pin de error
    //2
    OC2CONbits.OCM=0b000;
    OC2CONbits.OCSIDL=0;//operacion continua en idle
    OC2CONbits.OCTSEL=0; //timer2 es la fuente de reloj
    OC2RS=938;
    OC2R=938;
    OC2CONbits.OCM=0b110; //pwm sin pin de error
    //habilitar timer2 y config interrupciones
    T2CONbits.TON=1;//timer habilitado
    
    
    IFS0bits.OC1IF=0;
    IEC0bits.OC1IE=0;
    IFS0bits.OC2IF=0;
    IEC0bits.OC2IE=0;
    IPC1bits.T2IP=0x01;//prioridad 1 para timer 2
    IFS0bits.T2IF=0;// limpiar flag de  int timer2
    IEC0bits.T2IE=0;
    
    /*///////////////////////////////////////////////////////////////////////////
    ////////////////////////CONFIGURACION PUERTO SERIAL 1////////////////////////*/

    U1MODEbits.USIDL=1;  //puerto serial se apaga cuando el pic entra en sleep
    U1MODEbits.IREN=0;   //irda desactivado
    U1MODEbits.RTSMD=1;  //puerto serial en modo simple (no control de flujo)
    U1MODEbits.UEN=0b00; //solo se usan rx y tx para la transmicion
    U1MODEbits.WAKE=0;   //puerto serial no despierta al pic
    U1MODEbits.LPBACK=0; //modo de retroalimentcion deshabilitado
    U1MODEbits.ABAUD=0;  //autodetexion de baudage deshabilitada
    U1MODEbits.URXINV=0; //sin inversion de polaridad para recepcion
    U1MODEbits.BRGH=1;   //modo de alta velocidad
    U1MODEbits.PDSEL=0b00; //8 bits sin paridad
    U1MODEbits.STSEL=0;   //1 bit de parada

    U1STAbits.UTXISEL1=0; //salta interrupcion de transmision cuando
    U1STAbits.UTXISEL0=0; //1 caracter se envia
    U1STAbits.UTXINV=0;   //no se invierte la polaridad de transmicion idle=1
    U1STAbits.UTXBRK=0;   //pausa de sincronizacion deshabilitada

    U1STAbits.URXISEL=0b00; //salta interrupcion apenas se recibe un caracter
    U1STAbits.ADDEN=0;    //modo de deteccion de direccion deshabilitado

    IEC0bits.U1TXIE=0;    //interrupcion por envio deshabilitada
    IPC2bits.U1RXIP=1;    //prioridad 1 para recepción de datos
    IEC0bits.U1RXIE=1;    //se habilita interrupcion de recepcion

    ODCBbits.ODCB6=0;     //modo open collector para pin de transmision asi se puede trabajar a 5 V

    U1BRG=42; //para obtener 38461 baudios intentando llegar a 38400  se tiene 259 con modo de alta velocidad
               //para obtener 114942 baudios intentando llegar a 115200 se tiene 86 con modo de alta velocidad
               //para obtener 232558 baudios intentando llegar a 230400 se tiene 42 con modo de alta velocidad
               //para 9615 baudios buscando 9600 se tiene 259 con modo de baja velocidad
    U1MODEbits.UARTEN=1; //puerto serial habilitado
    U1STAbits.UTXEN=1;    //pin de transmicion habilitado
    __delay_us(106);  //esperar al menos 105 us
    
    /*///////////////////////////////////////////////////////////////////////////
    ////////////////////////CONFIGURACION PUERTO SERIAL 2////////////////////////*/

    U2MODEbits.USIDL=1;  //puerto serial se apaga cuando el pic entra en sleep
    U2MODEbits.IREN=0;   //irda desactivado
    U2MODEbits.RTSMD=1;  //puerto serial en modo simple (no control de flujo)
    U2MODEbits.UEN=0b00; //solo se usan rx y tx para la transmicion
    U2MODEbits.WAKE=0;   //puerto serial no despierta al pic
    U2MODEbits.LPBACK=0; //modo de retroalimentcion deshabilitado
    U2MODEbits.ABAUD=0;  //autodetexion de baudage deshabilitada
    U2MODEbits.URXINV=0; //sin inversion de polaridad para recepcion
    U2MODEbits.BRGH=1;   //modo de alta velocidad
    U2MODEbits.PDSEL=0b00; //8 bits sin paridad
    U2MODEbits.STSEL=0;   //1 bit de parada

    U2STAbits.UTXISEL1=0; //salta interrupcion de transmision cuando
    U2STAbits.UTXISEL0=0; //1 caracter se envia
    U2STAbits.UTXINV=0;   //no se invierte la polaridad de transmicion idle=1
    U2STAbits.UTXBRK=0;   //pausa de sincronizacion deshabilitada

    U2STAbits.URXISEL=0b00; //salta interrupcion apenas se recibe un caracter
    U2STAbits.ADDEN=0;    //modo de deteccion de direccion deshabilitado

    IEC1bits.U2TXIE=0;    //interrupcion por envio deshabilitada
    
    
    IPC7bits.U2RXIP=6;    //prioridad 6 para recepción de datos
    IFS1bits.U2RXIF = 0;  //se limpia flag de recepcion
    IEC1bits.U2RXIE=1;    //se habilita interrupcion de recepcion

       

    U2BRG=86; //para obtener 38461 baudios intentando llegar a 38400  se tiene 259 con modo de alta velocidad
               //para obtener 114942 baudios intentando llegar a 115200 se tiene 86 con modo de alta velocidad
               //para obtener 232558 baudios intentando llegar a 230400 se tiene 42 con modo de alta velocidad
               //para 9615 baudios buscando 9600 se tiene 259 con modo de baja velocidad
    U2MODEbits.UARTEN=1; //puerto serial habilitado
    U2STAbits.UTXEN=0;    //pin de transmicion deshabilitado
    __delay_us(106);  //esperar al menos 105 us
    
    /*//////////////////////////////////////////////////////////////////////////
    ////////////////////////CONFIGURACION DE TIMER1///////////////////////////*/
    //periodo de interrupcion: Tint=(PRx+1)*Pre*2/Fosc


    T1CONbits.TON=0;    //timer deshabilitado
    T1CONbits.TSIDL=1;  //timer desahabilitado con el procesador en reposo
    T1CONbits.TGATE=0;  //modo de compuerta deshabilitado
    T1CONbits.TCKPS=0b01; //prescaler a 8
//    T1CONbits.TCKPS=0b10; //prescaler a 64
    T1CONbits.TSYNC=0; //ignorado cuando la entrada del reloj es el oscilador interno
    TMR1=0;             //registro del timer1 a 0
    T1CONbits.TCS=0;   //fuente del reloj interno

    PR1=2499;           //con prescaler a 8--> para interrupcion cada 500 us-->2499
                        //con prescaler=64-->para una interrupcion cada5 ms-->3125 10 ms-->6250 20 ms--> 12500  y para 100ms-->62500

    IPC0bits.T1IP = 0x07;// Prioridad máxima para inttimer1
    IFS0bits.T1IF = 0;// limpiar flag de interrupcion 1
    IEC0bits.T1IE = 1;// habilitar interrupcion del timer1
    T1CONbits.TON = 1;// timer1 deshabilitado
    
    /////////////////////////////////////////////////////////////////////////////
   /////////////////////////CONFIGURACION DE ENCODER-1 /////////////////////////

   AD1PCFGLbits.PCFG4=1;  //an4 como entrada salida digital para uso con el encoder
   AD1PCFGLbits.PCFG5=1;  //an5 como entrada salida digital para uso con el encode
   QEI1CONbits.PCDOUT=0;  //sentido de giro no se refleja en el pin de sentido
   QEI1CONbits.QEISIDL=1; //quadrature se detiene en idle
   QEI1CONbits.QEIM=0b101;//quadrature en modo x2 y reset por coincidencia
   QEI1CONbits.SWPAB=0;   //fase a y b no estan intercambiadas

   QEI1CONbits.POSRES=0;  //index no resetea contador

   MAX1CNT=2300;         //se define el valor para reset por comparacion del contador se usa 2 ya que se esta en modo x2
   //filtro digital
   DFLT1CONbits.IMV=0b11;    //estados de ambos pines para reset del sistema en 0
   DFLT1CONbits.CEID=0;   //interrupciones por error en la cuenta desactivadas
   DFLT1CONbits.QEOUT=1;  //filtro digital desactivado
   DFLT1CONbits.QECK=0b011;//reloj del filtro a 1:16 FCY

   IPC14bits.QEI1IP=0x01;//prioridad 1 para interrupcion
   IFS3bits.QEI1IF=0;
   IEC3bits.QEI1IE=1; //interrupcion de desborde de encoder 1 habilitada
    /////////////////////////////////////////////////////////////////////////////
   /////////////////////////CONFIGURACION DE ENCODER-2 /////////////////////////
 
   
   
   QEI2CONbits.PCDOUT=0;  //sentido de giro no se refleja en el pin de sentido
   QEI2CONbits.QEISIDL=1; //quadrature se detiene en idle
   QEI2CONbits.QEIM=0b101;//quadrature en modo x2 y reset por coincidencia
   QEI2CONbits.SWPAB=0;   //fase a y b no estan intercambiadas

   QEI2CONbits.POSRES=0;  //index no resetea contador

   MAX2CNT=2300;         //se define el valor para reset por comparacion del contador se usa 2 ya que se esta en modo x2
   //filtro digital
   DFLT2CONbits.IMV=0b11;    //estados de ambos pines para reset del sistema en 0
   DFLT2CONbits.CEID=0;   //interrupciones por error en la cuenta desactivadas
   DFLT2CONbits.QEOUT=1;  //filtro digital desactivado
   DFLT2CONbits.QECK=0b011;//reloj del filtro a 1:16 FCY

   IPC18bits.QEI2IP=0x01;//prioridad 1 para interrupcion
   IFS4bits.QEI2IF=0;
   IEC4bits.QEI2IE=1; //interrupcion de desborde de encoder 1 habilitada
    
   ////////////////////////////////////////////////////////////////////////////
   /////////////////////////TIMER 4////////////////////////////////////////////
   
    T4CONbits.TON=0;//timer deshabilitado
    T4CONbits.TSIDL=0;//operacion continua con procesador en idle
    T4CONbits.TGATE=0; //modo compueta deshabilitado
    T4CONbits.TCKPS=0b01; //prescaler a 64
    T4CONbits.T32=0; //modo 32 bits deshabilitado
    T4CONbits.TCS=0; //fuente de reloj interna
    PR4=24999; //salta cada 5 ms
    
    IPC6bits.T4IP = 0x07;// Prioridad máxima para inttimer1
    IFS1bits.T4IF = 0;// limpiar flag de interrupcion 1
    IEC1bits.T4IE = 1;// habilitar interrupcion del timer1
    T4CONbits.TON = 1;// timer4 habilitado
    
}