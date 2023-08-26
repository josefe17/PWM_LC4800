/*
*   Multichannel DMX512 PWM firmware
*   Authors: Jose Fernando Gomez Díaz
*            Roberto Salinas Rosich
*
*   Thanks to Nocturno from Micropic (www.micropic.es), whose DMX receiver based this code.
*
*   Based on DMX dimmer
*
*   This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  To change from 25 series to 45 series comment and uncomment ad_index, read_addr function and
 *  TRIS register configuration
 */


#define _XTAL_FREQ 8000000

#include <xc.h>
#include "config.h"

/*Tri-State ports*/
#define IN   1
#define OUT  0

/*RS-485 enabling*/
#define dmx_en PORTBbits.RB4

/*DMX safety timer*/
#define t0_res_1 0b10000100
#define t0_res_2 0b10000101

/*Internal dimming parameters*/
#define TOTALCHANNELS       512
#define CHANNELS            4
#define DMX_ESPERA_BYTE     0
#define DMX_ESPERA_BREAK    1
#define DMX_ESPERA_START    2
#define DMX_ESPERA_DATO     3
#define DMX_RECEPCION_DATOS 4


/*Function prototypes*/

int read_addr(void);
void set_PWM2(unsigned char); //ojo al cambiar el modo pq el mapping es int!!!!!!!!!!!!
void set_PWM3(unsigned char);
void set_PWM4(unsigned char);
void set_PWM5(unsigned char);


/*DMX Variables*/
volatile unsigned char DMX_Estado = DMX_ESPERA_BREAK;  //FSM status
volatile unsigned char DatoRX;                          //Recived data
//const unsigned char dt_map[] = { 95, 95, 96, 97, 97, 98, 99, 99, 100, 100, 101, 102, 102, 103, 104, 104, 105, 105, 106, 107, 107, 108, 109, 109, 110, 110, 111, 112, 112, 113, 114, 114, 115, 116, 116, 117, 117, 118, 119, 119, 120, 121, 121, 122, 122, 123, 124, 124, 125, 126, 126, 127, 127, 128, 129, 129, 130, 131, 131, 132, 132, 133, 134, 134, 135, 136, 136, 137, 137, 138, 139, 139, 140, 141, 141, 142, 142, 143, 144, 144, 145, 146, 146, 147, 147, 148, 149, 149, 150, 151, 151, 152, 152, 153, 154, 154, 155, 156, 156, 157, 157, 158, 159, 159, 160, 161, 161, 162, 162, 163, 164, 164, 165, 166, 166, 167, 167, 168, 169, 169, 170, 171, 171, 172, 172, 173, 174, 174, 175, 176, 176, 177, 177, 178, 179, 179, 180, 181, 181, 182, 182, 183, 184, 184, 185, 186, 186, 187, 187, 188, 189, 189, 190, 191, 191, 192, 192, 193, 194, 194, 195, 196, 196, 197, 197, 198, 199, 199, 200, 201, 201, 202, 202, 203, 204, 204, 205, 206, 206, 207, 207, 208, 209, 209, 210, 211, 211, 212, 212, 213, 214, 214, 215, 216, 216, 217, 217, 218, 219, 219, 220, 221, 221, 222, 222, 223, 224, 224, 225, 226, 226, 227, 227, 228, 229, 229, 230, 231, 231, 232, 232, 233, 234, 234, 235, 236, 236, 237, 237, 238, 239, 239, 240, 241, 241, 242, 242, 243, 244, 244, 245, 246, 246, 247, 247, 248, 249, 249, 250, 251, 251, 252, 252, 253, 254, 255};
const unsigned char dt_map[] = { 72, 73, 74, 75, 75, 76, 77, 77, 78, 79, 80, 80, 81, 82, 82, 83, 84, 85, 85, 86, 87, 87, 88, 89, 90, 90, 91, 92, 92, 93, 94, 95, 95, 96, 97, 97, 98, 99, 100, 100, 101, 102, 102, 103, 104, 105, 105, 106, 107, 107, 108, 109, 110, 110, 111, 112, 112, 113, 114, 114, 115, 116, 117, 117, 118, 119, 119, 120, 121, 122, 122, 123, 124, 124, 125, 126, 127, 127, 128, 129, 129, 130, 131, 132, 132, 133, 134, 134, 135, 136, 137, 137, 138, 139, 139, 140, 141, 142, 142, 143, 144, 144, 145, 146, 147, 147, 148, 149, 149, 150, 151, 152, 152, 153, 154, 154, 155, 156, 157, 157, 158, 159, 159, 160, 161, 162, 162, 163, 164, 164, 165, 166, 167, 167, 168, 169, 169, 170, 171, 172, 172, 173, 174, 174, 175, 176, 177, 177, 178, 179, 179, 180, 181, 182, 182, 183, 184, 184, 185, 186, 187, 187, 188, 189, 189, 190, 191, 192, 192, 193, 194, 194, 195, 196, 197, 197, 198, 199, 199, 200, 201, 202, 202, 203, 204, 204, 205, 206, 207, 207, 208, 209, 209, 210, 211, 211, 212, 213, 214, 214, 215, 216, 216, 217, 218, 219, 219, 220, 221, 221, 222, 223, 224, 224, 225, 226, 226, 227, 228, 229, 229, 230, 231, 231, 232, 233, 234, 234, 235, 236, 236, 237, 238, 239, 239, 240, 241, 241, 242, 243, 244, 244, 245, 246, 246, 247, 248, 249, 249, 250, 251, 251, 252, 253, 254, 255 };
volatile int DMX_Indice = 0;                              //Loop variable for FrameDMX
volatile unsigned char FrameDMX[TOTALCHANNELS]={0};       //Received DMX values
volatile int dmx_start_address=0;                        //Starting address
volatile unsigned char dmx_rx_ok=0;                      //RX valid variable                   

/*Analog variables*/
volatile unsigned char analog_index=0;   //ADC channel variable
volatile unsigned char analog_buffer[4]={0,0,0,0}; // ADC out


union  // Estructura para hacer una copia del registro RCSTA
   {
   unsigned char registro;
   struct {
     unsigned char RX9D:1;
     unsigned char OERR:1;
     unsigned char FERR:1;
     unsigned char ADDEN:1;
     unsigned char CREN:1;
     unsigned char SREN:1;
     unsigned char RX9:1;
     unsigned char SPEN:1;
           } bits ;
  }Copia_RCSTA;


void main(void)
{        
    /*FOR 25K80*/
    
    TRISA  = 0b11111111;   //All analog inputs
    TRISB  = 0b11001111;   //RB0-3 as address input, RB3 as DMX enable, RB5 as PWM output, RB7 defined as input by USART
    TRISC  = 0b00111011;   //RC0-1 as input, RC2 as PWM output, RC3-5 as input, RC6-7 as PWM output
        
    OSCCON = 0b01111100;   //HS  oscillator @8Mhz
    //SSPCON1bits.SSPEN=0; 

    /*Global CCP & Timer2 configuration*/
    
    CCP2CONbits.CCP2M=12;   //CCP2 PWM mode
    CCP3CONbits.CCP3M=12;   //CCP3 PWM mode
    CCP4CONbits.CCP4M=12;   //CCP4 PWM mode
    CCP5CONbits.CCP5M=12;   //CCP5 PWM mode
    CCPTMRS=0x0;            //Timer2 Timebase
    T2CONbits.T2CKPS1=0;    //No prescaler
    T2CONbits.T2CKPS0=0;
    PR2=60;                //@8 Mhz => 32.786 KHz PWM & 8 bit resolution
    T2CONbits.TMR2ON=1;     //Timer2 ON
    set_PWM2(0);           //Set min voltage on outputs
    set_PWM3(0);
    set_PWM4(0);
    set_PWM5(0);
    
    /*ADC configurations*/
    
    ANCON0=0b11111111;      //Ports 0-3 as ADC input
    ANCON1=0;
    ADCON1bits.VCFG=0;      //AVDD ref
    ADCON1bits.VNCFG=0;     //AVSS red
    ADCON1bits.CHSN=0b101;  //AN4 Negative reference  0b101
    ADCON2bits.ADFM=0;      //Left justified
    ADCON2bits.ACQT=0;
    ADCON2bits.ADCS=7;      //RC osc TAD????????

    dmx_rx_ok=0;               //Pre invalidate DMX frames

    /*USART configurations*/
    
    TXSTA2bits.BRGH=1;           // Alta velocidad seleccionada.
    BAUDCON2bits.BRG16=1;        // Baudrate de 16 bits
    TXSTA2bits.SYNC=0;           // Seleccionamos transmisión asíncrona
    SPBRG2=7;                    // A 8MHz representa Baudios = 250KHz
    SPBRGH2=0;
    RCSTA2bits.RX9=1;            // Activada la recepción a 9 bits
    RCSTA2bits.SREN=0;           // Desactivada la recepción de un sólo byte
    RCSTA2bits.ADDEN=0;          // Desactivada la autodetección de dirección
    RCSTA2bits.FERR=0;           // No hay error de frame
    RCSTA2bits.OERR=0;           // No hay error de overrun
    RCSTA2bits.SPEN=1;           // USART activada
    RCSTA2bits.CREN=1;           // Recepción activada
    
    /*Interrupt Configuration*/
    
    RCONbits.IPEN=0;            //Interruption priority disable
    PIE3bits.RC2IE=1;           //EUSART Receiving Interrupt Enable
    PIR3bits.RC2IF=0;           //Clear EUSART interruption flag
    PIE1bits.ADIE = 0;          //Disable ADC interrupt
    INTCONbits.T0IE=1;          //Enable timer0 interrupts
    INTCONbits.T0IF=0;          //Clear timer0 interrupt
    INTCONbits.GIE_GIEH = 1;    //Global interrupts enable
    INTCONbits.PEIE_GIEL=1;     //Peripheral interrupts enable
    //INTCONbits.GIE_GIEH = 0;    //Global interrupts enable

    dmx_en=1;                   //Switch on RS-485 receiver
    dmx_start_address = 0;                   //Clear DMX starting address
    TMR0H=0; 
    TMR0L=0;
    T0CON=t0_res_2;             //Start TTL timer

    analog_index=0;
    while(1){
 
       dmx_start_address=read_addr();         //Read DIP switch address
                    
        analog_index=0;
        ADCON0bits.ADON=1;
        ADCON0bits.CHS=analog_index;
        while (analog_index<CHANNELS){   //ADC routine
            __delay_us(10);                 //Tadq delay; 1/8 Mhz =0.2 uS * 12 = 1.5 uS delaym
            ADCON0bits.GO_NOT_DONE=1;       //Start conversion            
            __delay_us(3);                  // Wait 3 us before changing mux to allow C to be disconnected
            ADCON0bits.CHS=++analog_index;
            while(ADCON0bits.GO_NOT_DONE);  //Wait for conversion
            analog_buffer[analog_index-1]=ADRESH;
            ADRESH=0;
            ADRESL=0;
        }

            /*
        set_PWM5(dt_map[analogs[0]]);
        set_PWM4(dt_map[FrameDMX[0]]);       
        set_PWM3(dt_map[analogs[1]]);
        set_PWM2(dt_map[analogs[2]]);
     
*/
        if (dmx_start_address>(TOTALCHANNELS-CHANNELS)){ //If address is higher than 508, 4th channel is just analog controlled
            set_PWM4(analog_buffer[3]);
        }
        else{
            if((FrameDMX[dmx_start_address+(CHANNELS-1)]>analog_buffer[3]) && dmx_rx_ok && dmx_en){
                set_PWM4(FrameDMX[dmx_start_address+(CHANNELS-1)]);
            }
            else{
               set_PWM4(analog_buffer[3]);
            }
        }
        
        if (dmx_start_address>(TOTALCHANNELS-CHANNELS+1)){ //If address is higher than 509, 3rd channel is just analog controlled
            set_PWM5(analog_buffer[2]);
        }
        else{
            if((FrameDMX[dmx_start_address+(CHANNELS-2)]>analog_buffer[2]) && dmx_rx_ok && dmx_en){
                set_PWM5(FrameDMX[dmx_start_address+(CHANNELS-2)]);
            }
            else{
               set_PWM5(analog_buffer[2]);
            }            
        }
        
        if (dmx_start_address>(TOTALCHANNELS-CHANNELS+2)){ //If address is higher than 510, 2nd channel is just analog controlled
            set_PWM2(analog_buffer[1]);
        }
        else{
            if((FrameDMX[dmx_start_address+(CHANNELS-3)]>analog_buffer[1]) && dmx_rx_ok && dmx_en){
                set_PWM2(FrameDMX[dmx_start_address+(CHANNELS-3)]);
            }
            else{
               set_PWM2(analog_buffer[1]);
            } 
        }
        
        if((FrameDMX[dmx_start_address]>analog_buffer[0]) && dmx_rx_ok && dmx_en){
            set_PWM3(FrameDMX[dmx_start_address]);
        }
        else{
            set_PWM3(analog_buffer[0]);
        }
 
    }
     
}

/*For 25k80*/

int read_addr(void){
    int number=0;
    volatile unsigned char portb_val=PORTB;
    volatile unsigned char portc_val=PORTC;
    
    number = ((portb_val & _PORTB_RB3_MASK) >> 3) | ((portb_val & _PORTB_RB2_MASK) >> 1) | ((portb_val & _PORTB_RB1_MASK) << 1) | ((portb_val & _PORTB_RB0_MASK) << 3) |
            ((portc_val & _PORTC_RC5_MASK) >> 1) | ((portc_val & _PORTC_RC4_MASK) << 1) | ((portc_val & _PORTC_RC3_MASK) << 3) | ((portc_val & _PORTC_RC1_MASK) << 6);
    number += (portc_val & _PORTC_RC0_MASK) * 256;
    
//    if  (PORTCbits.RC0) number+=256;
//    if  (PORTCbits.RC1) number+=128;
//    if  (PORTCbits.RC3) number+=64;
//    if  (PORTCbits.RC4) number+=32;
//    if  (PORTCbits.RC5) number+=16;
//    if  (PORTBbits.RB0) number+=8;
//    if  (PORTBbits.RB1) number+=4;
//    if  (PORTBbits.RB2) number+=2;
//    if  (PORTBbits.RB3) number+=1;

    return number;
}


void set_PWM2(unsigned char dc){
    CCPR2L=(dt_map[dc]>>2);
    CCP2CONbits.DC2B=(dt_map[dc] & 3);
}

void set_PWM3 (unsigned char dc){
    CCPR3L=(dt_map[dc]>>2);
    CCP3CONbits.DC3B=(dt_map[dc] & 3);
}

void set_PWM4 (unsigned char dc){
    CCPR4L=(dt_map[dc]>>2);
    CCP4CONbits.DC4B=(dt_map[dc] & 3);
}

void set_PWM5 (unsigned char dc){
    CCPR5L=(dt_map[dc]>>2);
    CCP5CONbits.DC5B=(dt_map[dc] & 3);
}

void __interrupt() ISR(void)
{
  if(INTCONbits.T0IF){
      INTCONbits.T0IF=0;    //Clear flag
      dmx_rx_ok=0; //Invalidate DMX values
      T0CON=0;  //Stop timer
  }
    
  while (PIR3bits.RC2IF) // ejecutamos mientras haya un dato pendiente de procesar
   {
    // Hacemos una copia del registro RCSTA porque sus bits cambian de valor
    // al leer RCREG y modificar CREN
    Copia_RCSTA.registro = RCSTA2;

    // En RCREG está el dato que acaba de recibir la USART
    DatoRX = RCREG2;

    // Si se reciben más de 3 bytes sin haberlos procesado, se produce un error
    // de Overrun. En este caso, se borra el error reiniciando CREN y dejamos
    // la interrupción preparada para procesar la siguiente trama DMX
    if (Copia_RCSTA.bits.OERR)
    {
      CREN2=0;
      CREN2=1;
      DMX_Estado = DMX_ESPERA_BYTE;
      return;
    }

    // Máquina de estados
    switch (DMX_Estado)
    {
      case DMX_ESPERA_BYTE:   // si estamos en este estado y hay error FRAME
      // es que nos ha pillado en medio de un Byte. Hay que seguir esperando
      // hasta que desaparezca el error.
        if (!Copia_RCSTA.bits.FERR)
              // Ha llegado un byte. Ahora esperaremos la señal Break
          DMX_Estado = DMX_ESPERA_BREAK;
        break;


      case DMX_ESPERA_BREAK:   // estamos esperando la señal Break
        // Esta señal se identifica porque aparece el error de Frame
        if (Copia_RCSTA.bits.FERR)
          // Tras recibir el error de Break, hay que esperar un byte de valor 0
          if (!DatoRX)
            DMX_Estado = DMX_ESPERA_START;
        break;
      case DMX_ESPERA_START: // ya hemos recibido el Break y ahora hay que
          // esperar un Byte con valor 0, que será la señal de Start
        // Mientras tanto, si recibimos un error de Frame, hay que volver a
        // empezar para recibir la señal de comienzo de trama.
        if (Copia_RCSTA.bits.FERR)
            DMX_Estado = DMX_ESPERA_BYTE;
        else {
          if (!DatoRX)
          {
            // Llegados a este punto, ya hemos recibido el Byte Start=0
            // y comenzamos la trama de valores DMX.
            DMX_Indice = 0;
            DMX_Estado = DMX_RECEPCION_DATOS;
          } else
            // Si el dato recibido no es 0, volvemos a empezar
            DMX_Estado = DMX_ESPERA_BREAK;
        }
        break;
      case DMX_RECEPCION_DATOS:
        // En este estado estamos recibiendo la trama de datos DMX
        // Si se detecta un error de Frame es que ha habido un error y estamos
        // al principio
        if (Copia_RCSTA.bits.FERR)
          if (!DatoRX)
            DMX_Estado = DMX_ESPERA_START;
          else
            DMX_Estado = DMX_ESPERA_BYTE;
        else
        {
          // Almacenamos el dato recibido en nuestro array
          FrameDMX[DMX_Indice++] = DatoRX;


          TMR0H=0; //Reset TTL frame
          TMR0L=0;
          T0CON=t0_res_2;

          // Si ha llegado al final de la capacidad, cambiamos al estado de espera
          // de nueva trama
          if (DMX_Indice >= TOTALCHANNELS || DMX_Indice >= dmx_start_address+CHANNELS){              
              dmx_rx_ok=1; //Valid frame
          }
          if (DMX_Indice >= TOTALCHANNELS ){
              DMX_Estado = DMX_ESPERA_BREAK;
          }

        }
        break;
   }
}

}