/* 
 * File:   Palm_Power.c
 * Author: Denns
 *
 * Created on August 16, 2018, 8:06 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <P24F16KA101.h>
#include <xc.h>
#define FOSC    8000000L
#define FCY    (FOSC/2)
#include <libpic30.h>  // __delayXXX() functions macros defined here
 //definitions
char uart_receive_array[10]; //SET UP USART RECEIVE ARRAY
int adx;
int ady;
char uart_rec = 0;
#define BAUD_RATE_UART1    9600L
//`functions
 void usart_setup(void);
 void usart_write(char);
 void A_TO_D_SETUP(void);
 void A_TO_D_conv(void);
 void send_string(void);
 void uart_decode(void);
 // Setup the PIC  
#pragma config POSCMOD = NONE             // Primary Oscillator Select (HS Oscillator mode selected)
#pragma config BOREN = BOR3 // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
#pragma config PWRTEN = ON // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON // MCLR Pin Enable bit (MCLR pin enabled; RA5 input pin disabled)
#pragma config FNOSC = FRC // Oscillator Select (Fast RC oscillator with Postscaler (FRCDIV)) changed to FRC - no postscaler
#pragma config FWDTEN = OFF // Watchdog Timer Enable (Watchdog Timer is disabled)
/*
 * 
 */
int main() {
      TRISB = 0xFFFF;
      TRISA = 0xFF;
      TRISBbits.TRISB7 = 1;
      TRISBbits.TRISB2 = 1; 
      //set up interrupt for UART1, receive and transmit  
      usart_setup();
      A_TO_D_SETUP();

      while(1)
      __delay_ms(100);
         A_TO_D_conv();
      if(uart_rec == 'y' ){
          uart_decode();
      }
                
      }
     
    return (EXIT_SUCCESS);
}
//DECODE RECIEVED DATA
  void uart_decode(void){
         uart_rec = 0;
        if ((uart_receive_array[0] == 'd') && (uart_receive_array[1] == 'n')
                && (uart_receive_array[2] == 'e') && (uart_receive_array[3] == 's')){
                send_string();
            
        }
            
    }
  
  
//send the string
void send_string()
{
    usart_write('p');
    adx = adx >>2; //Drop the lower 2 bits and keep the upper 8 bits
    usart_write(adx);
    usart_write(';');
    usart_write('r');
    ady = ady >>2; //Drop the lower 2 bits and keep the upper 8 bits
    usart_write(ady);
    usart_write(';');
    if (PORTBbits.RB0 == 1){
        usart_write('s');
        usart_write('1');
        usart_write('y');
        usart_write(';');
    }
    if (PORTBbits.RB0 == 0){
        usart_write('s');
        usart_write('1');
        usart_write('n');
        usart_write(';');
      }
    if (PORTBbits.RB2 == 1){
        usart_write('s');
        usart_write('2');
        usart_write('y');
        usart_write(';');
    }
    if (PORTBbits.RB2 == 0){
        usart_write('s');
        usart_write('2');
        usart_write('n');
        usart_write(';');
      }
    if (PORTBbits.RB4 == 1){
        usart_write('s');
        usart_write('3');
        usart_write('y');
        usart_write(';');
    }
    if (PORTBbits.RB4 == 0){
        usart_write('s');
        usart_write('3');
        usart_write('n');
        usart_write(';');
      }       
    }
  
//uart section
 void usart_setup(){
    U1MODE = 0; // Clear UART1 mode register
    U1STA = 0;    // Clear UART1 status register
    U1MODEbits.BRGH = 0;
    U1BRG = (FCY/(16*BAUD_RATE_UART1))-1; // Calculate value of baud rate register
    IEC0bits.U1TXIE = 0;           // Disable UART TX interrupt
    IEC0bits.U1RXIE = 1;            // Enable the UART RX interrupt
    U1STAbits.URXISEL = 0;         // Interrupt after one RX character is   received
    U1MODEbits.UARTEN = 1; // Enable UART
    U1MODEbits.RTSMD = 1; //SET FOR NO FLOW CONTROL
    U1STAbits.UTXEN = 1;     // Enable UART1 transmit
       return;
   }
 void usart_write(char data)
{
  while(!U1STAbits.TRMT);
  U1TXREG = data;
}
 
void __attribute__((interrupt,no_auto_psv)) _U1RXInterrupt( void )
{
// process USART receive interrupts. Shift all data up one register and save the new value
if(IFS0bits.U1RXIF) {
 uart_receive_array[9] = uart_receive_array[8];
 uart_receive_array[8] = uart_receive_array[7];
 uart_receive_array[7] = uart_receive_array[6];
 uart_receive_array[6] = uart_receive_array[5];
 uart_receive_array[5] = uart_receive_array[4];
 uart_receive_array[3] = uart_receive_array[2];
 uart_receive_array[2] = uart_receive_array[1];
 uart_receive_array[1] = uart_receive_array[0];
 uart_receive_array[0] = U1RXREG; //Read received data.   
 usart_write(uart_receive_array[0]); // Mirror value back
 IFS0bits.U1RXIF = 0;
 uart_rec = 'y';
}

  
 }
//A/D CONVERTER SECTION
void A_TO_D_SETUP (void)
{
    AD1PCFG = 0xFFFC; // AN0 and AN1 as analog, all other pins are digital
    AD1CON1 = 0x0000; // SAMP bit = 0 ends sampling 
    AD1CSSL = 0;
    AD1CON3 = 0x0002; // Manual Sample
    AD1CON2 = 0;
    AD1CON1bits.ADON = 1; // turn ADC ON
}
void A_TO_D_conv()
{
     //convert and save x value
    AD1CHS = 0x0000; // Connect AN0 
    AD1CON1bits.SAMP = 1; // start sampling
     __delay_ms(100);; // 100 ms time delay
    AD1CON1bits.SAMP = 0; // start converting
    while (!AD1CON1bits.DONE){}; // check for done
    adx = ADC1BUF0; // check for done
     //convert and save y value
       AD1CHS = 0x0001; // Connect AN1 
    AD1CON1bits.SAMP = 1; // start sampling
     __delay_ms(100);; // 100 ms time delay
    AD1CON1bits.SAMP = 0; // start converting
    while (!AD1CON1bits.DONE){}; // check for done
    ady = ADC1BUF0; // check for done
    }


