/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 * 
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//   MSP430F552x Demo - USCI_A0, Ultra-Low Pwr UART 9600 Echo ISR, 32kHz ACLK
//
//   Description: Echo a received character, RX ISR used. Normal mode is LPM3,
//   USCI_A0 RX interrupt triggers TX Echo.
//   ACLK = 32768Hz crystal, MCLK = SMCLK = DCO ~1.045MHz
//   Baud rate divider with 32768Hz XTAL @9600 = 32768Hz/9600 = 3.41
//   See User Guide for baud rate divider table
//
//                MSP430F552x
//             -----------------
//        /|\ |              XIN|-
//         |  |                 | 32kHz
//         ---|RST          XOUT|-
//            |                 |
//            |     P3.3/UCA0TXD|------------>
//            |                 | 9600 - 8N1
//            |     P3.4/UCA0RXD|<------------
//
//   Bhargavi Nisarga
//   Texas Instruments Inc.
//   April 2009
//   Built with CCSv4 and IAR Embedded Workbench Version: 4.21
//******************************************************************************

#include <include.h>
#include <string.h>
#include <stdlib.h>


char tstr[32];  //convert save
char a[32];

char *str = 0 ; //receive string
char *strp = 0 ;

//AD
double val_d0;
double val_d1;
double val_d2;
double val_d3;
unsigned int DMA_A0;                        // ADC conversion result is stored in this variable
unsigned int DMA_A1;                        // ADC conversion result is stored in this variable
unsigned int DMA_A2;                        // ADC conversion result is stored in this variable
unsigned int DMA_A3;                        // ADC conversion result is stored in this variable

unsigned int Pre_A3;

//timer0
int tri_flag = 0;
int tri_time = 0;
int tri_flags = 0;
int tri_times = 0;
unsigned short Tcal = 0;

//system
int RX_flag = 0;
int set_ack_flag = 0;
int SET_flag = 0;
int I_flag = 0;
int stalls_flag = 0;

int fct_stc_flag = 0;
int buttom_fct_flag = 0;
int buttom_fct_flags = 0;
unsigned char state = 0;
double Datai=0.0;
double Datavin=0.0;
double I_out = 0.0;

int stalls = 0;
int function = 0;
int mode = 0;
//SPI
unsigned short MST_Data,SLV_Data;

void DMAInit(void);
void fct_stc_opt(void);
void buttom_fct_stc_opt(void);
void buttom_fct_stcs_opt(void);
void cuto(int chn);

int main(void)
{
//    double Datav=123.4;

    str = a;
    int i;

//    char *str_1 = "abc" ;


  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT


  //Timer0
  TA0CCTL0 = CCIE;                          // CCR0 interrupt enabled
  TA0CCR0 = 5000;
  TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, upmode, clear TAR

  //GPIO
  P4DIR |= BIT7;                            // Set P4.7 to output direction
  P1DIR |= BIT0;                            // Set P1.0 to output direction
  P1DIR |= 0x3C;
  P3DIR |= BIT5+BIT6;
  P1OUT |= BIT5+BIT4;

  //buttom
  P2REN |= BIT1;
  P2OUT |= BIT1;
  P2IES |= BIT1;
  P2IFG &= ~BIT1;
  P2IE |= BIT1;

  P1REN |= BIT1;
  P1OUT |= BIT1;
  P1IES |= BIT1;
  P1IFG &= ~BIT1;
  P1IE |= BIT1;

  //IIC
  OLED_Init();
  OLED_ShowStr(0,2,"Please Set:",99,2);
//  Delay_ms(100);
//  OLED_Clear(0x00);
//end of IIC

  DMAInit();
  ADCInit();   //启动需要在增加 enable

  //UART
  UART_Init(UARTA0); //bluetooth
  UART_Init(UARTB0); //SPI

  UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
  //  UCB0IE |= UCRXIE;                         // Enable USCI_B0 RX interrupt

  //SPI
    P2OUT |= BIT5;                            // reset slave
    P2OUT &= ~BIT5;                           // Now with SPI signals initialized,
    for(i=50;i>0;i--);                        // Wait for slave to initialize

    MST_Data = 0x8FFF;                          // Initialize data values
    SLV_Data = 0x00;                          //
    while (!(UCB0IFG&UCTXIFG));               // USCI_B0 TX buffer ready?
  //end of SPI


  __bis_SR_register(LPM3_bits + GIE);       // Enter LPM0, enable interrupts
  while(1)
  {
//    OLED_ShowStr(12,2,(unsigned char *)F2S(Datav,tstr),6,1); //6*8
//    UartTX_Send("Done!\r\n",7);

      if(buttom_fct_flag != 0)
      {
    	  buttom_fct_stc_opt();
    	  buttom_fct_flag = 0;
      }

      if(buttom_fct_flags != 0)
      {
    	  buttom_fct_stcs_opt();
    	  buttom_fct_flags = 0;
      }

      if(function!=0) // not off
      {
          ADC12CTL0 |= ADC12SC;   // Start sampling/conversion
      }


//    OLED_ShowStr(0,0,(unsigned char *)F2S(DMA_A1,tstr),8,1); //6*8
      if(fct_stc_flag != 0)
      {
          fct_stc_flag = 0;
          fct_stc_opt();
      }
      switch(function)
      {
          case 0:
          {
//            OLED_OFF();
              __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3, interrupts enabled;
          }
          break;

          case 1:
          {
              if(mode == 0)
              {
                    if(set_ack_flag == 1)
                    {
//                      OLED_ON();
//                      OLED_Clear(0x00);
                        set_ack_flag = 0;
                        MST_Data = (u16)(Datavin*4095/4);
                        MST_Data |= 0x8000;
                        SPISend(MST_Data);
                        OLED_ShowStr(0,2,"I:",2,1); //6*8
                        OLED_ShowStr(16,2,(unsigned char *)F2S(Datai,tstr),8,1); //6*8
                        OLED_ShowStr(0,3,"stalls:",7,1); //6*8
                        OLED_ShowStr(42,3,(unsigned char *)I2S(stalls,tstr),6,1); //6*8
                    }
              }
              else   // current_pot
              {
                  Datavin = (double)DMA_A1*4/3.3;
                  MST_Data = (u16)(Datavin*4095/4);
                  MST_Data |= 0x8000;
                  SPISend(MST_Data);
                  I_out = (double)DMA_A2*3.3/4095*10;
                  OLED_ShowStr(0,2,"stalls:",7,1); //6*8
				  OLED_ShowStr(42,2,(unsigned char *)I2S(stalls,tstr),6,1); //6*8
//                OLED_ShowStr(0,2,"I_out:",6,1); //6*8
//                OLED_ShowStr(36,2,(unsigned char *)F2S(I_out,tstr),6,1); //6*8
                  if(!((Pre_A3 - ADC12MEM3)<40 | (ADC12MEM3 - Pre_A3)<40))
                  {
                      OLED_ShowStr(24,6,(unsigned char *)F2S(ADC12MEM3,tstr),4,1); //6*8
                  }
                  Pre_A3 = ADC12MEM3;

              }

          }
          break;

          case 2:
          {
            if(set_ack_flag == 1)
            {
                OLED_ON();
                set_ack_flag = 0;
            }
          }
          break;

          case 3:   //off
          {
//            OLED_OFF();
              //shutdown the circuit
              __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3, interrupts enabled;
          }
          break;

      }//end of function switch;
//      OLED_ShowStr(0,3,(unsigned char *)F2S(DMA_A0,tstr),4,1); //6*8
//      OLED_ShowStr(0,4,(unsigned char *)F2S(DMA_A1,tstr),4,1); //6*8
//      OLED_ShowStr(0,5,(unsigned char *)F2S(DMA_A2,tstr),4,1); //6*8
//      OLED_ShowStr(0,6,(unsigned char *)F2S(ADC12MEM3,tstr),4,1); //6*8

//      Delay_ms(200);
      // if(strcmp(str_1,str)==0)
      // {
      //     P1OUT |= BIT0;
      //     UartTX_Send("Great!\r\n",8);
      //     Delay_ms(200);
      // }
      // else P1OUT &= ~BIT0;
  } //end of while

}

// Echo back RXed character, confirm TX buffer is ready first
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
      static int i=0;

  switch(__even_in_range(UCA0IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
    UCA0TXBUF = UCA0RXBUF;                  // TX -> RXed character
    //save string
    if(RX_flag == 0)  //进中断则不是完成接受状态
    {
        if(UCA0RXBUF!='\n')
        {
            a[i++]=UCA0RXBUF;
        }
        else
        {
            a[i]='\0';
            i=0;
            RX_flag = 1;
            strp = str;
        }

    }


      if(RX_flag == 1)
      {
              RX_flag = 0;
              //function switch
//            if(strcmp("on",str)==0)
//            {
//                SET_flag = 1;
//            }
              if(strcmp("slp",str)==0)
              {
                  function = 3;
    //            set_ack_flag = 1;
                  state = 0;
                  mode = 0;
                  fct_stc_flag = 1;
//                OLED_Clear(0x00);
//                OLED_OFF();
                  __bic_SR_register_on_exit(LPM3_bits);
              }
              else if(strcmp("out",str)==0)
              {
                  function = 1;
                  mode = 0;
//                OLED_ShowStr(0,0,"Current_bt",11,1); //6*8
                  SET_flag = 1;
                  fct_stc_flag = 1;
    //            state = 0;
              }
              else if(strcmp("test",str)==0)
              {
                  function = 2;
//                OLED_ShowStr(0,0,"R_test",11,1); //6*8
                  fct_stc_flag = 1;
    //            state = 0;
                  __bic_SR_register_on_exit(LPM3_bits);
              }
              else if(strcmp("off",str)==0)
              {
                  function = 0;
    //            set_ack_flag = 1;
                  state = 0;
                  mode = 0;
                  fct_stc_flag = 1;
//                OLED_Clear(0x00);
//                OLED_OFF();
                  __bic_SR_register_on_exit(LPM3_bits);

              }



              if(SET_flag)
              {
                  switch(function)
                  {
                  //Done nothing
                    case 0:
                    {
                        ;
                    }
                    break;

                    //output current
                    case 1:
                    {
                          if(strcmp("I:",str)==0)
                          {
                            state = 1;
    //                      __bic_SR_register_on_exit(LPM3_bits);
                            break;
                          }
                          else if(strcmp("Stalls",str)==0 | strcmp("stalls",str)==0)
                          {
                            state = 2;
    //                      __bic_SR_register_on_exit(LPM3_bits);
                            break;
                          }

                          if(state == 1) //wait for current
                          {
                              Datai = atof(str);
                              I_flag = 1;

                          }
                          else if(state == 2)
                          {
                              stalls = (int)(string_to_float(str));
                              stalls_flag = 1;
    //                        state = 201;
                          }

                          if(stalls_flag!=0 & I_flag!=0)
                          {
                              stalls_flag = 0;
                              I_flag = 0;

                              switch(stalls)
                              {
                                  case 0:
                                      break;
                                  case 1:
                                  {
    //                                state = 101;
                                      Datavin= Datai/5; // 不同的档位输入给定不同
                                      state = 0;
                                      SET_flag= 0;
                                      set_ack_flag = 1;
                                      __bic_SR_register_on_exit(LPM3_bits);
                                  }
                                  break;

                                  case 2:
                                  {
                                      state = 0;
                                      SET_flag = 0;
                                      set_ack_flag = 1;
                                      __bic_SR_register_on_exit(LPM3_bits);
                                  }
                                  break;

                                  case 3:
                                  {
                                      state = 0;
                                      SET_flag = 0;
                                      set_ack_flag = 1;
                                      __bic_SR_register_on_exit(LPM3_bits);
                                  }
                                  break;
                              }

                          }
                    }
                    break;
                    //end of output current

                    case 2:   //R_test
                    {

                        __bic_SR_register_on_exit(LPM3_bits);
                    }
                    break;

                    case 3:
                    {
                        ;  //shouldn't excute to this place
                    }
                    break;

                  }
              }
              //end of set
      }//end of receive


    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;  
  }
}

//------------------------------------------------------------------------------
// DMA Interrupt Service Routine
//------------------------------------------------------------------------------
#pragma vector=DMA_VECTOR
__interrupt void DMA_ISR(void)
{
  switch(__even_in_range(DMAIV,16))
  {
    case 0: break;
    case 2:                                 // DMA0IFG = DMA Channel 0
      P1OUT ^= BIT0;                        // Toggle P1.0 - PLACE BREAKPOINT HERE AND CHECK DMA_DST VARIABLE
      val_d0=val_d0+((DMA_A0-val_d0)/7);
    __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
      break;
    case 4:                                 // DMA1IFG = DMA Channel 1
//     P4OUT ^= BIT7;
      val_d1=val_d1+((DMA_A1-val_d1)/7);
      __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
      break;
    case 6:                         // DMA2IFG = DMA Channel 2
//     P4OUT ^= BIT7;
     val_d2=val_d2+((DMA_A2-val_d2)/7);
     __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
     break;
    case 8: break;                          // DMA3IFG = DMA Channel 3
    case 10: break;                         // DMA4IFG = DMA Channel 4
    case 12: break;                         // DMA5IFG = DMA Channel 5
    case 14: break;                         // DMA6IFG = DMA Channel 6
    case 16: break;                         // DMA7IFG = DMA Channel 7
    default: break;
  }
}

// Port 2 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) Port_2 (void)
#else
#error Compiler not supported!
#endif
{
//  P4OUT ^= BIT7;                            // P1.0 = toggle
  tri_flag = 1;
  P2IFG &= ~BIT1;                          // P1.4 IFG cleared
}

// Port 1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
#else
#error Compiler not supported!
#endif
{
//  P4OUT ^= BIT7;                            // P1.0 = toggle
  tri_flags = 1;
  P1IFG &= ~BIT1;                          // P1.1 IFG cleared
}


// Timer0 A0 interrupt service routine  ~57HZ 手算    MCLK ~1.045MHz
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    unsigned char temp;
    unsigned char temps;

//  P1OUT ^=BIT0;
    Tcal++;
//  UartTX_Send((char *)Tcal,8);
    if(tri_flag == 1)
    {
        tri_time++;
        if(tri_time >3)
        {
            tri_time = 0;
            tri_flag = 0;
            temp = P2IN & ~BIT1;
            if(P2IN == temp)
            {
//              if(function == 0) function++;
//              OLED_ON();
//              OLED_Clear(0x00);
            	buttom_fct_flag = 1;
            }
        }
    }

    if(tri_flags !=0)
    {
        tri_times++;
        if(tri_times >3)
        {
            tri_times = 0;
            tri_flags = 0;
            temps = P1IN & ~BIT1;
            if(P1IN == temps)
            {
//              if(function == 0) function++;
//              OLED_ON();
//              OLED_Clear(0x00);
            	buttom_fct_flags = 1;
            }
        }
    }
  __bic_SR_register_on_exit(LPM0_bits);
}

void cuto(int chn)
{
	//SPI output = 0
	MST_Data = 0;
    MST_Data |= 0x8000;
	SPISend(MST_Data);
	//cut IO
	P3OUT &= ~BIT6;
	P3OUT &= ~BIT5;

	P1OUT |= BIT5+BIT4;
	P1OUT &= ~BIT3;
	P1OUT &= ~BIT2;

	//connecti IO
	switch(chn)
	{
		case 1:
		{
			P3OUT &= ~BIT6;
			P3OUT |= BIT5;
			P1OUT &= ~BIT3;
			P1OUT |= BIT2;
			P1OUT &= ~BIT5;
			P1OUT &= ~BIT4;
		}
		break;

		case 2:
		{
			P3OUT |= BIT6;
			P3OUT &= ~BIT5;
			P1OUT |= BIT3;
			P1OUT &= ~BIT2;
			P1OUT &= ~BIT5;
			P1OUT &= ~BIT4;
		}
		break;

		case 3:
		{
			P3OUT |= BIT6;
			P3OUT |= BIT5;
			P1OUT |= BIT3;
			P1OUT |= BIT2;
			P1OUT &= ~BIT5;
			P1OUT &= ~BIT4;
		}
		break;

		default:
		break;
	}
	//OLED


}
void buttom_fct_stc_opt(void)
{
	switch(function)
	 {
		 case 0:
		 {
			 function = 1;
			 mode = 1;            //滑变
	//                      OLED_ShowStr(0,0,"Current_Pot",11,1); //6*8
			 fct_stc_flag = 1;
		 }
		 break;
		 case 1:
		 {
			 switch(mode)
			 {
				 case 0:
				 {
					 mode = 1;            //滑变
	//                              OLED_ShowStr(0,0,"Current_Pot",11,1); //6*8
					 fct_stc_flag = 1;
				 }
				 break;

				 case 1:
				 {
					 function = 3;       //sleep
					 mode = 0;
					 fct_stc_flag = 1;
				 }
				 break;
			 }
		 }
		 break;

		 case 3:
		 {
			 function = 2;        //测电阻
			 mode = 0;
	//                              OLED_ShowStr(0,0,"R_test",6,1); //6*8
			 fct_stc_flag = 1;
		 }
		 break;

		 case 2:
		 {
			 function = 0;                //off
			 mode = 0;
	//                      OLED_ShowStr(0,0,"Current_bt",10,1); //6*/8
			 fct_stc_flag = 1;
		 }
		 break;
	 }
}

void buttom_fct_stcs_opt(void)
{
	P4OUT ^= BIT7;
	if(function == 1 & mode == 1)
	{
		stalls++;
		if(stalls>3) stalls = 1;
	}
}

void DMAInit(void)
{
      DMACTL0 = DMA0TSEL_24+DMA1TSEL_24;        // DMA0 1 - ADC12IFGx
      DMACTL1 = DMA2TSEL_24;         //DMA3TSEL_24;                                                                    // DMA2
      DMACTL4 = DMARMWDIS;                      // Read-modify-write disable

      // Setup DMA0
      __data16_write_addr((unsigned short) &DMA0SA,(uint32) &ADC12MEM0);
//    __data16_write_addr((unsigned short) &DMA0SA,(unsigned short) &ADC12MEM0);
                                                // Source block address
      __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) &DMA_A0);
                                                // Destination single address
      DMA0SZ = 1;                               // Block size
      DMA0CTL &= ~DMAIFG;
      DMA0CTL = DMADT_4+DMAEN+DMADSTINCR_0+DMAIE; // Rpt single tranfer, dst, Int

      // Setup DMA1
      __data16_write_addr((unsigned short) &DMA1SA,(unsigned long) &ADC12MEM1);
                                                // Source block address
      __data16_write_addr((unsigned short) &DMA1DA,(unsigned long) &DMA_A1);
                                                // Destination single address
      DMA1SZ = 1;                               // Block size
      DMA1CTL &= ~DMAIFG;
      DMA1CTL = DMADT_4+DMAEN+DMADSTINCR_0+DMAIE; // Rpt single tranfer, dst, Int

      // Setup DMA2
      __data16_write_addr((unsigned short) &DMA2SA,(unsigned long) &ADC12MEM2);
                                                // Source block address
      __data16_write_addr((unsigned short) &DMA2DA,(unsigned long) &DMA_A2);
                                                // Destination single address
      DMA2SZ = 1;                               // Block size
      DMA2CTL &= ~DMAIFG;
      DMA2CTL = DMADT_4+DMAEN+DMADSTINCR_0+DMAIE; // Rpt single tranfer, dst, Int

//    // Setup DMA3
//    __data16_write_addr((unsigned short) &DMA3SA,(unsigned long) &ADC12MEM3);
//                                              // Source block address
//    __data16_write_addr((unsigned short) &DMA3DA,(unsigned long) &DMA_A3);
//                                              // Destination single address
//    DMA3SZ = 1;                               // Block size
//    DMA3CTL &= ~DMAIFG;
//    DMA3CTL = DMADT_4+DMAEN+DMADSTINCR_0+DMAIE; // Rpt single tranfer, dst, Int
}

void fct_stc_opt(void)
{
  int i = 0;
    switch(function)
    {
        case 0:  //off
        {
          OLED_OFF();
          for (i = 1; i > 0; i--)
          {
            P4OUT |= BIT7;
            Delay_ms(50);
            P4OUT &= ~BIT7;
            Delay_ms(50);
          }
        }
        break;

        case 1:
        {
            switch(mode)
            {
                case 0:
                {
//                  mode = 1;
                    OLED_ON();
                    OLED_Clear(0x00);
                    OLED_ShowStr(0,0,"Current_bt",11,1); //6*8
                }
                break;

                case 1:
                {
//                  function = 2;
                    OLED_ON();
                    OLED_Clear(0x00);
                    OLED_ShowStr(0,0,"Current_Pot",11,1); //6*8
                }
                break;
            }
        }
        break;

        case 2: //R_test
        {
//          function = 1;
//          mode = 0;
        	OLED_ON();
            OLED_Clear(0x00);
            OLED_ShowStr(0,0,"R_test",11,1); //6*8
        }
        break;

        case 3:   //sleep
        {
          OLED_OFF();
          for (i = 3; i > 0; i--)
          {
            P4OUT |= BIT7;
            Delay_ms(50);
            P4OUT &= ~BIT7;
            Delay_ms(50);
          }
        }
        break;
    }
  fct_stc_flag = 0;
}

//
//#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
//#pragma vector=USCI_B0_VECTOR
//__interrupt void USCI_B0_ISR(void)
//#elif defined(__GNUC__)
//void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
//#else
//#error Compiler not supported!
//#endif
//{
//  volatile unsigned int i;
//
//  switch(__even_in_range(UCB0IV,4))
//  {
//    case 0: break;                          // Vector 0 - no interrupt
//    case 2:                                 // Vector 2 - RXIFG
//      while (!(UCB0IFG&UCTXIFG));           // USCI_B0 TX buffer ready?
//
//      if (UCB0RXBUF==SLV_Data)              // Test for correct character RX'd
//        P1OUT |= 0x01;                      // If correct, light LED
//      else
//        P1OUT &= ~0x01;                     // If incorrect, clear LED
//
//      MST_Data++;                           // Increment data
//      SLV_Data++;
//      UCB0TXBUF = MST_Data;                 // Send next value
//
//      for(i = 20; i>0; i--);                // Add time between transmissions to
//                                            // make sure slave can process information
//      break;
//    case 4: break;                          // Vector 4 - TXIFG
//    default: break;
//  }
//}

