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
//******************************************************************************  //從這裡開始看
//  MSP430G2x33/G2x53 Demo - ADC10, Sample A10 Temp and Convert to oC and oF      //範例介紹
//
//  Description: A single sample is made on A10 with reference to internal
//  1.5V Vref. Software sets ADC10SC to start sample and conversion - ADC10SC
//  automatically cleared at EOC. ADC10 internal oscillator/4 times sample
//  (64x) and conversion. In Mainloop MSP430 waits in LPM0 to save power until
//  ADC10 conversion complete, ADC10_ISR will force exit from any LPMx in
//  Mainloop on reti. Temperaure in oC stored in IntDegC, oF in IntDegF.
//  Uncalibrated temperature measured from device to device will vary with
//  slope and offset - please see datasheet.
//  ACLK = n/a, MCLK = SMCLK = default DCO ~1.2MHz, ADC10CLK = ADC10OSC/4
//
//                MSP430G2x33/G2x53
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |A10              |
//
//  D. Dang
//  Texas Instruments Inc.
//  December 2010
//   Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10        //中文化：Edward Chen, 2015/03/25
//******************************************************************************
#include <msp430.h>

long temp;
long IntDegF;
long IntDegC;

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT                     //停止看門狗
  ADC10CTL1 = INCH_10 + ADC10DIV_3;         // Temp Sensor ADC10CLK/4       //INCH_10為溫度感應器所在腳位；ADC10DIV_3設定ADC模組時脈
  ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;              //設定ADC參數，可參考晶片使用手冊
  __enable_interrupt();                     // Enable interrupts.           //開啟系統總interrupts
  TACCR0 = 30;                              // Delay to allow Ref to settle //設定計時器計數器初始值(因為Vref開啟至穩定需要時間)
  TACCTL0 |= CCIE;                          // Compare-mode interrupt.      //開啟計數器interrupt
  TACTL = TASSEL_2 | MC_1;                  // TACLK = SMCLK, Up mode.      //設定計數器時脈來源，計數方向(向上數至溢位)
  LPM0;                                     // Wait for delay.              //關閉CPU(等待計數器溢位interrupt後將會喚醒CPU)
  TACCTL0 &= ~CCIE;                         // Disable timer Interrupt      //關閉計數器interrupt
  __disable_interrupt();													//關閉系統總interrupts

  while(1)
  {
    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start//開始取樣並轉換(ADC的運作內容)
    __bis_SR_register(CPUOFF + GIE);        // LPM0 with interrupts enabled //關閉CPU並開啟系統總interrupts(ADC轉換完畢會發生interrupt喚醒CPU)

    // oF = ((A10/1024)*1500mV)-923mV)*1/1.97mV = A10*761/1024 - 468        //10-bits ADC數值至華氏換算公式
    temp = ADC10MEM;                                                        //讀取ADC轉換結果暫存器
    IntDegF = ((temp - 630) * 761) / 1024;                                  //實際換算程式

    // oC = ((A10/1024)*1500mV)-986mV)*1/3.55mV = A10*423/1024 - 278        //10-bits ADC數值至攝氏換算公式
    temp = ADC10MEM;
    IntDegC = ((temp - 673) * 423) / 1024;
    
  }
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR                                                // ADC10 中斷(interrupt)副程式
__interrupt void ADC10_ISR (void)
{
  __bic_SR_register_on_exit(CPUOFF);       // Clear CPUOFF bit from 0(SR)  //清除CPUOFF值(也就是讓CPU恢復工作模式)
}                                                                          //然後就回到主程式


#pragma vector=TIMER0_A0_VECTOR                                            // TIMER0 A0 中斷(interrupt)副程式
__interrupt void ta0_isr(void)
{
  TACTL = 0;                                                               //清除計數器的值(歸零)
  LPM0_EXIT;                              // Exit LPM0 on return           //清除CPUOFF值(也就是讓CPU恢復工作模式)
}                                                                          //然後就回到主程式
