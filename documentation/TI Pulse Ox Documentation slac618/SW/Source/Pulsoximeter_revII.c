/* --COPYRIGHT--,BSD
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
 * --/COPYRIGHT--*/
//*****************************************************************************
//    MSP430FG437 based pulse oximeter demonstration - Version II
//    V. Chan and S. Underwood
//    May 2005
//    Modified by Bhargavi Nisarga
//    April 2008
//*****************************************************************************
#include  <msp430xG43x.h>
#include "stdint.h"
#include "intrinsics.h"
#include "math.h"

// LCD Segment Configuration
#define seg_a       0x01
#define seg_b       0x02
#define seg_c       0x04
#define seg_d       0x08
#define seg_e       0x40
#define seg_f       0x10
#define seg_g       0x20
#define seg_h       0x80

#define NUM_0   (seg_a | seg_b | seg_c | seg_d | seg_e | seg_f)
#define NUM_1   (seg_b | seg_c)
#define NUM_2   (seg_a | seg_b | seg_d | seg_e | seg_g)
#define NUM_3   (seg_a | seg_b | seg_c | seg_d | seg_g)
#define NUM_4   (seg_b | seg_c | seg_f | seg_g)
#define NUM_5   (seg_a | seg_c | seg_d | seg_f | seg_g)
#define NUM_6   (seg_a | seg_c | seg_d | seg_e | seg_f | seg_g)
#define NUM_7   (seg_a | seg_b | seg_c)
#define NUM_8   (seg_a | seg_b | seg_c | seg_d | seg_e | seg_f | seg_g)
#define NUM_9   (seg_a | seg_b | seg_c | seg_d | seg_f | seg_g)
#define NUM_A   (seg_a | seg_b | seg_c | seg_e | seg_f | seg_g)
#define NUM_B   (seg_c | seg_d | seg_e | seg_f | seg_g)
#define NUM_C   (seg_a | seg_d | seg_e | seg_f)
#define NUM_D   (seg_b | seg_c | seg_d | seg_e | seg_g)
#define NUM_E   (seg_a | seg_d | seg_e | seg_f | seg_g)
#define NUM_F   (seg_a | seg_e | seg_f | seg_g)

const unsigned char hex_table[] =
{
    NUM_0,NUM_1,NUM_2,NUM_3,NUM_4,NUM_5,NUM_6,NUM_7,
    NUM_8,NUM_9,NUM_A,NUM_B,NUM_C,NUM_D,NUM_E,NUM_F
};

int32_t mul16(register int16_t x, register int16_t y);
                                           
//FIR filter coefficient for removing 50/60Hz and 100/120Hz from the signals
#if 0 
static const int16_t coeffs[9] =
{
    5225,
    5175,
    7255,
    9453,
    11595,
    13507,
    15016,
    15983,
    16315
};
#else
static const int16_t coeffs[12] =
{
    688,
    1283,
    2316,
    3709,
    5439,
    7431,
    9561,
    11666,
    13563,
    15074,
    16047,
    16384
};
#endif

// SaO2 Look-up Table
const unsigned int Lookup [43] = {100,100,100,100,99,99,99,99,99,99,98,98,98,98,
                                  98,97,97,97,97,97,97,96,96,96,96,96,96,95,95,
                                  95,95,95,95,94,94,94,94,94,93,93,93,93,93};
//
//  #define FIRST_STAGE_TARGET_HIGH         3900
//  #define FIRST_STAGE_TARGET_LOW          3600
//  #define FIRST_STAGE_TARGET_HIGH_FINE    4096
//  #define FIRST_STAGE_TARGET_LOW_FINE     3500

// LED Target Range 
#define FIRST_STAGE_TARGET_HIGH         3500
#define FIRST_STAGE_TARGET_LOW          3000
#define FIRST_STAGE_TARGET_HIGH_FINE    4096
#define FIRST_STAGE_TARGET_LOW_FINE     2700
#define FIRST_STAGE_STEP                5
#define FIRST_STAGE_FINE_STEP           1

// UART Transmission Structure Definition
enum scope_type_e
{
    SCOPE_TYPE_OFF = 0,
    SCOPE_TYPE_HEART_SIGNALS,
    SCOPE_TYPE_RAW_SIGNALS,
    SCOPE_TYPE_LED_DRIVE,
};
int scope_type = SCOPE_TYPE_HEART_SIGNALS;
//int scope_type = SCOPE_TYPE_RAW_SIGNALS;

int ir_dc_offset = 2000;
int vs_dc_offset = 2000;
int ir_LED_level;
int vs_LED_level;
int ir_sample;
int vs_sample;
char is_IR; 
int ir_heart_signal;
int vs_heart_signal;
int ir_heart_ac_signal;
int vs_heart_ac_signal;
unsigned int rms_ir_heart_ac_signal;
unsigned int rms_vs_heart_ac_signal;
int32_t ir_2nd_dc_register = 0;
int32_t vs_2nd_dc_register = 0;
unsigned long log_sq_ir_heart_ac_signal;
unsigned long log_sq_vs_heart_ac_signal;
unsigned long sq_ir_heart_ac_signal;
unsigned long sq_vs_heart_ac_signal;
unsigned int pos_edge = 0;
unsigned int edge_debounce;
unsigned int heart_beat_counter;
unsigned int log_heart_signal_sample_counter;
unsigned int heart_signal_sample_counter;

volatile unsigned int j;

/* The results */
unsigned int heart_rate;
unsigned int heart_rate_LSB = 0;
unsigned int SaO2, Ratio;
unsigned int SaO2_LSB = 0;

/* Function prototypes */
//unsigned long isqrt32(register unsigned long h);
int16_t dc_estimator(register int32_t *p, register int16_t x);
int16_t ir_filter(int16_t sample);
int16_t vs_filter(int16_t sample);
void set_LCD(void);
void display_number(int value, int start, int width);
void display_pulse(int on);
void display_correcting(int x, int on);

void main(void)
{
    double f1;
    int32_t x;
    int32_t y;    
    
    WDTCTL = WDTPW | WDTHOLD; 
    SCFI0 |= FN_4;                          // x2 DCO frequency, 8MHz nominal
                                            // DCO
    SCFQCTL = 91;                           // 32768 x 2 x (91 + 1) = 6.03 MHz
    FLL_CTL0 = DCOPLUS + XCAP14PF;          // DCO+ set so freq = xtal x D x
                                            //(N + 1)
    // Loop until 32kHz crystal stabilizes
    do
    {
      IFG1 &= ~OFIFG;                       // Clear oscillator fault flag
      for (j = 50000; j; j--);              // Delay
    }
    while (IFG1 & OFIFG);                   // Test osc fault flag
    
    // Setup GPIO
    P1DIR = 0xFF;
    P1OUT = 0;
    P2DIR = 0xFF;
    P2DIR |= BIT2 + BIT3;                   // P2.2 and P2.3 o/p direction - 
                                            // drives PNP transistors in H-Bridge
    P2OUT = 0;
    P3DIR = 0xFF;
    P3OUT = 0;
    P4DIR = 0xFF;
    P4OUT = 0;
    P5DIR = 0xFF;
    P5OUT = 0;
    P6OUT = 0; 
    
    /* Setup LCD */
    set_LCD(); 
    
    /* First amplifier stage - transconductance configuration */
    P6SEL |= (BIT0 | BIT1 | BIT2);          // Select OA0O
                                            // -ve=OA0I0, +ve=OA0I1
    OA0CTL0 = OAN_0 | OAP_1 | OAPM_3 | OAADC1;
    OA0CTL1 = 0;    
    
    /* Second amplifier stage */
    P6SEL |= (BIT3 | BIT4);                 // Select 0A1O 0A1I
                                            // -ve=OA1I0, +ve=DAC1
                                            // -ve=OA1I0, +ve=DAC1
//    OA1CTL0 = OAN_0 | OAP_3 | OAPM_3 | OAADC1;
//    OA1CTL1 = 0x00;   
                                            // Inverted input internally 
                                            // connected to OA0 output 
    OA1CTL0 = OAN_2 + OAP_3 + OAPM_3 + OAADC1;
    OA1CTL1 = OAFBR_7 + OAFC_6;             // OA as inv feedback amp, internal
                                            // gain = 15;    
    
    /* Configure DAC 1 to provide bias for the amplifier */
    P6SEL |= BIT7;
    DAC12_1CTL = DAC12CALON | DAC12IR | DAC12AMP_7 | DAC12ENC;
    DAC12_1DAT = 0; 
    
    /* Configure DAC 0 to provide variable drive to the LEDs */
    DAC12_0CTL =  DAC12CALON | DAC12IR | DAC12AMP_7 | DAC12ENC; // VRef+, high speed/current, 
                                            // DAC12OPS=0 => DAC12_0 output on P6.6 (pin 5) */
                                            // Configure P2.2 and P2.3 to 
                                            // provide variable drive to LEDs
    P2OUT |= BIT2;                          // turn off source for D2
    P2OUT &= ~BIT3;                         // turn on source for D3        
    DAC12_0DAT = 3340;    

                                            // Set initial values for the LED brightnesses 
    ir_LED_level = 1300;
    vs_LED_level = 1450;    
    
    /* Configure ADC12 */
    ADC12CTL0 &= ~ENC;                      // Enable conversions
                                            // Turn on the ADC12, and
                                            // set the sampling time
    ADC12CTL0 = ADC12ON + MSC + SHT0_4 + REFON + REF2_5V;
    ADC12CTL1 = SHP + SHS_1 + CONSEQ_1;     // Use sampling timer, single sequence,
                                            // TA1 trigger(SHS_1), start with ADC12MEM0
    ADC12MCTL0 = INCH_1 + SREF_1;           // ref+=Vref, channel = A1 = OA0
    ADC12MCTL1 = INCH_3 + SREF_1 + EOS;     // ref+=Vref, channel = A3 = OA1
    ADC12IE = BIT1;                         // ADC12MEM1 interrupt enable
    ADC12CTL0 |= ENC;                       // Enable the ADC
    ADC12CTL0 |= ADC12SC;                   // Start conversion    
    
    /* Configure Timer */
    TACTL = TASSEL0 + TACLR;                // ACLK, clear TAR,
    TACCTL1 = OUTMOD_2;
    TACCTL0 = CCIE;
                                            // This gives a sampling rate of
                                            // 512sps
    TACCR0 = 31;                            // Do two channels, at
                                            // 512sps each.
    TACCR1 = 10;                            // Allow plenty of time for the
                                            // signal to become stable before
                                            // sampling   
    TACTL |= MC_1;                          // Timer A on, up mode     
                                            
    /*Configure USART, so we can report readings to a PC */
    P2DIR |= BIT4;
    P2SEL |= BIT4;
    
    UCTL0 |= SWRST;
    ME1 |= UTXE0;                           // Enable USART1 TXD
    UCTL0 |= CHAR;                          // 8-bit char, SWRST=1
    UTCTL0 |= SSEL1;                        // UCLK = SMCLK
    UBR00 = 52;                             // 115200 from 6.02MHz = 52.33
    UBR10 = 0x00;
    UMCTL0 = 0x45;                          // Modulation = 0.375
    UCTL0 &= ~SWRST;                        // Initialise USART    
   
    while(1) 
    {
        __bis_SR_register(LPM0_bits + GIE); 
 
        /* Heart Rate Computation */
        f1 = 60.0*512.0*3.0/(float)log_heart_signal_sample_counter;
        heart_rate = f1;
        display_number(heart_rate, 3, 3);
        heart_rate_LSB = heart_rate & 0x00FF;
        
        /* SaO2 Computation */
        x = log_sq_ir_heart_ac_signal/log_heart_signal_sample_counter;
        y = log_sq_vs_heart_ac_signal/log_heart_signal_sample_counter;
        Ratio = (unsigned int) (100.0*log(y)/log(x));
        if (Ratio > 66)
          SaO2 = Lookup[Ratio - 66];        // Ratio - 50 (Look-up Table Offset) - 16 (Ratio offset)
        else if (Ratio > 50)
          SaO2 = Lookup[Ratio - 50];        // Ratio - 50 (Look-up Table Offset)
        else
          SaO2 = 100;
        display_number(SaO2, 7, 3); 
        SaO2_LSB = SaO2 & 0x00FF;
    }
}


// Timer A0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A0(void)
{
    int i;
    if ((DAC12_0CTL & DAC12OPS))            // D2 enabled in demo board
    {
                                            // Immediately enable the visible
                                            // LED, to allow time for the
                                            // transimpedance amp to settle
        DAC12_0CTL &= ~DAC12ENC;
        P2OUT &= ~BIT3;                     // turn on source for D3
        DAC12_0CTL &= ~DAC12OPS;            // Disable IR LED, enable visible LED
        DAC12_0CTL |= DAC12ENC;
        DAC12_0DAT = vs_LED_level;
        DAC12_1DAT = vs_dc_offset;          // Load op-amp offset value for visible
        P2OUT |= BIT2;                      // turn off source for D2        
        
        is_IR = 0;                          // IR LED OFF
        
        ir_sample = ADC12MEM0;              // Read the IR LED results 
        i = ADC12MEM1; 
                                            // Enable the next conversion sequence. 
                                            // The sequence is started by TA1 
        ADC12CTL0 &= ~ENC;
        ADC12CTL0 |= ENC;  
        
                                            // Filter away 50/60Hz electrical pickup, 
                                            // and 100/120Hz room lighting optical pickup 
        ir_heart_signal = ir_filter(i);
                                            // Filter away the large DC
                                            // component from the sensor */
        ir_heart_ac_signal = ir_heart_signal - dc_estimator(&ir_2nd_dc_register, ir_heart_signal);  
        
        /* Bring the IR signal into range through the second opamp */    
        if (i >= 4095)
        {
            if (ir_dc_offset > 100)
                ir_dc_offset--;
        }
        else if (i < 100)
        {
            if (ir_dc_offset < 4095)
                ir_dc_offset++;
        }        

        sq_ir_heart_ac_signal += (mul16(ir_heart_ac_signal, ir_heart_ac_signal) >> 10);

                                            //Tune the LED intensity to keep
                                            //the signal produced by the first
                                            //stage within our target range.
                                            //We don't really care what the
                                            //exact values from the first
                                            //stage are. They need to be
                                            //quite high, because a weak
                                            //signal will give poor results
                                            //in later stages. However, the
                                            //exact value only has to be
                                            //within the range that can be
                                            //handled properly by the next
                                            //stage. */        
        
        if (ir_sample > FIRST_STAGE_TARGET_HIGH
            ||
            ir_sample < FIRST_STAGE_TARGET_LOW)
        {
                                            //We are out of the target range
                                            //Starting kicking the LED
                                            //intensity in the right
                                            //direction to bring us back
                                            //into range. We use fine steps
                                            //when we are close to the target
                                            //range, and coarser steps when
                                            //we are far away.
            if (ir_sample > FIRST_STAGE_TARGET_HIGH)
            {
                if (ir_sample >= FIRST_STAGE_TARGET_HIGH_FINE)
                    ir_LED_level -= FIRST_STAGE_STEP;
                else
                    ir_LED_level -= FIRST_STAGE_FINE_STEP;
                                            // Clamp to the range of the DAC 
                if (ir_LED_level < 0)
                    ir_LED_level = 0;
            } 
            else
            {
                if (ir_sample < FIRST_STAGE_TARGET_LOW_FINE)
                    ir_LED_level += FIRST_STAGE_STEP;
                else
                    ir_LED_level += FIRST_STAGE_FINE_STEP;
                                            // Clamp to the range of the DAC 
                if (ir_LED_level > 4095)
                    ir_LED_level = 4095;
            }
        } 

        /* UART Transmission - IR heart signals */      
        switch (scope_type)
        {
        case SCOPE_TYPE_HEART_SIGNALS:
            i = (ir_heart_ac_signal >> 6) + 128;
                                            // Saturate to a byte 
            if (i >= 255)                   // Make sure the data != 0x0 or 0xFF
                i = 254;                    // as 0x0 and 0xFF are used for sync
            else if (i <= 0)                // bytes in the LABVIEW GUI
                i = 1;
            
           TXBUF0 = 0x00;                   // Byte 1 - 0x00 (synchronization byte)
           while (!(IFG1 & UTXIFG0)); 
           TXBUF0 = 0xFF;                   // Byte 2 - 0xFF (synchronization byte)
           while (!(IFG1 & UTXIFG0)); 
           TXBUF0 = i;                      // Byte 3 - IR Heart signal (AC only)
           while (!(IFG1 & UTXIFG0));
           TXBUF0 = heart_rate_LSB;         // Byte 4 - Heart rate data
           while (!(IFG1 & UTXIFG0));
           TXBUF0 = SaO2_LSB;               // Byte 5 - %SaO2 data
           break;
            
        case SCOPE_TYPE_RAW_SIGNALS:
            while (!(IFG1 & UTXIFG0));            
            TXBUF0 = ir_sample >> 4;
            break;
        case SCOPE_TYPE_LED_DRIVE:
            TXBUF0 = ir_LED_level >> 4;
            break;
        }   
        
        /* Track the beating of the heart */
        heart_signal_sample_counter++;
        if (pos_edge)
        {
            if (edge_debounce < 120)
            {
                edge_debounce++;
            }
            else
            {
                if (ir_heart_ac_signal < -200)
                {
                    edge_debounce = 0;
                    pos_edge = 0;
                    display_pulse(0);
                }
            }
        }
        else
        {
            if (edge_debounce < 120)
            {
                edge_debounce++;
            }
            else
            {
                if (ir_heart_ac_signal > 200)
                {
                    edge_debounce = 0;
                    pos_edge = 1;
                    display_pulse(1);
                    display_correcting(1, 0);
                    if (++heart_beat_counter >= 3)
                    {
                        log_heart_signal_sample_counter = heart_signal_sample_counter;
                        log_sq_ir_heart_ac_signal = sq_ir_heart_ac_signal;
                        log_sq_vs_heart_ac_signal = sq_vs_heart_ac_signal;
                        heart_signal_sample_counter = 0;
                        sq_ir_heart_ac_signal = 0;
                        sq_vs_heart_ac_signal = 0;
                        heart_beat_counter = 0;
                        _BIC_SR_IRQ(LPM0_bits);
                                            // Do a dummy wake up roughly
                                            // every 2 seconds
                    }
                }
            }
        }
    }
    else                                    //D3 enabled in demoboard
    {
                                            //Immediately enable the IR LED,
                                            //to allow time for the
                                            //transimpedance amp to settle */
        DAC12_0CTL &= ~DAC12ENC; 
        P2OUT &= ~BIT2;                     //turn on source for D3        
        DAC12_0CTL |= DAC12OPS;             // Disable visible LED, enable IR LED
        DAC12_0CTL |= DAC12ENC;
        DAC12_0DAT = ir_LED_level;
        DAC12_1DAT = ir_dc_offset;          // Load op-amp offset value for IR
        P2OUT |= BIT3;                      //turn off source for D2
        
        is_IR = 1;                          // IR LED ON
        
        vs_sample = ADC12MEM0;              //Read the visible LED results 
        i = ADC12MEM1;
        
                                            //Enable the next conversion sequence. 
                                            //The sequence is started by TA1 
        ADC12CTL0 &= ~ENC;
        ADC12CTL0 |= ENC;  


                                            //Filter away 50/60Hz electrical
                                            //pickup, and 100/120Hz room
                                            //lighting optical pickup */
        vs_heart_signal = vs_filter(i);
                                            //Filter away the large DC
                                            //component from the sensor */
        vs_heart_ac_signal = vs_heart_signal - dc_estimator(&vs_2nd_dc_register, vs_heart_signal);
        
       /* Bring the VS signal into range through the second opamp */
        if (i >= 4095)
        {
            if (vs_dc_offset > 100)
                vs_dc_offset--;
        }
        else if (i < 100)
        {
            if (vs_dc_offset < 4095)
                vs_dc_offset++;
        }        

        sq_vs_heart_ac_signal += (mul16(vs_heart_ac_signal, vs_heart_ac_signal) >> 10);
        
        if (vs_sample > FIRST_STAGE_TARGET_HIGH
            ||
            vs_sample < FIRST_STAGE_TARGET_LOW)
        {
            /* We are out of the target range */
            display_correcting(1, 1);
            if (vs_sample > FIRST_STAGE_TARGET_HIGH)
            {
                if (vs_sample >= FIRST_STAGE_TARGET_HIGH_FINE)
                    vs_LED_level -= FIRST_STAGE_STEP;
                else
                    vs_LED_level -= FIRST_STAGE_FINE_STEP;
                if (vs_LED_level < 0)
                    vs_LED_level = 0;
            }
            else
            {
                if (vs_sample < FIRST_STAGE_TARGET_LOW_FINE)
                    vs_LED_level += FIRST_STAGE_STEP;
                else
                    vs_LED_level += FIRST_STAGE_FINE_STEP;
                if (vs_LED_level > 4095)
                    vs_LED_level = 4095;
            }
        }
    }
       
}
    
#pragma vector=ADC_VECTOR
__interrupt void ADC12ISR(void)
{
    ADC12IFG &= ~BIT1;                      // Clear the ADC12 interrupt flag
    DAC12_0DAT = 0;                         // Turn OFF the LED 
    DAC12_1DAT = 0; 
    // Turn OFF the H-Bridge completely
    if(is_IR)                               // If IR LED was ON in TA0 ISR
      P2OUT |= BIT2;                        // P2.2 = 1
    else                                    // Else if VS LED ON in TA0 ISR
      P2OUT |= BIT3;                        // P2.3 = 1
}

int16_t ir_filter(int16_t sample)
{
    static int16_t buf[32];
    static int offset = 0;
    int32_t z,test;
    int i;
                                            //Filter hard above a few Hertz,
                                            //using a symmetric FIR.
                                            //This has benign phase
                                            //characteristics */
    buf[offset] = sample;
    z = mul16(coeffs[11], buf[(offset - 11) & 0x1F]);
    for (i = 0;  i < 11;  i++)
        z += mul16(coeffs[i], buf[(offset - i) & 0x1F] + buf[(offset - 22 + i) & 0x1F]);
    offset = (offset + 1) & 0x1F;
    return  z >> 15;
}

int16_t vs_filter(int16_t sample)
{
    static int16_t buf[32];
    static int offset = 0;
    int32_t z;
    int i;

                                            //Filter hard above a few Hertz,
                                            //using a symmetric FIR.
                                            //This has benign phase
                                            //characteristics */
    buf[offset] = sample;
    z = mul16(coeffs[11], buf[(offset - 11) & 0x1F]);
    for (i = 0;  i < 11;  i++)
        z += mul16(coeffs[i], buf[(offset - i) & 0x1F] + buf[(offset - 22 + i) & 0x1F]);
    offset = (offset + 1) & 0x1F;
    return  z >> 15;
}

/*unsigned long isqrt32(register unsigned long h)
{
    register unsigned long x;
    register unsigned long y;
    register int i;

                                            //Calculate a 32 bit bit square
                                            //root of a 32 bit integer,
                                            //where the top 16 bits
                                            //of the result is the integer
                                            //part of the result, and the
                                            //low 16 bits are fractional.
    x =
    y = 0;
    for (i = 0;  i < 32;  i++)
    {
        x = (x << 1) | 1;
        if (y < x)
            x -= 2;
        else
            y -= x;
        x++;
        y <<= 1;
        if ((h & 0x80000000))
            y |= 1;
        h <<= 1;
        y <<= 1;
        if ((h & 0x80000000))
            y |= 1;
        h <<= 1;
    }
    return  x;
} */

int16_t dc_estimator(register int32_t *p, register int16_t x)
{
    /* Noise shaped DC estimator. */
    *p += ((((int32_t) x << 16) - *p) >> 9);
    return (*p >> 16);
}

/* LCD number Display */
void display_number(int value, int start, int width)
{
  unsigned int i;
  unsigned int Output;
  char *pLCD = (char *)&LCDMEM[7-start]; 

  for (i = 16, Output = 0; i; i--)              // BCD Conversion, 16-Bit
  {
    Output = __bcd_add_short(Output, Output);
    if (value & 0x8000)
      Output = __bcd_add_short(Output, 1);
    value <<= 1;
  }
  
  for (i = 0; i < width; i++)                           // Process 4 digits
  {
    *pLCD++ = hex_table[Output & 0x0f];           // Segments to LCD
    Output >>= 4;                               // Process next digit
  }
}

/* LCD Pulse Display */
void display_pulse(int on)
{
    if (on)
        LCDMEM[7] |= 0x4;
    else
        LCDMEM[7] &= ~0x4; 
}

/* LCD Correcting info Display */
void display_correcting(int x, int on)
{
    if (on)
        LCDMEM[3] |= ((x)  ?  seg_a  :  seg_d);
    else
        LCDMEM[3] &= ~((x)  ?  seg_a  :  seg_d);
}

/* Configure LCD */
void set_LCD(void)
{
    volatile unsigned int i;
    for(i=0;i<11;i++)                         // Clear LCD memory
    {
      LCDMEM[i] = 0x00;
    }    

    /* Turn on the COM0-COM3 and R03-R33 pins */
    P5SEL |= (BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2);

    LCDCTL = 0x7F;                          // Selected function: Analog generator on
                                            // Low impedance of AG
                                            // 4Mux active
                                            // all outputs are Seg
                                            // S0-S23 are LCD segment lines
    BTCTL = BTFRFQ0;	                    // Start Basic Timer 1s + LCD 64Hz
}

