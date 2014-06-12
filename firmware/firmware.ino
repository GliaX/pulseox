volatile int bits=0;
volatile int flag=0;
volatile int bytetosend=0;

void setup()
{
  TACCR0 = 272; //set timer to tick every bit at 57600bps
  TACCTL0 |= CCIE; //enable timer interrupt
  TACTL=TASSEL_2|ID_0|MC_1|TACLR; //set timer clock to SMCLK, no divider, up mode, clear
pinMode(P1_0,INPUT);//ADC input from photodiode
pinMode(P1_1,INPUT);//~30x ADC input from photodiode
pinMode(P2_0,OUTPUT);//LED
pinMode(P2_1,OUTPUT);//LED
pinMode(P2_2,OUTPUT);//LED
pinMode(P2_5,OUTPUT);//debug pin for logic analyzer timing measurement
pinMode(P2_7,OUTPUT);//serial out
digitalWrite(P2_0,LOW);
digitalWrite(P2_1,LOW);
digitalWrite(P2_2,LOW);
digitalWrite(P2_5,LOW);
digitalWrite(P2_7,HIGH);//UART signals idle high
  
}
__attribute__((interrupt(TIMER0_A0_VECTOR)))
void ta_handler(void)
{
    if(flag){ //enforce one clock delay between bytes
     flag=0;
    return; 
    }
    if(!bits){
      P2OUT |= (1<<7); //out of bits to send, set line high
      //todo: disable timer
      flag=1;
      return;
    }
    if(bytetosend&0x1){ //if last bit is high, set line high
      P2OUT|=(1<<7);
    }else{
      P2OUT &= ~(1<<7);//set line low
    }
    bytetosend>>=1;//right shift, drop bit we just sent
    --bits;
}

void sendbyte(unsigned char b){
while(bits)__asm("nop;");//busywait while sending
bytetosend=((0x0100+b)<<1);//add stop bit to our byte, and left-shift to add start bit
bits=10;//1 byte, plus start and stop bit.
}

__attribute__((interrupt(ADC10_VECTOR)))
void ADC10_ISR(void)
{
    __bic_SR_register_on_exit(CPUOFF); // return to active mode
}
void loop()
{
int irval=0;
int i;
int redval=0;
int a0,a1;
ADC10CTL1 = ADC10SSEL_0 | ADC10DIV_0; //set ADC clock to ADC10OSC, no divider
ADC10CTL0 = ADC10ON | ADC10SHT_3 | ADC10IE |ADC10SR; //default reference, max sample/hold time, low sample rate, interrupt enabled
ADC10AE0 = 3; //enable channels 0 and 1
ADC10CTL1 = ADC10SSEL_0 | ADC10DIV_0 ;//channel 0, use |(4096) for channel 1
__delay_cycles(128);

while(1){
    P2OUT|=32; //debug pin high
    P2OUT|=1; //set pin 0 high
    delayMicroseconds(10);
    ADC10CTL0 = ADC10ON | ADC10SHT_3 | ADC10IE |ADC10SR;
    ADC10CTL1 = ADC10SSEL_0 | ADC10DIV_0;//channel 0
    ADC10CTL0 |= ENC | ADC10SC; // enable ADC and start conversion
    while (ADC10CTL1 & ADC10BUSY) { // sleep and wait for completion
        __bis_SR_register(CPUOFF + GIE); // LPM0 with interrupts enabled
    }
    a0=0;
    for(i=0;i<16;++i){ADC10CTL0 = ADC10ON | ADC10SHT_3 | ADC10IE |ADC10SR;
    ADC10CTL1 = ADC10SSEL_0 | ADC10DIV_0;//channel 0
    ADC10CTL0 |= ENC | ADC10SC; // enable ADC and start conversion
    while (ADC10CTL1 & ADC10BUSY) { // sleep and wait for completion
        __bis_SR_register(CPUOFF + GIE); // LPM0 with interrupts enabled
    }
    a0+=ADC10MEM;
    }
    irval=irval-(irval>>3)+a0;
    ADC10CTL1 = ADC10SSEL_0 | ADC10DIV_0|(4096);//channel 1
    ADC10CTL0 |= ENC | ADC10SC; // enable ADC and start conversion
    sendbyte((irval>>8)&0xff);
    sendbyte((irval)&0xff);
    while (ADC10CTL1 & ADC10BUSY) { // sleep and wait for completion
        __bis_SR_register(CPUOFF + GIE); // LPM0 with interrupts enabled
    }
    a1=ADC10MEM;
    P2OUT&=248; //set pins low
    P2OUT|=6; //set pins 1 and 2 high
    delayMicroseconds(10);
    ADC10CTL0 = ADC10ON | ADC10SHT_3 | ADC10IE |ADC10SR;
    ADC10CTL1 = ADC10SSEL_0 | ADC10DIV_0;//channel 0
    ADC10CTL0 |= ENC | ADC10SC; // enable ADC and start conversion
    while (ADC10CTL1 & ADC10BUSY) { // sleep and wait for completion
        __bis_SR_register(CPUOFF + GIE); // LPM0 with interrupts enabled
    }
    a0=0;
    for(i=0;i<16*4;++i){ADC10CTL0 = ADC10ON | ADC10SHT_3 | ADC10IE |ADC10SR;
    ADC10CTL1 = ADC10SSEL_0 | ADC10DIV_0;//channel 0
    ADC10CTL0 |= ENC | ADC10SC; // enable ADC and start conversion
    while (ADC10CTL1 & ADC10BUSY) { // sleep and wait for completion
        __bis_SR_register(CPUOFF + GIE); // LPM0 with interrupts enabled
    }
    a0+=ADC10MEM;
    }
    redval=redval-(redval>>3)+a0;
    P2OUT&=223;//debug pin low
    ADC10CTL1 = ADC10SSEL_0 | ADC10DIV_0|(4096);//channel 1
    ADC10CTL0 |= ENC | ADC10SC; // enable ADC and start conversion
    sendbyte((redval>>8)&0xff);
    sendbyte((redval)&0xff);
    while (ADC10CTL1 & ADC10BUSY) { // sleep and wait for completion
        __bis_SR_register(CPUOFF + GIE); // LPM0 with interrupts enabled
    }
    P2OUT&=248; //set pins low
    a1=ADC10MEM;
    ADC10CTL0 &= ~(ADC10ON | REFON);

    sendbyte('\n');
     delay(16);
    
}
}
