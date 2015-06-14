
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/stm32/pwr.h>

#include <libopencm3/stm32/gpio.h>


#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>

uint8_t channel_array[] = { ADC_CHANNEL0 };//, ADC_CHANNEL1, ADC_CHANNEL_TEMP};


static void disp_write(unsigned char dat, unsigned char cmd)
{
    unsigned int i,x;
    if(cmd==1){
		gpio_clear(GPIOA, GPIO6);
	}else{
		gpio_set(GPIOA, GPIO6);
	}
    gpio_clear(GPIOA, GPIO5);// SCK=0;
	gpio_clear(GPIOA, GPIO10);//CS1=0;

	for(x=0;x<8;x++)
	{
	    gpio_clear(GPIOA, GPIO5);//SCK=0;
	for (i = 0; i < 3; i++)	/* Wait a bit. */
			__asm__("nop");
		//_nop_();
		//_nop_();
		if(dat&0x80){ //set DATA value
			gpio_set(GPIOA, GPIO7);
		}else{
			gpio_clear(GPIOA, GPIO7);
		}
		dat<<=1;
	for (i = 0; i < 3; i++)	/* Wait a bit. */
			__asm__("nop");
		//_nop_();
		//_nop_();
		gpio_set(GPIOA, GPIO5);//SCK=1;
	for (i = 0; i < 3; i++)	/* Wait a bit. */
			__asm__("nop");
	
		//_nop_();
		//_nop_();
	}
	gpio_set(GPIOA, GPIO10);//CS1=1;
	for (i = 0; i < 10; i++)	/* Wait a bit. */
			__asm__("nop");
	return;
}
static void disp_write16(unsigned int dat, unsigned char cmd)
{
    unsigned int i,x;
    if(cmd){
		gpio_clear(GPIOA, GPIO6);
	}else{
		gpio_set(GPIOA, GPIO6);
	}
    gpio_clear(GPIOA, GPIO5);// SCK=0;
	gpio_clear(GPIOA, GPIO10);//CS1=0;

	for(x=0;x<16;x++)
	{
	    gpio_clear(GPIOA, GPIO5);//SCK=0;
	for (i = 0; i < 3; i++)	/* Wait a bit. */
			__asm__("nop");
		//_nop_();
		//_nop_();
		if(dat&0x80){ //set DATA value
			gpio_set(GPIOA, GPIO7);
		}else{
			gpio_clear(GPIOA, GPIO7);
		}
		dat<<=1;
	for (i = 0; i < 3; i++)	/* Wait a bit. */
			__asm__("nop");
		//_nop_();
		//_nop_();
		gpio_set(GPIOA, GPIO5);//SCK=1;
	for (i = 0; i < 3; i++)	/* Wait a bit. */
			__asm__("nop");
	
		//_nop_();
		//_nop_();
	}
	gpio_set(GPIOA, GPIO10);//CS1=1;
	for (i = 0; i < 10; i++)	/* Wait a bit. */
			__asm__("nop");
	return;
}


static void setrow(unsigned char row){
	disp_write(0xb0|row,1);
}
static void setcol(unsigned char col){
	disp_write(0x10|(col>>4),1);
	disp_write(0x0f&col,1);
	
}



static void adc_setup(void)
{
	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_clock_enable(RCC_GPIOA);
	
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	
	adc_power_off(ADC1);
	adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
	adc_calibrate_start(ADC1);
	adc_calibrate_wait_finish(ADC1);
	adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	//adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_041DOT5);
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
	adc_disable_analog_watchdog(ADC1);
	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 8000; i++) {    /* Wait a bit. */
		__asm__("nop");
	}

}


void rtc_isr(void)
{
        RTC_ISR &= ~(RTC_ISR_ALRAF|RTC_ISR_ALRAWF);
        exti_reset_request(EXTI17);
	
}

static void rtc_setup(void)
{
	rcc_periph_clock_enable(RCC_PWR);
	pwr_disable_backup_domain_write_protect();
	RCC_BDCR |= (1<<16);
    RCC_BDCR &= ~(1<<16);
	rcc_osc_on(LSI);
	rcc_wait_for_osc_ready(LSI);
	RCC_BDCR &= ~(RCC_BDCR_RTCSEL_HSE|RCC_BDCR_RTCSEL_LSE);
	RCC_BDCR |= RCC_BDCR_RTCSEL_LSI;
	RCC_BDCR |= (1<<15); //RTC enable
	rcc_periph_clock_enable(RCC_RTC);
	rtc_unlock();
	RTC_ISR |=RTC_ISR_INIT;
	while ((RTC_ISR & RTC_ISR_INITF) != RTC_ISR_INITF)__asm__("nop");
	rtc_set_prescaler(127, 212);
	RTC_ISR &= ~(RTC_ISR_INIT);
	PWR_CR &= ~PWR_CR_PDDS;
	PWR_CR &= ~PWR_CR_LPDS;
	PWR_CSR |= PWR_CSR_EWUP;
	SCB_SCR |= SCB_SCR_SLEEPDEEP;
	DBGMCU_CR |= DBGMCU_CR_STOP;
	DBGMCU_CR |= DBGMCU_CR_STANDBY;
	DBGMCU_CR |= DBGMCU_CR_SLEEP;
	rtc_lock();
	RCC_BDCR |= (1<<15); //RTC enable
	rcc_periph_clock_enable(RCC_RTC);
	rtc_wait_for_synchro();
	exti_enable_request(EXTI17);
	exti_set_trigger(EXTI17,EXTI_TRIGGER_RISING);
	nvic_enable_irq(NVIC_RTC_IRQ);
	rtc_unlock();
	RTC_CR &= ~RTC_CR_ALRAE;
	while((RTC_ISR & RTC_ISR_ALRAWF)!=RTC_ISR_ALRAWF);
	RTC_ALRMAR = RTC_ALRMXR_MSK4|RTC_ALRMXR_MSK3|RTC_ALRMXR_MSK2|RTC_ALRMXR_MSK1;
	RTC_ALRMASSR |= ((3)<<24);
	//RTC_TAFCR|=((1<<18)|(1<<19));
	RTC_CR = RTC_CR_ALRAIE|RTC_CR_ALRAE;//|RTC_CR_COE|RTC_CR_COSEL;
	RTC_ISR &= ~(RTC_ISR_ALRAF|RTC_ISR_ALRAWF);
	rtc_lock();
	rtc_wait_for_synchro();	
}

static void timer_setup(void)
{
	rcc_periph_clock_enable(RCC_TIM3);
	TIM3_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;
	/* Period */
	TIM3_ARR = 2000;
	/* Prescaler */
	TIM3_PSC = 0;
	TIM3_EGR = TIM_EGR_UG;
	TIM3_CCMR2 |= TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE;
	TIM3_CCER |= TIM_CCER_CC4P | TIM_CCER_CC4E;
	TIM3_CCR4 = 1000;
	TIM3_CR1 |= TIM_CR1_ARPE;
	
}

static void usart_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART1. */
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO14);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO14);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 38400);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

static void gpio_setup(void)
{
	/* Enable GPIOB clock. */
	/* Manually: */
	//RCC_AHBENR |= RCC_AHBENR_GPIOCEN;
	/* Using API functions: */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA); //for display comms
	rcc_periph_clock_enable(RCC_GPIOF); //for debug sync

	gpio_mode_setup(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
	

	/* Set GPIO1 (in GPIO port B) to 'output push-pull'. */
	/* Using API functions: */
	//gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1);
	gpio_set_af(GPIOB, 1, GPIO1);
	
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10);

	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
	gpio_clear(GPIOA, GPIO1|GPIO2|GPIO3|GPIO4);

	//gpio_clear(GPIOA, GPIO6);
	//disp_write(0xe2,1); //reset
	
	gpio_clear(GPIOA, GPIO9);//RST=0;
	unsigned int i,j,k;
    gpio_set(GPIOA, GPIO10);//CS=1;
	gpio_set(GPIOA, GPIO9);//RST=1;
	for (i = 0; i < 1000000; i++)__asm__("nop");
	gpio_clear(GPIOA, GPIO9);//RST=0;
	for (i = 0; i < 100000; i++)__asm__("nop");
	gpio_set(GPIOA, GPIO9);//RST=1;
	for (i = 0; i < 100000; i++)__asm__("nop");
	disp_write(0xe2,1);
	for (i = 0; i < 100000; i++)__asm__("nop");
	
	disp_write(0xae,1);//display off
    disp_write(0xa6,1);//non-reversed
    disp_write(0xa0,1);//adc normal
    disp_write(0x28,1);//operation off
    disp_write(0xa2,1);//1/9 bias
    disp_write(0xee,1);// read-modify-write off
    disp_write(0xc8,1);// set SHL
    disp_write(0x28|0x07,1);//Power_Control(0x07);
    disp_write(0x20|0x05,1);//Regulor_Resistor_Select(0x05);
    disp_write16(0x8115,1);//Set_Contrast_Control_Register(Contrast_level);
	disp_write(0x40|0x00,1);//Initial_Dispay_Line(0x00);
	disp_write(0xaf,1);//DISPLAY_ON();
	for (i = 0; i < 100000; i++)__asm__("nop");
	for(i=0;i<0x08;i++)
	{
		setrow(i);//Set_Page_Address(i);
		setcol(0);
   		for(j=0;j<0x80;j++)
		{
		    disp_write(0,0);
		}
	}
	
	unsigned char font[]={
		0,48,74,74,74,50,124,0,
		0,127,56,68,68,68,56,0,
		0,60,66,66,66,36,0,0,
		0,56,68,68,68,56,127,0
		};
	
	/*
	for(i=0;i<0x04;i++)
	{
		setrow(i);
		setcol((i&1)?0x04:0x00);
   		for(j=0;j<0x10;j++)
		{
		    for(k=0;k<0x04;k++)
		        disp_write(255,0);
		    for(k=0;k<0x04;k++)
		        disp_write(0,0);
		}
	}
	for(i=4;i<0x08;i++)
	{
		setrow(i);
		setcol((i&1)?0x04:0x00);
   		for(j=0;j<0x4;j++)
		{
		    for(k=0;k<0x08;k++)
		        disp_write(font[k+8*j],0);
		}
	}*/
    return;
	
	
}

int main(void)
{
	
	long i;
	unsigned char position=0;
	volatile int16_t i1,i2,i3,i4;
	volatile uint16_t plot,plot2;
	volatile int32_t i1os=0, i1dc=0;
	volatile int32_t i2os=0, i2dc=0;
	volatile int32_t i3os=0, i3dc=0;
	volatile int32_t i4os=0, i4dc=0;
	volatile int32_t Ros=0;
	volatile int32_t prev=0;
	
	rcc_clock_setup_in_hsi_out_8mhz();
	rtc_setup();
	timer_setup();
	gpio_setup();
	adc_setup();
	for (i = 4800000; i; --i) {
			__asm__ volatile("nop");
		}
	/*
	for (i = 4800000; i; --i) {
			__asm__ volatile("nop");
		}
	for (i = 4800000; i; --i) {
			__asm__ volatile("nop");
		}
	for (i = 4800000; i; --i) {
			__asm__ volatile("nop");
		}
	*/
	pwr_disable_power_voltage_detect();
	//usart_setup();	
	rcc_clock_setup_in_hsi_out_48mhz();
	
	while (1) {
		rcc_clock_setup_in_hsi_out_48mhz();
		rtc_wait_for_synchro();
		rcc_periph_clock_enable(RCC_ADC);
		rcc_periph_clock_enable(RCC_GPIOA);
		rcc_periph_clock_enable(RCC_GPIOB);

		adc_power_on(ADC1);
		
		GPIOA_BSRR = GPIO1; //power up detector
		for (i = 400; i ; --i) {   /* Wait a bit. */
			__asm__("nop");
		}
		GPIOA_BSRR = GPIO2|GPIO3; 
		for (i = 150; i; --i) {   
			__asm__("nop");
		}
		gpio_set(GPIOF, GPIO0);
		adc_start_conversion_regular(ADC1);
		while (!(adc_eoc(ADC1)));
		gpio_clear(GPIOF, GPIO0);
		i1= adc_read_regular(ADC1); //yellow 587nm 280mcd
		for (i = 20; i; --i) {   
			__asm__("nop");
		}
		
		GPIOA_BSRR = (GPIO2|GPIO3)<<16;
		for (i = 300; i; --i) {   
			__asm__("nop");
		}
		
		GPIOA_BSRR = GPIO4;
		for (i = 10; i; --i) {   
			__asm__("nop");
		}
		GPIOA_BSRR = (GPIO4)<<16;
		for (i = 3; i; --i) {   
			__asm__("nop");
		}
		gpio_set(GPIOF, GPIO0);
		adc_start_conversion_regular(ADC1);
		while (!(adc_eoc(ADC1)));
		gpio_clear(GPIOF, GPIO0);
		i2= adc_read_regular(ADC1); //orange 601nm 160mcd
		
		for (i = 300; i; --i) {   
			__asm__("nop");
		}
		GPIOA_BSRR = GPIO4|GPIO3;
		for (i = 7; i; --i) {   
			__asm__("nop");
		}
		GPIOA_BSRR = (GPIO4|GPIO3)<<16;
		for (i = 3; i; --i) {   
			__asm__("nop");
		}
		gpio_set(GPIOF, GPIO0);
		adc_start_conversion_regular(ADC1);
		while (!(adc_eoc(ADC1)));
		gpio_clear(GPIOF, GPIO0);
		i3= adc_read_regular(ADC1); //red 640nm 100mcd
		
		for (i = 300; i; --i) {   /* Wait a bit. */
			__asm__("nop");
		}
		GPIOA_BSRR = GPIO2;
		for (i = 6; i; --i) {   /* Wait a bit. */
			__asm__("nop");
		}
		__asm__("nop");
		__asm__("nop");
		__asm__("nop");
		__asm__("nop");
		__asm__("nop");
		__asm__("nop");
		__asm__("nop");
		__asm__("nop");
		__asm__("nop");
		__asm__("nop");
		GPIOA_BSRR = (GPIO2)<<16;
		for (i = 5; i; --i) {   
			__asm__("nop");
		}
		gpio_set(GPIOF, GPIO0);
		adc_start_conversion_regular(ADC1);
		while (!(adc_eoc(ADC1)));
		gpio_clear(GPIOF, GPIO0);
		i4= adc_read_regular(ADC1); //ir
		
		for (i = 300; i; --i) {   /* Wait a bit. */
			__asm__("nop");
		}
		GPIOA_BSRR = GPIO1<<16; //power down detector
		rcc_periph_clock_disable(RCC_ADC);
		adc_power_off(ADC1);
		rcc_clock_setup_in_hsi_out_8mhz();
		/*
		rcc_periph_clock_enable(RCC_USART1);
		usart_enable(USART1);
		
		rcc_clock_setup_in_hsi_out_8mhz();
		usart_send_blocking(USART1, (i1>>8));
		usart_send_blocking(USART1, (i1&0xff));
		usart_send_blocking(USART1, (i2>>8));
		usart_send_blocking(USART1, (i2&0xff));
		usart_send_blocking(USART1, (i3>>8));
		usart_send_blocking(USART1, (i3&0xff));
		usart_send_blocking(USART1, (i4>>8));
		usart_send_blocking(USART1, (i4&0xff));
		usart_send_blocking(USART1, '\n');
		usart_wait_send_ready(USART1);
		//usart_disable(USART1);
		for (i = 400; i; --i) {   // Wait until the last bit is transferred.
			__asm__("nop");
		}
		
		//rcc_periph_clock_disable(RCC_GPIOA);
		rcc_periph_clock_disable(RCC_USART1);
		*/
		/*for (i = 10; i; --i) {
			__asm__ volatile("nop");
		}
		*/
		for (i = 14000; i; --i) {
			__asm__ volatile("nop");
		}
		
	//	__asm__("wfe");
	//	__asm__("wfe");
	
		#define OS_AC 3
		#define OS_DC 5
		#define OS_R 3
		
		i1os-=((i1os)>>OS_AC);
		i1os+=(i1);
		i1dc-=((i1dc)>>OS_DC);
		i1dc+=(i1);
		i2os-=((i2os)>>OS_AC);
		i2os+=(i2);
		i2dc-=((i2dc)>>OS_DC);
		i2dc+=(i2);
		i3os-=((i3os)>>OS_AC);
		i3os+=(i3);
		i3dc-=((i3dc)>>OS_DC);
		i3dc+=(i3);
		i4os-=((i4os)>>OS_AC);
		i4os+=(i4);
		i4dc-=((i4dc)>>OS_DC);
		i4dc+=(i4);
		volatile int32_t rac=((int32_t)(i3os>>OS_AC)-(int32_t)(i3dc>>OS_DC));// /(int32_t)(i3dc>>OS_DC);  //division not substraction
		volatile int32_t irac=((int32_t)(i4os>>OS_AC)-(int32_t)(i4dc>>OS_DC));// /(int32_t)(i4dc>>OS_DC);
		Ros-=(Ros>>OS_R);
		Ros+=10*rac/irac;
		
		//if(irac==0)irac=1;
		//i1bl+=1;
		//if(i1bl==4096)i1bl=0;
		plot2=0;
		if((i1os>>OS_AC)>(i1dc>>OS_DC)|(i2os>>OS_AC)>(i2dc>>OS_DC)|(i3os>>OS_AC)>(i3dc>>OS_DC)|(i4os>>OS_AC)>(i4dc>>OS_DC)){
			//plot=100*rac/irac;
			plot2=100;
			TIM3_CR1 |= TIM_CR1_CEN;
		
		}else{
			TIM3_CR1 &= ~TIM_CR1_CEN;
			rcc_periph_clock_disable(RCC_GPIOB);
		
			//plot=0;
		}
		plot=0;
		if(irac>rac){
			plot=100*(i4dc>>OS_DC)/(i3dc>>OS_DC);
		}
		//plot=100*(i4os>>OS_AC)/(i3os>>OS_AC);
		//(i4dc>>OS_DC);//Ros>>OS_R;
		prev=(i3os>>OS_AC)-(i3dc>>OS_DC);
		//if(plot<0)plot=0;
		//plot=((i1&0xff8)-(i1bl>>3))<<3;
		
		
		for(i=0;i<0x08;i++){
		setrow(i);
		setcol(position>>1);
		disp_write(0,0);
		disp_write(255,0);
		}
		unsigned char oldrow=7-((plot>>5)&0x07);
		unsigned char newrow=7-((plot2>>5)&0x07);
		setrow(oldrow);
		setcol(position>>1);
		if(newrow==oldrow){
			disp_write((1<<(7-((plot>>2)&0x7)))|(1<<(7-((plot2>>2)&0x7))),0);
		}else{
			disp_write(1<<(7-((plot>>2)&0x7)),0);
			setrow(7-((plot2>>5)&0x07));
			setcol(position>>1);
			disp_write(1<<(7-((plot2>>2)&0x7)),0);
		}
		if(position==0xff) position=0;
		else ++position;
		rcc_periph_clock_disable(RCC_GPIOA);
		
		
	}

}
