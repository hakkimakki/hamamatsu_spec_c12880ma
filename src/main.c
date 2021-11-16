#include <zephyr.h>
#include <device.h>

//For GPIO
#include <drivers/gpio.h>

//For ADC
#include <szl_adc.h>
#include <drivers/adc.h>

// For USB CDC AACM 
#include <usb/usb_device.h>
#include <drivers/uart.h>
BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

//For stdlib
#include <stdbool.h>

//For Counter
#include <drivers/counter.h>

#define TIMER DT_LABEL(DT_NODELABEL(tc4))



//#define SPEC_TRG         A0
#define SPEC_ST          15 //PortA D5
#define SPEC_CLK         18 //PortA D10
#define SPEC_VIDEO       2 //Analog Channel PIN A1
#define PHOTO_DIODE      0 //Analog Channel PIN A0

const struct device* porta;
const struct device* portb;
const struct device* counter;

#define SPEC_CHANNELS    288 // New Spec Channel
int16_t data[SPEC_CHANNELS];

uint32_t delayTime = 1; // delay time in us for CLK

uint32_t min_integ_micros; //Minimal Integration Time
uint32_t integ_time_us = 0; //Integration Time
uint32_t duration_micros;

uint32_t intg_start;
uint32_t intg_stop;



void stopresetStartCounter(){
  counter_stop(counter);
  static const struct counter_top_cfg cfg = {.ticks = 0xFFFFFFFF};
  counter_set_top_value(counter,&cfg);
  counter_start(counter);
}

uint32_t readCounter_us(){
  static uint32_t tcks;
  counter_get_value(counter,&tcks);
  return counter_ticks_to_us(counter,tcks);
}



/*
 * Function for precise delay with cycle counter polling.
*/
void prec_delay(uint32_t micros) {
  static uint32_t start;
  start = readCounter_us();
  while (micros > (readCounter_us() - start))
  {
    ;
  }  
}




/*
 * Function for one clock Cycle
 *
*/
void pulse_clock(int cycles){
  static int i;
  for(i = 0; i < cycles; i++){
    gpio_pin_set(porta, SPEC_CLK,(int)true);
    prec_delay(delayTime);
    gpio_pin_set(porta, SPEC_CLK,(int)false);
    prec_delay(delayTime);
  }
}

/*
 * Clocking over a time
 *
*/
void pulse_clock_timed(uint32_t duration_micros){
  static uint32_t start;

  /* capture initial time stamp */
  start = readCounter_us();

  /* do work for some (short) period of time */
  while ((duration_micros) > (readCounter_us()-start)){
    gpio_pin_set(porta, SPEC_CLK,(int)true);
    prec_delay(delayTime);
    gpio_pin_set(porta, SPEC_CLK,(int)false);
    prec_delay(delayTime);
  }
}

void measure_min_integ_micros() {

  stopresetStartCounter();
  static uint32_t start, stop;

  /* capture initial time stamp */
  start = readCounter_us();
  //48 clock cycles are required after ST goes low  
  pulse_clock(48);
  stop = readCounter_us();  

  min_integ_micros = stop-start;
}

void set_integration_time(uint32_t microseconds) {
  integ_time_us = microseconds;
  duration_micros = microseconds;
}

void setup(){

  porta = device_get_binding("PORTA");
  portb = device_get_binding("PORTB");
  counter = device_get_binding("TIMER_4");  

  //Set desired pins to OUTPUT
  gpio_pin_configure(porta, SPEC_CLK, GPIO_OUTPUT); 
  gpio_pin_configure(porta, SPEC_ST, GPIO_OUTPUT); 
  gpio_pin_configure(porta, 2, GPIO_INPUT);

  gpio_pin_set(porta, SPEC_CLK,(int)true); // Set SPEC_CLK High
  gpio_pin_set(porta, SPEC_CLK,(int)false); // Set SPEC_CLK Low
  
  //Initalize Microseconds
  measure_min_integ_micros();
}


/*
 * This functions reads spectrometer data from SPEC_VIDEO
 * Look at the Timing Chart in the Datasheet for more info
*/
void readSpectrometer(){
  // Start clock cycle and set start pulse to signal start
  gpio_pin_set(porta, SPEC_CLK,(int)true);
  prec_delay(delayTime);
  gpio_pin_set(porta, SPEC_CLK,(int)false);
  gpio_pin_set(porta, SPEC_ST,(int)true);
  prec_delay(delayTime); 

  //pixel integration starts after three clock pulses
  pulse_clock(3);
  //measure effective integration time 
  intg_start = readCounter_us();
  //Integrate pixels for a while
  pulse_clock_timed(duration_micros); 
  //Set _ST_pin to low
  gpio_pin_set(porta, SPEC_ST,(int)false);
  //integration stops at pulse 48 th pulse after ST went low
  pulse_clock(48);
  //stop measure eff intg time
  intg_stop = readCounter_us();
  //pixel output is ready after last pulse #88 after ST wen low
  pulse_clock(40);
  //Read out the first sample to throw it away (high impedance input)
  data[0] = szl_adc_readOneChannel(SPEC_VIDEO);
  //Read from SPEC_VIDEO
  for(int i = 0; i < SPEC_CHANNELS; i++){
    //data[i] = szl_adc_raw_to_millivolts(szl_adc_readOneChannel(SPEC_VIDEO));
    data[i] = (szl_adc_readOneChannel(SPEC_VIDEO));
    pulse_clock(1);
  }
}


/*
 * The function below prints out data to the terminal or 
 * processing plot
 */
void printData(){

  printk("<data>,");  
  for (int i = 0; i < SPEC_CHANNELS; i++){
    
    printk("%d,",data[i]);
    
  }
  printk("\n");

  printk("<minIntgTime_us>%u\n",min_integ_micros);
  printk("<sollIntgTime_us>%u\n",duration_micros);
  printk("<istIntgTime_us>%u\n",intg_stop - intg_start);

  static int32_t raw_value;
  raw_value = szl_adc_readOneChannel(PHOTO_DIODE);
  printk("<ADC_raw>%u\n",raw_value);
  printk("<ADC_mV>%d\n",szl_adc_raw_to_millivolts(raw_value)); 
  printf("<ADC_uA>%f\n",((float)szl_adc_raw_to_millivolts(raw_value)/(float)1000/(float)100000)*(float)1e6);
  printf("<ADC_W/m2 @ 530nm>%f\n",((float)szl_adc_raw_to_millivolts(raw_value)/(float)1000/(float)100000)*0.38/(2.16e-6));   
}

void loop(){
  
  stopresetStartCounter();
  set_integration_time(0);
  measure_min_integ_micros();
  k_sched_lock();
  readSpectrometer();
  k_sched_unlock();
  printData();
  k_sleep(K_SECONDS(1));    
}



/**@brief Function for application main entry.
 */
void main(void) {

  // Init Board
  printk("Init Board...\n"); 
	if (usb_enable(NULL)) {
		return;
	}

  //Init Application
  k_sleep(K_MSEC(5000));
  printk("Init Application...\n"); 
  setup();

  // Start Application
  printk("Starting Application...\n"); 
  while(1){
    loop();
  }  
}


