#include <zephyr.h>
#include <device.h>

//For GPIO
#include <drivers/gpio.h>

//For ADC
#include <szl_adc.h>

// For USB CDC AACM 
#include <usb/usb_device.h>
#include <drivers/uart.h>
BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

//For stdlib
#include <stdbool.h>





//#define SPEC_TRG         A0
#define SPEC_ST          15 //PortA D5
#define SPEC_CLK         18 //PortA D10
#define SPEC_VIDEO       0 //Analog Channel 0

const struct device* porta;
const struct device* portb;

#define SPEC_CHANNELS    288 // New Spec Channel
uint16_t data[SPEC_CHANNELS];

uint32_t delayTime = 1; // delay time in us for CLK

uint32_t min_integ_micros; //Minimal Integration Time
uint32_t integ_time_us = 100; //Integration Time

uint32_t timings[10];




/*
 * Function for one clock Cycle
 *
*/
void pulse_clock(int cycles){
  for(int i = 0; i < cycles; i++){
    gpio_pin_set(porta, SPEC_CLK,(int)true);
    k_busy_wait(delayTime);
    gpio_pin_set(porta, SPEC_CLK,(int)false);
    k_busy_wait(delayTime);
  }
}

/*
 * Get passed time in ns (1 Rollover is OK!) -> 2³² / F_CPU = ~ 90s.
 *
*/
uint32_t measure_us_one_rollover(uint32_t start_cycles, uint32_t stop_cycles){
  if(start_cycles >= stop_cycles){
    return k_ticks_to_us_floor32(1) + k_cyc_to_us_floor32(stop_cycles);
  } else {
    return k_cyc_to_us_floor32(stop_cycles - start_cycles);
  }
}



/*
 * Clocking over a time
 *
*/
void pulse_clock_timed(uint32_t duration_micros){
  static uint32_t start_cycles;

  /* capture initial time stamp */
  start_cycles = k_cycle_get_32();

  /* do work for some (short) period of time */
  while ((duration_micros) > k_cyc_to_us_floor32(k_cycle_get_32()-start_cycles)){
    gpio_pin_set(porta, SPEC_CLK,(int)true);
    k_busy_wait(delayTime);
    gpio_pin_set(porta, SPEC_CLK,(int)false);
    k_busy_wait(delayTime);
  }
}

void measure_min_integ_micros() {
  static uint32_t start_cycles, stop_cycles;

  /* capture initial time stamp */
  start_cycles = k_cycle_get_32();

  //48 clock cycles are required after ST goes low  
  pulse_clock(48);
  stop_cycles = k_cycle_get_32();
    

  min_integ_micros = measure_us_one_rollover(start_cycles,stop_cycles);
  printk("Min. Integ. Time: %uus\n",min_integ_micros);
}

void set_integration_time(uint32_t microseconds) {
  integ_time_us = microseconds;
}

void setup(){

  porta = device_get_binding("PORTA");
  portb = device_get_binding("PORTB");

  //Set desired pins to OUTPUT
  gpio_pin_configure(porta, SPEC_CLK, GPIO_OUTPUT); 
  gpio_pin_configure(porta, SPEC_ST, GPIO_OUTPUT); 

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

  //compute integration time
  static uint32_t duration_micros;
  duration_micros = integ_time_us;
  if(integ_time_us >= min_integ_micros){
    duration_micros -= min_integ_micros; //correction based on 48 pulses after ST goes low
  } else {
    duration_micros = 0;
  } 

  // Start clock cycle and set start pulse to signal start
  gpio_pin_set(porta, SPEC_CLK,(int)true);
  k_busy_wait(delayTime);
  gpio_pin_set(porta, SPEC_CLK,(int)false);
  gpio_pin_set(porta, SPEC_ST,(int)true);
  k_busy_wait(delayTime);

  //pixel integration starts after three clock pulses
  pulse_clock(3);
  timings[0] = k_cycle_get_32();
  //Integrate pixels for a while
  pulse_clock_timed(duration_micros);
  //Set _ST_pin to low
  gpio_pin_set(porta, SPEC_ST,(int)false);
  timings[1] = k_cycle_get_32();
  //Sample for a period of time
  //integration stops at pulse 48 th pulse after ST went low
  pulse_clock(48);
  timings[2] = k_cycle_get_32();
  //pixel output is ready after last pulse #88 after ST wen low
  pulse_clock(40);
  timings[3] = k_cycle_get_32();
  //Read from SPEC_VIDEO
  for(int i = 0; i < SPEC_CHANNELS; i++){
    data[i] = readOneChannel(SPEC_VIDEO);
    pulse_clock(1);
  }
  timings[4] = k_cycle_get_32(); 
}



/*
 * The function below prints out data to the terminal or 
 * processing plot
 */
void printData(){

  printk("<data>,");
  
  for (int i = 0; i < SPEC_CHANNELS; i++){
    
    printk("%u,",data[i]);
    
  }
  
  printk("\n");
}

void loop(){
   
  readSpectrometer();
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


