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

uint32_t min_integ_micros; //Minimal Integration Time
uint32_t integ_time_us = 100; //Integration Time


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
 * Clocking over a time
 *
*/
void pulse_clock_timed(uint32_t duration_micros){
  static uint32_t start_time;

  /* capture initial time stamp */
  start_time = k_cycle_get_32();

  /* do work for some (short) period of time */
  while ((duration_micros*1000) < SYS_CLOCK_HW_CYCLES_TO_NS(k_cycle_get_32() - start_time)){
    gpio_pin_set(porta, SPEC_CLK,(int)true);
    k_busy_wait(delayTime);
    gpio_pin_set(porta, SPEC_CLK,(int)false);
    k_busy_wait(delayTime);
  }
}

void measure_min_integ_micros() {
  static uint32_t start_time;
  /* capture initial time stamp */
  start_time = k_cycle_get_32();
  //48 clock cycles are required after ST goes low  
  pulse_clock(48);
  min_integ_micros = SYS_CLOCK_HW_CYCLES_TO_NS(k_cycle_get_32() - start_time)*1000;
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

  uint32_t delayTime = 1; // delay time

  // Start clock cycle and set start pulse to signal start
  gpio_pin_set(porta, SPEC_CLK,(int)false);
  k_busy_wait(delayTime);
  gpio_pin_set(porta, SPEC_CLK,(int)true);
  k_busy_wait(delayTime);
  gpio_pin_set(porta, SPEC_CLK,(int)false);
  gpio_pin_set(porta, SPEC_ST,(int)true);
  k_busy_wait(delayTime);

  //Sample for a period of time
  for(int i = 0; i < 15; i++){

      gpio_pin_set(porta, SPEC_CLK,(int)true);
      k_busy_wait(delayTime);
      gpio_pin_set(porta, SPEC_CLK,(int)false);
      k_busy_wait(delayTime); 
 
  }

  //Set SPEC_ST to low
  gpio_pin_set(porta, SPEC_ST,(int)false);

  //Sample for a period of time
  for(int i = 0; i < 85; i++){

      gpio_pin_set(porta, SPEC_CLK,(int)true);
      k_busy_wait(delayTime);
      gpio_pin_set(porta, SPEC_CLK,(int)false);
      k_busy_wait(delayTime); 
      
  }



  //One more clock pulse before the actual read
  gpio_pin_set(porta, SPEC_CLK,(int)true);
  k_busy_wait(delayTime);
  gpio_pin_set(porta, SPEC_CLK,(int)false);
  k_busy_wait(delayTime);

  //Read from SPEC_VIDEO
  for(int i = 0; i < SPEC_CHANNELS; i++){

      data[i] = (uint16_t)readOneChannel(SPEC_VIDEO);
      
      gpio_pin_set(porta, SPEC_CLK,(int)true);
      k_busy_wait(delayTime);
      gpio_pin_set(porta, SPEC_CLK,(int)false);
      k_busy_wait(delayTime);
        
  }

  //Set SPEC_ST to high
  gpio_pin_set(portb, SPEC_ST,(int)true);

  //Sample for a small amount of time
  for(int i = 0; i < 7; i++){
    
      gpio_pin_set(porta, SPEC_CLK,(int)true);
      k_busy_wait(delayTime);
      gpio_pin_set(porta, SPEC_CLK,(int)false);
      k_busy_wait(delayTime);
    
  }

  gpio_pin_set(porta, SPEC_CLK,(int)true);
  k_busy_wait(delayTime);
  
}



/*
 * The function below prints out data to the terminal or 
 * processing plot
 */
void printData(){
  
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
  printk("Init Application...\n"); 
  setup();

  // Start Application
  printk("Starting Application...\n"); 
  while(1){
    loop();
  }  
}


