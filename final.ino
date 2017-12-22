/*
 * Protocol: 
 * Send character M to access the Menu of the generator
 * Then send freq in Hz 
 * And then send the Qmpl in Volts
 * A three signal sine will be generated, at any time if M is sent again by serial, then the process will restart
 */
 
/* 
 * Log 20/12/17 
 * V 1.21
 *      Add a reset command in the loop() by typping "reset at any time"
 *      entry_data_float was a integer, needed to be a float
 *      if the freq is wrong the next time the parameters are introduced demands the amplitude twice
 * V 1.2
 *      Improve serial com with carriage return and prevent first unnacurate serial data comming from the analyzer
 * Log 20/12/17 
 * V 1.1
 *      Added a more complete communication protocol in get_sin_parameters()
 * Log 15/12/17
 * V 1.0
 *      Changed analyse_samples value from T = 20 to T = 50 for more accuracy
 *      Changed v3_pointer from 173 to 153 since (231*2)/3 = 155

*/

float entry_data[] = {0, 0};

int top = 230;                 // Top value in single slope pwm generation
int samples_needed = 231;      // Size of the generation buffer
int T = 50;                     // Number of periods to capture for frequency determination
int analyse_samples = (40 * T);    // Size of analyse buffer of ADC

int *generated_samples;         // Buffer that holds calculated pwm values for pwm generation

int *measured_samples_1;        // Buffers that holds adc values for the 3 sinusoids
int *measured_samples_2;
int *measured_samples_3;

int v1_in = 0;                  // helper pointers
int v2_in = 0;
int v3_in = 0;

const int v1_out = 2;           // Digital pins for three phase generation
const int v2_out = 5;
const int v3_out = 7;

int v1_pointer = 0;             // pointers that provide 120degrees phase difference
int v2_pointer = 76;
int v3_pointer = 153;

int pin = 0;                    // helper pointer

int pw = (int)(0.57 * top);     // Pwm dutycycle that generates 1,7V output for reference value

bool flag_reset_read_buffers = false;

bool buffer_1_full = false;
bool buffer_2_full = false;
bool buffer_3_full = false;

int v1_maxvalue = 0;
int v1_minvalue = 4096;                     // 2^12 (12 bit resolution)
float v1_high_level = 0;
float v1_low_level = 0;
float v1_amplitude_pk_pk = 0;
int v1_offset = 0;
float v1_freq = 0;

int v2_maxvalue = 0;
int v2_minvalue = 4096;                     // 2^12 (12 bit resolution)
float v2_high_level = 0;
float v2_low_level = 0;
float v2_amplitude_pk_pk = 0;
int v2_offset = 0;
float v2_freq = 0;

int v3_maxvalue = 0;
int v3_minvalue = 4096;                     // 2^12 (12 bit resolution)
float v3_high_level = 0;
float v3_low_level = 0;
float v3_amplitude_pk_pk = 0;
int v3_offset = 0;
float v3_freq = 0;

float amplitude = 0.0;
float frequency = 50.0;
bool startup = true;
bool flag_correct_values = true;

int serial_print = 0;
bool reset = false;

bool stop_generation = false;
int hulp_count = 0;

/*
    Debug function
*/
void print_sample_table()
{
  Serial.begin(9600);
  for (int i = 0; i < samples_needed; i++) {
    Serial.print("Sample ");
    Serial.print(i);
    Serial.print(" value ");
    Serial.println(generated_samples[i]);
  }
  Serial.end();
}

/*
    Debug function
*/
void print_buffer()
{
  Serial.println("pin 1");
  for (int i = 0; i < analyse_samples; i++) {
    Serial.println(measured_samples_1[i]);
  }

  Serial.println("");
  Serial.println("pin 2");
  for (int i = 0; i < analyse_samples; i++) {
    Serial.println(measured_samples_2[i]);
  }

  Serial.println("");
  Serial.println("pin 3");
  for (int i = 0; i < analyse_samples; i++) {
    Serial.println(measured_samples_3[i]);
  }
}

void setup() {

  /*
     Generation of the three phase generator
  */

  pinMode(v1_out, OUTPUT);
  pinMode(v2_out, OUTPUT);
  pinMode(v3_out, OUTPUT);

  // Allocate memory for generated samples
  generated_samples = (int*)malloc(samples_needed * sizeof(int));
  
  // Get frequency and amplitude from PI
  
  Serial.begin(9600);    //Start COM
  
  get_sin_parameters();

  // Calculate pwm values for corresponding sinus values
  calculate_sinusoid();

  // Print table on serial monitor for check
  //print_sample_table();

  // Make a generic clock
  setup_generic_clock_tc();

  // Enable port multiplexer for digital pins
  config_port_settings();

  // Configure TC to generate pwm signal
  setup_tc3();
  setup_tcc0();
  setup_tcc1();

  // Start TC
  tc_start();

  /*
      Three phase analyzer
  */

  measured_samples_1 = (int *) malloc(analyse_samples * sizeof(int));     //Allocate the buffer where the first sine samples are measured
  measured_samples_2 = (int *) malloc(analyse_samples * sizeof(int));      //Allocate the buffer where the first sine samples are measured
  measured_samples_3 = (int *) malloc(analyse_samples * sizeof(int));      //Allocate the buffer where the first sine samples are measured

  // Empty the previous read buffers and initialization parameters
  reset_read_buffers();

  // Setup ADC
  // Setup generic clock
  setup_generic_clock_adc();

  // config ports adc
  config_ports_adc();

  // setup adc
  setup_adc();

  // start adc
  adc_start();

}

void loop()
{
  
  if (serial_print == 15) {
    get_amplitude();
    get_frequency();

    Serial.print("[1]low level v1 =  ");
    Serial.print(v1_low_level, 2);
    Serial.print("      high level v1 =  ");
    Serial.print(v1_high_level, 2);
    Serial.print("      amplitude pk pk v1 = ");
    Serial.print(v1_amplitude_pk_pk, 2);
    Serial.print("      frequency v1 = ");
    Serial.println(v1_freq, 2);

    Serial.print("[2]low level v2 =  ");
    Serial.print(v2_low_level);
    Serial.print("      high level v2 =  ");
    Serial.print(v2_high_level, 2);
    Serial.print("      amplitude pk pk v2 = ");
    Serial.print(v2_amplitude_pk_pk, 2);
    Serial.print("      frequency v2 = ");
    Serial.println(v2_freq, 2);

    Serial.print("[3]low level v3 =  ");
    Serial.print(v3_low_level);
    Serial.print("      high level v3 =  ");
    Serial.print(v3_high_level, 2);
    Serial.print("      amplitude pk pk v3 = ");
    Serial.print(v3_amplitude_pk_pk, 2);
    Serial.print("      frequency v3 = ");
    Serial.println(v3_freq, 2);

    //print_buffer();
    Serial.println("");

    reset_read_buffers();
  }

  if (reset) {
    reset = false;
    //NVIC_SystemReset();      // processor software reset
    
    delay(20);
    setup();
  }

  while (Serial.available()) {
    // get the new byte:
    int in_char = Serial.read();
    
    if (in_char == byte(0x4D)) {            //4D is hex for M
      // Reset of the pwm generation
      Serial.println("Reset settings");
      reset = true;
    } else if (in_char == byte(0xD)) {
        // Ignore this byte
        // Equals to carriage return
    } else {
        Serial.println("COM port expects only M");
    }
  }

  if(stop_generation) {
    Serial.println("Generation of PWM is stopped");
    tcReset();
    TCC0->CTRLA.reg |= TCC_CTRLA_SWRST; //reset
    while (TCC0->SYNCBUSY.bit.SWRST == 1);
    stop_generation = false;
  }

  serial_print++;
  delay(100);
}

void get_sin_parameters()
{  
  if (startup == true) {
    entry_data[0] = frequency;
    entry_data[1] = amplitude;

    startup = false;
  } else {
    Serial.println("Enter FREQ in Hz");
    entry_data[0] = read_parameter();
    frequency = entry_data[0];
    if ((frequency < 40 || frequency > 60) && frequency != 0) {
        Serial.println("Wait for M");
        flag_correct_values = false;
        frequency = 50;
    } else {
        flag_correct_values = true;
    }

    Serial.println("Enter Amplitude in Volts (max amplitude = 3.3V)");
    entry_data[1] = read_parameter();
    amplitude = entry_data[1];
    if (amplitude < 0 || amplitude > 3.3) {
        amplitude = 0.0;
        flag_correct_values = false;
        Serial.println("Wait for M");
    } else {
        flag_correct_values = true;
    }

    if (flag_correct_values) {
        Serial.println("Settings OK");
    }

    if (frequency == 0 && amplitude == 0) {
        stop_generation = true;
    }
  }
}

float read_parameter() { //Wait for serial data and reads it in. This function reads in the parameters that are entered into the serial terminal
  float f;
  String number = "";
  char marker = '\r';

  bool new_data = false;
  while (!Serial.available());
  while (Serial.available() > 0 || new_data == false) {
    char inChar = Serial.read();
    if (inChar != byte(0) && inChar != byte(255)) {
        if (inChar != marker) { 
            number += (char)inChar;
        } else {
            number += '\0';
            f = number.toFloat();
            Serial.println(f, 2);
            new_data = true;
        }
    }

    hulp_count++;

    if(hulp_count == 10000) {
        Serial.println("Timeout");
        hulp_count = 0;
        return -1;
    }
  }

  hulp_count = 0;
  return f;     //get int that was entered on Serial monitor
}

void calculate_sinusoid()
{
  int i;
  const float pi = 3.141592;
  int cmp_value;
  float percentage;
  float radian;
  float amplitude_reference = samples_needed / 3.3;

  for (i = 0; i < samples_needed; i++) {
    radian = (i * (360 / (float)samples_needed)) * (pi / 180);
    cmp_value = (int)(sin(radian) * ((amplitude_reference * amplitude) / 2) + ((amplitude_reference * amplitude) / 2));
    generated_samples[i] = cmp_value;
  }
}

void setup_generic_clock_tc()
{
  int division_factor = 9;
  int generic_clock = 4;

  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(division_factor) |          // Divide the main clock down by some factor to get generic clock
                    GCLK_GENDIV_ID(generic_clock);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(generic_clock);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
}

void setup_generic_clock_adc()
{
  int division_factor = 9;
  int generic_clock = 6;

  ADC->CTRLA.reg = 1;                 // Reset ADC

  REG_PM_APBCMASK |= PM_APBCMASK_ADC;

  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(division_factor) |          // Divide the main clock down by some factor to get generic clock
                    GCLK_GENDIV_ID(generic_clock);            // Select Generic Clock (GCLK) 6
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_GENEN |
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(generic_clock);          // Select GCLK6
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
}

void config_port_settings()
{
  // Enable pins for pwm generation
  PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[5].ulPort].PINCFG[g_APinDescription[5].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;

  //Multiplex pwm pins and select the TC or TCC to use
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXE_E | PORT_PMUX_PMUXO_E;
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;
  PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;
}

void config_ports_adc()
{
  // Input pin for ADC Arduino
  REG_PORT_DIRCLR1 = PORT_PB08;
  REG_PORT_DIRCLR1 = PORT_PB09;
  REG_PORT_DIRCLR1 = PORT_PA04;

  // Enable multiplexing
  PORT->Group[g_APinDescription[A1].ulPort].PINCFG[g_APinDescription[A1].ulPort].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[A2].ulPort].PINCFG[g_APinDescription[A2].ulPort].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[A3].ulPort].PINCFG[g_APinDescription[A3].ulPort].bit.PMUXEN = 1;

  PORT->Group[g_APinDescription[A1].ulPort].PMUX[g_APinDescription[A1].ulPin >> 1].reg = PORT_PMUX_PMUXE_B | PORT_PMUX_PMUXO_B;
  PORT->Group[g_APinDescription[A3].ulPort].PMUX[g_APinDescription[A3].ulPin >> 1].reg = PORT_PMUX_PMUXE_B;
}

void setup_tc3()
{
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN_GCLK4 |
                      GCLK_CLKCTRL_ID_TCC2_TC3 ;              //bind generic clock 4 to TCC2 and TC3
  while (GCLK->STATUS.bit.SYNCBUSY);

  Tc* TC = (Tc*) TC3;
  tcReset(); //reset TC3

  TC->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8;
  TC->COUNT8.CTRLA.reg |= TC_CTRLA_WAVEGEN_NPWM;
  TC->COUNT8.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2;

  TC->COUNT8.PER.reg = top;                                   // top value of single slope pwm generation
  TC->COUNT8.CC[0].reg = generated_samples[v1_pointer];       // value where dutycycle goes low on channel 0
  TC->COUNT8.CC[1].reg = generated_samples[v2_pointer];       // value where dutycycle goes low on channel 1
  while (tcIsSyncing());

  // Configure interrupt request
  set_priority_tc();

  TC->COUNT8.INTENSET.bit.OVF = 1;                        // Enables interrupt on overflow
  while (tcIsSyncing()); //wait until TC5 is done syncing
}

void setup_tcc0()
{
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN_GCLK4 |
                      GCLK_CLKCTRL_ID_TCC0_TCC1 ;         // bind generic clock 4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);

  Tcc* TC = (Tcc*) TCC0;
  TC->CTRLA.reg &= ~TCC_CTRLA_ENABLE;             //Disable TCC0
  while (TC->SYNCBUSY.bit.ENABLE == 1);

  TC->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;          // Use single slope PWM generation
  while (TC->SYNCBUSY.bit.WAVE == 1);

  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2;       // Prescaler of 2

  TC->PER.reg = top;
  while (TC->SYNCBUSY.bit.PER == 1);

  TC->CC[3].reg = generated_samples[v3_pointer];      // value where dutycycle goes down
  while (TC->SYNCBUSY.bit.CC3 == 1);

  // Configure interrupt request
  set_priority_tcc();

  TC->INTENSET.bit.OVF = 1;
}

void setup_tcc1()
{
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN_GCLK4 |
                      GCLK_CLKCTRL_ID_TCC0_TCC1 ;     // bind generic clock 4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);

  Tcc* TC = (Tcc*) TCC1;
  TC->CTRLA.reg &= ~TCC_CTRLA_ENABLE;             //Disable TC
  while (TC->SYNCBUSY.bit.ENABLE == 1);

  TC->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;          // Use single slope PWM generation
  while (TC->SYNCBUSY.bit.WAVE == 1);

  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2;

  TC->PER.reg = top;
  while (TC->SYNCBUSY.bit.PER == 1);

  TC->CC[1].reg = pw;
  while (TC->SYNCBUSY.bit.CC1 == 1);

  set_priority_tcc();
}

void setup_adc()
{
  GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN |
                       GCLK_CLKCTRL_GEN_GCLK6 |
                       GCLK_CLKCTRL_ID_ADC;           // bind generic clock 6 to ADC
  while (GCLK->STATUS.bit.SYNCBUSY);

  ADC->REFCTRL.reg |= ADC_REFCTRL_REFSEL_INTVCC1;

  ADC->AVGCTRL.reg |= ADC_AVGCTRL_ADJRES(0) |
                      ADC_AVGCTRL_SAMPLENUM_1;

  REG_ADC_SAMPCTRL |= ADC_SAMPCTRL_SAMPLEN(0);

  ADC->INPUTCTRL.bit.INPUTSCAN = 2;                   // Enable pinscan -> toggles between the three analog pins to do an adc conversion
  ADC->INPUTCTRL.bit.INPUTOFFSET = 0;

  ADC->INPUTCTRL.reg |= ADC_INPUTCTRL_GAIN_DIV2 |
                        ADC_INPUTCTRL_MUXNEG_GND |
                        ADC_INPUTCTRL_MUXPOS_PIN2;     // start pin of the adc inputscan, pin 2 = A1 (AIN[2])
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);

  // CLK_ADC settings,
  ADC->CTRLB.reg |= ADC_CTRLB_PRESCALER_DIV128 |   // prescaler of 128
                    ADC_CTRLB_RESSEL_12BIT |      // bit resolution (8, 10 or 12 bit resolution)
                    ADC_CTRLB_FREERUN;            // Run ADC continously, 6/7 ADC_CLKs needed for 1 conversion -> 3 conversions for each timer click
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);

  set_priority_adc();

  ADC->INTENSET.reg |= ADC_INTENSET_RESRDY; // enable ADC result ready interrupt
  while (ADC->STATUS.bit.SYNCBUSY);
}

void set_priority_tc()
{
  NVIC_SetPriority(TC3_IRQn, 0);
  NVIC_EnableIRQ(TC3_IRQn);
}

void set_priority_tcc()
{
  NVIC_SetPriority(TCC0_IRQn, 0);
  NVIC_EnableIRQ(TCC0_IRQn);
  NVIC_SetPriority(TCC1_IRQn, 0);
  NVIC_EnableIRQ(TCC1_IRQn);
}

void set_priority_adc()
{
  NVIC_SetPriority(ADC_IRQn, 0); //set priority of the interrupt
  NVIC_EnableIRQ(ADC_IRQn); // enable ADC interrupts
}

bool tcIsSyncing()
{
  return TC3->COUNT8.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void tc_start()
{
  TC3->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd

  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE; //set the CTRLA register
  while (TCC0->SYNCBUSY.bit.ENABLE == 1);

  TCC1->CTRLA.reg |= TCC_CTRLA_ENABLE; //set the CTRLA register
  while (TCC1->SYNCBUSY.bit.ENABLE == 1);
}

void adc_start()
{
  ADC->CTRLA.reg = 2;
}

void tcReset()
{
  TC3->COUNT8.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC3->COUNT8.CTRLA.bit.SWRST);
}

void reset_read_buffers()
{
  int i = 0;

  if (flag_reset_read_buffers == false) {
    for (i = 0; i < analyse_samples; i++) {
      measured_samples_1[i] = 0;
      measured_samples_2[i] = 0;
      measured_samples_3[i] = 0;
    }
  }

  if (flag_reset_read_buffers > true) {
    i = 0;
  }

  flag_reset_read_buffers = true;
  v1_amplitude_pk_pk = 0;
  v1_maxvalue = 0;
  v1_minvalue = 4096;

  v2_amplitude_pk_pk = 0;
  v2_maxvalue = 0;
  v2_minvalue = 4096;

  v3_amplitude_pk_pk = 0;
  v3_maxvalue = 0;
  v3_minvalue = 4096;

  v1_in = 0;
  v2_in = 0;
  v3_in = 0;

  buffer_1_full = false;
  buffer_2_full = false;
  buffer_3_full = false;

  serial_print = 0;
}

void get_amplitude() {
  int i = 0;

  if (buffer_1_full == true) {       //if the buffer is filled
    for (i = 0; i < analyse_samples; i++) {
      if (measured_samples_1[i] < 4096 && measured_samples_1[i] > 0) {
        if (v1_maxvalue < measured_samples_1[i]) {
          v1_maxvalue = measured_samples_1[i];
        }

        if (v1_minvalue > measured_samples_1[i]) {
          v1_minvalue = measured_samples_1[i];
        }
      }

      v1_low_level = v1_minvalue * (3.3 / 4096);
      v1_high_level = v1_maxvalue * (3.3 / 4096);

      v1_offset = v1_maxvalue - v1_minvalue;
      v1_amplitude_pk_pk = v1_high_level - v1_low_level;
    }
  }

  if (buffer_2_full == true) {       //if the buffer is filled
    for (i = 0; i < analyse_samples; i++) {
      if (measured_samples_2[i] < 4096 && measured_samples_2[i] > 0) {
        if (v2_maxvalue < measured_samples_2[i]) {
          v2_maxvalue = measured_samples_2[i];
        }

        if (v2_minvalue > measured_samples_2[i]) {
          v2_minvalue = measured_samples_2[i];
        }
      }

      v2_low_level = v2_minvalue * (3.3 / 4096);
      v2_high_level = v2_maxvalue * (3.3 / 4096);

      v2_offset = v2_maxvalue - v2_minvalue;
      v2_amplitude_pk_pk = v2_high_level - v2_low_level;
    }
  }

  if (buffer_3_full == true) {       //if the buffer is filled
    for (i = 0; i < analyse_samples; i++) {
      if (measured_samples_3[i] < 4096 && measured_samples_3[i] > 0) {
        if (v3_maxvalue < measured_samples_3[i]) {
          v3_maxvalue = measured_samples_3[i];
        }

        if (v3_minvalue > measured_samples_3[i]) {
          v3_minvalue = measured_samples_3[i];
        }
      }

      v3_low_level = v3_minvalue * (3.3 / 4096);
      v3_high_level = v3_maxvalue * (3.3 / 4096);

      v3_offset = v3_maxvalue - v3_minvalue;
      v3_amplitude_pk_pk = v3_high_level - v3_low_level;
    }
  }
}

void get_frequency()
{
  int i;
  int clk_samd = 48000000;
  int divider = 9;
  int prescaler = 128;
  int conversion_cycles = 7;
  float GCLK_ADC = clk_samd / divider;
  float CLK_ADC = GCLK_ADC / prescaler;
  float conversion_frequency = (1 / ((1 / CLK_ADC) * conversion_cycles));
  float samples_channel = conversion_frequency / 3;

  float v1_period = 0;
  float v2_period = 0;
  float v3_period = 0;

  float samples_period_1 = 0;
  float samples_period_2 = 0;
  float samples_period_3 = 0;

  if (buffer_1_full == true) {
    for (i = 0; i < analyse_samples; i++) {
      if ((measured_samples_1[i] < v1_offset) && (measured_samples_1[i + 1] > v1_offset))
        v1_period += 0.5;
      if ((measured_samples_1[i] > v1_offset) && (measured_samples_1[i + 1] < v1_offset))
        v1_period += 0.5;
    }

    samples_period_1 = analyse_samples / v1_period;
    v1_freq = samples_channel / samples_period_1;
  }

  if (buffer_2_full == true) {
    for (i = 0; i < analyse_samples; i++) {
      if ((measured_samples_2[i] < v2_offset) && (measured_samples_2[i + 1] > v2_offset))
        v2_period += 0.5;
      if ((measured_samples_2[i] > v2_offset) && (measured_samples_2[i + 1] < v2_offset))
        v2_period += 0.5;
    }

    samples_period_2 = analyse_samples / v2_period;
    v2_freq = samples_channel / samples_period_2;
  }

  if (buffer_3_full == true) {
    for (i = 0; i < analyse_samples; i++) {
      if ((measured_samples_3[i] < v3_offset) && (measured_samples_3[i + 1] > v3_offset))
        v3_period += 0.5;
      if ((measured_samples_3[i] > v3_offset) && (measured_samples_3[i + 1] < v3_offset))
        v3_period += 0.5;
    }

    samples_period_3 = analyse_samples / v3_period;
    v3_freq = samples_channel / samples_period_3;
  }
}

//this function gets called by the interrupt
void TC3_Handler (void)
{
  //digitalWrite(9, HIGH);
  if (TC3->COUNT8.INTFLAG.bit.OVF == 1) {
    v1_pointer++;
    v2_pointer++;

    if (v1_pointer == samples_needed) {
      v1_pointer = 0;
    } else if (v2_pointer == samples_needed) {
      v2_pointer = 0;
    }

    TC3->COUNT8.CC[0].reg = generated_samples[v1_pointer];
    TC3->COUNT8.CC[1].reg = generated_samples[v2_pointer];
  }

  // END OF YOUR CODE
  TC3->COUNT8.INTFLAG.bit.OVF = 1; //Reset overflow interrupt

}


void TCC0_Handler (void) 
{

  if (TCC0->INTFLAG.bit.OVF == 1) {
    v3_pointer++;

    if (v3_pointer == samples_needed) {
      v3_pointer = 0;
    }

    TCC0->CC[3].reg = generated_samples[v3_pointer];
  }

  // END OF YOUR CODE
  TCC0->INTFLAG.bit.OVF = 1; //Reset overflow interrupt

  //digitalWrite(9, LOW);
}

// ADC interupt handler
// interupt at result ready
// write result to buffer and change channel
void ADC_Handler()
{
  //digitalWrite(8, HIGH);

  if (pin == 0) {
    measured_samples_1[v1_in] = REG_ADC_RESULT;
    v1_in++;

    if (v1_in == analyse_samples) {
      buffer_1_full = true;
      v1_in = 0;
    }

  } else if (pin == 1) {
    measured_samples_2[v2_in] = REG_ADC_RESULT;
    v2_in++;

    if (v2_in == analyse_samples) {
      buffer_2_full = true;
      v2_in = 0;
    }
  } else if (pin == 2) {
    measured_samples_3[v3_in] = REG_ADC_RESULT;
    v3_in++;

    if (v3_in == analyse_samples) {
      buffer_3_full = true;
      v3_in = 0;
    }
  }

  pin++;
  if (pin == 3) {
    pin = 0;
  }

  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY; //Need to reset interrupt
  //digitalWrite(8, LOW);
}
