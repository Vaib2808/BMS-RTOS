#include <FlexCAN_T4.h>
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include <EEPROM.h>
#include <SD.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6813.h"
#include "arduino_freertos.h"
#include "semphr.h"
#include <SD.h>
#include <SPI.h>
File myFile;
const int chipSelect = BUILTIN_SDCARD;

// Define ENABLED and DISABLED
#define pin19 19
#define pin18 18
#define ENABLED 1
#define DISABLED 0

// Use Arduino namespace
using namespace arduino;

// Semaphore handles
SemaphoreHandle_t yBinarySemaphore;
SemaphoreHandle_t xBinarySemaphore;
#define NUM_TX_MAILBOXES 2
#define NUM_RX_MAILBOXES 2

// CAN setup
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;
CAN_message_t msg;
CAN_message_t msg1;
CAN_message_t msg2;

// Buffers and data
char buf[40];
String checks;
int i = 0;
uint8_t data[8];
uint8_t data1[8];
int8_t dataTherm[8];
int value1, adcValue;
unsigned long value3;
float sum=0;
int control=1;
 uint8_t volthi=17; 
 uint8_t voltlo=148;  
 uint8_t currhi=0;  
 uint8_t currlo=10;


// Pin and voltage thresholds
const int OUTPUT_PIN = 20;
const uint16_t OV_THRESHOLD = 46000;
const uint16_t UV_THRESHOLD = 25000;

// Pin and temperature thresholds

const uint16_t OV_THRESHOLDT = 45;
const uint16_t UV_THRESHOLDT = 20;

// BMS settings
const uint8_t TOTAL_IC = 6;
const uint8_t ADC_OPT = ADC_OPT_DISABLED;
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;
const uint8_t ADC_DCP = DCP_DISABLED;
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL;
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;
const uint8_t SEL_ALL_REG = REG_ALL;
const uint8_t SEL_REG_A = REG_1;
const uint8_t SEL_REG_B = REG_2;

// Measurement loop setup
const uint16_t MEASUREMENT_LOOP_TIME = 500;
const uint8_t WRITE_CONFIG = DISABLED;
const uint8_t READ_CONFIG = DISABLED;
const uint8_t MEASURE_CELL = ENABLED;
const uint8_t MEASURE_AUX = DISABLED;
const uint8_t MEASURE_STAT = ENABLED;
const uint8_t PRINT_PEC = ENABLED;

// Global battery variables
cell_asic BMS_IC[TOTAL_IC];
bool REFON = true;
bool ADCOPT = false;
bool GPIOBITS_A[5] = {false, false, true, true, true};
bool GPIOBITS_B[4] = {false, false, false, false};
uint16_t UV = UV_THRESHOLD;
uint16_t OV = OV_THRESHOLD;
bool DCCBITS_A[12] = {false, false, false, false, false, false, false, false, false, false, false, false};
bool DCCBITS_B[7] = {false, false, false, false, false, false, false};
bool DCTOBITS[4] = {true, false, true, false};
bool FDRF = false;
bool DTMEN = true;
bool PSBITS[2] = {false, false};

// Threshold crossing flags
float threscrossed = false;
float threscrossedt = false;
float threscrossedc = false;
static bool flag = false;
static unsigned long timeitcrossedthres = 0;
float total_voltage = 0.0;
float ic_summed_voltage = 0.0;
static  bool counterUpdated = false;

//current sensor variables
int count;
int count1;
int count2;
float chargingvoltage;
float chargingcurrent;
float status;
int ADC_read_200;
float curr_read_200;
int it;
float sumcurr=0;
float curravg;
int latchflag=1;
float datasum;
char filename[30];
int flagfile=0;



// Define DATALOG_DISABLED
#define DATALOG_DISABLED 0
void getUniqueFilename(char* filename, const char* form) {
  if(flagfile==0)
  {
  int counter = EEPROM.read(1);
  counter=counter+1;
  EEPROM.write(1,counter); 
  flagfile=1;
  }
  
  /*if(counterUpdated == false){
     int counter = EEPROM.read(1);// Retrieve the counter from EEPROM
    //Serial.println(counter);
    EEPROM.write(1,counter); 
    }*/
    int counter = EEPROM.read(1);
 
    snprintf(filename, 30, form, counter);
  
}

void print_cells(uint8_t datalog_en) {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    if (datalog_en == 0) {
      /*Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      Serial.print(", ");*/
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
        Serial.print("a,");
        Serial.print(current_ic+1);
        Serial.print(",");
        Serial.print(i + 1, DEC);
        Serial.print(",");
        Serial.println(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        sum=sum+BMS_IC[current_ic].cells.c_codes[i] * 0.0001;
    
        //Serial.print(",");
      }
      
      //Serial.println();
    }


  }
  //Serial.print("TOTAL VOLTAGE");
  //Serial.print(sum);
  //Serial.println("\n");
  Serial.print("e,");
  Serial.println(sum);
  Serial3.print("i,");
  Serial3.print(data[5]);
  Serial3.print(",");
  Serial3.println(sum);
  datasum=sum;
  sum=0;
}

void print_SUM_OF_CELLS(uint8_t datalog_en) {
  total_voltage = 0.0;
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(" IC ");
    Serial.print(current_ic + 1, DEC);
    Serial.print(" Voltage per Segment:");
    ic_summed_voltage = BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 30 * 4;
    Serial.print(ic_summed_voltage);
    Serial.print(",");
    total_voltage += ic_summed_voltage;
  }
  Serial.println("\nTotal voltage of the TSAC: ");
  Serial.println(total_voltage, 3);
  Serial.println("\n");
}

void current() {
    for(it=0;it<5;it++)
    {
    ADC_read_200 = analogRead(A14)-47;
    curr_read_200 = (ADC_read_200*0.4692082111-250)+0.38;
    sumcurr=sumcurr+curr_read_200;
    }
    curravg=sumcurr/5;
    Serial.print("c,");
    Serial.println(curravg);
    ADC_read_200 = analogRead(A14);
    if (ADC_read_200<5&&ADC_read_200>=0) {
        threscrossedc = true;
        Serial.println("current sensor disconnected");
      }
    else if (curravg>150||curravg<-150) {
        threscrossedc = true;
        Serial.println("over overcurrent");
      }
    else{
      threscrossedc = false;
    }
  if (threscrossed==true||threscrossedt==true||threscrossedc==true) {
    digitalWrite(OUTPUT_PIN, HIGH);
  } 
  else {
    
    digitalWrite(OUTPUT_PIN, LOW);
  }
    curravg=0;
    sumcurr=0;
}


void latch_pcb(uint8_t datalog_en) {
  count=0;
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    if(current_ic<5)
    {
    
    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
      uint16_t cell_voltage = BMS_IC[current_ic].cells.c_codes[i];
      if (cell_voltage < UV_THRESHOLD && current_ic!=3 && i!=BMS_IC[0].ic_reg.cell_channels-2 && i!=BMS_IC[0].ic_reg.cell_channels-1) {
        threscrossed = true;
        Serial.print("Under voltage detected in IC ");
        Serial.print(current_ic + 1);
        Serial.print(", Cell ");
        Serial.println(i + 1);
      }

      else if (cell_voltage > OV_THRESHOLD) {
        threscrossed = true;
        Serial.print("Over voltage detected in IC ");
        Serial.print(current_ic + 1);
        Serial.print(", Cell ");
        Serial.println(i + 1);
      }
      else{
        count=count+1;
      }
    }
  }
  else if(current_ic==5){
     for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels -2; i++) {
      uint16_t cell_voltage = BMS_IC[current_ic].cells.c_codes[i];
      if (cell_voltage < UV_THRESHOLD) {
        threscrossed = true;
        Serial.print("Under voltage detected in IC ");
        Serial.print(current_ic + 1);
        Serial.print(", Cell ");
        Serial.println(i + 1);
      }

      else if (cell_voltage > OV_THRESHOLD) {
        threscrossed = true;
        Serial.print("Over voltage detected in IC ");
        Serial.print(current_ic + 1);
        Serial.print(", Cell ");
        Serial.println(i + 1);
      }
      else{
        count=count+1;
      }

    }
  

  }
  }
  if(count==106)
  {
    threscrossed= false;
  }
  

  if (threscrossed==true||threscrossedt==true||threscrossedc==true) {
    digitalWrite(OUTPUT_PIN, HIGH);
    count=0;

  } 
  else {
    
    digitalWrite(OUTPUT_PIN, LOW);
  }
    Serial.print("d,");
    Serial.println(threscrossed);
    Serial.print("j,");
    Serial.println(threscrossedt);
    Serial.print("k,");
    Serial.println(threscrossedc);
}

void check_error(int error) {
  if (error == -1) {
    Serial.println("A PEC error was detected in the received data");
  }
}

void bms(void *pvParameters) {
  while (1) {
 
    int8_t error = 0;
    wakeup_sleep(TOTAL_IC);
    LTC6813_adcvsc(ADC_CONVERSION_MODE, ADC_DCP);
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
    check_error(error);
    print_cells(DATALOG_DISABLED);
    latch_pcb(DATALOG_DISABLED);
    current();
    if (Serial.available() > 0) {
        char command1 = Serial.read();
        if (command1 == 'a') {
        int command = Serial.read();
        if (command == 0) {
            // Start action
            control=0;
            Serial.println("Received start command");
        }
        else if (command == 1) {
            // Start action
            control=1;
            Serial.println("Received start command");
    }
    }
     if (Serial.available() >= 5) {
    char command2 = Serial.read();
    if (command2 == 'b') {
      volthi = Serial.read();  
      voltlo = Serial.read();  
      currhi = Serial.read();  
      currlo = Serial.read(); 
          }
          }
}
  int commv=volthi*256+voltlo;
  int commc=currhi*256+currlo;
  Serial.print("y,");
  Serial.println(commv);
  Serial.print("x,");
  Serial.println(commc);
   
    msg1.id = 0x1806E5F4;
    msg1.len = 8;
    msg1.flags.extended =1;
    msg1.buf[0] = volthi;
    msg1.buf[1] = voltlo;
    msg1.buf[2] = currhi;
    msg1.buf[3] = currlo;
    msg1.buf[4] = control;
    msg1.buf[5] = 0x00;
    msg1.buf[6] = 0x00;
    msg1.buf[7] = 0x00;
    Can1.write(msg1);
    Serial.print("z,");
    Serial.println(control);

    myFile = SD.open(filename, FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to");
    Serial.println(filename);
    myFile.print(datasum);
    myFile.print(",");
    myFile.print(curravg);
    myFile.print(",");
    myFile.print(data[5]);
    myFile.print(",");
    myFile.print(millis());
    myFile.println();
	// close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.print("error opening");
    Serial.println(filename);
  }

   
  
    //vTaskDelay(200);

    //j++;
  }
}

void interruptdata(void *pvParameters) {
  while (1) {
    xSemaphoreTake(yBinarySemaphore, portMAX_DELAY);
    /*Serial.print(data[0]);
    Serial.print(" | ");
    Serial.print(data[1]);
    Serial.print(" | ");
    Serial.print(data[2]);
    Serial.print(" | ");
    Serial.print(data[3]);
    Serial.print(" | ");
    Serial.print(data[4]);
    Serial.print(" | ");
    Serial.print(data[5]);
    Serial.print(" | ");
    Serial.print(data[6]);
    Serial.print(" | ");
    Serial.print(data[7]);
    Serial.println();*/
    count2=count2+1;
    Serial.print("b,");
    Serial.print(data[1]);
    Serial.print(",");
    Serial.println(data[2]);
    if (data[4] < UV_THRESHOLDT || data[2] < UV_THRESHOLDT) {
        threscrossedt = true;
        Serial.print("Under temp detected in IC ");
        Serial.print(data[1]);
        Serial.print(",");
        Serial.print(data[4]);
        Serial.println("Thermistor");
        count1=0;
        count2=0;
      }
    
    else if (data[5] > OV_THRESHOLDT || data[2] > OV_THRESHOLDT) {
        threscrossedt = true;
        Serial.print("Over temp detected in IC ");
        Serial.print(data[1]);
        Serial.println("thermistor");
        count1=0;
        count2=0;
     
      }
    else{
      count1=count1+1;
    }

  if(count1==200)
  {
    threscrossedt=false;
  }
  

  if (threscrossed==true||threscrossedt==true||threscrossedc==true) {
    digitalWrite(OUTPUT_PIN, HIGH);
  } 
  else {
    
    digitalWrite(OUTPUT_PIN, LOW);
  }
  if (count2==210)
  {
    count1=0;
    count2=0;
  }
    //xSemaphoreGive(yBinarySemaphore);
    //portYIELD();
  }
}

void interruptdata1(void *pvParameters) {
  while (1) {
    xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
    chargingvoltage=data1[0]*256+data1[1];
    chargingcurrent=data1[2]*256+data1[3];
    status=data1[4];
    Serial.print("f,");
    Serial.println(chargingvoltage);
    Serial.print("g,");
    Serial.println(chargingcurrent);
    Serial.print("h,");
    Serial.println(status);
    if(digitalRead(pin19)==LOW&&digitalRead(pin18)==HIGH)
    {
      control=0;
      latchflag=1;
    }
    else if(digitalRead(pin18)==LOW)
    {
      control=1;
      //if(latchflag==1)
      //{
      digitalWrite(OUTPUT_PIN, HIGH);
      latchflag=0;
      //}
      
    }

  }
    
}

void canSniff(const CAN_message_t &msg) {
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  if (msg.id == 0x1838F380) {
    for (i = 0; i < 8; i++) {
      data[i] = msg.buf[i];
    }
  }
  else if(msg.id == 0x1838F381) {
    for (i = 0; i < 8; i++) {
      data[i] = msg.buf[i];
    }
  }
  xSemaphoreGiveFromISR(yBinarySemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void canSniff1(const CAN_message_t &msg2) {
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  if (msg2.id ==  0x18FF50E5) {
    for (i = 0; i < 8; i++) {
      data1[i] = msg2.buf[i];
    }
  }

  xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void setup() {
  
  digitalWrite(OUTPUT_PIN, LOW);
  Serial.begin(9600);

  getUniqueFilename(filename, "amsdata_%d.csv");

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  
  // open the file. 
  myFile = SD.open(filename, FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to");
    Serial.println(filename);
    myFile.print("TOTAL VOLTAGE");
    myFile.print(",");
    myFile.print("CURRENT");
    myFile.print(",");
    myFile.print("TEMP");
    myFile.print(",");
    myFile.print("TIME");
    myFile.println();
	// close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.print("error opening");
    Serial.println(filename);
  }
  
  Serial3.begin(9600);
  delay(400);
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV64); // This will set the Linduino to have a 1MHz Clock
  LTC6813_init_cfg(TOTAL_IC, BMS_IC);
  LTC6813_init_cfgb(TOTAL_IC,BMS_IC);
  for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
  {
    LTC6813_set_cfgr(current_ic,BMS_IC,REFON,ADCOPT,GPIOBITS_A,DCCBITS_A, DCTOBITS, UV, OV);
    LTC6813_set_cfgrb(current_ic,BMS_IC,FDRF,DTMEN,PSBITS,GPIOBITS_B,DCCBITS_B);   
  }   
  LTC6813_reset_crc_count(TOTAL_IC,BMS_IC);
  LTC6813_init_reg_limits(TOTAL_IC,BMS_IC);
  wakeup_sleep(TOTAL_IC);
   Can0.begin();
  Can0.setBaudRate(250000);
  Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i<NUM_RX_MAILBOXES; i++){
    Can0.setMB((FLEXCAN_MAILBOX)i,RX,EXT);
  }
  for (int i = NUM_RX_MAILBOXES; i<(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++){
    Can0.setMB((FLEXCAN_MAILBOX)i,TX,EXT);
  }
  Can0.setMBFilter(REJECT_ALL);
  Can0.enableMBInterrupts();
  Can0.onReceive(MB0,canSniff);
  Can0.onReceive(MB1,canSniff);
  //Can0.onReceive(MB2,canSniff);
  Can0.setMBFilter(MB0, 0x1838F380);
  Can0.setMBFilter(MB1, 0x1838F381);
  
  Can0.mailboxStatus();

  Can1.begin();
  Can1.setBaudRate(500000);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  Can1.onReceive(canSniff1);
  Can1.mailboxStatus();
  pinMode(pin19,INPUT_PULLUP);
  pinMode(pin18,INPUT);
  
  /*Can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = NUM_TX_MAILBOXES + NUM_RX_MAILBOXES; i<6; i++){
    Can1.setMB((FLEXCAN_MAILBOX)i,RX,EXT);
  }
  for (int i = 6; i<8; i++){
    Can1.setMB((FLEXCAN_MAILBOX)i,TX,EXT);
  }
  Can1.setMBFilter(REJECT_ALL);
  Can1.enableMBInterrupts();
  Can1.onReceive(MB4,canSniff1);
  Can1.onReceive(MB5,canSniff1);
  
  Can1.setMBFilter(MB4,  0x18FF50E5);*/
  //Can1.setMBFilter(MB5, 0x1838F381);
  
 
  yBinarySemaphore = xSemaphoreCreateBinary();
  xBinarySemaphore = xSemaphoreCreateBinary();
  xTaskCreate(bms, "BMS", 2000, NULL, 1, NULL);
  xTaskCreate(interruptdata, "INTERRUPTDATA", 2000, NULL, 3, NULL);
  xTaskCreate(interruptdata1, "INTERRUPTDATA1", 2000, NULL, 2, NULL);
  
 
  vTaskStartScheduler();
   //digitalWrite(OUTPUT_PIN, LOW);
  //pinMode(OUTPUT_PIN, OUTPUT);
}

void loop() {
  // Empty loop as FreeRTOS is handling tasks
}