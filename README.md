
# BMS-RTOS

Freertos implementation on battery management system master(Teensy 4.1). by Vaibhav and Chithrinesh


## TASKS:
```bash
  xTaskCreate(bms, "BMS", 2000, NULL, 1, NULL);
  xTaskCreate(interruptdata, "INTERRUPTDATA", 2000, NULL, 3, NULL);
  xTaskCreate(interruptdata1, "INTERRUPTDATA1", 2000, NULL, 2, NULL);
```
Three tasks are created with stack size of 2000 bytes.

BMS task has lowest priority and this task handles reading of current data from current sensor and voltage data from LTC6813 through iso-spi.

interruptdata task has the highest priority and its used to process temperature data sent by orion thermistor pack through can.

interruptdata1 task has priority of 2 and its used to process data sent from charger through can.
## Scheduling algorithm:
Prioritized Preemptive Scheduling with Time Slicing is chosen as the scheduling algorithm and it can be enabled by confuguring the settings of FreeRTOSConfig.h as below:

![Image Alt](https://github.com/chithrinesh/RTOS-DOCUMENTATION/blob/main/WhatsApp%20Image%202025-02-28%20at%206.54.22%20PM.jpeg?raw=true)

Preemptive scheduling algorithms will immediately 'preempt' the Running state task if a task that has a
 priority higher than the Running state task enters the Ready state.

  Time slicing is used to share processing time between tasks of equal priority, even when the tasks do
 not explicitly yield or enter the Blocked state. A time slice is equal to the time between two
 RTOS tick interrupts.
## Deferring interrupts:
An interrupt service routine must record the cause of the interrupt, and clear the interrupt. 
Any other processing necessitated by the interrupt can often be performed in a task, allowing 
the interrupt service routine to exit as quickly as is practical. This is called ‘deferred interrupt 
processing’, because the processing necessitated by the interrupt is ‘deferred’ from the ISR to 
a task.

![Image Alt](https://github.com/chithrinesh/RTOS-DOCUMENTATION/blob/main/WhatsApp%20Image%202025-02-28%20at%206.54.22%20PM%20(1).jpeg?raw=true)

## How deferring interrupts is utilised in the code:
Interrupt method is used to read CAN(Controller area network) data from orion thermistor pack and charger.Kernel object semaphores are used to defer the ISR to task interruptdata and interruptdata1 respectively to keep ISR short and to make sure we dont lose any new interrupts.

```bash
portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
```
ensures context switch to higher priority task and avoid delay waiting for tick interrupt to switch to high priority task.It should be used at the end of ISR.

creation of semaphore:
```bash
  yBinarySemaphore = xSemaphoreCreateBinary();//creation of semaphore
  xBinarySemaphore = xSemaphoreCreateBinary();
```
giving semaphore from ISR to high priority task waiting for semaphore: 
```bash
  xSemaphoreGiveFromISR(yBinarySemaphore, &xHigherPriorityTaskWoken);//gives semaphore to task waiting for semaphore
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  ```
Higher priority task waiting for semaphore to process the data from ISR:
```bash
  void interruptdata(void *pvParameters) {
  while (1) {
    xSemaphoreTake(yBinarySemaphore, portMAX_DELAY);
```
## Functions of the BMS:
1.Monitor voltage of 106 cells by receiving data from ltc6813 battery stack monitor using spi at speed of 1Mhz.

2.Recive temperature from 100 thermistors connected to orion thermistor module using can at bitrate of 250 Kbps.

3.Send and receive data from elcon 6.6kw charger through can at bitrate of 500Kbps.

4.Send and receive data from bms gui via serial port at baud rate of 9600 bps.

5.Log the cell voltages and the temperatures using the onboard SD card.

6.Send an output to the AMS Latch PCB when Cell Voltages or Temps exceed their limits to open the contactor of HV battery.
## Hardware setup:

![Image Alt](https://github.com/user-attachments/assets/13b0f3eb-775f-45e1-b9c5-7403bffada3c)

## Refernces

[Mastering Freertos book](https://freertos.org/Documentation/02-Kernel/07-Books-and-manual/01-RTOS_book)

