// Dhruv Anand - 400535222
// Meausurement LED - PN0 (D2)
// UART LED - PN1 (D1)
// Additional Status LED - PF4 (D3) (using for TOF on/off)
// Assigned Bus Speed - 20MHz

//bus calculation
// SysClk = fVCO / (PSYSDIV + 1)
// 480 MHz / (PSYSDIV + 1) = 20 MHz
// PSYSDIV = 24 - 1 = 23

//PN1 is D1
//PN0 is D2
//PF4 is D3
//PF0 is D4


#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"


#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3  
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
}


//#####################################################################
//#####################################################################
//#####################################################################
//#####################################################################
//#####################################################################

void PortF_Init(void) 
{
    SYSCTL_RCGCGPIO_R |= 0x20; 								//enable clock PortF (bit 5), 0x20 = 32, ex: 0000 |= 0010 => 0010
    while((SYSCTL_PRGPIO_R & 0x20) == 0){}; 	//wait until PortF is ready (loop runs until it detects a 1 at that port's preif ready register)
    GPIO_PORTF_DIR_R |= 0x11; 								//set pins 0 and 4 as output, 0x11 = 17
    GPIO_PORTF_DEN_R |= 0x11; 								//enable digital function for pins 0 and 4
}

void PortH_Init(void) 
{
    SYSCTL_RCGCGPIO_R |= 0x80; 								//enable clock PortH (bit 7), 0x80 = 128, ex: 0000 |= 1000 => 1000
    while((SYSCTL_PRGPIO_R & 0x80) == 0){}; 	//wait until PortH is ready
    GPIO_PORTH_DIR_R |= 0x0F; 								//set pins 0-3 as output, 0x0F = 15
    GPIO_PORTH_DEN_R |= 0x0F; 								//enable digital function for pins 0-3
    GPIO_PORTH_AFSEL_R &= ~0x0F; 							//disable alternate function for pins 0-3, ex: 0001 & 1110 = 0000 (bit 0 cleared)
    GPIO_PORTH_AMSEL_R &= ~0x0F; 							//disable analog function for pins 0-3
}

void PortJ_Init(void) 
{
    SYSCTL_RCGCGPIO_R |= 0x100; 							//enable clock PortJ (bit 8), 0x100 = 256, ex: 0000 |= 0001 => 0001
    while((SYSCTL_PRGPIO_R & 0x100) == 0){}; 	//wait until PortJ is ready
    GPIO_PORTJ_DIR_R &= ~0x03; 								//set pins 0 and 1 as input, 0x03 = 3, ex: 1111 & 1100 => 1100
    GPIO_PORTJ_DEN_R |= 0x03; 								//enable digital function for pins 0 and 1
    GPIO_PORTJ_PUR_R |= 0x03; 								//enable pull-up resistors for pins 0 and 1
}

void PortN_Init(void) 
{
    SYSCTL_RCGCGPIO_R |= 0x1000; 							//enable clock PortN (bit 12)
    while((SYSCTL_PRGPIO_R & 0x1000) == 0){}; //wait until PortN is ready
    GPIO_PORTN_DIR_R |= 0x03; 								//set pins 0 and 1 as output, 0x03 = 3 0011
    GPIO_PORTN_DEN_R |= 0x03; 								//enable digital function for pins 0 and 1
}

void step_motor(int delay, int direction) 
{
	if(direction) //direction = 1 = CW
	{
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10us(delay);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait10us(delay);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10us(delay);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10us(delay);
	} 
	else //CCW
	{
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10us(delay);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10us(delay);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait10us(delay);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10us(delay);
	}
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) 
{
  uint8_t byteData, sensorState = 0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, i = 0;
  uint16_t wordData;
  uint16_t Distance;
  uint8_t dataReady;
  uint8_t RdByte;
  uint16_t RdWord;

  // initialize
  PLL_Init();
	PortF_Init();
  PortH_Init();
	PortJ_Init();
	PortN_Init();
  SysTick_Init();
  onboardLEDs_Init();
  I2C_Init();
  UART_Init();
  
	//preparing TOF sensor
  // hello world!
  UART_printf("Program Begins\r\n");
  int mynumber = 1;
  sprintf(printf_buffer, "2DX ToF Program Studio Code %d\r\n", mynumber);
  UART_printf(printf_buffer);

  // Basic I2C read functions to check I2C functionality
  status = VL53L1X_GetSensorId(dev, &wordData);
  sprintf(printf_buffer, "(Model_ID, Module_Type)=0x%x\r\n", wordData); //format data
  UART_printf(printf_buffer); //print the formatted data onto uart

  // Wait for ToF device to boot
  while (sensorState == 0) {
    status = VL53L1X_BootState(dev, &sensorState);
    SysTick_Wait10ms(10);
  }
  FlashAllLEDs();
  UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");

  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt */
  status = VL53L1X_SensorInit(dev);      /* Initialize the sensor */
  Status_Check("SensorInit", status);

  // Start ranging
  status = VL53L1X_StartRanging(dev);

	//TOF is ready, now implement below whats needed
	//#####################################################################
	//#####################################################################
	
	int scanReady = 0; //if true, the tof is ready to scan, awaiting moment
	int spinStatus = 0; //spinning or not
	int takeMeasure = 0; //rady to take a measurement
	int steps = 512; //steps for 360 deg
	int displacement; //disp from positive reference
	int stepCount = 0; //amount of steps taken
	int depth = 0; //increment after 360deg
	int direc = 1; //direction CW initially 
	
	while(1) //main loop to run program
	{
		//confirm 20MHz clock speed works
		//turn all leds on first using 10ms delay with 200000 ticks. SHould be on for 1ms. Then change to the old one for 120Mhz. Then it should be on for 12s rather than 1s
		//it will be on for 12s because: RealDelay = 1.200.000 ticks/20.000.000Hz = 60ms each. 60ms*200 = 12s
//		while(1)
//		{
//			GPIO_PORTN_DATA_R ^= 0b00000011; 								
//			GPIO_PORTF_DATA_R ^= 0b00010001; 									
//			SysTick_Wait10ms(200);														//2s delay bc 10ms * 200 = 2s 
//			GPIO_PORTN_DATA_R ^= 0b00000011;			
//			GPIO_PORTF_DATA_R ^= 0b00010001; 									
//			SysTick_Wait10ms(200);														//2s delay	
//		}			
		
		
		//press PJ0 to start spinning the motor 
		if(!(GPIO_PORTJ_DATA_R & 0x01)) //not bc active low. 0x01 bc chaning bit 0 (=1 in decimal)
		{
			//FlashLED2(1);
			spinStatus ^= 1; //pressing button 1 activate or deactivates the stepper motor
			while(!(GPIO_PORTJ_DATA_R & 0x01));  // Debounce (loop stays active until it detects the realease (1 again))
		}
		
		//press PJ1 to turn on TOF
		if (!(GPIO_PORTJ_DATA_R & 0x02)) //0x02 because change bit 1
		{
			GPIO_PORTF_DATA_R ^= 0x10; // Toggle PF4 (Additional Status LED) 0x10 because changing bit 4 (decimal 16)
			scanReady ^= 1; //TOF is ready to scan when needed
			while (!(GPIO_PORTJ_DATA_R & 0x02));  // Debounce
		}
		
		//If TOF is ready and recives a signal to take a measurement, scan
		if(scanReady == 1 && takeMeasure == 1)
		{
			// Wait for the data to be ready
			while (dataReady == 0) 
			{
				status = VL53L1X_CheckForDataReady(dev, &dataReady); //I2C read the data ready
				FlashLED3(1);
				VL53L1_WaitMs(dev, 5);
			}
			dataReady = 0;
			
			//read distance values from TOF
			status = VL53L1X_GetDistance(dev, &Distance);
			
			//calc displacement from 0deg, neg or pos, taking pos refrence
			if(direc == 0)
			{
				displacement = steps - stepCount;
			}
			else
				displacement = stepCount;
			
			//flash UART status led PN1
			FlashLED1(1);	//PN1 the UART LED
			
			//send data for distance, depth, and steps
			sprintf(printf_buffer, "%u, %u, %u\r\n", Distance, displacement, depth); //uart format the data
			UART_printf(printf_buffer); //uart send to com5
			
			//reset take measurments indicator
			takeMeasure = 0;
				
			status = VL53L1X_ClearInterrupt(dev); //clear interrupt for next ready
			SysTick_Wait10us(500);  // Wait for 5 ms before moving to the next position
			
		}
		
		//spin motor while spin status is true
		if(spinStatus == 1)
		{
			//make a motor movement
			step_motor(250, direc);
			stepCount++;
		}
		
		//every 11.25 deg of rotation, flash data led and enable take Measurement
		if(stepCount % 16 == 0 && stepCount != 0)
		{
			FlashLED2(1); //(d2) measurement led
			SysTick_Wait(10000000);
			takeMeasure = 1; //system ready to send data
		}
		
		//every 360deg, reset the system and wait for start button to press again. Note we go opposite direction now to untangle cable and increase depth
		if(stepCount > 512)
		{
			direc ^= 1;
			spinStatus = 0;
			depth++;
			stepCount = 0;
		}
		
	}//while loop end
	
  // Stop ranging when done
  VL53L1X_StopRanging(dev);
} 

