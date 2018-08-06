/*****************************************************************************
 *   A demo example using several of the peripherals on the base board
 *
 *   Copyright(C) 2011, EE2024
 *   All rights reserved.
 *
 ******************************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

#include "joystick.h"
#include "rotary.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "temp.h"
#include "led7seg.h"
#include "light.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#define TEMP_SCALAR_DIV10 1
#define TEMP_NUM_HALF_PERIODS 340
#define MAX_LIGHT 15000
#define MIN_LIGHT 0
#define ACC_DIV_MEASURE 64.0

static int TEMP_THRESHOLD = 290;
static int OBSTACLE_THRESHOLD = 3000;
static float ACC_THRESHOLD = 0.4;

static const uint8_t SEGMENT_DISPLAY[16] =
{
	 //digits 0-9
	 0x24, 0xAF, 0xE0, 0xA2, 0x2B, 0x32, 0x30, 0xA7, 0x20, 0x22,
	 // A-F
	 0x21, 0x38, 0x74, 0xA8, 0x70, 0x71,
};

typedef enum {
	ACC_OFF,
	ACC_NORMAL,
	ACC_HIGH,
} ACC_STATE;

typedef enum {
	TEMP_OFF,
	TEMP_NORMAL,
	TEMP_HIGH,
} TEMP_STATE;

typedef enum {
	LIGHT_OFF,
	LIGHT_NORMAL,
	LIGHT_HIGH,
} LIGHT_STATE;

typedef enum {
    STATIONARY,
    LAUNCH,
    RETURN,
} MODE;

typedef struct {
	MODE modeState;
	ACC_STATE accState;
	TEMP_STATE tempState;
	LIGHT_STATE lightState;
} STATE;

typedef struct {
	int32_t temperature;
	int halfPeriods;
	uint32_t temperatureT1;
	uint32_t temperatureT2;
} TEMP;

typedef struct {
	float acc_x;
	float acc_y;
	int32_t temp;
	uint32_t light;
	uint16_t brightness;
} DATA;

typedef struct {
	int8_t acc_x[6];
	int8_t acc_y[6];
	uint8_t temp[6];
	uint8_t light[6];
	uint8_t uart[100];
} DISPLAY;

int8_t xoff, yoff;
int8_t x, y, z;
uint8_t msg_buf[64];
uint32_t buf_count = 0;

static uint8_t clear_warning = 0;

// Optimization flags
static int temp_change = 1;
static int acc_change = 0;
static int light_change = 0;
static int telemetry_change = 0;
static int obstacle_avoid = 0;
static int message_received = 0;
static int launch_toggle = 0;
static int mode_toggle = 0;

// De-bouncing flags
static int firstPress = 1;
static int secondPress = 0;
static int launchFirstPress = 1;
static int launchSecondPress = 0;

uint32_t launchFirstPressTime;
uint32_t launchSecondPressTime = 0;
uint32_t firstPressTime;
uint32_t secondPressTime = 0;
volatile uint32_t msTicks;

static DATA data = {0, 0, 0, 0};
static DISPLAY display = {"", "", "", "", ""};
static TEMP temp = {0, 0, 0, 0};

static STATE state = {STATIONARY, ACC_OFF, TEMP_NORMAL, LIGHT_OFF};

//   **************************    FUNCTION PROTOTYPES      **************************
///////// Init functions /////////
void init_all(void);
static void init_ssp(void);
static void init_i2c(void);
static void init_uart(void);
static void init_sw3(void);
static void init_sw4(void);
void init_light_sensor(void);
void init_temp_sensor(void);
void init_timer0(void);

///////// Main Functions /////////
void warn_blink(void);
void countDown(void);
void configStationary(void);
void updateStationary(void);
void configLaunch(void);
void updateLaunch(void);
void configReturn(void);
void updateReturn(void);
void modeToggle(void);
void telemetryMsg(void);
void checkLaunchMsg(void);
void checkReturnMsg(void);

///////// Temp Functions /////////
void temp_enable(void);
void temp_disable(void);
void adjusted_temp_read(TEMP*);
void checkTemp(void);

///////// Acc Functions /////////
void checkAcc(void);

///////// Light Functions /////////
void light_sensor_enable(void);
void light_sensor_disable(void);
static uint16_t getBrightness(uint32_t);

///////// UART Functions /////////
void UART3_Rx_Interrupt(void);
void set_uart_message(uint8_t* , char* );
void set_uart_string(void);

///////// Peripheral Functions /////////
void checkManualSettings(void);

void SysTick_Handler(void) {
	msTicks++;
	warn_blink();
}
uint32_t getTicks(void) {
	return msTicks;
}

void warn_blink(void) {
	if (state.accState == ACC_HIGH && state.tempState == TEMP_HIGH) {
		if (msTicks % 333 == 0) {
				rgb_setLeds(RGB_BLUE);
			}
		if ((msTicks + 167) % 333 == 0) {
			rgb_setLeds(RGB_RED);
		}
	} else if (state.accState == ACC_HIGH) {
		if (msTicks % 333 == 0) {
			rgb_setLeds(RGB_BLUE);
		}
		if ((msTicks + 167) % 333 == 0) {
			rgb_setLeds(0);
		}
	} else if (state.tempState == TEMP_HIGH) {
		if (msTicks % 333 == 0) {
			rgb_setLeds(RGB_RED);
		}
		if ((msTicks + 167) % 333 == 0) {
			rgb_setLeds(0);
		}
	} else {
		rgb_setLeds(0);
	}
}

//   **************************    EINT0 MODE TOGGLE INTERRUPT AND FUCNTIONS      **************************
void EINT0_IRQHandler(void)  {

	// SW3 External Interrupt
	if ((LPC_SC -> EXTINT) & 0x01) {

		// Every other mode except in launch (1 press is mode toggle)
		if (state.modeState != LAUNCH) {

			if (firstPress) {
				firstPressTime = getTicks(); // Record time when first press
				firstPress = 0;
				secondPress = 1;
			} else if (secondPress) {
				secondPressTime = getTicks();
				secondPress = 0;
				firstPress = 1;
			}

			// If the delay between first and second press is larger than 1 second
			if (secondPressTime == 0) {
				mode_toggle = 1;
			} else if ((int)(secondPressTime - firstPressTime) > 1000) {
				mode_toggle = 1;
			} else { // Reset sw3 to prepare for next presses
				firstPress = 1;
				firstPressTime = 0;
				secondPress = 0;
				secondPressTime = 0;
			}

		} else {

			if (launchFirstPress) {
				launchFirstPressTime = getTicks();
				launchFirstPress = 0;
				launchSecondPress = 1;
			} else if (launchSecondPress) {
				launchSecondPressTime = getTicks();
				launchSecondPress = 0;
				launchFirstPress = 1;
			}

			if (launchSecondPressTime == 0) {
				launch_toggle  = 0;

			} else if (launchSecondPressTime - launchFirstPressTime <= 1000) {
				mode_toggle = 1;
				launch_toggle = 1;
			} else {
				launchFirstPressTime = launchSecondPressTime;
				launchSecondPress = 1;
				launchFirstPress = 0;
			}
		}

		// Reset sw3 to prepare for next presses
		if (launch_toggle) {
			launchFirstPress = 1;
			launchFirstPressTime = 0;
			launchSecondPress = 0;
			launchSecondPressTime = 0;
			firstPress = 1;
			firstPressTime = 0;
			secondPress = 0;
			secondPressTime = 0;
		}
	}
	// clear interrupt
	LPC_SC->EXTINT |= (1<<0);
}

void modeToggle(void) {

	if ((state.modeState == STATIONARY) && (state.tempState != TEMP_HIGH)) {
			countDown();
	// if second mode toggle is recorded and within 1 second of the first mode toggle
	} else if ((state.modeState == LAUNCH) && (launch_toggle)) {
		configReturn();
		state.modeState = RETURN;
		launch_toggle = 0;

	} else if (state.modeState == RETURN) {
		checkLightWarning();
		configStationary();
		state.modeState = STATIONARY;
	}
}

void countDown(void) {
	int i=14;
	uint32_t currentTime, initialTime;

	initialTime = getTicks();

	while (state.tempState != TEMP_HIGH) {
		currentTime = getTicks();
		if (currentTime - initialTime < 1000) {
			led7seg_setChar(SEGMENT_DISPLAY[i], TRUE);
		}
		else {
			initialTime = getTicks();
			if (i==0) {
				configLaunch();
				state.modeState = LAUNCH;
				break;
			}
	    	else {
				  i--;
	    	}
	    }
	}

	// Abort count down
	if (state.tempState == TEMP_HIGH) {
		led7seg_setChar(SEGMENT_DISPLAY[15], TRUE);
	}
}

//   **************************     EINT3 GPIO LIGHT, TEMP INTERRUPTS     **************************
void EINT3_IRQHandler(void)
{
	// Light Sensor Interrupt (Obstacle detection)
	if ((LPC_GPIOINT->IO2IntStatF >> 5) & 0x1) {
		if (state.lightState == LIGHT_NORMAL) {
			state.lightState = LIGHT_HIGH;
			light_change = 1;
			light_setHiThreshold(MAX_LIGHT);
			light_setLoThreshold(OBSTACLE_THRESHOLD);

		}
		else if (state.lightState == LIGHT_HIGH){
			state.lightState = LIGHT_NORMAL;
			obstacle_avoid = 1;
			light_setHiThreshold(OBSTACLE_THRESHOLD);
			light_setLoThreshold(0);
		}
			light_clearIrqStatus();
			LPC_GPIOINT->IO2IntClr = (1 << 5);
	}

	// Temp sensor Interrupt (Monitor Fuel Tank)
	if ((LPC_GPIOINT->IO0IntStatF >> 2) & 0x1) {

			adjusted_temp_read(&temp);

			if ((state.tempState == TEMP_NORMAL) && (temp.temperature > TEMP_THRESHOLD)) {
				state.tempState = TEMP_HIGH;
				temp_change = 1;
			}
			LPC_GPIOINT->IO0IntClr |= (1 << 2);
	}
}

//   **************************     LIGHT SENSOR FUNCTIONS    **************************

void light_sensor_enable(void) {
	state.lightState = LIGHT_NORMAL;
	LPC_GPIOINT->IO2IntEnF |= 1<<5;
}

void light_sensor_disable(void) {
	light_shutdown();
	LPC_GPIOINT->IO2IntEnF &= ~(1<<5);
	state.lightState = LIGHT_OFF;
}

//   **************************     TEMPERATURE SENSOR FUNCTIONS      **************************
void temp_disable(void) {
	LPC_GPIOINT->IO0IntEnF &= ~(1<<2);
	LPC_GPIOINT->IO0IntClr |= (1 << 2);
	state.tempState = TEMP_OFF;
}

void temp_enable(void) {
	LPC_GPIOINT->IO0IntEnF |= (1<<2);
	temp.halfPeriods = 0;
	temp.temperature = 0;
	temp.temperatureT1 = 0;
	temp.temperatureT2 = 0;
	state.tempState = TEMP_NORMAL;
}

void adjusted_temp_read(TEMP* temp) {
	if (!temp->temperatureT1 && !temp->temperatureT2) {
		temp->temperatureT1 = getTicks();
	} else if (temp->temperatureT1 && !temp->temperatureT2) {
		temp->halfPeriods++;
		if (temp->halfPeriods == TEMP_NUM_HALF_PERIODS/2) {
			temp->temperatureT2 = getTicks();
			if (temp->temperatureT2 > temp->temperatureT1) {
				temp->temperatureT2 = temp->temperatureT2 - temp->temperatureT1;
			}
			else {
				temp->temperatureT2 = (0xFFFFFFFF - temp->temperatureT1 + 1) + temp->temperatureT2;
			}
			temp->temperature = ((2*1000*temp->temperatureT2) / (TEMP_NUM_HALF_PERIODS*TEMP_SCALAR_DIV10) - 2731);
			temp->temperatureT1 = 0;
			temp->temperatureT2 = 0;
			temp->halfPeriods = 0;
		}
	}
}

//   ********************************    UART 3 INTERRUPT      ********************************
void UART3_IRQHandler(void) {
	 UART3_StdIntHandler();
}

void UART3_Rx_Interrupt(void) {

	if(UART_Receive(LPC_UART3, &msg_buf[buf_count], 1, NONE_BLOCKING) == 1) {
		 if(msg_buf[buf_count] == '\n'){
			message_received = 1;
		 }
			 buf_count++;

		 if (buf_count == 64)  {
			 buf_count = 0;
		 }
	}
}

void set_uart_message(uint8_t* UART_DISPLAY, char* string) {
	strcpy((char*)UART_DISPLAY, "");
	strcat((char*)UART_DISPLAY, string);
}

void set_uart_string(){
    msg_buf[buf_count-1] = '\0';
    buf_count = 0;
}

//   ***********************************    TIMER 0 INTERRUPT      ************************************
void TIMER0_IRQHandler(void) {
	// check if TIMER0 M0 interrupt
	if ((LPC_TIM0-> IR >> 0) & 0x1) {
		if (state.modeState == LAUNCH || state.modeState == RETURN) {
			telemetry_change = 1; // every 10seconds flag for telemetry data to be sent
		}
		LPC_TIM0->IR |= (1<<0); // clear TIMER0 interrupt
	}
}

void telemetryMsg(void) {
	char s[40] = "";
	if (telemetry_change) {
		if (state.modeState == LAUNCH) {
			sprintf(s, "TEMP: %.2f ACC X : %.2f ACC Y : %.2f\r\n", data.temp/10.0, data.acc_x, data.acc_y);
			set_uart_message(display.uart, s);
			UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
			telemetry_change = 0;
		}

		if (state.modeState == RETURN) {
			sprintf(s,  "Obstacle distance: %d\r\n", (int)data.light);
			set_uart_message(display.uart, s);
			UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
			telemetry_change = 0;
		}
	}
}

//   ***********************************    TIMER 1 INTERRUPT      ************************************
void TIMER1_IRQHandler(void) {
	// check if TIMER0 M0 interrupt
	if ((LPC_TIM1-> IR >> 0) & 0x1) {
		if (state.modeState == RETURN) {
			updateReturn(); // every 10seconds flag for telemetry data to be sent
		}
		LPC_TIM1->IR |= (1<<0); // clear TIMER0 interrupt
	}
}

//   ********************************    SW4 CLEAR_WARNING FUNCTION    ********************************

void checkClearWarning(void) {
	clear_warning = (GPIO_ReadValue(1) >> 31) & 0x01;

	if (clear_warning == 0) {

			if (state.tempState == TEMP_HIGH) {
				state.tempState = TEMP_NORMAL;
				temp_change = 1;
			}
			if (state.accState == ACC_HIGH) {
				state.accState = ACC_NORMAL;
				acc_change = 1;
			}
	}
}

void checkLightWarning(void) {
	if (state.lightState == LIGHT_HIGH) {
		state.lightState = LIGHT_NORMAL;
		light_setHiThreshold(OBSTACLE_THRESHOLD);
		light_setLoThreshold(0);
		set_uart_message(display.uart,  "Obstacle Avoided.\r\n");
		UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
	}

}

//   ********************************    OTHER PERIPHERAL FUNCTIONS    ********************************
void checkManualSettings(void) {

	uint8_t rState = 0; //rotary state
	uint8_t jState = 0; //joystick state

	rState = rotary_read();
	jState = joystick_read();

	char s[40] = "";

	if (state.tempState == TEMP_HIGH || state.accState == ACC_HIGH) {

		if (jState) {
			if(rState == ROTARY_LEFT) {
				if (state.accState == ACC_HIGH) {
					state.accState = ACC_NORMAL;
					acc_change = 1;
				}
				if (jState == JOYSTICK_UP) {
					if (ACC_THRESHOLD <= 0.7) {
						ACC_THRESHOLD += 0.1;
					}
				} else if (jState == JOYSTICK_DOWN) {
					if (ACC_THRESHOLD >= 0.3) {
						ACC_THRESHOLD -= 0.1;
					}
				}
				sprintf(s,  "New ACC_THRESHOLD = %.2f\r\n", ACC_THRESHOLD);
				set_uart_message(display.uart, s);
				UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
			} else if (rState == ROTARY_RIGHT) {

				if (state.tempState == TEMP_HIGH) {
					state.tempState = TEMP_NORMAL;
					temp_change = 1;
				}

				if (jState == JOYSTICK_UP) {
					if (TEMP_THRESHOLD <= 310) {
						TEMP_THRESHOLD += 10;
					}
				} else if (jState == JOYSTICK_DOWN) {
					if (TEMP_THRESHOLD >= 260) {
						TEMP_THRESHOLD -= 10;
					}
				}
				sprintf(s,  "New TEMP_THRESHOLD = %d\r\n", TEMP_THRESHOLD);
				set_uart_message(display.uart, s);
				UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
			}
		}
	}
}


//   ************************************   WITHIN MODE FUNCTIONS    ************************************

void configStationary(void) {

	// Entering for first time into stationary from return mode
	state.lightState = LIGHT_OFF;
	oled_clearScreen(OLED_COLOR_BLACK);
	set_uart_message(display.uart, "Entering STATIONARY Mode\r\n");
	UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);

	pca9532_setLeds(0x0000, 0xFFFF);

	// temp sensor enabled
	temp_enable();
	state.lightState = LIGHT_OFF;

	oled_putString(0, 0, (uint8_t *) "STATIONARY", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(0, 10, (uint8_t *) "TEMP" , OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	led7seg_setChar(SEGMENT_DISPLAY[15], TRUE);
}

void stationaryMode() {

	checkClearWarning();

	updateStationary();

	if (message_received) { // Message sent in wrong mode
		set_uart_message(display.uart,  "Report not enabled in this mode.\r\n");
		UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
		message_received = 0;
		buf_count = 0;
	}

	if (state.tempState == TEMP_HIGH) {
		if (temp_change) {
			oled_putString(0, 40, (uint8_t *) "Temp. too high", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			set_uart_message(display.uart,  "Temp. too high.\r\n");
			UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
			temp_change = 0;
		}

	} else if (temp_change) {
		oled_putString(0, 40, (uint8_t *) "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		temp_change = 0;
	}
}

void updateStationary(void) {
	data.temp = temp.temperature;
	char s[16] = "";
	sprintf(s, "%2.2f", data.temp/10.0);
	oled_putString(60, 10, (uint8_t *) s, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

void configLaunch(void) {
	 // Entering launch for first time from Stationary
	set_uart_message(display.uart, "Entering LAUNCH Mode\r\n");
	UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
	state.accState = ACC_NORMAL;

	oled_clearScreen(OLED_COLOR_BLACK);
	oled_putString(0, 0, (uint8_t *) "LAUNCH", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(0, 10, (uint8_t *) "TEMP" , OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(0, 20, (uint8_t *) "ACC" , OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	led7seg_setChar(SEGMENT_DISPLAY[0], TRUE);
}

void launchMode() {

	checkAcc();
	checkClearWarning();
	updateLaunch();
	checkLaunchMsg();

	if (state.tempState == TEMP_HIGH) {
		if (temp_change) {
			oled_putString(0, 40, (uint8_t *) "Temp. too high", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			set_uart_message(display.uart,  "Temp. too high.\r\n");
			UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
			temp_change = 0;
		}
	} else if (temp_change) {
		oled_putString(0, 40, (uint8_t *) "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		temp_change = 0;
	}

	if (state.accState == ACC_HIGH) {
		if (acc_change) {
			oled_putString(0, 50, (uint8_t *) "Veer off course", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			set_uart_message(display.uart,  "Veer off course.\r\n");
			UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
			acc_change = 0;
		}
	} else if(acc_change) {
		oled_putString(0, 50, (uint8_t *) "                ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		acc_change = 0;
	}
}

void checkAcc(void) {
	acc_read(&x, &y, &z);
	x = x + xoff;
	y = y + yoff;

	data.acc_x = fabs (x / ACC_DIV_MEASURE) ;
	data.acc_y = fabs (y / ACC_DIV_MEASURE) ;

	if (state.accState == ACC_NORMAL) {
		if ((data.acc_x >  ACC_THRESHOLD) || (data.acc_y >  ACC_THRESHOLD)) {
			state.accState = ACC_HIGH;
			acc_change = 1;
		}
	}
}

void updateLaunch(void) {

	data.temp = temp.temperature;
	char s[16] = "";
	sprintf(s, "%.2f", data.temp/10.0);
	oled_putString(60, 10, (uint8_t *) s, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	// after exiting measure, interrupt re-starts again

	char xy[16] = "";
	sprintf(xy, "%.2f, %.2f", data.acc_x, data.acc_y);
	oled_putString(30, 20, (uint8_t *) xy, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

void checkLaunchMsg(void) {

	char s[40] = "";
	if (state.tempState == TEMP_NORMAL && state.accState == ACC_NORMAL) {
		if (message_received) {
			message_received = 0;
			set_uart_string();
			if (strcmp(msg_buf, "RPT\r")==0) {
				sprintf(s,"TEMP: %.2f ACC X : %.2f ACC Y : %.2f\r\n",data.temp/10.0, data.acc_x, data.acc_y );
				set_uart_message(display.uart, s);
				UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
		    } else {
				set_uart_message(display.uart,  "Unknown message received.\r\n");
				UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
		    }
		}
	}else if (message_received) {
			set_uart_message(display.uart,  "Cannot report in warning state.\r\n");
			UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
			message_received = 0;
			buf_count = 0;
		}
}

void configReturn(void) {
	 // Entering Return for first time from launch
	set_uart_message(display.uart, "Entering RETURN Mode\r\n");
	UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);

	state.lightState = LIGHT_NORMAL;
	state.accState = ACC_OFF;
	temp_disable();

	oled_clearScreen(OLED_COLOR_BLACK);
	oled_putString(0, 0, (uint8_t *) "RETURN", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	// led7seg_setChar(SEGMENT_DISPLAY[0], TRUE); 7seg is the same from launch mode
}

void updateReturn(void) {
	data.light = light_read();
	data.brightness = getBrightness(data.light);
	pca9532_setLeds(data.brightness, 0xFFFF);
}

static uint16_t getBrightness(uint32_t light) {
	if (light < 242) {
		return 0x0000;
	} else if (light < 484) { // 1
		return 0x0003;
	} else if (light < 726) { // 2
		return 0x0007;
	} else if (light < 968) { // 3
		return 0x000F;
	} else if (light < 1210) { // 4
		return 0x001F;
	} else if (light < 1452) { // 5
		return 0x003F;
	} else if (light < 1694) { // 6
		return 0x007F;
	} else if (light < 1936) { // 7
		return 0x00FF;
	} else if (light < 2178) { // 8
		return 0x01FF;
	} else if (light < 2420) { // 9
		return 0x03FF;
	} else if (light < 2662) { // 10
		return 0x07FF;
	} else if (light < 2904) { // 11
		return 0x0FFF;
	} else if (light < 3146) { // 12
		return 0x1FFF;
	} else if (light < 3388) { // 13
		return 0x3FFF;
	} else if (light < 3630) { // 14
		return 0x7FFF;
	} else {
		return 0xFFFF;
	}
}

void returnMode() {

	if (state.lightState == LIGHT_NORMAL) {
		if (obstacle_avoid) {
			obstacle_avoid = 0;
			oled_putString(0, 10, (uint8_t *) "             ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			set_uart_message(display.uart,  "Obstacle Avoided.\r\n");
			UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
		}
	} else if (state.lightState == LIGHT_HIGH) {
		if (light_change) {
			light_change = 0;
			oled_putString(0, 10, (uint8_t *) "Obstacle near", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			set_uart_message(display.uart,  "Obstacle near.\r\n");
			UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
		}
	}

	checkReturnMsg();
}

void checkReturnMsg(void) {

	char x[40] = "";

	if (state.lightState == LIGHT_NORMAL) {
		if (message_received) {
			message_received = 0;
			set_uart_string();
			if (strcmp(msg_buf, "RPT\r") == 0 ) {
				sprintf(x,"Obstacle distance: %d\r\n", (int)data.light);
				set_uart_message(display.uart, x);
				UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
		    } else {
				set_uart_message(display.uart,  "Unknown message received.\r\n");
				UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
		    }
		}
	}else if (message_received) {
		set_uart_message(display.uart,  "Cannot report in warning state.\r\n");
		UART_Send(LPC_UART3, display.uart, strlen((char*)display.uart), BLOCKING);
		message_received = 0;
		buf_count = 0;
 	}
}


//   ******************************    ALL INITIALIZATIONS      ******************************

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);
}

void init_uart(void) {
	UART_CFG_Type uartCfg;

	UART_FIFO_CFG_Type fifoCfg;

	//uart pinsel
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);

	// init default UART config , BR 9600
    UART_ConfigStructInit(&uartCfg);
    // change baudrate to 115200
	uartCfg.Baud_rate = 115200;
	UART_Init(LPC_UART3, &uartCfg);

	UART_FIFOConfigStructInit(&fifoCfg);
	UART_FIFOConfig(LPC_UART3, &fifoCfg);

	UART_SetupCbs(LPC_UART3, 0, UART3_Rx_Interrupt); //Call back function
	UART_TxCmd(LPC_UART3, ENABLE);
	UART_IntConfig(LPC_UART3, UART_INTCFG_RBR, ENABLE);
}

void init_light_sensor(void) {
	light_enable();
	light_setRange(LIGHT_RANGE_16000);
	LPC_GPIOINT->IO2IntEnF |= (1<<5);
	light_setHiThreshold(OBSTACLE_THRESHOLD);
	light_clearIrqStatus();
}


void init_temp_sensor(void) {
	PINSEL_CFG_Type PinCfg;

	PinCfg.Funcnum = 0;
 	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	GPIO_SetDir(0, (1<<2), 0);
}

static void init_sw3(void) {
	//SW3 MODE_TOGGLE
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 1;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PINSEL_ConfigPin(&PinCfg);

	// Enable EINT0 Interrupt
	LPC_SC->EXTMODE |= 0x01;
	LPC_SC->EXTPOLAR &= 0xFFFE; // set EINT0 to trigger on falling edge
}

static void init_sw4(void) {
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
 	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;

	//SW4 CLEAR_WARNING
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	// (portnum, pinnum, in/out (0/1) )
	GPIO_SetDir(0, 1<<31, 0); // SW4
}

void init_timer0(void) {
	LPC_SC->PCONP |= (1<<1); //Power up TIMER0
	LPC_SC->PCLKSEL0 |= (0b01 << 2);
	LPC_TIM0->TCR = 2; // reset & hold TIMER0
	LPC_TIM0->MR0 = SystemCoreClock*10; // Intterupt every 10seconds
	LPC_TIM0->MCR |= 1<<0; // Interrupt on Match0 compare
	LPC_TIM0->MCR |= 1<<1; // reset TIMER0 on match 0
}

void init_timer1(void) {
	LPC_SC->PCONP |= (1 << 2); // power up TIMER1
	// PCLK_peripheral = SystemCoreClock
	LPC_SC->PCLKSEL0 |= (0b01 << 4);
	LPC_TIM1->TCR = 2; // reset & hold TIMER1
	LPC_TIM1->MR0 = (SystemCoreClock/4);// interrupt every 250 millisecond
	LPC_TIM1->MCR |= 1 << 0; // interrupt on Match1 compare
	LPC_TIM1->MCR |= 1 << 1; // reset TIMER1 on Match 1
}

void init_all() {

    init_i2c();
    init_ssp();
    init_uart();

	led7seg_init();
	rotary_init();
	joystick_init();
    pca9532_init();
    pca9532_setLeds(0x0000, 0xffff);
	rgb_init();
	oled_init();
	acc_init();
	acc_read(&x, &y, &z);
	xoff = 0 - x;
	yoff = 0 - y;
	oled_clearScreen(OLED_COLOR_BLACK);

	//EINT3_GPIO
	init_light_sensor();

    //EINT3_GPIO
	init_temp_sensor();

	//EINT0 MODE_TOGGLE
    init_sw3();

	//TIMER0
	init_timer0();

	//TIMER1
	init_timer1();

    // CLEAR_WARNING
	init_sw4();

	NVIC_SetPriorityGrouping(5);

	NVIC_SetPriority(SysTick_IRQn, 0x00);
	NVIC_SetPriority(EINT0_IRQn, 0x40);
	NVIC_SetPriority(TIMER1_IRQn, 0x48);
	NVIC_SetPriority(TIMER0_IRQn, 0x50);
	NVIC_SetPriority(EINT3_IRQn, 0x80);
	NVIC_SetPriority(UART3_IRQn, 0x88);

	NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_ClearPendingIRQ(UART3_IRQn);

    NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
    NVIC_EnableIRQ(TIMER0_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);
    NVIC_EnableIRQ(UART3_IRQn);

}

int main (void) {
    SysTick_Config(SystemCoreClock/1000);
	init_all();
	LPC_TIM0->TCR = 1;//start timer0
	LPC_TIM1->TCR = 1;//start timer1
	configStationary();
	telemetry_change = 0;

    while (1)
    {
    	telemetryMsg();
    	checkManualSettings();
    	if (mode_toggle) {
    		modeToggle();
    		mode_toggle = 0;
    	}
    	switch (state.modeState) {
    	case STATIONARY :
    		stationaryMode();
    		break;

    	case LAUNCH :
    		launchMode();
    		break;

    	case RETURN :
    		returnMode();
    		break;
    	}
    }
}


