/************************************************************************************/
/* This is the marelab OSMOSIS Control												*/
/* The firmeware works in a standalone mode with RS485 Interface					*/
/* - RS485																			*/
/* - MODBUS																			*/
/* 																					*/
/* - I2C Controlling 3xMosfet Switch for DC24V										*/
/* - I2C 12Bit ADC for Water Quality 												*/
/* - 4 Input I/O for Sensors														*/
/*																					*/
/* How does it work																	*/
/* First the Sensor for LowLevel Water Tank is checked if its FALSE = 0				*/
/* then the MosfetSwitch1 is turn ON. This is activating a 24V pressure			    */
/* pump and a 24V Water in Vent and the Water Clear Vent ON MosfetSwitch2.			*/
/* The WaterCleanVent is opened for 20sec then closed. The Mosfet1 keeps ON			*/
/* until WaterLevel Switch2 is turn ON.												*/
/*																					*/
/* PumpOsmose Pump that raise the FreshWater preasure								*/
/* PumpRefill Pump that moves water from Osmoses									*/
/* VentFresh  Vent for Fresh water to Osmoses										*/
/* VentClean  Vent to clen the osmoses membran										*/	
/*																					*/
/* Logic:																			*/
/* SAQUA PumpRefill:																*/
/* The PumpeRefill is only active when SOLow = High && SAQUA = 0 the signal must be	*/
/* stable over the amount of time before it is switched.							*/
/*																					*/
/* Osmosis water:																	*/
/* The Osmosis Pump & Vent is active when the Sensor changes from last state		*/
/* SOHigh = active to low and next state change is SOLow = low. The direction of	*/
/* the state change is used to determine if is goes from High water to Low. Or from	*/
/* Low Water to High. Only if the state change is done from High water to low water	*/
/* the osmosis system is activated.													*/
/*																					*/	
/* Osmosis cleaning:																*/
/* The osmosis cleaning is done first osmosis starts and then after (n) Minutes		*/
/* in repeat as long as Osmosis water is produced. The Duration of cleaning and		*/
/* periodic time is configurable.													*/
/*																					*/
/* Safety mechanism:																*/
/* First if Sensor conditions are found that aren't possible in operation all is	*/
/* shutdown. Displays shows Error.													*/
/* Beside the detectable Sensor conditions additional timings are used				*/
/* to minimize the risk. For the Tank Refill a duration can be configured that		*/
/* if the tank refill run exceeds that time the hole process and device is stopen	*/
/* The same mechanism is used for a Osmosis run from SensorLow to SensorHigh. If	*/
/* these values are configured right a defected Sensor will not have a dramatic		*/
/* impact on the Tank or a overflow of the Osmosis Tank.							*/
/*																					*/
/* Logic Table to control Pumps & Vents												*/
/* ---------------------------------------------------------------------------------*/	
/*					SOHigh	SOLow	SAQUA											*/
/* Osmose Empty		  0		  0			0   PumpOsmose VentFresh ON PumpRefill OFF	*/
/* Osmose Empty		  0	  	  0			1   PumpOsmose VentFresh ON PumpRefill OFF	*/
/* Osmose OK		  0		  1			0   PumpOsmose VentFresh ON PumpRefill ON	*/
/* Osmose OK		  0		  1			1	All OFF IDLE							*/
/* ERROR			  1		  0			0   All OFF									*/
/* ERROR		      1		  0			1   All OFF									*/
/* Osmose OK		  1		  1			0   PumpOsmose VentFresh OFF PumpRefill ON	*/
/* Osmose OK		  1		  1			1   PumpOsmose VentFresh OFF PumpRefill OFF	*/
/************************************************************************************/
/* OSMOSE CLEANING
/* OSMOSE ON -> N min CleanVent open -> n min production -> CleanVent Open N min    */
/* Screen Design STATUS		                                                        */
/*																					*/
/* Overview   WaterQuaily         													*/
/* -------------------------------------											*/
/* Mode		: OSMOSE CLEAN REFILL ERROR												*/
/* WaterQuaily : 0012mS																*/
/* SHigh=ON SLow=ON SAqua=ON 														*/
/* -------------------------------------											*/
/* Auto Osmose Refill Setup															*/
/*																					*/
/************************************************************************************/
/* Screen Design SETUP		                                                        */
/*																					*/
/* SETUP   WaterQuaily         														*/
/* -------------------------------------											*/
/* SENSOR OSMOSE (setup sensor timeoffset for stable value)							*/
/* SENSOR TANK	 (setup sensor timeoffset for stable value)							*/
/* CLEAN		 (Duration of Cleaning)												*/
/* MODBUS 		 (MODBUS Setup)														*/
/* -------------------------------------											*/
/* BACK																				*/
/*																					*/
/************************************************************************************/
/* Screen Design SENSOR OSMOSE	                                                    */
/*																					*/
/* SETUP   WaterQuaily         														*/
/* -------------------------------------											*/
/* SENSOR OSMOSE (setup sensor timeoffset for stable value)							*/
/* SENSOR TANK	 (setup sensor timeoffset for stable value)							*/
/* CLEAN		 (Duration of Cleaning)												*/
/* MODBUS 		 (MODBUS Setup)														*/
/* -------------------------------------											*/
/* BACK																				*/
/*																					*/
/************************************************************************************/



	


#include <stdlib.h>
//#include "Arduino.h"
#include <Wire.h>
#include <EEPROM.h>

#include "EEPROMAnything.h"
#include "libmodbus/Modbusino.h"
#include "OneWire/OneWire.h"
#include <avr/pgmspace.h>
//#include "TimerOne.h"
#include "marelab_dimm.h"
#include "OLED.h"

// Pins used By Module
#define RESET_PIN 				2	/* THIS PI HAS TO SET HIGH if LOW RESET is activate					*/
#define TX_RS485_PIN            4   /* Pin to enable arduino to write to RS485							*/
#define BUTTON_ANALOG_PIN		0   /* the number of the analog pi the buttons are connected to			*/
#define LED_PIN				    3   /* Pin to switch on LED												*/

// Osmose Control
#define SOHIGH				    6   /* Sensor Omsmose Tank High											*/
#define SOLOW				    7   /* Sensor Omsmose Tank Low											*/
#define SAQUA					8   /* Sensor Reef Tank Water Hight										*/
struct TSENSOR{
	uint8_t			S_PORT;
	unsigned long	S_DBOUNCE;
	unsigned long	S_STABLE;
	boolean			S_LAST_STATE;
	boolean			S_OK;
};
struct TSENSOR AQUA;
struct TSENSOR FWTANK_TOP;
struct TSENSOR FWTANK_BOT;
uint8_t GlobalStateSensor;			/* AllSensors as Byte */
uint8_t GlobalStateSensorOld;
#define OSMOSE_MODE_AUTO     1
#define OSMOSE_MODE_OSMOSE   2
#define OSMOSE_MODE_REFILL   3

unsigned long OSMOSE_CLEAN_RUNTIME;
unsigned long OSMOSE_CLEAN_REPEATTIME;

boolean osmose_active;
boolean pump_active;
boolean osmose_active_firsttime;
boolean CleanOn;
unsigned long OSMOSE_START_TIME;
unsigned long OSMOSE_RUN_TIME;

// Software Safty feature to handle mailfunction of Sensors
unsigned long OSMOSE_MAX_RUNTIME;			// Max duration between low and high sensor if > OSMOSE to ERROR
unsigned long OSMOSE_MAX_PUMP_RUNTIME;		// Max Duration between Refill start and Sensor Tank Signal if > ERROR

uint8_t OSMOSE_MODE;



// Modbus Settings
#define MODBUS_BAUD			115200	/* Modbus Baud */
#define MARELAB_TYPE			1	/* Set the device ID to get identified 								*/
#define MARELAB_FIRWMARE  		2
#define MODBUS_REGISTER_COUNT 	31  /* COUNT of Modbus to Led Registers starting with 0					*/

	

/* These are the Modbus Registers where we can read & write to        */
/* Mapping Modbus Register to LED Lamp Functions */
#define MARELAB_DEVICE_ID 		0	/* Device ID to identify marelab 	*/
#define MARELAB_VERSION			1	/* Firmware ID to identify 		*/
#define MARELAB_REGISTER_COUNT  2
#define LED_COLOR1 				3	/* DIM Value channel 1			*/
#define LED_COLOR2 				4
#define LED_COLOR3 				5
#define LED_COLOR4 				6
#define LED_COLOR5 				7
#define LED_COLOR6 				8
#define LED_COLOR7 				9
#define LED_COLOR8 				10
#define LED_COLOR9 				11
#define LED_COLOR10 			12
#define LED_COLOR11 			13
#define LED_COLOR12 			14
#define LED_COLOR13 			15
#define LED_COLOR14 			16
#define LED_COLOR15 			17
#define LED_COLOR16 			18
#define DIMM_TEMPERATUR 		19	/* Chassi temperature of the dimmer	*/
#define UNIX_DATE1				20	/* Timestap high of Dimmer RTC		*/
#define UNIX_DATE2				21	/* Timestap low  of Dimmer RTC		*/
#define MCOMMAND        		22
/*
 0 nothing
 1 free
 2 setRTC Time
 10 setPWM fills 20-26
 11 getPWM fills 20-26 Two reads
 20 getTemp Sensor 1
 21 getTemp Sensor 2
 */
#define MDIM_START_MIN   		23  	/* defines startpoint in sec. PWM val = 0 */
#define MDIM_START_MAX   		24 		/* defines startendpoint in sec.  */
#define MDIM_START_VALUE 		25 		/* PWM value for startendpoint */
#define MDIM_END_MAX     		26 		/* defines endpoint in sec. */
#define MDIM_END_VALUE   		27 		/* PWM value for endpoint */
#define MDIM_END_MIN     		28 		/* defines endpoint in sec. PWM val = 0 */
#define MDIM_CHANNEL     		29 		/* the dimchannel the values belongs to */
#define MDIM_MODUS              30      /* CONTROLLED BY 01 STANDALONE /  00 NETWORK WLAN/MOD / 02 NETWORK WLAN /04 NETWORK WLAN */

enum ACCESS_MODE {
	ACC_WLAN_MODBUS = 0, ACC_STANDALONE, ACC_MOD, ACC_WLAN
};

/* WATCHDOG DEFENITIONS */
#define WATCHDOG_OFF    (0)
#define WATCHDOG_8S     (_BV(WDP3) | _BV(WDP0) | _BV(WDE))

#define I2C_ADRESS_MODBUS_ADR	0x38

uint16_t mb_reg[MODBUS_REGISTER_COUNT];

#define OLED_RESET 		4
#define NUMFLAKES 		10
#define XPOS 			0
#define YPOS 			1
#define DELTAY 			2


OLED screen; /* marelab Graphic Lib 				*/


uint8_t WhichDisplay = 0; /* Global Graphic Display State 	*/

const int ledPin = 13;

unsigned int MODCODES = 0;

OneWire ds(10);


/********************************************************************************************/
/*  Multiple buttons on an analog input for a 5 Botton joystick switch                      */
/*  This will read the value from the analog pin and report when a button was pressed. A    */
/*  button press will only be reported once the button is released.                         */
/********************************************************************************************/

// The button related constants
#define BUTTONS_NUMBER  5
enum BUTTON_STATE {
	BT_NULL = 0, BT_ENTER, BT_LEFT, BT_RIGHT, BT_UP, BT_DOWN
};
//const uint8_t button[] = {1,2,3,4,5};
const uint16_t buttonLowRange[] = { 0, 1000, 900, 800, 650, 570 };
const uint16_t buttonHighRange[] = { 100, 1024, 950, 870, 720, 630 };
//									Middle/LEFT/RIGHT/ UP / DOWN
// Variables that will change:
uint8_t buttonState;             // the current reading from the input pin
uint8_t lastButtonState = BT_NULL;   // the previous reading from the input pin
uint16_t reading;
uint8_t MENU_POS;
uint8_t KEYBOARD_POS_X;
uint8_t KEYBOARD_POS_Y;
/********************************************************************************************/
/* DISPLAYS                                                                                 */
/********************************************************************************************/
enum MENU_DISPLAYS {
	DS_LOGO = 0,
	DS_MAIN,
	DS_BOOT,
	DS_MENU,
	DS_MODBUS_SETUP,
	DS_TIME_SETUP,
	DS_IP_SETUP,
	DS_DIMM,
	DS_INFO,
	DS_SCREENSAVER,
	DS_KEYBOARD,
	DS_KEYBOARD2
};

enum MENU_IP_POS {
	IP_POS_ACTIVE = 1,
	IP_POS_SSID = 2,
	IP_POS_WPA = 3,
	IP_POS_BACK = 4,
	IP_POS_SAVE = 5
};
uint8_t MODBUS_TEMP_ID = 0;	// its needed if the ModBus ID isn't chg over display
#define MENU_SIZE_SETUP  6		   // Modbus/Time/Ip/Dimm/Info/Back
#define MENU_SIZE_MODBUS_SETUP  4  // Modbus ID/ ACTIVE CHECKBOX/BACK/SAVE
#define MENU_SIZE_TIME          5  // HH/MM/SS BACK SAVE
#define MENU_SIZE_IP            5  // ON-OFF/IP1/IP2/IP3/IP4/BACK/SAVE
#define MENU_SIZE_DIMM          13 // ON-OFF/IP1/IP2/IP3/IP4/BACK/SAVE
#define MENU_SIZE_INFO          1  // ON-OFF/IP1/IP2/IP3/IP4/BACK/SAVE

#define SCREEN_SAVER_TIMEOUT    120 // Time until screensaver is activated
unsigned long tempTime = 0;		// Transfer Time Object between GUI & EEPROM
//char tempSSID[19]	= {' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
//char tempWPA[19] 	= {' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
uint8_t CURSOR = 0;

/********************************************************************************************/

struct DimChannel {
	uint16_t DIM_START_MIN;
	uint16_t DIM_START_MAX;
	uint16_t DIM_START_VALUE;
	uint16_t DIM_END_MAX;
	uint16_t DIM_END_VALUE;
	uint16_t DIM_END_MIN;
};

struct MarelabEEPROM {
	uint8_t MARELAB_MODUS; /* CONTROLLED BY 01 STANDALONE /  00 NETWORK WLAN/MOD / 02 NETWORK MOD /04 NETWORK WLAN */
	uint8_t MODBUS_ID; /* Slave ID of this device 			                                                    */
	char tempSSID[19]; /* MAX 32 Bytes                                                                         */
	char tempWPA[19]; /* MAX 63 Bytes                                                                         */
};

MarelabEEPROM ee_Config;
DimChannel DimChannelSetup;
uint8_t Channel = 0;

/***********************************************************/
/* READ EEPROM MARLAB CONFIG                               */
/***********************************************************/
void ReadMarelabConfig() {
	EEPROM_readAnything(512, ee_Config);
	mb_reg[MDIM_MODUS] = ee_Config.MARELAB_MODUS;
}
/***********************************************************/
/* WRITE EEPROM MARLAB CONFIG                               */
/***********************************************************/
void WriteMarelabConfig() {
	EEPROM_writeAnything(512, ee_Config);
}

/*
void TimeToModBusRegister(DateTime time) {
	const long LOW_MASK = ((1L << 32) - 1);
	mb_reg[UNIX_DATE1] = (uint16_t) (time.unixtime() >> 16);
	mb_reg[UNIX_DATE2] = (uint16_t) (time.unixtime() & LOW_MASK);
}
*/



// For 128x64 Pixel
/*
void DrawLedGraph(void) {
	uint8_t yh = 40;   // Max height of a bar
	uint8_t y = 0;
	uint16_t yscale = 1024 / yh;
	uint16_t reg;

	screen.DrawLine(20, 63, 20, (63 - yh));
	screen.DrawLine(18, 63, 127, 63);  //0 %
	screen.Text(0, 56, "0");
	screen.DrawLine(18, 43, 127, 43);  //50 %
	screen.Text(0, 40, "50");
	screen.DrawLine(18, 23, 127, 23);  //100 %
	screen.Text(0, 20, "100");
	screen.DrawLine(20, 63, 127, 63);
	screen.DrawLine(127, 63, 127, (63 - 40));
//*  display.fillRect(22, 30, 94, 31 , BLACK);
	for (uint8_t i = LED_COLOR1; i < 19; i++) {
		// scaling the 10Bit 1024 Value to y achse
		reg = mb_reg[i];
		y = reg / yscale;
		//screen.DrawFilledRect(((i-LED_COLOR1)*6)+22, yh+(yh-y), 4, y );
		screen.DrawFilledRect(((i - LED_COLOR1) * 6) + 26, 63 - y, 4, 63);
	}

}
*/

void drawInt(uint8_t x, uint8_t y, uint16_t value, uint8_t place, bool invers) {
	char buffer[4];

	itoa(value, buffer, 10);
	if (((place == 2) && (value < 10))) {
		if (invers) {
			screen.TextI(x, y, "0");
			screen.TextI(x + 6, y, buffer);
		} else {
			screen.Text(x, y, "0");
			screen.Text(x + 6, y, buffer);
		}
	} else if ((place == 3) && (value < 10)) {
		if (invers) {
			screen.TextI(x, y, "00");
			screen.TextI(x + 12, y, buffer);
		} else {
			screen.Text(x, y, "00");
			screen.Text(x + 12, y, buffer);
		}
	} else if ((place == 3) && (value > 9) && (value < 100)) {
		if (invers) {
			screen.TextI(x, y, "0");
			screen.TextI(x + 6, y, buffer);
		} else {
			screen.Text(x, y, "0");
			screen.Text(x + 6, y, buffer);
		}
	} else {
		if (invers) {
			screen.TextI(x, y, buffer);
		} else {
			screen.Text(x, y, buffer);
		}
	}

}

void DisplayStatus() {
	char buffer[4];
	// ICON STATUS BAR

	if (ee_Config.MARELAB_MODUS == ACC_WLAN_MODBUS) {
		screen.DrawXBM(112, 0, 16, 16, ICON_MODBUS);
		screen.DrawXBM(95, 0, 16, 16, ICON_WLAN);
	} else if (ee_Config.MARELAB_MODUS == ACC_STANDALONE) {
		screen.DrawXBM(112, 0, 16, 16, ICON_STANDALONE);
	} else if (ee_Config.MARELAB_MODUS == ACC_MOD) {
		screen.DrawXBM(112, 0, 16, 16, ICON_MODBUS);
	} else if (ee_Config.MARELAB_MODUS == ACC_WLAN) {
		screen.DrawXBM(112, 0, 16, 16, ICON_WLAN);
	}

	// Temperatur of Dimmer Hardware
	itoa(GlobalStateSensor, buffer, 10);
	screen.Text(60, 40, buffer);
	screen.Text(1, 40, "State");

	if (OSMOSE_MODE == OSMOSE_MODE_AUTO){
		screen.Text(1, 5, "AUTO");
	}

	if (AQUA.S_LAST_STATE && AQUA.S_OK){
		screen.Text(1, 56, "A-1");
	}else if (!AQUA.S_LAST_STATE && AQUA.S_OK){
		screen.Text(1, 56, "A-0");
	}else{
		screen.Text(1, 56, "A-?");
	}
	
	if (FWTANK_BOT.S_LAST_STATE && FWTANK_BOT.S_OK){		
		screen.Text(30, 56, "B-1");
	}else if (!FWTANK_BOT.S_LAST_STATE && FWTANK_BOT.S_OK){	
		screen.Text(30, 56, "B-0");
	}else{
		screen.Text(30, 56, "B-?");
	}
	
	if (FWTANK_TOP.S_LAST_STATE && FWTANK_TOP.S_OK){
		screen.Text(60, 56, "T-1");
	}else if (!FWTANK_TOP.S_LAST_STATE && FWTANK_TOP.S_OK){
		screen.Text(60, 56, "T-0");
		//MODE OSMOSE
	}else{
		screen.Text(60, 56, "T-?");
	}
	
		 /* Logic Table to control Pumps & Vents												*/
		 /* ---------------------------------------------------------------------------------*/
		 /*					SOHigh	SOLow	SAQUA											*/
		 /*0 Osmose Empty	  0		  0			0   PumpOsmose VentFresh ON PumpRefill OFF	*/
		 /*1 Osmose Empty	  0	  	  0			1   PumpOsmose VentFresh ON PumpRefill OFF	*/
		 /*2 Osmose OK		  0		  1			0   PumpOsmose VentFresh ON PumpRefill ON	*/
		 /*3 Osmose OK		  0		  1			1	PumpOsmose VentFresh OFF PumpRefill ON	*/
		 /*4 ERROR			  1		  0			0   All OFF									*/
		 /*5 ERROR		      1		  0			1   All OFF									*/
		 /*6 Osmose OK		  1		  1			0   PumpOsmose VentFresh OFF PumpRefill ON	*/
		 /*7 Osmose OK		  1		  1			1   PumpOsmose VentFresh OFF PumpRefill OFF	*/
	
	//if (AQUA.S_OK && FWTANK_TOP.S_OK & FWTANK_BOT.S_OK){
	//}else{
	//	GlobalStateSensor = GlobalStateSensorOld;
	//}
	
	if (OSMOSE_MODE == OSMOSE_MODE_AUTO){
		if (GlobalStateSensor==0)		{ osmose_active = TRUE;pump_active=FALSE;}		// PumpOsmose VentFresh ON PumpRefill OFF
		else if (GlobalStateSensor==1)	{ osmose_active = TRUE;pump_active=TRUE;}		// PumpOsmose VentFresh OFF PumpRefill OFF
		else if (GlobalStateSensor==2 && GlobalStateSensorOld != GlobalStateSensor )	// PumpOsmose VentFresh ON PumpRefill ON
		{
			if (GlobalStateSensorOld==6 || GlobalStateSensorOld == 7){ // wenn von voll nach leer dann nicht aktivieren
				osmose_active	= FALSE;
				pump_active		= FALSE;
			}
			else{
				osmose_active	= TRUE;
				pump_active		= FALSE;
			}
		}		
		
		else if (GlobalStateSensor==3 && GlobalStateSensorOld != GlobalStateSensor)	// PumpOsomse VentFresh OFF PumpRefill ON
		{
			if (GlobalStateSensorOld==6 || GlobalStateSensorOld == 7){ // wenn von voll nach leer dann nicht aktivieren
				osmose_active = FALSE;
				pump_active=FALSE;
			}
			else{
				osmose_active = TRUE;
				pump_active=FALSE;
			}
		}		
		else if (GlobalStateSensor==4)	{screen.Text(1, 18, "ERROR Sensor");  osmose_active = FALSE;pump_active=FALSE;}		// All OFF ERROR
		else if (GlobalStateSensor==5)	{screen.Text(1, 18, "ERROR Sensor");  osmose_active = FALSE;pump_active=FALSE;}		// All OFF ERROR
		else if (GlobalStateSensor==6)	{osmose_active = FALSE;pump_active=TRUE;}		// PumpOsmose VentFresh OFF PumpRefill ON
		else if (GlobalStateSensor==7)	{osmose_active = FALSE;pump_active=FALSE; }	// PumpOsmose VentFresh OFF PumpRefill OFF
		GlobalStateSensorOld = GlobalStateSensor;
	}
	
	if (osmose_active ){
		screen.Text(1, 18, "Osmose-On");
	}else
	{
		screen.Text(1, 18, "Osmose-Off");
	}
	if (pump_active ){
		screen.Text(65, 18, "Pump-On");
	}else
	{
		screen.Text(65, 18, "Pump-Off");
	}

 
    uint16_t run = OSMOSE_RUN_TIME/1000;
	ltoa(run, buffer, 10);
	screen.Text(1, 27, "OS");
	screen.Text(20, 27, buffer);
	
	if (CleanOn && osmose_active)
		screen.Text(60, 27, "CLEAN");
	else if (!CleanOn && osmose_active)
		screen.Text(60, 27, "OSMOSE");
	else
		screen.Text(60, 27, "IDLE");
		
	screen.DrawLine(0, 16, 128, 16);
	screen.DrawLine(0, 54, 128, 54);
	
	// RUNTIME
	unsigned long runtime = millis()/1000;
	ltoa(runtime, buffer, 10);
	screen.Text(70, 40, "S");
	screen.Text(75, 40, buffer);
	// Graph of 16 LEDs
	//DrawLedGraph();
	screen.Show();
}

void draw_Menu() {
	char buffer[4];
	screen.TextP(0, 5, T_SETUP, false);
	screen.DrawLine(0, 14, 128, 14);

	itoa(MENU_POS, buffer, 10);
	screen.Text(85, 5, buffer);

	if (MENU_POS == 1) {
		screen.TextP(8, 20, T_MODBUS_ADRESS, true);
	} else {
		screen.TextP(8, 20, T_MODBUS_ADRESS, false);
	}

	if (MENU_POS == 2) {
		screen.TextP(8, 30, T_TIME_SETUP, true);
	} else {
		screen.TextP(8, 30, T_TIME_SETUP, false);
	}

	if (MENU_POS == 3) {
		screen.TextP(8, 40, T_IP_ADRESS, true);
	} else {
		screen.TextP(8, 40, T_IP_ADRESS, false);
	}

	if (MENU_POS == 4) {
		screen.TextP(60, 20, T_DIMM_SETUP, true);
	} else {
		screen.TextP(60, 20, T_DIMM_SETUP, false);
	}

	if (MENU_POS == 5) {
		screen.TextP(60, 30, T_INFO_SETUP, true);
	} else {
		screen.TextP(60, 30, T_INFO_SETUP, false);
	}

	if (MENU_POS == 6) {
		screen.TextP(8, 50, T_BACK, true);
	} else {
		screen.TextP(8, 50, T_BACK, false);
	}
}

void drawModbusSetup() {
	char buffer[4];
	screen.TextP(0, 5, T_SETUP, false);
	screen.TextP(50, 5, T_MODBUS_ADRESS, false);
	screen.DrawLine(0, 14, 128, 14);

	screen.TextP(0, 20, T_MODSETUP_D, false);
	screen.TextP(8, 35, T_MODBUS_ID, false);

	// Get the ModbusID
	itoa(MODBUS_TEMP_ID, buffer, 10);
	if (MENU_POS == 1) {
		screen.TextI(30, 35, buffer);
		//!!!!!ICONS FOR LEF RIGHT
		//screen.DrawLine(24, 45, 42, 45);
	} else {
		screen.Text(30, 35, buffer);
	}

	if ((ee_Config.MARELAB_MODUS == ACC_WLAN_MODBUS)
			|| (ee_Config.MARELAB_MODUS == ACC_MOD)) {
		screen.DrawRect(100, 35, 10, 10);
		screen.DrawFilledRect(102, 37, 6, 6);
	} else {
		screen.DrawRect(100, 35, 10, 10);
	}

	if (MENU_POS == 2) {
		screen.TextP(52, 35, T_ACTIVE, true);

	} else {
		screen.TextP(52, 35, T_ACTIVE, false);
		screen.DrawRect(100, 35, 10, 10);
	}

	if (MENU_POS == 3) {
		screen.TextP(8, 50, T_BACK, true);
	} else {
		screen.TextP(8, 50, T_BACK, false);
	}

	if (MENU_POS == 4) {
		screen.TextP(100, 50, T_SAVE, true);
	} else {
		screen.TextP(100, 50, T_SAVE, false);
	}
}



void drawTime(uint8_t xt1, uint8_t yt1, uint8_t menu, unsigned long hh,unsigned long mm,unsigned long ss,bool hhmm) {
	if (MENU_POS == menu) {
		drawInt(xt1, yt1, hh, 2, true);
	} else {
		drawInt(xt1, yt1, hh, 2, false);
	}
	screen.Text(xt1 + 12, yt1, ":");
	if (MENU_POS == (menu + 1)) {
		drawInt(xt1 + 18, yt1, mm, 2, true);
	} else {
		drawInt(xt1 + 18, yt1, mm, 2, false);
	}
	if (hhmm) {
		screen.Text(xt1+30, yt1, ":");

		if (MENU_POS == 3) {
			drawInt(xt1+36, yt1, ss, 2, true);
		} else {
			drawInt(xt1+36, yt1, ss, 2, false);
		}
	}
}

/*
void drawTimeSetup() {

	uint8_t hh, mm, ss;

	screen.TextP(0, 5, T_SETUP, false);
	screen.TextP(50, 5, T_TIME_SETUP, false);
	screen.DrawLine(0, 14, 128, 14);

	TimeSpan timeObj(tempTime);
	hh = timeObj.hours();
	mm = timeObj.minutes();
	ss = timeObj.seconds();

	// Get the ModbusID
	drawTime(30, 30, 1, hh, mm,ss,true);



	if (MENU_POS == 4) {
		screen.TextP(8, 50, T_BACK, true);
	} else {
		screen.TextP(8, 50, T_BACK, false);
	}

	if (MENU_POS == 5) {
		screen.TextP(100, 50, T_SAVE, true);
	} else {
		screen.TextP(100, 50, T_SAVE, false);
	}
}
*/

/*****************************************************************/
/* Customize Dimmer                                              */
/*****************************************************************/


void drawInfo() {
	char buffer[15];
	screen.TextP(0, 5, T_SETUP, false);
	screen.TextP(50, 5, T_INFO_SETUP, false);
	screen.DrawLine(0, 14, 128, 14);

	// Display Information
	// Marelab TypeID
	// Marelab Software ID
	// WLAN & MODBUS Status

	// TypeID
	screen.TextP(8, 20, T_MARELAB, false);
	screen.TextP(50, 20, T_MODBUS_ID, false);
	itoa(mb_reg[MARELAB_DEVICE_ID], buffer, 10);
	screen.Text(80, 20, buffer);

	// Software Firmware Version
	screen.TextP(8, 30, T_FIRMWARE, false);
	screen.TextP(58, 30, T_MODBUS_ID, false);
	itoa(MARELAB_DEVICE_ID, buffer, 10);
	screen.Text(80, 30, buffer);

	// Modbus ID
	screen.TextP(8, 40,T_MODBUS_ADRESS, false);
	screen.TextP(48, 40,T_MODBUS_ID, false);
	itoa(ee_Config.MODBUS_ID, buffer, 10);
	screen.Text(80, 40, buffer);


	//screen.Text(26,53,wifi.showIP());

	// Display wich technics can be used to acess marelab
	/*
	 * CONTROLLED BY
	 * 01 STANDALONE /
	 * 00 NETWORK WLAN/MOD /
	 * 02 NETWORK WLAN /
	 * 04 NETWORK WLAN
	 */

	if (ee_Config.MARELAB_MODUS == ACC_WLAN_MODBUS) {
		screen.DrawXBM(108, 20, 16, 16, ICON_MODBUS);
		screen.DrawXBM(108, 40, 16, 16, ICON_WLAN);
	} else if (ee_Config.MARELAB_MODUS == ACC_STANDALONE) {
		screen.DrawXBM(108, 20, 16, 16, ICON_STANDALONE);

	} else if (ee_Config.MARELAB_MODUS == ACC_MOD) {
		screen.DrawXBM(108, 20, 16, 16, ICON_MODBUS);
	} else if (ee_Config.MARELAB_MODUS == ACC_WLAN) {
		screen.DrawXBM(108, 20, 16, 16, ICON_WLAN);
	} else if (ee_Config.MARELAB_MODUS > ACC_WLAN) { // Undifined ERROR no ACCESS MODE

	}

	if (MENU_POS == 1) {
		screen.TextP(8, 53, T_BACK, true);
	} else {
		screen.TextP(8, 53, T_BACK, false);
	}
}



void drawKeyboard() {
	uint8_t x, y;
	screen.TextP(0, 5, T_SETUP, false);
	screen.TextP(50, 5, T_IP_ADRESS, false);
	screen.DrawLine(0, 14, 128, 14);

	screen.TextP(0, 20, ASCII_1, false);
	screen.TextP(0, 30, ASCII_2, false);
	screen.TextP(0, 40, ASCII_3, false);
	screen.TextP(0, 50, ASCII_4, false);

	x = KEYBOARD_POS_X * 6;
	y = KEYBOARD_POS_Y * 10;
	screen.DrawLine(x, y + 20, x + 6, y + 20);
	screen.DrawLine(x, y + 29, x + 6, y + 29);
}
void drawKeyboard2() {
	uint8_t x, y;
	screen.TextP(0, 5, T_SETUP, false);
	screen.TextP(50, 5, T_IP_ADRESS, false);
	screen.DrawLine(0, 14, 128, 14);

	screen.TextP(0, 20, ASCII_1_2, false);
	screen.TextP(0, 30, ASCII_2_2, false);

	x = KEYBOARD_POS_X * 6;
	y = KEYBOARD_POS_Y * 10;
	screen.DrawLine(x, y + 20, x + 6, y + 20);
	screen.DrawLine(x, y + 29, x + 6, y + 29);

}

void draw(void) {
	screen.firstPage();
	do {
		switch (WhichDisplay) {
		case DS_LOGO: {	// Logo
			screen.DrawXBM(10, 20, 104, 20, marelab_logo);
			screen.Show();
			break;
		}
		case DS_MAIN: {	// Main Display
			DisplayStatus();
			break;
		}
		case DS_BOOT: {	// Boot Display
			screen.TextP(35, 30, T_BIOS_UPDATE, false);
			break;
		}
		case DS_MENU: {	// Main Menu
			draw_Menu();
			break;
		}
		case DS_MODBUS_SETUP: {	// Modbus Change
			drawModbusSetup();
			break;
		}
		case DS_TIME_SETUP: {	// Time Setup
			//drawTimeSetup();
			break;
		}
		case DS_IP_SETUP: {	// Modbus Change	
			break;
		}
		case DS_DIMM: {	// IP Setup
			break;
		}
		case DS_INFO: {	// IP Setup
			drawInfo();
			break;
		}
		case DS_SCREENSAVER: {
			break;
		}
		case DS_KEYBOARD: {
			drawKeyboard();
			break;
		}
		case DS_KEYBOARD2: {
			drawKeyboard2();
			break;
		}
		}
	} while (screen.nextPage());

}

void checkActiveButtons() {
	// read the state of the switch into a local variable:
	reading = analogRead(BUTTON_ANALOG_PIN);
	uint8_t tmpButtonState = BT_NULL;
	//= BT_NULL;               // The current reading from the input pin

	for (uint8_t i = 0; i <= BUTTONS_NUMBER; i++) {
		if (reading >= buttonLowRange[i] && reading <= buttonHighRange[i]) {
			tmpButtonState = i;
			break;
		}
	}

	// Only set the button active once it is released:
	//if((lastButtonState!=tmpButtonState) && (tmpButtonState == BT_NULL)){
	if ((lastButtonState != BT_NULL) && (tmpButtonState != lastButtonState)) {
		buttonState = lastButtonState;
	}

	// save the reading.  Next time through the loop,
	// it'll be the lastButtonState:
	lastButtonState = tmpButtonState;
}

void SetTimeOnScreen(uint8_t menu, unsigned long &hh, unsigned long &mm) {
	// houre ID ++
	if ((buttonState == BT_RIGHT) && (MENU_POS == menu)) {
		if ((hh >= 0) && (hh <= 23))
			hh++;
		else
			hh = 0;
	}
	// houre ID --
	else if ((buttonState == BT_LEFT) && (MENU_POS == menu)) {
		if ((hh >= 1) && (hh <= 23))
			hh--;
		else
			hh = 23;
	}

	// minute ID --
	if ((buttonState == BT_RIGHT) && (MENU_POS == (menu + 1))) { // Modbus ID minus
		if ((mm >= 0) && (mm <= 58))
			mm++;
		else
			mm = 0;
	}
	// minute ID ++
	else if ((buttonState == BT_LEFT) && (MENU_POS == (menu + 1))) { // Modbus ID plus
		if ((mm >= 1) && (mm <= 59))
			mm--;
		else
			mm = 59;
	}
}

void calcMenu(int MENU_SIZE) {
	/*	if ((buttonState == BT_ENTER) && (MENU_POS == 1)) { // ENTER SETUP
	 MENU_POS = 1;
	 WhichDisplay = DS_MENU;
	 }
	 */
	// SETUP DOWN
	if (buttonState == BT_DOWN) {
		if (MENU_POS < MENU_SIZE) {
			MENU_POS++;
		} else {
			MENU_POS = 1;
		}
	} else if (buttonState == BT_UP) {
		if (MENU_POS > 1) {
			MENU_POS--;
		} else {
			MENU_POS = MENU_SIZE;
		}
	}
}
/*************************************************/
/* control where we are in the menu and call the */
/* right function for next action                */
/*************************************************/
/* Middle/LEFT/RIGHT/ UP / DOWN                  */
/* 1      2    3      4    5                     */
/* MENU_POS 0 = NOTHING
 *DS_LOGO=0,
 DS_MAIN=1,
 DS_BOOT=2,
 DS_MENU=3,
 DS_MODBUS_SETUP=4,
 DS_TIME_SETUP=5,
 DS_IP_SETUP=6                   */
/*************************************************/
void DoMenu() {
	checkActiveButtons();     // Check if a button was pressed
	if (buttonState != BT_NULL) {

		if (WhichDisplay == DS_SCREENSAVER) {
			WhichDisplay = DS_MAIN;
			MENU_POS = 1;
			///MsTimer2::start();
		} else {
			///MsTimer2::start(); // Restart so we do not get the ScreenSaver until the next timeout

			// SETUP SELECTED
			if ((WhichDisplay == DS_MAIN) && (buttonState == BT_ENTER)
					&& (MENU_POS == 1)) {
				MENU_POS = 1;
				WhichDisplay = DS_MENU;
			}

			else if (WhichDisplay == DS_MENU) {
				calcMenu(MENU_SIZE_SETUP);
				if (buttonState == BT_ENTER) { // One of the setup menus selected
					if (MENU_POS == 1) {
						WhichDisplay = DS_MODBUS_SETUP;		// ModBus ID change
						MODBUS_TEMP_ID = ee_Config.MODBUS_ID;
					} else if (MENU_POS == 2) {
						WhichDisplay = DS_TIME_SETUP;		// Time Setup
					} else if (MENU_POS == 3) {
						WhichDisplay = DS_IP_SETUP;			// IP Setup Selected
					} else if (MENU_POS == 6) {				// Back to mainmenu
						WhichDisplay = DS_MAIN;
					} else if (MENU_POS == 4) {					// DIMM setup
						WhichDisplay = DS_DIMM;
					} else if (MENU_POS == 5) {						// Info
						WhichDisplay = DS_INFO;
					}
					MENU_POS = 1;
				}
			}
			/***********************************************************/
			/* Menu Section for Modbus                                 */
			/***********************************************************/
			// MODBUS DOWN
			else if (WhichDisplay == DS_MODBUS_SETUP) {
				calcMenu(MENU_SIZE_MODBUS_SETUP);

				// MODBUS BACK
				if ((buttonState == BT_ENTER) && (MENU_POS == 3)) { // Back to SetupMenu
					MENU_POS = 1;
					WhichDisplay = DS_MENU;
				}
				// MODBUS SAVE
				else if ((buttonState == BT_ENTER) && (MENU_POS == 4)) { // Save Adr Modbus
					ee_Config.MODBUS_ID = MODBUS_TEMP_ID;
					WriteMarelabConfig();
					modbus_configure(MODBUS_BAUD, ee_Config.MODBUS_ID,
							TX_RS485_PIN,
							MODBUS_REGISTER_COUNT, 0);
					MENU_POS = 1;
					WhichDisplay = DS_MENU;
				}

				// MODBUS ACTIVATE
				else if ((buttonState == BT_RIGHT) && (MENU_POS == 2)) { // Modbus ID minus
					if (ee_Config.MARELAB_MODUS == ACC_WLAN) {
						ee_Config.MARELAB_MODUS = ACC_WLAN_MODBUS;
					} else if (ee_Config.MARELAB_MODUS == ACC_STANDALONE) {
						ee_Config.MARELAB_MODUS = ACC_MOD;
					} else if (ee_Config.MARELAB_MODUS > ACC_WLAN) {
						ee_Config.MARELAB_MODUS = ACC_MOD;
					}
				}
				// MODBUS DEACTIVATE
				else if ((buttonState == BT_LEFT) && (MENU_POS == 2)) { // Modbus ID plus
					if (ee_Config.MARELAB_MODUS == ACC_MOD) {
						ee_Config.MARELAB_MODUS = ACC_STANDALONE;
					} else if (ee_Config.MARELAB_MODUS == ACC_WLAN_MODBUS) {
						ee_Config.MARELAB_MODUS = ACC_WLAN;
					} else if (ee_Config.MARELAB_MODUS > ACC_WLAN) {
						ee_Config.MARELAB_MODUS = ACC_STANDALONE;
					}
				}

				// MODBUS ID --
				else if ((buttonState == BT_RIGHT) && (MENU_POS == 1)) { // Modbus ID minus
					if (MODBUS_TEMP_ID >= 1 && MODBUS_TEMP_ID < 254)
						MODBUS_TEMP_ID++;
					else
						MODBUS_TEMP_ID = 1;
				}
				// MODBUS ID ++
				else if ((buttonState == BT_LEFT) && (MENU_POS == 1)) { // Modbus ID plus
					if (MODBUS_TEMP_ID >= 2 && MODBUS_TEMP_ID <= 254)
						MODBUS_TEMP_ID--;
					else
						MODBUS_TEMP_ID = 254;
				}
			}
			/***********************************************************/
			/* Menu Section for TIME Setup                             */
			/***********************************************************/
			// Time DOWN
			else if (WhichDisplay == DS_TIME_SETUP) {
				unsigned long hh, mm, ss;
				//TimeSpan timeObj(tempTime);
				//hh = timeObj.hours();
				//mm = timeObj.minutes();
				//ss = timeObj.seconds();

				calcMenu(MENU_SIZE_TIME);
				//SetTimeOnScreen(1, hh, mm);

				// sec ID --
				if ((buttonState == BT_RIGHT) && (MENU_POS == 3)) { // Modbus ID minus
					if (ss >= 0 && ss < 60)
						ss++;
					else
						ss = 0;
				}
				// sec ID ++
				else if ((buttonState == BT_LEFT) && (MENU_POS == 3)) { // Modbus ID plus
					if (ss >= 1 && ss <= 59)
						ss--;
					else
						ss = 59;
				}

				//tempTime = (hh * 3600) + (mm * 60) + ss;

				// TIME BACK
				if ((buttonState == BT_ENTER) && (MENU_POS == 4)) { // Back to SetupMenu
					MENU_POS = 1;
					WhichDisplay = DS_MENU;
				}
				// TIME SAVE
				else if ((buttonState == BT_ENTER) && (MENU_POS == 5)) { // Save Time
					//DateTime now(2016, 12, 1,hh, mm, ss);
					//rtc.adjust(now);
					MENU_POS = 1;
					WhichDisplay = DS_MENU;
				}
			}
			/***********************************************************/

			/***********************************************************/
			/* Menu Section for IP Setup                             */
			/***********************************************************/
			// IP DOWN
			else if (WhichDisplay == DS_IP_SETUP) {
				//calcMenu(MENU_SIZE_IP);

				if (buttonState == BT_DOWN) {
					if (MENU_POS >= IP_POS_ACTIVE) {
						MENU_POS++;
					} else if (MENU_POS >= IP_POS_SAVE) {
						MENU_POS = IP_POS_ACTIVE;
					}
				}

				else if (buttonState == BT_UP) {
					if (MENU_POS <= IP_POS_ACTIVE) {
						MENU_POS = IP_POS_SAVE;
					} else if (MENU_POS > IP_POS_ACTIVE) {
						MENU_POS--;
					}
				}

				// Activate
				if ((buttonState == BT_RIGHT) && (MENU_POS == IP_POS_ACTIVE)) {
					if (ee_Config.MARELAB_MODUS == ACC_STANDALONE) {
						ee_Config.MARELAB_MODUS = ACC_WLAN;
					} else if (ee_Config.MARELAB_MODUS == ACC_MOD) {
						ee_Config.MARELAB_MODUS = ACC_WLAN_MODBUS;
					}
				} else if ((buttonState == BT_LEFT)
						&& (MENU_POS == IP_POS_ACTIVE)) {
					if (ee_Config.MARELAB_MODUS == ACC_WLAN) {
						ee_Config.MARELAB_MODUS = ACC_STANDALONE;
					} else if (ee_Config.MARELAB_MODUS == ACC_WLAN_MODBUS) {
						ee_Config.MARELAB_MODUS = ACC_MOD;
					} else if (ee_Config.MARELAB_MODUS > ACC_WLAN) {
						ee_Config.MARELAB_MODUS = ACC_STANDALONE;
					}
				}

				else if ((buttonState == BT_ENTER)
						&& ((MENU_POS == IP_POS_SSID)
								|| (MENU_POS == IP_POS_WPA))) {
					WhichDisplay = DS_KEYBOARD;
					KEYBOARD_POS_X = 0;
					KEYBOARD_POS_Y = 0;
				}

				// SSID KEY
				else if ((buttonState == BT_LEFT)
						&& ((MENU_POS == IP_POS_SSID)
								|| (MENU_POS == IP_POS_WPA))) {
					if (CURSOR <= 0)
						CURSOR = 18;
					else
						CURSOR--;
				} else if ((buttonState == BT_RIGHT)
						&& ((MENU_POS == IP_POS_SSID)
								|| (MENU_POS == IP_POS_WPA))) {
					if (CURSOR <= 17)
						CURSOR++;
					else
						CURSOR = 0;
				}

				// IP BACK
				else if ((buttonState == BT_ENTER)
						&& (MENU_POS == IP_POS_BACK)) { // Back to SetupMenu
					MENU_POS = 1;
					WhichDisplay = DS_MENU;
				}
				// IP SAVE
				else if ((buttonState == BT_ENTER)
						&& (MENU_POS == IP_POS_SAVE)) { // Save Adr Modbus
					WriteMarelabConfig();
					MENU_POS = 1;
					WhichDisplay = DS_MENU;
				}
			}

			/***********************************************************/
			/* Menu Section for KeyBoard1                              */
			/***********************************************************/
			else if (WhichDisplay == DS_KEYBOARD) {
				if (buttonState == BT_DOWN) {
					if (KEYBOARD_POS_Y < 3) {
						KEYBOARD_POS_Y++;
					} else {
						KEYBOARD_POS_Y = 0;
					}
				} else if (buttonState == BT_UP) {
					if (KEYBOARD_POS_Y >= 1) {
						KEYBOARD_POS_Y--;
					} else {
						KEYBOARD_POS_Y = 3;
					}
				} else if (buttonState == BT_LEFT) {
					if (KEYBOARD_POS_X >= 1) {
						KEYBOARD_POS_X--;
					} else {
						KEYBOARD_POS_X = 17;
					}

				} else if (buttonState == BT_RIGHT) {
					if (KEYBOARD_POS_X < 16) {
						KEYBOARD_POS_X++;
					} else {
						KEYBOARD_POS_X = 0;
					}
				} else if ((buttonState == BT_ENTER) && (KEYBOARD_POS_Y == 3)
						&& (KEYBOARD_POS_X >= 14)) {
					WhichDisplay = DS_KEYBOARD2;
					KEYBOARD_POS_Y = 0;
					KEYBOARD_POS_X = 0;
				} else if ((buttonState == BT_ENTER)
						&& (MENU_POS == IP_POS_SSID)) {
					if (KEYBOARD_POS_Y == 0) {
						ee_Config.tempSSID[CURSOR] = pgm_read_byte(
								&(ASCII_1[KEYBOARD_POS_X]));
					} else if (KEYBOARD_POS_Y == 1) {
						ee_Config.tempSSID[CURSOR] = pgm_read_byte(
								&(ASCII_2[KEYBOARD_POS_X]));
					} else if (KEYBOARD_POS_Y == 2) {
						ee_Config.tempSSID[CURSOR] = pgm_read_byte(
								&(ASCII_3[KEYBOARD_POS_X]));
					} else if (KEYBOARD_POS_Y == 3) {
						ee_Config.tempSSID[CURSOR] = pgm_read_byte(
								&(ASCII_4[KEYBOARD_POS_X]));
					}
					WhichDisplay = DS_IP_SETUP;
				} else if ((buttonState == BT_ENTER)
						&& (MENU_POS == IP_POS_WPA)) {
					if (KEYBOARD_POS_Y == 0) {
						ee_Config.tempWPA[CURSOR] = pgm_read_byte(
								&(ASCII_1[KEYBOARD_POS_X]));
					} else if (KEYBOARD_POS_Y == 1) {
						ee_Config.tempWPA[CURSOR] = pgm_read_byte(
								&(ASCII_2[KEYBOARD_POS_X]));
					} else if (KEYBOARD_POS_Y == 2) {
						ee_Config.tempWPA[CURSOR] = pgm_read_byte(
								&(ASCII_3[KEYBOARD_POS_X]));
					} else if (KEYBOARD_POS_Y == 3) {
						ee_Config.tempWPA[CURSOR] = pgm_read_byte(
								&(ASCII_4[KEYBOARD_POS_X]));
					}
					WhichDisplay = DS_IP_SETUP;
				}
			}
			/***********************************************************/
			/* Menu Section for KeyBoard2                              */
			/***********************************************************/
			else if (WhichDisplay == DS_KEYBOARD2) {
				if (buttonState == BT_DOWN) {
					if (KEYBOARD_POS_Y < 3) {
						KEYBOARD_POS_Y++;
					} else {
						KEYBOARD_POS_Y = 0;
					}
				} else if (buttonState == BT_UP) {
					if (KEYBOARD_POS_Y >= 1) {
						KEYBOARD_POS_Y--;
					} else {
						KEYBOARD_POS_Y = 3;
					}
				} else if (buttonState == BT_LEFT) {
					if (KEYBOARD_POS_X >= 1) {
						KEYBOARD_POS_X--;
					} else {
						KEYBOARD_POS_X = 17;
					}

				} else if (buttonState == BT_RIGHT) {
					if (KEYBOARD_POS_X < 16) {
						KEYBOARD_POS_X++;
					} else {
						KEYBOARD_POS_X = 0;
					}
				} else if ((buttonState == BT_ENTER) && (KEYBOARD_POS_Y == 1)
						&& (KEYBOARD_POS_X >= 14)) {
					WhichDisplay = DS_KEYBOARD;
					KEYBOARD_POS_X = 0;
					KEYBOARD_POS_Y = 0;
				} else if (buttonState == BT_ENTER) {
					if (KEYBOARD_POS_Y == 0) {
						ee_Config.tempSSID[CURSOR] = pgm_read_byte(
								&(ASCII_1_2[KEYBOARD_POS_X]));
					} else if (KEYBOARD_POS_Y == 1) {
						ee_Config.tempSSID[CURSOR] = pgm_read_byte(
								&(ASCII_2_2[KEYBOARD_POS_X]));
					}
					if ((MENU_POS > 1) && (MENU_POS <= 19)) {
						WhichDisplay = DS_IP_SETUP;
					} else {
						MENU_POS = 2;
						WhichDisplay = DS_IP_SETUP;
					}
				}
			}
			/***********************************************************/

			/***********************************************************/
			/* Menu Section for Dimm Setup                             */
			/***********************************************************/
			// Dimm DOWN
			else if (WhichDisplay == DS_DIMM) {
				unsigned long hh, mm;

				calcMenu(MENU_SIZE_DIMM);

				if ((MENU_POS == 1) && (buttonState == BT_LEFT)) {
					if (Channel > 0)
						Channel--;
					else
						Channel = 15;
					EEPROM_readAnything((Channel * 12), DimChannelSetup);
				} else if ((MENU_POS == 1) && (buttonState == BT_RIGHT)) {
					if (Channel < 15)
						Channel++;
					else
						Channel = 0;
					EEPROM_readAnything((Channel * 12), DimChannelSetup);
				}

				else if ((MENU_POS == 2) || (MENU_POS == 3)) {
					//TimeSpan timeObj(60UL * DimChannelSetup.DIM_START_MIN);
					//hh = timeObj.hours();
					//mm = timeObj.minutes();
					//SetTimeOnScreen(2, hh, mm);
					//DimChannelSetup.DIM_START_MIN = (hh * 60) + mm;
				}

				else if ((MENU_POS == 4) || (MENU_POS == 5)) {
					//TimeSpan timeObj(60UL * DimChannelSetup.DIM_START_MAX);
					//hh = timeObj.hours();
					//mm = timeObj.minutes();
					//SetTimeOnScreen(4, hh, mm);
					//DimChannelSetup.DIM_START_MAX = (hh * 60) + mm;
				}

				if ((MENU_POS == 6) && (buttonState == BT_LEFT)) {
					if (DimChannelSetup.DIM_START_VALUE > 0)
						DimChannelSetup.DIM_START_VALUE--;
					else
						DimChannelSetup.DIM_START_VALUE = 100;
				} else if ((MENU_POS == 6) && (buttonState == BT_RIGHT)) {
					if (DimChannelSetup.DIM_START_VALUE < 100)
						DimChannelSetup.DIM_START_VALUE++;
					else
						DimChannelSetup.DIM_START_VALUE = 0;
				}

				else if ((MENU_POS == 7) || (MENU_POS == 8)) {
					//TimeSpan timeObj(60UL * DimChannelSetup.DIM_END_MAX);
					//hh = timeObj.hours();
					//mm = timeObj.minutes();
					//SetTimeOnScreen(7, hh, mm);
					//DimChannelSetup.DIM_END_MAX = (hh * 60) + mm;
				}

				else if ((MENU_POS == 9) || (MENU_POS == 10)) {
					//TimeSpan timeObj(60UL * DimChannelSetup.DIM_END_MIN);
					//hh = timeObj.hours();
					//mm = timeObj.minutes();
					//SetTimeOnScreen(9, hh, mm);
					//DimChannelSetup.DIM_END_MIN = (hh * 60) + mm;
				}

				if ((MENU_POS == 11) && (buttonState == BT_LEFT)) {
					if (DimChannelSetup.DIM_END_VALUE > 0)
						DimChannelSetup.DIM_END_VALUE--;
					else
						DimChannelSetup.DIM_END_VALUE = 100;
				} else if ((MENU_POS == 11) && (buttonState == BT_RIGHT)) {
					if (DimChannelSetup.DIM_END_VALUE < 100)
						DimChannelSetup.DIM_END_VALUE++;
					else
						DimChannelSetup.DIM_END_VALUE = 0;
				}

				// Dimm BACK
				if ((buttonState == BT_ENTER) && (MENU_POS == 12)) { // Back to SetupMenu
					MENU_POS = 1;
					WhichDisplay = DS_MENU;
				}
				// Dimm SAVE
				else if ((buttonState == BT_ENTER) && (MENU_POS == 13)) { // Save Adr Modbus
					MENU_POS = 1;
					EEPROM_writeAnything((Channel * 12), DimChannelSetup);
					//WhichDisplay = DS_MENU;
				}
			}
			/***********************************************************/

			/***********************************************************/
			/* Menu Section for INFO Setup                             */
			/***********************************************************/
			// INFO DOWN
			else if (WhichDisplay == DS_INFO) {
				calcMenu(MENU_SIZE_INFO);
				// INFO BACK
				if ((buttonState == BT_ENTER) && (MENU_POS == 1)) { // Back to SetupMenu
					MENU_POS = 1;
					WhichDisplay = DS_MENU;
				}
			}
		}
	}
}

void ScreenSaver() {
	WhichDisplay = DS_SCREENSAVER;
	MENU_POS = 1;
}




void UpdateSensor(struct TSENSOR &sensor)
{
	// Check Sensors & set Actors
	if (digitalRead(sensor.S_PORT) != sensor.S_LAST_STATE ){
		sensor.S_STABLE = millis();
		sensor.S_LAST_STATE = digitalRead(sensor.S_PORT);
		sensor.S_OK=FALSE;
	}
			  
	//Senor stable over TimeOffset ?
	if ((millis() - sensor.S_STABLE) >= sensor.S_DBOUNCE){
		sensor.S_OK = TRUE;	
	}
			  
}

void setup() {
	// the setup routine runs once when you press reset:
	pinMode(LED_PIN, OUTPUT);
	pinMode(SAQUA,INPUT);
	AQUA.S_PORT = SAQUA;
	AQUA.S_DBOUNCE = 4000L;
	AQUA.S_STABLE=0L;
	AQUA.S_LAST_STATE = digitalRead(SAQUA);
	AQUA.S_OK = FALSE;
	pinMode(SOHIGH,INPUT);
	FWTANK_TOP.S_PORT = SOHIGH;
	FWTANK_TOP.S_DBOUNCE = 4000L;
	FWTANK_TOP.S_STABLE=0L;
	FWTANK_TOP.S_LAST_STATE = digitalRead(SOHIGH);
	FWTANK_TOP.S_OK = FALSE;
	pinMode(SOLOW,INPUT);
	FWTANK_BOT.S_PORT = SOLOW;
	FWTANK_BOT.S_DBOUNCE = 4000L;
	FWTANK_BOT.S_STABLE=0L;
	FWTANK_BOT.S_LAST_STATE = digitalRead(SOLOW);
	FWTANK_BOT.S_OK = FALSE;
	
	
	
	//STATUS_OSMOSE_CLEAN		= FALSE;
	OSMOSE_CLEAN_RUNTIME	= 60000;
	OSMOSE_CLEAN_REPEATTIME = 120000;
	OSMOSE_MODE				= OSMOSE_MODE_AUTO;
	
	osmose_active = FALSE;
	osmose_active_firsttime=TRUE;
	boolean CleanOn = FALSE;
	OSMOSE_START_TIME=0;
	OSMOSE_RUN_TIME=0;
	pump_active = FALSE;
	OSMOSE_MAX_RUNTIME = 0;
	
	
	digitalWrite(RESET_PIN, HIGH);
	delay(100);
	pinMode(RESET_PIN, OUTPUT);
	unsigned long timeout = SCREEN_SAVER_TIMEOUT * 1000UL;
	///MsTimer2::set(timeout, ScreenSaver);
	///MsTimer2::start();
	ReadMarelabConfig();

	Channel = 0;
	EEPROM_readAnything((Channel * 12), DimChannelSetup);

	WhichDisplay = DS_LOGO;
	screen.begin();
	draw();
	delay(1000);
	WhichDisplay=DS_MAIN;
	MENU_POS=1;
		
	mb_reg[MARELAB_DEVICE_ID] = MARELAB_TYPE; /* Set the device ID to get identified 	*/
	mb_reg[MARELAB_VERSION] = MARELAB_FIRWMARE; /* Set the Firmware Version ID 			*/
	mb_reg[MARELAB_REGISTER_COUNT] = MODBUS_REGISTER_COUNT; /* Stores the amount of registers for modbus */

	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//ee_Config.MARELAB_MODUS = ACC_STANDALONE;					/* Modbus & WLAN Access = 0 */

	mb_reg[LED_COLOR1] = 100;
	mb_reg[LED_COLOR2] = 200;
	mb_reg[LED_COLOR3] = 300;
	mb_reg[LED_COLOR4] = 400;
	mb_reg[LED_COLOR5] = 500;
	mb_reg[LED_COLOR6] = 600;
	mb_reg[LED_COLOR7] = 700;
	mb_reg[LED_COLOR8] = 800;
	mb_reg[LED_COLOR9] = 900;
	mb_reg[LED_COLOR10] = 1000;
	mb_reg[LED_COLOR11] = 900;
	mb_reg[LED_COLOR12] = 800;
	mb_reg[LED_COLOR13] = 700;
	mb_reg[LED_COLOR14] = 600;
	mb_reg[LED_COLOR15] = 500;
	mb_reg[LED_COLOR16] = 400;

	mb_reg[DIMM_TEMPERATUR] = 48;
	mb_reg[UNIX_DATE1] = 0;
	mb_reg[UNIX_DATE2] = 0;
	mb_reg[MCOMMAND] = 0;

	// Reading EEPROM
	DimChannel DimChSet;
	EEPROM_readAnything(0, DimChSet);
	mb_reg[MDIM_START_MIN] = DimChSet.DIM_START_MIN;
	mb_reg[MDIM_START_MAX] = DimChSet.DIM_START_MAX;
	mb_reg[MDIM_START_VALUE] = DimChSet.DIM_START_VALUE;
	mb_reg[MDIM_END_MAX] = DimChSet.DIM_END_MAX;
	mb_reg[MDIM_END_VALUE] = DimChSet.DIM_END_VALUE;
	mb_reg[MDIM_END_MIN] = DimChSet.DIM_END_MIN;
	mb_reg[MDIM_CHANNEL] = 0;

	//modbusino_slave.setup(MODBUS_BAUD);		/* ModBus Baud Rate			*/
	//modbus_configure(MODBUS_BAUD, ee_Config.MODBUS_ID, TX_RS485_PIN,MODBUS_REGISTER_COUNT, 0);


	Serial.begin(19200);
	pinMode(TX_RS485_PIN, OUTPUT);
	digitalWrite(TX_RS485_PIN, LOW);



	pinMode(LED_PIN, OUTPUT);  				// Set ledPin mode
	pinMode(BUTTON_ANALOG_PIN, INPUT); 	// Joystick Analog Input
	MENU_POS = 1;


	 //pwm.begin();
	 //pwm.setPWMFreq(1600);  // This is the maximum PWM frequency

	  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
	  // some i2c devices dont like this so much so if you're sharing the bus, watch
	  // out for this!
	#ifdef TWBR
	  // save I2C bitrate
	  uint8_t twbrbackup = TWBR;
	  // must be changed after calling Wire.begin() (inside pwm.begin())
	  TWBR = 12; // upgrade to 400KHz!
	#endif

}

char buf[128];
String inputString;
boolean stringComplete = false;
byte byteRead;
char inChar;

uint8_t count=0;


void loop() {
	
	//pinMode(LED_PIN, OUTPUT);
	//digitalWrite(LED_PIN, HIGH);
	//delay(1000);
	//digitalWrite(LED_PIN, LOW);
	//delay(1000);
	
		mb_reg[MCOMMAND] = 0; /* after each loop setting the modbus command register to 0 */
	/*if (modbus_update(mb_reg) < 0)
		//digitalWrite(13, HIGH);
		// else
		digitalWrite(13, LOW);
	*/
	// Command Interpreter
	// Executes the command in transfered in the command
	// register. After executen it is set to 0
	/* MCOMMAND -> 1  = getRTC Time			*/
	if (mb_reg[MCOMMAND] == 1) {

	}
	/* MCOMMAND -> 2  = setRTC Time			*/
	else if (mb_reg[ MCOMMAND] == 2) {
		//DateTime newTime(MakeTime(mb_reg[ UNIX_DATE1], mb_reg[ UNIX_DATE2]));
		//rtc.adjust(newTime);
	}
	/* MCOMMAND -> 10  = setPWM fills Register 20-26	    */
	/* Writes the DIMM Parameter for a PWM Channel to EEPROM */
	else if (mb_reg[ MCOMMAND] == 10) {
		uint16_t chandel2update;
		DimChannel DimChSet;
		DimChSet.DIM_START_MIN = mb_reg[ MDIM_START_MIN];
		DimChSet.DIM_START_MAX = mb_reg[ MDIM_START_MAX];
		DimChSet.DIM_START_VALUE = mb_reg[ MDIM_START_VALUE];
		DimChSet.DIM_END_MAX = mb_reg[ MDIM_END_MAX];
		DimChSet.DIM_END_VALUE = mb_reg[ MDIM_END_VALUE];
		DimChSet.DIM_END_MIN = mb_reg[ MDIM_END_MIN];
		chandel2update = mb_reg[ MDIM_CHANNEL];
		EEPROM_writeAnything((chandel2update * 12), DimChSet);
	}
	/* MCOMMAND -> 11  = getPWM fills Register 20-26						*/
	/* Reads the PWM Channel Dimm Values from EEPROM    					*/
	/* To read the registers correct we need two Requests 					*/
	/* 1.) write MultipleRegisters with command (11) -> get and PWM Chanel (MDIM CHANNEL) 	*/
	/*        Program transfers EEPROM content to the Register for reading			*/
	/* 2.) read ReadHoldingRegister (03) -> transfers the PWM Channel Data			*/
	else if (mb_reg[ MCOMMAND] == 11) {
		uint16_t chandel2update;
		chandel2update = mb_reg[ MDIM_CHANNEL];
		DimChannel DimChSet;
		EEPROM_readAnything((chandel2update * 12), DimChSet);
		mb_reg[ MDIM_START_MIN] = DimChSet.DIM_START_MIN;
		mb_reg[ MDIM_START_MAX] = DimChSet.DIM_START_MAX;
		mb_reg[ MDIM_START_VALUE] = DimChSet.DIM_START_VALUE;
		mb_reg[ MDIM_END_MAX] = DimChSet.DIM_END_MAX;
		mb_reg[ MDIM_END_VALUE] = DimChSet.DIM_END_VALUE;
		mb_reg[ MDIM_END_MIN] = DimChSet.DIM_END_MIN;
	}

	/* RESET FOR BOOTLOADING */
	else if (mb_reg[ MCOMMAND] == 200) {
		///MsTimer2::stop();
		WhichDisplay = 2;
		draw();
		digitalWrite(RESET_PIN, LOW);
	}
	if (ee_Config.MARELAB_MODUS == ACC_STANDALONE){
	 // CalcDimmValue();				 // Calculate Dimmer values for all 16 channels in standalone mode
	}
	DoMenu();
	draw(); // Redraw display
	buttonState = BT_NULL;        // Reset before the next check

	
	// Sensor Actor work
	UpdateSensor(AQUA);
	UpdateSensor(FWTANK_BOT);
	UpdateSensor(FWTANK_TOP);
	
	GlobalStateSensor = 0;
	if (AQUA.S_OK)
	   digitalWrite(LED_PIN, AQUA.S_LAST_STATE);
	   
	 /* Logic Table to control Pumps & Vents												*/
	 /* ---------------------------------------------------------------------------------*/
	 /*					SOHigh	SOLow	SAQUA											*/
	 /* Osmose Empty	  0		  0			0   PumpOsmose VentFresh ON PumpRefill OFF	*/
	 /* Osmose Empty	  0	  	  0			1   PumpOsmose VentFresh ON PumpRefill OFF	*/
	 /* Osmose OK		  0		  1			0   PumpOsmose VentFresh ON PumpRefill ON	*/
	 /* Osmose OK		  0		  1			1	All OFF IDLE							*/
	 /* ERROR			  1		  0			0   All OFF									*/
	 /* ERROR		      1		  0			1   All OFF									*/
	 /* Osmose OK		  1		  1			0   PumpOsmose VentFresh OFF PumpRefill ON	*/
	 /* Osmose OK		  1		  1			1   PumpOsmose VentFresh OFF PumpRefill OFF	*/  
	 
	if (AQUA.S_OK && FWTANK_TOP.S_OK & FWTANK_BOT.S_OK)
	{
		GlobalStateSensor = (AQUA.S_LAST_STATE<<0) | (FWTANK_BOT.S_LAST_STATE<<1) | (FWTANK_TOP.S_LAST_STATE<<2);
	}
	else
	{
		GlobalStateSensor = GlobalStateSensorOld;
	}
	
	// OSMOSE STEUERUNG
	/////////////////////////////////////////////////////////
	// FirstTime
	if (osmose_active && osmose_active_firsttime){
		OSMOSE_START_TIME = millis();
		CleanOn = TRUE;
		osmose_active_firsttime = FALSE;
	}
	
	// OSMOSE is Running
	if (osmose_active){
		OSMOSE_RUN_TIME = millis()-OSMOSE_START_TIME;
		//osmose_active_firsttime = FALSE;
	}
	else
	{
		OSMOSE_START_TIME = millis();
		CleanOn = FALSE;
		osmose_active_firsttime = TRUE;
	}
	
	// Stop active cleaning
	if (osmose_active && (OSMOSE_RUN_TIME >= OSMOSE_CLEAN_RUNTIME)){
		CleanOn = FALSE;
		//OSMOSE_START_TIME = millis();
	}
	
	// Repeat Cleaning
	if (!CleanOn && osmose_active && ( OSMOSE_RUN_TIME >= OSMOSE_CLEAN_REPEATTIME)){
		CleanOn = TRUE;	 
		osmose_active_firsttime = TRUE;   
	}
	////////////////////////////////////////////////////////////////////////////////
	
	//inputString="";

	while (Serial.available()) {
			inChar = (char) Serial.read();
			if ((inChar != '\r')||(inChar != '\n')){
				buf[count] = inChar; // Store it
				count++; // Increment where to write next
				buf[count] = '\0'; // Null terminate the string
			}
			if ((inChar == '\r')||(inChar == '\n')){
				String t(buf);
				//swSerial.println(buf);
				//swSerial.println("AT+CWJAP=\"marelab.wlan\",\"1234567812345678\"");
				digitalWrite(TX_RS485_PIN, HIGH);
				delay(10);
				Serial.println(buf);
				delay(10);
				digitalWrite(TX_RS485_PIN, LOW);
				count=0;
			}
	}
/*
	if (swSerial.available()) {
		digitalWrite(TX_RS485_PIN, HIGH);
		delay(10);
		char inChar;
		while (swSerial.available()) {
			inChar = (char) swSerial.read();
			Serial.print(inChar);
		}
		Serial.flush();
		delay(10);
		digitalWrite(TX_RS485_PIN, LOW);
	}
	*/
	

}

