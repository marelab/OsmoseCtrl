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
#include <Wire.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include "libmodbus/Modbusino.h"
#include "OneWire/OneWire.h"
#include <avr/pgmspace.h>
#include "marelab_dimm.h"
#include "OLED.h"
#include "MCP3221.h"
#include "PCA9554.h"

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
	unsigned int	S_DBOUNCE;
	unsigned long	S_STABLE;
	boolean			S_LAST_STATE;		// STATE OF THE SENSOR THAT GETS UPDATE IN EVERY LOOP
	boolean         S_STATE;			// STATE OF THE SENSOR THAT ONLY GETS UPDATED IF ITS STABLE OVER DBOUNCE TIME
};
struct TSENSOR AQUA;
struct TSENSOR FWTANK_TOP;
struct TSENSOR FWTANK_BOT;

uint8_t GlobalStateSensor;			/* AllSensors as Byte */
uint8_t GlobalStateSensorOld;

#define PUMP_VENT_CH		0		/* I2C Channel for Pump & Vent FreshWater */
#define VENT_CLEAN          1
#define PUMP_REFILL         2


boolean osmose_active;
boolean pump_active;
boolean osmose_active_firsttime;
boolean CleanOn;

byte ERROR_STATE;

unsigned long OSMOSE_START_TIME;
unsigned long OSMOSE_RUN_TIME;
unsigned long REFILL_RUN_TIME;
unsigned long REFILL_START_TIME;
unsigned long SCREEN_SAVER_TIME;
unsigned long CLEAN_START_TIME;

//unsigned long OSMOSE_CLEAN_RUNTIME;
//unsigned long OSMOSE_CLEAN_REPEATTIME;
// Software Safty feature to handle mailfunction of Sensors
//unsigned long OSMOSE_MAX_RUNTIME;			// Max duration between low and high sensor if > OSMOSE to ERROR
//unsigned long OSMOSE_MAX_PUMP_RUNTIME;		// Max Duration between Refill start and Sensor Tank Signal if > ERROR





// Modbus Settings
#define MODBUS_BAUD			    19200	/* Modbus Baud */
#define MARELAB_TYPE			1000	/* Set the device ID to get identified 								*/
#define MARELAB_FIRWMARE  		1
#define MODBUS_REGISTER_COUNT 	31  /* COUNT of Modbus to Led Registers starting with 0					*/

	

/* These are the Modbus Registers where we can read & write to        */
/* Mapping Modbus Register to LED Lamp Functions */
#define MARELAB_DEVICE_ID 		0	/* Device ID to identify marelab 	*/
#define MARELAB_VERSION			1	/* Firmware ID to identify 		*/
#define MARELAB_REGISTER_COUNT  2

#define MOD_OSMOSE_MODE 		3	/* 0-MANUAL 1-AUTO R/W */
#define OSMOSE_MODE_MANUEL		0
#define OSMOSE_MODE_AUTO		1


#define MOD_OSMOSE_STATE 		4

/* Modbus Register for Dbounce & stable R/W 
   Because the sensor can bounce between on and off a Time as
   DBOUNCE value in seconds is configured. The Sensor input is set 
   active only when STABLESx >= DBOUNCEx. 
   - DBOUNCE configures the time a Sensor has to be in a stable 
     not changed state.
   - STABLESx is the time the Sensor is stable 
*/
#define	MOD_DBOUNCE_AQUA		5
#define	MOD_STABLES_AQUA		6

#define	MOD_DBOUNCE_TANKTOP		7
#define	MOD_STABLES_TANKTOP		8

#define	MOD_DBOUNCE_TANKBOT		9
#define	MOD_STABLES_TANKBOT		10
  
#define MCOMMAND        		22


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

//OneWire ds(10);
MCP3221 *adc;
byte i2cAdrADC				= 0x4D;	
const int I2CadcVRef		= 4980; //Measured millivolts of voltage input to ADC (can measure VCC to ground on MinipH for example)
uint16_t adcRaw, adcCompare = 0;
float temperatute;
uint16_t adcAVG;
//const int oscV = 185; //voltage of oscillator output after voltage divider in millivolts i.e 120mV (measured AC RMS) ideal output is about 180-230mV range
const int	oscV			= 200; //voltage of oscillator output after voltage divider in millivolts i.e 120mV (measured AC RMS) ideal output is about 180-230mV range
const float kCell			= 1.0; //set our Kcell constant basically our microsiemesn conversion 10-6 for 1 10-7 for 10 and 10-5 for .1
const float Rgain			= 1000.0; //this is the measured value of the R9 resistor in ohms

#define Write_Check			0x1234

//Our parameter, for ease of use and eeprom access lets use a struct


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
const uint16_t buttonLowRange[] =  { 0,1010  ,830, 920, 1000, 690};
const uint16_t buttonHighRange[] = { 0,1020, 860, 950, 1008, 710};
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
	DS_DBOUNCE,
	DS_CLEANING,
	DS_SAFETY,
	DS_MODBUS_SETUP,
	DS_MODE,
	DS_INFO,
	DS_SCREENSAVER
};


uint8_t MODBUS_TEMP_ID = 0;	// its needed if the ModBus ID isn't chg over display
#define MENU_SIZE_SETUP			7  // Modbus/Debounce/Info/Back
#define MENU_SIZE_MODBUS_SETUP  4  // Modbus ID/ ACTIVE CHECKBOX/BACK/SAVE
#define MENU_SIZE_DBOUNCE       5  // ON-OFF/IP1/IP2/IP3/IP4/BACK/SAVE
#define MENU_SIZE_INFO          1  // ON-OFF/IP1/IP2/IP3/IP4/BACK/SAVE
#define MENU_SIZE_SAFETY        4  // ON-OFF/IP1/IP2/IP3/IP4/BACK/SAVE
#define MENU_SIZE_CLEANING      4  // ON-OFF/IP1/IP2/IP3/IP4/BACK/SAVE
#define MENU_SIZE_MODE          4  // ON-OFF/IP1/IP2/IP3/IP4/BACK/SAVE

#define SCREEN_SAVER_TIMEOUT    120 // Time until screensaver is activated
unsigned long tempTime = 0;		// Transfer Time Object between GUI & EEPROM
uint8_t CURSOR = 0;


/********************************************************************************************/



struct MarelabEEPROM {
	uint8_t MARELAB_MODUS; /* CONTROLLED BY 01 STANDALONE / 02 NETWORK MOD */
	uint8_t MODBUS_ID; /* Slave ID of this device 			                                                    */
	
	// OSMOSE Mode
	uint8_t OSMOSE_MODE;  /*0 Manuel 1 Auto */
	
	// Debounce settings
	uint16_t OSMOSE_DBOUNCE_AQUA;  // ms time
	uint16_t OSMOSE_DBOUNCE_TOP;   // ms time
	uint16_t OSMOSE_DBOUNCE_BOTTOM;// ms time
	
	// Membran Cleaning
	unsigned long OSMOSE_CLEAN_DURATION; // Time in sec the CleanVent is opened to clean the osmosis membran
	unsigned long OSMOSE_CLEAN_REPEAT;   // Time in min after a new membran cleaning is initiated
	
	// Saftey Feature
	unsigned long OSMOSE_MAX_CYCLE_TIME; // Time a complete filling of the OSMOSIS Tank can take
	unsigned long OSMOSE_MAX_REFILL_TANK; // Max time a tank refill with osmosis water can take
	
	unsigned int SCREENSAVER_TIMEOUT;
	
	//EC_PARAMETERS ECparams;
	unsigned int WriteCheck;
	int eCLowCal, eCHighCal;
	//float eCStep;
};

MarelabEEPROM ee_Config;


/***********************************************************/
/* READ EEPROM MARLAB CONFIG                               */
/***********************************************************/
void ReadMarelabConfig() {
	EEPROM_readAnything(512, ee_Config);
	//mb_reg[MDIM_MODUS] = ee_Config.MARELAB_MODUS;
}
/***********************************************************/
/* WRITE EEPROM MARLAB CONFIG                               */
/***********************************************************/
void WriteMarelabConfig() {
	EEPROM_writeAnything(512, ee_Config);
}

/*
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
*/
void DisplayStatus() {
	char buffer[6];
	// ICON STATUS BAR

	
	if (ee_Config.MARELAB_MODUS == ACC_STANDALONE) {
		screen.DrawXBM(112, 0, 16, 16, ICON_STANDALONE);
	} else if (ee_Config.MARELAB_MODUS == ACC_MOD) {
		screen.DrawXBM(112, 0, 16, 16, ICON_MODBUS);
	} 
	
	

	// GLOBAL STATE OF OSMOSE PROCESS
	//itoa(GlobalStateSensor, buffer, 10);
	mb_reg[MOD_OSMOSE_STATE] = GlobalStateSensor;
	//screen.Text(100, 5, buffer);
		
	
   
	// Display Sensor Status 
	if (ee_Config.OSMOSE_MODE ==  OSMOSE_MODE_MANUEL){
		screen.TextP(94, 56, T_OSMOSE_MANUEL, true);
	}
	else if (ee_Config.OSMOSE_MODE == OSMOSE_MODE_AUTO){
		screen.TextP(94, 56, T_OSMOSE_AUTO, true);
	}

	if (AQUA.S_STATE){
		screen.Text(1, 56, "A-1");
	}else{
		screen.Text(1, 56, "A-0");
	}
	
	if (FWTANK_BOT.S_STATE){		
		screen.Text(30, 56, "B-1");
	}else {	
		screen.Text(30, 56, "B-0");
	}
	
	if (FWTANK_TOP.S_STATE){
		screen.Text(60, 56, "T-1");
	}else{
		screen.Text(60, 56, "T-0");
	}
	
	// EC VALUE DISPLAY
	screen.setFontGross();
	if ((adcAVG > 65000) || (adcAVG < 25) )
		adcAVG = 0;
	ltoa(adcAVG, buffer, 10);
	screen.Text(60, 18, buffer);
	screen.setFontKLein();
	screen.Text(112, 24, "uS");
	screen.DrawLine(58, 16, 58, 35);
	screen.DrawLine(58, 35, 128, 35);
 
	// STATUS ICONS OF OSMOSE
	if (ERROR_STATE != 0){
		if (ERROR_STATE==1)
			screen.TextP(1, 2, T_ERROR,TRUE);
		if (ERROR_STATE==2)
			screen.TextP(1, 2, T_ERROR_TIME,TRUE);	
	//if(!FWTANK_BOT.S_STATE && FWTANK_TOP.S_STATE){
	}else{
		if (osmose_active && !CleanOn){
			screen.TextP(1, 2, T_OSMOSE,TRUE);
			screen.DrawXBM(94, 0, 16, 16, ICON_WATERON);
			uint16_t run = OSMOSE_RUN_TIME/1000/60;
			ltoa(run, buffer, 10);
			screen.TextP(1, 18, T_MODERUNMIN,FALSE);
			screen.Text(1, 28, buffer);
			screen.Text(24, 28, "/");
			ltoa(ee_Config.OSMOSE_MAX_CYCLE_TIME, buffer, 10);
			screen.Text(32, 28, buffer);
		}
		else if (osmose_active && CleanOn){
			screen.DrawXBM(76, 0, 16, 16, ICON_CLEAN);
			screen.DrawXBM(94, 0, 16, 16, ICON_WATERON);
			screen.TextP(1, 2, T_CLEAN,TRUE);
			// Mode Runtime COUNTDOWN
			unsigned long run =  (millis()-CLEAN_START_TIME ) / 1000;
			ltoa(run, buffer, 10);
			screen.TextP(1, 18, T_MODERUNSEC,FALSE);
			screen.Text(1, 28, buffer);
			screen.Text(24, 28, "/");
			ltoa(ee_Config.OSMOSE_CLEAN_DURATION, buffer, 10);
			screen.Text(32, 28, buffer);
		}
		if (pump_active){
			screen.DrawXBM(58, 0, 16, 16, ICON_REFILL);
			screen.TextP(1, 2, T_REFILL,TRUE);
			// Mode Runtime COUNTDOWN
			
			uint16_t run = REFILL_RUN_TIME/1000;
			ltoa(run, buffer, 10);
			screen.TextP(1, 40, T_REFILLRUNSEC,FALSE);
			screen.Text(74, 40, buffer);
			screen.Text(94, 40, "/");
			ltoa(ee_Config.OSMOSE_MAX_REFILL_TANK, buffer, 10);
			screen.Text(102, 40, buffer);
			
		}
		if(!pump_active && !CleanOn && !osmose_active){
			screen.TextP(1, 2, T_IDLE,TRUE);
		}
	}
	
	screen.DrawLine(0, 16, 128, 16);
	screen.DrawLine(0, 54, 128, 54);
	
	// RUNTIME
	/*
	unsigned long runtime = millis()/1000;
	ltoa(runtime, buffer, 10);
	screen.Text(70, 40, "S");
	screen.Text(75, 40, buffer);
	screen.Show();
	*/
}



void draw_Menu() {
	char buffer[4];
	screen.TextP(0, 5, T_SETUP, false);
	screen.DrawLine(0, 14, 128, 14);

	itoa(MENU_POS, buffer, 10);
	screen.Text(85, 5, buffer);
	

	if (MENU_POS == 1) {
		screen.TextP(8, 20, T_CLEANING, true);
		} else {
		screen.TextP(8, 20, T_CLEANING, false);
	}

	if (MENU_POS == 2) {
		screen.TextP(60, 20, T_SAFETY, true);
		} else {
		screen.TextP(60, 20, T_SAFETY, false);
	}

	if (MENU_POS == 3) {
		screen.TextP(8, 30, T_BOUNCE_SETUP, true);
	} else {
		screen.TextP(8, 30, T_BOUNCE_SETUP, false);
	}
	
	if (MENU_POS == 4) {
		screen.TextP(60, 30, T_MODBUS_ADRESS, true);
		} else {
		screen.TextP(60, 30, T_MODBUS_ADRESS, false);
	}
	
	if (MENU_POS == 5) {
		screen.TextP(8, 40, T_OSMOSE_MODE, true);
	} else {
		screen.TextP(8, 40, T_OSMOSE_MODE, false);
	}

	if (MENU_POS == 6) {
		screen.TextP(60, 40, T_INFO_SETUP, true);
	} else {
		screen.TextP(60, 40, T_INFO_SETUP, false);
	}

	if (MENU_POS == 7) {
		screen.TextP(1, 55, T_BACK, true);
	} else {
		screen.TextP(1, 55, T_BACK, false);
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

void drawDBounce(){
	char buffer[4];
	screen.TextP(0, 5, T_SETUP, false);
	screen.TextP(50, 5, T_BOUNCE_SETUP, false);
	screen.DrawLine(0, 14, 128, 14);
	
	if (MENU_POS == 1)
		screen.TextP(0, 18, T_SAQUA, true);
	else
		screen.TextP(0, 18, T_SAQUA, false);
	ltoa((AQUA.S_DBOUNCE/1000), buffer, 10);
	screen.Text(90, 18, buffer);
	
	if (MENU_POS == 2)
		screen.TextP(0, 30, T_STOP, true);
	else
		screen.TextP(0, 30, T_STOP, false);
	ltoa((FWTANK_TOP.S_DBOUNCE/1000), buffer, 10);
	screen.Text(90, 30, buffer);
	
	if (MENU_POS == 3)
		screen.TextP(0, 42, T_SBOT, true);
	else
		screen.TextP(0, 42, T_SBOT, false);
	ltoa((FWTANK_BOT.S_DBOUNCE/1000), buffer, 10);
	screen.Text(90, 42, buffer);
	
	screen.DrawLine(0, 54, 128, 54);

	if (MENU_POS == 4) {
		screen.TextP(1, 56, T_BACK, true);
	} else {
		screen.TextP(1, 56, T_BACK, false);
	}

	if (MENU_POS == 5) {
		screen.TextP(100, 56, T_SAVE, true);
	} else {
		screen.TextP(100, 56, T_SAVE, false);	
	}
}

void drawSafety(){
	char buffer[4];
	screen.TextP(0, 5, T_SETUP, false);
	screen.TextP(50, 5, T_SAFETY, false);
	screen.DrawLine(0, 14, 128, 14);
	
	if (MENU_POS == 1)
	screen.TextP(0, 18, T_SAFETY_CYLE , true);
	else
	screen.TextP(0, 18, T_SAFETY_CYLE, false);
	
	ltoa(ee_Config.OSMOSE_MAX_CYCLE_TIME, buffer, 10);
	screen.Text(90, 18, buffer);
	
	if (MENU_POS == 2)
	screen.TextP(0, 30, T_SAFETY_REFILL, true);
	else
	screen.TextP(0, 30, T_SAFETY_REFILL, false);
	
	ltoa(ee_Config.OSMOSE_MAX_REFILL_TANK, buffer, 10);
	screen.Text(90, 30, buffer);
	
	if (MENU_POS == 3) {
		screen.TextP(1, 56, T_BACK, true);
		} else {
		screen.TextP(1, 56, T_BACK, false);
	}

	if (MENU_POS == 4) {
		screen.TextP(100, 56, T_SAVE, true);
		} else {
		screen.TextP(100, 56, T_SAVE, false);
	}
}

void drawCleaning(){
	char buffer[4];
	screen.TextP(0, 5, T_SETUP, false);
	screen.TextP(50, 5, T_CLEANING, false);
	screen.DrawLine(0, 14, 128, 14);
		
	if (MENU_POS == 1)
		screen.TextP(0, 18, T_CLEAN_DURA, true);
	else
		screen.TextP(0, 18, T_CLEAN_DURA, false);
	
	ltoa(ee_Config.OSMOSE_CLEAN_DURATION, buffer, 10);
	screen.Text(90, 18, buffer);
	
	if (MENU_POS == 2)
		screen.TextP(0, 30, T_CLEAN_REPE, true);
	else
		screen.TextP(0, 30, T_CLEAN_REPE, false);
	
	ltoa(ee_Config.OSMOSE_CLEAN_REPEAT, buffer, 10);
	screen.Text(90, 30, buffer);
	
	if (MENU_POS == 3) {
		screen.TextP(1, 56, T_BACK, true);
		} else {
		screen.TextP(1, 56, T_BACK, false);
	}

	if (MENU_POS == 4) {
		screen.TextP(100, 56, T_SAVE, true);
		} else {
		screen.TextP(100, 56, T_SAVE, false);
	}
}


void drawMode(){
	//char buffer[4];
	screen.TextP(0, 5, T_SETUP, false);
	screen.TextP(50, 5, T_OSMOSE_MODE, false);
	screen.DrawLine(0, 14, 128, 14);
	
	if (MENU_POS == 1)
		screen.TextP(0, 18, T_OSMOSE_MANUEL, true);
	else
		screen.TextP(0, 18, T_OSMOSE_MANUEL, false);
		
	if (ee_Config.OSMOSE_MODE == OSMOSE_MODE_MANUEL) {
		screen.DrawRect(100, 18, 10, 10);
		screen.DrawFilledRect(102, 20, 6, 6);
		} else {
		screen.DrawRect(100, 18, 10, 10);
	}
	
	
	
	if (MENU_POS == 2)
		screen.TextP(0, 30, T_OSMOSE_AUTO, true);
	else
		screen.TextP(0, 30, T_OSMOSE_AUTO, false);
	
	if (ee_Config.OSMOSE_MODE == OSMOSE_MODE_AUTO) {
		screen.DrawRect(100, 30, 10, 10);
		screen.DrawFilledRect(102, 32, 6, 6);
		} else {
		screen.DrawRect(100, 30, 10, 10);
	}
	
	if (MENU_POS == 3) {
		screen.TextP(1, 56, T_BACK, true);
		} else {
		screen.TextP(1, 56, T_BACK, false);
	}

	if (MENU_POS == 4) {
		screen.TextP(100, 56, T_SAVE, true);
		} else {
		screen.TextP(100, 56, T_SAVE, false);
	}
}

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

	if (ee_Config.MARELAB_MODUS == ACC_STANDALONE) {
		screen.DrawXBM(108, 20, 16, 16, ICON_STANDALONE);
	} else if (ee_Config.MARELAB_MODUS == ACC_MOD) {
		screen.DrawXBM(108, 20, 16, 16, ICON_MODBUS);
	} 

	if (MENU_POS == 1) {
		screen.TextP(8, 53, T_BACK, true);
	} else {
		screen.TextP(8, 53, T_BACK, false);
	}
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
		case DS_DBOUNCE: {	// DBOUNCE CONFIG
			drawDBounce();
			break;
		}
		case DS_INFO: {	// IP Setup
			drawInfo();
			break;
		}
		case DS_SAFETY: {	// Safety Setup
			drawSafety();
			break;
		}
		case DS_CLEANING: {	// Cleaning Setup
			drawCleaning();
			break;
		}
		case DS_MODE: {	// Mode Setup
			drawMode();
			break;
		}
		case DS_SCREENSAVER: {
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
		SCREEN_SAVER_TIME = millis();
	}

	// save the reading.  Next time through the loop,
	// it'll be the lastButtonState:
	lastButtonState = tmpButtonState;
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
 DS_TIME_SETUP=5                */
/*************************************************/
void DoMenu() {
	checkActiveButtons();     // Check if a button was pressed
	
	if (((millis()-SCREEN_SAVER_TIME)/1000) >= SCREEN_SAVER_TIMEOUT){
			WhichDisplay = DS_SCREENSAVER;
			MENU_POS = 1;
	}
	
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
					if (MENU_POS == 4) {
						WhichDisplay = DS_MODBUS_SETUP;		// ModBus ID change
						MODBUS_TEMP_ID = ee_Config.MODBUS_ID;
					} else if (MENU_POS == 3) {					// DIMM setup
						WhichDisplay = DS_DBOUNCE;
					} else if (MENU_POS == 7) {				// Back to mainmenu
						WhichDisplay = DS_MAIN;
					} else if (MENU_POS == 5) {						// Info
						WhichDisplay = DS_MODE;
					} else if (MENU_POS == 6) {						// Info
						WhichDisplay = DS_INFO;
					}else if (MENU_POS == 2) {					// DIMM setup
						WhichDisplay = DS_SAFETY;
					} else if (MENU_POS == 1) {					// DIMM setup
						WhichDisplay = DS_CLEANING;
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
			/* Menu Section for DBOUNCE Setup                          */
			/***********************************************************/
			else if (WhichDisplay == DS_DBOUNCE) {
				//unsigned long hh, mm;

				calcMenu(MENU_SIZE_DBOUNCE);

				if ((MENU_POS == 1) && (buttonState == BT_LEFT)) {
					if (AQUA.S_DBOUNCE >= 1000)
						AQUA.S_DBOUNCE = AQUA.S_DBOUNCE-1000;
					else
						AQUA.S_DBOUNCE = 32000;
				} else if ((MENU_POS == 1) && (buttonState == BT_RIGHT)) {
					if (AQUA.S_DBOUNCE  <= 32000)
						AQUA.S_DBOUNCE  = AQUA.S_DBOUNCE + 1000;
					else
						AQUA.S_DBOUNCE  = 0;
				}

				if ((MENU_POS == 2) && (buttonState == BT_LEFT)) {
					if (FWTANK_TOP.S_DBOUNCE >= 1000)
						FWTANK_TOP.S_DBOUNCE = FWTANK_TOP.S_DBOUNCE-1000;
					else
						FWTANK_TOP.S_DBOUNCE = 32000;
					} 
				else if ((MENU_POS == 2) && (buttonState == BT_RIGHT)) {
					if (FWTANK_TOP.S_DBOUNCE <= 32000)
						FWTANK_TOP.S_DBOUNCE = FWTANK_TOP.S_DBOUNCE + 1000;
					else
						FWTANK_TOP.S_DBOUNCE = 0;
				}
				
				if ((MENU_POS == 3) && (buttonState == BT_LEFT)) {
					if (FWTANK_BOT.S_DBOUNCE >= 1000)
						FWTANK_BOT.S_DBOUNCE = FWTANK_BOT.S_DBOUNCE-1000;
					else
						FWTANK_BOT.S_DBOUNCE = 32000;
				}
				else if ((MENU_POS == 3) && (buttonState == BT_RIGHT)) {
					if (FWTANK_BOT.S_DBOUNCE <= 32000)
						FWTANK_BOT.S_DBOUNCE = FWTANK_BOT.S_DBOUNCE + 1000;
					else
						FWTANK_BOT.S_DBOUNCE = 0;
				}

				
				if ((buttonState == BT_ENTER) && (MENU_POS == 4)) { // Back to SetupMenu
					MENU_POS = 1;
					WhichDisplay = DS_MENU;
				}
				
				else if ((buttonState == BT_ENTER) && (MENU_POS == 5)) { // Save Adr Modbus
					MENU_POS = 1;
					ee_Config.OSMOSE_DBOUNCE_AQUA	= AQUA.S_DBOUNCE;
					ee_Config.OSMOSE_DBOUNCE_TOP	= FWTANK_TOP.S_DBOUNCE;
					ee_Config.OSMOSE_DBOUNCE_BOTTOM	= FWTANK_BOT.S_DBOUNCE;
					WriteMarelabConfig();	
					WhichDisplay = DS_MENU;
				}
			}
			/***********************************************************/

			/***********************************************************/
			/* Menu Section for CLEANING Setup                          */
			/***********************************************************/
			else if (WhichDisplay == DS_CLEANING) {
				calcMenu(MENU_SIZE_CLEANING);
				// Max 10 min for a cleaning
				if ((MENU_POS == 1) && (buttonState == BT_LEFT)) {
					if (ee_Config.OSMOSE_CLEAN_DURATION >= 2)
						ee_Config.OSMOSE_CLEAN_DURATION = ee_Config.OSMOSE_CLEAN_DURATION-1;
					else
						ee_Config.OSMOSE_CLEAN_DURATION = 300;
					} 
					else if ((MENU_POS == 1) && (buttonState == BT_RIGHT)) {
						if (ee_Config.OSMOSE_CLEAN_DURATION  <= 300)
							ee_Config.OSMOSE_CLEAN_DURATION = ee_Config.OSMOSE_CLEAN_DURATION + 1;
						else
							ee_Config.OSMOSE_CLEAN_DURATION  = 1;
				}
				// Repeat min 30 min max 240 min
				if ((MENU_POS == 2) && (buttonState == BT_LEFT)) {
					if (ee_Config.OSMOSE_CLEAN_REPEAT >= 31)
						ee_Config.OSMOSE_CLEAN_REPEAT = ee_Config.OSMOSE_CLEAN_REPEAT-1;
					else
						ee_Config.OSMOSE_CLEAN_REPEAT = 240;
				}
				else if ((MENU_POS == 2) && (buttonState == BT_RIGHT)) {
					if (ee_Config.OSMOSE_CLEAN_REPEAT <= 240)
						ee_Config.OSMOSE_CLEAN_REPEAT = ee_Config.OSMOSE_CLEAN_REPEAT + 1;
					else
						ee_Config.OSMOSE_CLEAN_REPEAT = 30;
				}
				
				// Dimm BACK
				if ((buttonState == BT_ENTER) && (MENU_POS == 3)) { // Back to SetupMenu
					MENU_POS = 1;
					WhichDisplay = DS_MENU;
				}
				// Dimm SAVE
				else if ((buttonState == BT_ENTER) && (MENU_POS == 4)) { // Save Adr Modbus
					MENU_POS = 1;
					WriteMarelabConfig();
					WhichDisplay = DS_MENU;
				}
			}
			/***********************************************************/
			
			/***********************************************************/
			/* Menu Section for SAFETY Setup                          */
			/***********************************************************/
			else if (WhichDisplay == DS_SAFETY) {
				calcMenu(MENU_SIZE_CLEANING);
				
				if ((MENU_POS == 1) && (buttonState == BT_LEFT)) {
					if (ee_Config.OSMOSE_MAX_CYCLE_TIME >= 1)
					ee_Config.OSMOSE_MAX_CYCLE_TIME = ee_Config.OSMOSE_MAX_CYCLE_TIME-1;
					else
					ee_Config.OSMOSE_MAX_CYCLE_TIME = 32000;
				}
				else if ((MENU_POS == 1) && (buttonState == BT_RIGHT)) {
					if (ee_Config.OSMOSE_MAX_CYCLE_TIME  <= 32000)
					ee_Config.OSMOSE_MAX_CYCLE_TIME = ee_Config.OSMOSE_MAX_CYCLE_TIME + 1;
					else
					ee_Config.OSMOSE_MAX_CYCLE_TIME  = 0;
				}
				
				if ((MENU_POS == 2) && (buttonState == BT_LEFT)) {
					if (ee_Config.OSMOSE_MAX_REFILL_TANK >= 1)
					ee_Config.OSMOSE_MAX_REFILL_TANK = ee_Config.OSMOSE_MAX_REFILL_TANK-1;
					else
					ee_Config.OSMOSE_MAX_REFILL_TANK = 120;
				}
				else if ((MENU_POS == 2) && (buttonState == BT_RIGHT)) {
					if (ee_Config.OSMOSE_MAX_REFILL_TANK <= 120)
					ee_Config.OSMOSE_MAX_REFILL_TANK = ee_Config.OSMOSE_MAX_REFILL_TANK + 1;
					else
					ee_Config.OSMOSE_MAX_REFILL_TANK = 0;
				}
				
				// Dimm BACK
				if ((buttonState == BT_ENTER) && (MENU_POS == 3)) { // Back to SetupMenu
					MENU_POS = 1;
					WhichDisplay = DS_MENU;
				}
				// Dimm SAVE
				else if ((buttonState == BT_ENTER) && (MENU_POS == 4)) { // Save Adr Modbus
					MENU_POS = 1;
					WriteMarelabConfig();
					WhichDisplay = DS_MENU;
				}
			}
			/***********************************************************/


			/***********************************************************/
			/* Menu Section for Mode Setup                          */
			/***********************************************************/
			else if (WhichDisplay == DS_MODE) {
				calcMenu(MENU_SIZE_MODE);
				
				// Manual
				if (((buttonState == BT_LEFT)||(buttonState == BT_RIGHT)) && (MENU_POS == 1)) { // Modbus ID minus
					if (ee_Config.OSMOSE_MODE == OSMOSE_MODE_MANUEL) {
						ee_Config.OSMOSE_MODE = OSMOSE_MODE_AUTO;
					}else{
						ee_Config.OSMOSE_MODE = OSMOSE_MODE_MANUEL;
					}
				}
				
				if (((buttonState == BT_LEFT)||(buttonState == BT_RIGHT)) && (MENU_POS == 2)) { // Modbus ID minus
					if (ee_Config.OSMOSE_MODE == OSMOSE_MODE_MANUEL) {
						ee_Config.OSMOSE_MODE = OSMOSE_MODE_AUTO;
					}else{
						ee_Config.OSMOSE_MODE = OSMOSE_MODE_MANUEL;
					}
				}	
				
				// MODE BACK
				if ((buttonState == BT_ENTER) && (MENU_POS == 3)) { // Back to SetupMenu
					MENU_POS = 1;
					WhichDisplay = DS_MENU;
				}
				// MODE SAVE
				else if ((buttonState == BT_ENTER) && (MENU_POS == 4)) { // Save Adr Modbus
					MENU_POS = 1;
					WriteMarelabConfig();
					WhichDisplay = DS_MENU;
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

void reset_Params(void)
{
	//Restore to default set of parameters!
	ee_Config.WriteCheck				= Write_Check;
	ee_Config.eCLowCal					= 200; //assume ideal probe and amp conditions 1/2 of 4096
	ee_Config.eCHighCal					= 1380; //using ideal probe slope we end up this many 12bit units away on the 4 scale
	ee_Config.MARELAB_MODUS				= ACC_STANDALONE;
	ee_Config.OSMOSE_MODE				= OSMOSE_MODE_AUTO; //OSMOSE_MODE_MANUEL;
	// Default Debounce settings
	ee_Config.OSMOSE_DBOUNCE_AQUA		= 3000;		// ms time
	ee_Config.OSMOSE_DBOUNCE_TOP		= 3000;		// ms time
	ee_Config.OSMOSE_DBOUNCE_BOTTOM		= 3000;		// ms time
	// Membran Cleaning
	ee_Config.OSMOSE_CLEAN_DURATION		= 15;		// Time in sec the CleanVent is opened to clean the osmosis membran
	ee_Config.OSMOSE_CLEAN_REPEAT		= 1;		// Time in min after a new membran cleaning is initiated
	// Saftey Feature
	ee_Config.OSMOSE_MAX_CYCLE_TIME		= 10;		// Time a complete filling of the OSMOSIS Tank can take min
	ee_Config.OSMOSE_MAX_REFILL_TANK	= 20;		// Max time a tank refill with osmosis water can take sec
	// Screen Saver Time for activate
	ee_Config.SCREENSAVER_TIMEOUT		= 120;		// Time until Screensaver gets activated
	
	WriteMarelabConfig();
}

void calcEc(int raw)
{
	float tempmv, tempgain, Rprobe;
	tempmv = (float)adc->calcMillivolts(raw);
	tempgain = (tempmv / (float)oscV) - 1.0; // what is our overall gain again so we can cal our probe leg portion
	Rprobe = (Rgain / tempgain); // this is our actually Resistivity
	//adcAVG
	adcAVG = ((1000000) * kCell) / Rprobe; // this is where we convert to uS inversing so removed neg exponant
	//adcAVG = temp / 1000.0; //convert to EC from uS
}

void UpdateSensor(struct TSENSOR &sensor)
{
	if (digitalRead(sensor.S_PORT) != sensor.S_LAST_STATE ){
		sensor.S_STABLE = millis();
		sensor.S_LAST_STATE = digitalRead(sensor.S_PORT);
		return;	
	}
	//Senor stable over Time DBOUNCE?
	if( (millis() - sensor.S_STABLE) >= sensor.S_DBOUNCE){
		sensor.S_STATE = sensor.S_LAST_STATE;
		sensor.S_STABLE = millis();
	}
}

void setup() {
	
	/* Reading EEPROM									*/
	ReadMarelabConfig();
	if (ee_Config.WriteCheck != Write_Check){
		reset_Params();
	}
	ERROR_STATE = 0;
	// the setup routine runs once when you press reset:
	pinMode(SAQUA,INPUT);
	AQUA.S_PORT				= SAQUA;
	AQUA.S_DBOUNCE			= ee_Config.OSMOSE_DBOUNCE_AQUA;
	AQUA.S_STABLE			= millis();
	AQUA.S_LAST_STATE		= 0;
	AQUA.S_STATE			= 0;
	
	pinMode(SOHIGH,INPUT);
	FWTANK_TOP.S_PORT		= SOHIGH;
	FWTANK_TOP.S_DBOUNCE	= ee_Config.OSMOSE_DBOUNCE_TOP;
	FWTANK_TOP.S_STABLE		= millis();
	FWTANK_TOP.S_LAST_STATE = 0;
	FWTANK_TOP.S_STATE		= 0;
	
	pinMode(SOLOW,INPUT);
	FWTANK_BOT.S_PORT		= SOLOW;
	FWTANK_BOT.S_DBOUNCE	= ee_Config.OSMOSE_DBOUNCE_BOTTOM;
	FWTANK_BOT.S_STABLE		= millis();
	FWTANK_BOT.S_LAST_STATE = 0;
	FWTANK_BOT.S_STATE		= 0;
	
	osmose_active = FALSE;
	pump_active = FALSE;
	osmose_active_firsttime=TRUE;
//	boolean CleanOn = FALSE;
	OSMOSE_START_TIME	= 0;
	OSMOSE_RUN_TIME		= 0;
	REFILL_START_TIME	= 0;
	REFILL_RUN_TIME		= 0;

	
	
	
	digitalWrite(RESET_PIN, HIGH);
	delay(100);
	pinMode(RESET_PIN, OUTPUT);
	//unsigned long timeout = SCREEN_SAVER_TIMEOUT * 1000UL;

	
	// Initialize the Pca9554 library
	Wire.begin();
	adc = new MCP3221(i2cAdrADC, I2CadcVRef);
	
	Pca9554.begin();
	// Initialization could also be done like this
	// Set pin mode of pin 0 to output
	
	Pca9554.pinMode(0, OUTPUT);
	Pca9554.pinMode(1, OUTPUT);
	Pca9554.pinMode(2, OUTPUT);
	Pca9554.pinMode(3, OUTPUT);
	
	// Make it low
	Pca9554.digitalWrite(0, LOW);
	Pca9554.digitalWrite(1, LOW);
	Pca9554.digitalWrite(2, LOW);
	Pca9554.digitalWrite(3, LOW);


	WhichDisplay = DS_LOGO;
	screen.begin();
	draw();
	delay(1000);
	WhichDisplay=DS_MAIN;
	MENU_POS=1;
		
	mb_reg[MARELAB_DEVICE_ID]		= MARELAB_TYPE; /* Set the device ID to get identified 	*/
	mb_reg[MARELAB_VERSION]			= MARELAB_FIRWMARE; /* Set the Firmware Version ID 			*/
	mb_reg[MARELAB_REGISTER_COUNT]	= MODBUS_REGISTER_COUNT; /* Stores the amount of registers for modbus */

	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//ee_Config.MARELAB_MODUS = ACC_STANDALONE;					/* Modbus & WLAN Access = 0 */	
	mb_reg[MCOMMAND] = 0;

	//modbusino_slave.setup(MODBUS_BAUD);		/* ModBus Baud Rate			*/
	//modbus_configure(MODBUS_BAUD, ee_Config.MODBUS_ID, TX_RS485_PIN,MODBUS_REGISTER_COUNT, 0);
	pinMode(TX_RS485_PIN, OUTPUT);
	digitalWrite(TX_RS485_PIN, LOW);
	modbus_configure(MODBUS_BAUD, 2, TX_RS485_PIN,MODBUS_REGISTER_COUNT, 0);

	//modbus_configure(19200, 2,TX_RS485_PIN,MODBUS_REGISTER_COUNT, 0);
	ee_Config.MARELAB_MODUS = ACC_MOD;

	//pinMode(LED_PIN, OUTPUT);  			// Set ledPin mode
	pinMode(BUTTON_ANALOG_PIN, INPUT); 	// Joystick Analog Input
	MENU_POS = 1;

	#ifdef TWBR
	  // save I2C bitrate
	  uint8_t twbrbackup = TWBR;
	  // must be changed after calling Wire.begin() (inside pwm.begin())
	  TWBR = 12; // upgrade to 400KHz!
	#endif
}


uint8_t count=0;
unsigned long LAST_LEITWERT=0;

// TEST
void loop() {
	byte GlobalStatTmp;
	mb_reg[MCOMMAND] = 0; /* after each loop setting the modbus command register to 0 */
	modbus_update(mb_reg);
	//if (modbus_update(mb_reg) < 0)
    //		digitalWrite(13, LOW);
	//else
	//	digitalWrite(13, HIGH);
		
	
	
	//*/
	// Command Interpreter
	// Executes the command in transfered in the command
	// register. After executen it is set to 0
	/* MCOMMAND -> 1  = getRTC Time			*/
	if (mb_reg[MCOMMAND] == 1) {

	}
	
	/* 
		MCOMMAND -> 2  = BOUNCE & STABLE VALUES from MODBUS
		 Register into vars	
	*/
	else if (mb_reg[ MCOMMAND] == 2) {
		/*	AQUA.S_DBOUNCE			= mb_reg[MOD_DBOUNCE_AQUA]*1000;
			AQUA.S_STABLE			= mb_reg[MOD_STABLES_AQUA]*1000;		
			FWTANK_TOP.S_DBOUNCE	= mb_reg[MOD_DBOUNCE_TANKTOP]*1000;
			FWTANK_TOP.S_STABLE		= mb_reg[MOD_STABLES_TANKTOP]*1000;
			FWTANK_BOT.S_DBOUNCE	= mb_reg[MOD_DBOUNCE_TANKBOT]*1000;
			FWTANK_BOT.S_STABLE		= mb_reg[MOD_STABLES_TANKBOT]*1000;
			*/
	}
	else if (mb_reg[ MCOMMAND] == 10) {
	}
	else if (mb_reg[ MCOMMAND] == 11) {
	}

	/* RESET FOR BOOTLOADING */
	else if (mb_reg[ MCOMMAND] == 200) {
		///MsTimer2::stop();
		WhichDisplay = 2;
		draw();
		digitalWrite(RESET_PIN, LOW);
	}
	
	buttonState = BT_NULL;        // Reset before the next check
	// Sensor Actor work
	
	GlobalStatTmp	= (AQUA.S_STATE<<0)			| (FWTANK_BOT.S_STATE<<1)		| (FWTANK_TOP.S_STATE<<2);
	
	UpdateSensor(AQUA);
	UpdateSensor(FWTANK_BOT);
	UpdateSensor(FWTANK_TOP);
	
	GlobalStateSensor = 0;
	   
	 /* Logic Table to control Pumps & Vents												*/
	 /* ---------------------------------------------------------------------------------*/
	 /*					SOHigh	SOLow	SAQUA											*/
	 /* 0 Osmose Empty	  0		  0			0   PumpOsmose VentFresh ON PumpRefill OFF	*/
	 /* 1 Osmose Empty	  0	  	  0			1   PumpOsmose VentFresh ON PumpRefill OFF	*/
	 /* 2 Osmose OK		  0		  1			0   PumpOsmose VentFresh ON PumpRefill ON	*/
	 /* 3 Osmose OK		  0		  1			1	All OFF IDLE							*/
	 /* 4 ERROR			  1		  0			0   All OFF									*/
	 /* 5 ERROR		      1		  0			1   All OFF									*/
	 /* 6 Osmose OK		  1		  1			0   PumpOsmose VentFresh OFF PumpRefill ON	*/
	 /* 7 Osmose OK		  1		  1			1   PumpOsmose VentFresh OFF PumpRefill OFF	*/  
	 
	//GlobalStateSensor		= (AQUA.S_LAST_STATE<<0)	| (FWTANK_BOT.S_LAST_STATE<<1)	| (FWTANK_TOP.S_LAST_STATE<<2);
	GlobalStateSensor		= (AQUA.S_STATE<<0)			| (FWTANK_BOT.S_STATE<<1)		| (FWTANK_TOP.S_STATE<<2);
	if (GlobalStatTmp != GlobalStateSensor)
		GlobalStateSensorOld = GlobalStatTmp;
	 
	 if (ee_Config.OSMOSE_MODE == OSMOSE_MODE_AUTO){
		 if	  (GlobalStateSensor == 0)
		 {
			osmose_active = TRUE;
			pump_active   = FALSE;	 		  
		 }
		 else if(GlobalStateSensor == 1)
		 {
			 osmose_active = TRUE;
			 pump_active   = FALSE;
		 }
		 else if(GlobalStateSensor == 2)
		 {
			if (GlobalStateSensorOld==0 || GlobalStateSensorOld == 1){ // wenn von voll nach leer dann nicht aktivieren
				osmose_active = TRUE;
				pump_active   = TRUE;
			}else{
				pump_active   = TRUE;    
			}
		 }
		 else if(GlobalStateSensor == 3)
		 {
			if (GlobalStateSensorOld==0 || GlobalStateSensorOld == 1){ // wenn von voll nach leer dann nicht aktivieren
				osmose_active = TRUE;
				pump_active   = FALSE;	  
			}
			else{
				pump_active   = FALSE;
			}
		 }
		 else if(GlobalStateSensor == 4)
		 {
			 osmose_active	= FALSE;
			 pump_active	= FALSE;	
			 ERROR_STATE = 1;	
		 }
		 else if(GlobalStateSensor == 5)
		 {
			 osmose_active	= FALSE;
			 pump_active	= FALSE;
			 ERROR_STATE = 1;	 
		 }
		 else if(GlobalStateSensor == 6)
		 {
			 osmose_active = FALSE;
			 pump_active   = TRUE;
		 }
		 else if(GlobalStateSensor == 7)
		 {
			 osmose_active = FALSE;
			 pump_active   = FALSE;		 
		 }
	 }
	 
	/* REFILL TIME CONTROL */
	if ((pump_active) && (REFILL_START_TIME == 0))
	{
		REFILL_START_TIME = millis();
	}else
	{
		REFILL_RUN_TIME = millis()-REFILL_START_TIME;
	}
	if (!pump_active){
		REFILL_START_TIME = 0;
		REFILL_RUN_TIME   = 0;
	}
	
	/* OSMOSE STEUERUNG    */
	// FirstTime
	if (osmose_active && osmose_active_firsttime){
		OSMOSE_START_TIME = millis();
		CleanOn = TRUE;
		osmose_active_firsttime = FALSE;
		CLEAN_START_TIME = OSMOSE_START_TIME;
	}
	
	// OSMOSE is Running
	if (osmose_active){
		OSMOSE_RUN_TIME = millis()-OSMOSE_START_TIME;
	}
	else
	{
		OSMOSE_START_TIME = millis();
		CleanOn = FALSE;
		osmose_active_firsttime = TRUE;
	}
	
	/* CLEANING STEUERUNG STOP CLEANING*/
	if (osmose_active && (millis() >= (CLEAN_START_TIME + (ee_Config.OSMOSE_CLEAN_DURATION*1000)))) {
		CleanOn = FALSE;
		CLEAN_START_TIME = millis() + (ee_Config.OSMOSE_CLEAN_REPEAT*60*1000) - (ee_Config.OSMOSE_CLEAN_DURATION*1000);
	}
	else if (osmose_active && (millis() >= CLEAN_START_TIME) &&  ((CLEAN_START_TIME + (ee_Config.OSMOSE_CLEAN_DURATION*1000)) >= millis()))
	{
		CleanOn = TRUE;
	}
	
	
	/* CHECK IF SENSOR ERROR OR RUNTIME OF OSMOSE(MIN) OR REFILL IS TO LONG */
	if ((GlobalStateSensor==4) || (GlobalStateSensor==5) || ((OSMOSE_RUN_TIME/1000/60) >= ee_Config.OSMOSE_MAX_CYCLE_TIME) || ((REFILL_RUN_TIME/1000)>ee_Config.OSMOSE_MAX_REFILL_TANK))
	{
		if ((GlobalStateSensor==4) || (GlobalStateSensor==5))
			ERROR_STATE = 1;
		else
			ERROR_STATE = 2;
		// SENSOR FAILURE SO WE SET ERROR AND ALL OFF
		Pca9554.digitalWrite(PUMP_VENT_CH,		LOW);
		Pca9554.digitalWrite(PUMP_REFILL,		LOW);
		Pca9554.digitalWrite(VENT_CLEAN,		LOW);
	}else{
		ERROR_STATE = 0;
		if (osmose_active)
			Pca9554.digitalWrite(PUMP_VENT_CH,		HIGH);
		else
			Pca9554.digitalWrite(PUMP_VENT_CH,		LOW);
	
		if (pump_active)
			Pca9554.digitalWrite(PUMP_REFILL,		HIGH);
		else
			Pca9554.digitalWrite(PUMP_REFILL,		LOW);
		if (CleanOn)
			Pca9554.digitalWrite(VENT_CLEAN,		HIGH);
		else
			Pca9554.digitalWrite(VENT_CLEAN,		LOW);
	}
	////////////////////////////////////////////////////////////////////////////////
	

	//LEITWERT ADC
	
	//You can also use the MCP3221 library at scope level letting instances get created and destroyed as needed
	//Many instances can be ran as well to match the number of devices on the bus.)
	//MCP3221 secondI2CADC(altI2CAddress, I2CadcVRef);
	
	if (LAST_LEITWERT > 500){
		//adcAVG = adc->calcRollingAVG();	//This accumulates over time dropping the oldest and shifting an array adding int he newest set for 10 in library .h can be upped keep in mind ram usage#
		int adcRaw = adc->calcRollingAVG();
		calcEc(adcRaw);
		LAST_LEITWERT = 0;
	}else
	{
		LAST_LEITWERT = millis()-LAST_LEITWERT;
	}
	
	
	/****************************************************/
	/* DISPLAY UPDATE									*/
	/****************************************************/
	DoMenu();
	draw(); // Redraw display
}