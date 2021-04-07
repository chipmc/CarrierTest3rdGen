/*
* Project CarrierTest
* Description: Tests the 3rd Generation Device Carrier Boron / Xenon / Argon Carrier
* Author: Charles McClelland
* Date: Started 11-17-2019 
* 
* Implements the following Tests
* 1 - Test i2c bus and FRAM functionality
* 2 - Test the TMP-36 for temperature
* 3 - Test the User Switch - Requires physical Press
* 4 - Test that the RTC is keeping time
* 5 - Test an alarm on the RTC
* 6 - Test Battery charging - Can take a while based on state of charge
* 7 - Test Deep Sleep functionality
* 
* v0.10 - Initial release under version control
* v0.20 - Added the MCP79410 Library
* v0.30 - Redid the program structure
* v0.40 - Moved to the new library for FRAM
* v0.50 - Changed pin definitions for the new v1.2 Boron Carrier
* v0.60 - Added i2c scan and improved the RTC alarm testing
* v0.65 - Added signal strength messaging at connection.  This is Boron Specific / need to fix this for Xenon / Argon
* v0.70 - Updated for new carrier board 1.3
* v1.00 - Updated for new carrier board 1.5 
* Using the Watchdog App note - https://github.com/particle-iot/app-notes/tree/master/AN023-Watchdog-Timers
* API Documentation - https://rickkas7.github.io/AB1805_RK/
*/


// Particle Product definitions
PRODUCT_ID(12529);                                  // Boron Connected Counter Header
PRODUCT_VERSION(2);
char currentPointRelease[5] ="2.00";


namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Where we store the memory map version number
    controlRegisterAddr   = 0x01,
    currentTestAddr       = 0x02,                   // What test are we on?  Some perform reset
    timeZoneAddr          = 0x03,                   // Store the local time zone data
    tomeZoneOffsetAddr    = 0x04,                   // Store the DST offset value 0 - 2
    deepSleepStartAddr    = 0x05,                   // Time we started the deep sleep test
    testdataAddr          = 0x09,
  };
};

const int FRAMversionNumber = 1;

// Included Libraries
#include "3rdGenDevicePinoutdoc.h"                 // Documents pinout
#include "MB85RC256V-FRAM-RK.h"
#include "AB1805_RK.h"

SerialLogHandler logHandler;                        // For RTC alerts and events

// This is the maximum amount of time to allow for connecting to cloud. If this time is
// exceeded, do a deep power down. This should not be less than 10 minutes. 11 minutes
// is a reasonable value to use.
const std::chrono::milliseconds connectMaxTime = 11min;

// Prototypes and System Mode calls
SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);         // Means my code will not be held up by Particle processes.
FuelGauge batteryMonitor;       // Prototype for the fuel gauge (included in Particle core library)
MB85RC64 fram(Wire, 0);
AB1805 ab1805(Wire);

enum State {INITIALIZATION_STATE, I2C_SCAN, FRAM_TEST, TMP36_TEST, USERSW_TEST, RTCTIME_TEST, RTCALARM_TEST, CHARGING_TEST, DEEPSLEEP_TEST, ERROR_STATE, IDLE_STATE};
State state = INITIALIZATION_STATE;

// Pin Constants for Boron
const int blueLED  = D7;                                         // This LED is on the Electron itself
const int userSwitch = D4;                                       // User switch with a pull-up resistor
const int tmp36Pin = A4;                                         // Simple Analog temperature sensor
const int wakeUpPin = D8;                                        // This is the Particle Electron WKP pin

// Pin Constants - Sensor
const int intPin =        SCK;                      // Pressure Sensor inerrupt pin
const int disableModule = MOSI;                     // Bringining this low turns on the sensor (pull-up on sensor board)
const int ledPower =      MISO;                     // Allows us to control the indicator LED on the sensor board

// This section is where we will initialize sensor specific variables, libraries and function prototypes
// Pressure Sensor Variables
volatile bool sensorDetect = false;                 // This is the flag that an interrupt is triggered
Timer countSignalTimer(1000, countSignalTimerISR, true);  // This is how we will ensure the BlueLED stays on long enough for folks to see it.

// Program Variables
byte currentState;                                               // Store the current state for the tests that might cause a reset
volatile bool watchdogInterrupt = false;                         // variable used to see if the watchdogInterrupt had fired
char resultStr[64];
char SignalString[64];                                           // Used to communicate Wireless RSSI and Description
byte rebootOrNot;
int outOfMemory = -1;
bool cloudConnected = false;
uint64_t cloudConnectStarted = 0;


// setup() runs once, when the device is first turned on.
void setup() {
  pinMode(userSwitch,INPUT);                                      // Button for user input
  pinMode(blueLED, OUTPUT);                                       // declare the Blue LED Pin as an output

  // Pressure / PIR Module Pin Setup
  pinMode(intPin,INPUT_PULLDOWN);                   // pressure sensor interrupt
  pinMode(disableModule,OUTPUT);                    // Turns on the module when pulled low
  pinMode(ledPower,OUTPUT);                         // Turn on the lights

  digitalWrite(disableModule,false);                                // Enable or disable the sensor
  digitalWrite(ledPower,HIGH);                                  // For the pressure sensor, this is how you activate it

  Particle.variable("Release",currentPointRelease);
  Particle.variable("Signal", SignalString);
  
  // The sample board has D8 connected to FOUT for wake interrupts
  ab1805.withFOUT(wakeUpPin).setup();

  // Note whether the RTC is set before calling resetConfig() as this will make
  // isRTCSet return false.
  bool rtcSet = ab1805.isRTCSet();

  // Reset the AB1805 configuration to default values
  ab1805.resetConfig();

  // Enable watchdog
  ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);

  // The wakeReason is set during setup() so it's safe to call it after resetConfig.
  AB1805::WakeReason wakeReason = ab1805.getWakeReason();
  if (wakeReason == AB1805::WakeReason::DEEP_POWER_DOWN) {
      Log.info("woke from DEEP_POWER_DOWN");
  }
  else
  if (wakeReason == AB1805::WakeReason::ALARM) {
      // We were wakened by the alarm
      Particle.publish("Startup","woke by alarm (periodic interrupt)",PRIVATE);
  }
  else
  if (!rtcSet) {
      // RTC has not been set, get time from the cloud
      Particle.publish("Startup","RTC not set yet, getting time from cloud",PRIVATE);
  }

  waitUntil(Particle.connected);

  setPowerConfig();                                                 // Enables faster charging

  fram.begin();                                                    // Initializes Wire but does not return a boolean on successful initialization

  rebootOrNot = EEPROM.read(0);

  if (rebootOrNot==1) {
    EEPROM.write(0,0);
    ab1805.stopWDT();
    waitUntil(meterParticlePublish);
    Particle.publish("Update","Deep sleep test complete - all tests passed", PRIVATE);
  }
  else {
    waitUntil(meterParticlePublish);
    Particle.publish("Status", "Beginning Test Run",PRIVATE);
  }

  state = IDLE_STATE;

  attachInterrupt(intPin, sensorISR, RISING);                       // Pressure Sensor interrupt from low to high
}


void loop() {
  // Be sure to call ab1805.loop() on every call to loop()
  ab1805.loop();
  
  switch (state) {
    case IDLE_STATE: {
      if (rebootOrNot==1) slowFlash();
      else state = I2C_SCAN;
    } break;
    case I2C_SCAN:
      i2cScan() ? state = FRAM_TEST : state = ERROR_STATE;
      waitUntil(meterParticlePublish);
      Particle.publish("Result",resultStr, PRIVATE);
    break;
    case FRAM_TEST:
      framTest() ? state = TMP36_TEST : state=ERROR_STATE;
      waitUntil(meterParticlePublish);
      Particle.publish("Result",resultStr,PRIVATE);
    break;
    case TMP36_TEST:
      getTemperature() ? state = USERSW_TEST : state = ERROR_STATE;
      waitUntil(meterParticlePublish);
      Particle.publish("Result",resultStr,PRIVATE);
    break;
    case USERSW_TEST: {                                                             // Test the user switch
      static bool firstPublish = false;
      if (!firstPublish) {
        waitUntil(meterParticlePublish);
        Particle.publish("Prompt","Please press user switch", PRIVATE);
        firstPublish = true;
      }
      if (digitalRead(userSwitch) == LOW) {
        waitUntil(meterParticlePublish);
        Particle.publish("Result","Switch Test Passed - Press detected", PRIVATE);
        state = RTCTIME_TEST;
      }
    } break;
    case RTCTIME_TEST:
      rtcClockTest() ? state = RTCALARM_TEST : state = ERROR_STATE;
      waitUntil(meterParticlePublish);
      Particle.publish("Result",resultStr, PRIVATE);
    break;
    case RTCALARM_TEST: 
      rtcAlarmTest() ? state = CHARGING_TEST : state = ERROR_STATE;
      waitUntil(meterParticlePublish);
      Particle.publish("Result",resultStr, PRIVATE);
    break;
    case CHARGING_TEST:
      if (batteryChargeTest()) {
        waitUntil(meterParticlePublish);
        Particle.publish("Result",resultStr, PRIVATE);
        state = DEEPSLEEP_TEST;
      }
    break;
    case DEEPSLEEP_TEST:
      EEPROM.write(0,1);                                              // Since we will reboot, put a flag in EEPROM
      waitUntil(meterParticlePublish);            
      Particle.publish("Information","Deep Sleep Test - for 10 seconds starts in 5 seconds",PRIVATE);
      delay(5000);
      ab1805.deepPowerDown(10);
      delay(11000);
      state = ERROR_STATE;                                             // If test is successful we will not get to this step
    break;
    case ERROR_STATE: 
      waitUntil(meterParticlePublish);
      Particle.publish("Error","Testing halted",PRIVATE);
      state = IDLE_STATE;
    break;
    default:
      state = IDLE_STATE;
    break;
  }

  // Monitor the cloud connection state and do a deep power down if a 
  // failure to connect exceeds connectMaxTime (typically 11 minutes).
  if (Particle.connected()) {
      if (!cloudConnected) {
          cloudConnected = true;
          uint32_t elapsed = (uint32_t)(System.millis() - cloudConnectStarted);
          Log.info("cloud connected in %lu ms", elapsed);
      }
  }
  else {
      if (cloudConnected) {
          cloudConnected = false;
          cloudConnectStarted = System.millis();
          Log.info("lost cloud connection");
      }
      uint32_t elapsed = (uint32_t)(System.millis() - cloudConnectStarted);
      if (elapsed > connectMaxTime.count()) {
          Log.info("failed to connect to cloud, doing deep reset");
          delay(100);
          ab1805.deepPowerDown();
      }
  }
}


void sensorISR() {
  digitalWrite(blueLED,HIGH);
  countSignalTimer.resetFromISR();
}

void countSignalTimerISR() {
  digitalWrite(blueLED,LOW);
}

bool i2cScan() {                                            // Scan the i2c bus and publish the list of devices found
	byte error, address;
	int nDevices = 0;
  strncpy(resultStr,"i2c device(s) found at: ",sizeof(resultStr));

	for(address = 1; address < 127; address++ )
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0)
		{
      char tempString[4];
      snprintf(tempString, sizeof(tempString), "%02X ",address);
      strncat(resultStr,tempString,4);
			nDevices++;
      if (nDevices == 9) break;                    // All we have space to report in resultStr
		}

		else if (error==4) {
      snprintf(resultStr,sizeof(resultStr),"Unknown error at address %02X", address);
      return 0;
		}
	}

	if (nDevices == 0) {
    snprintf(resultStr,sizeof(resultStr),"No I2C devices found");
    return 0;
  }

  return 1;
}

// Power Management function
void setPowerConfig() {
  SystemPowerConfiguration conf;
  System.setPowerConfiguration(SystemPowerConfiguration());  // To restore the default configuration

  conf.powerSourceMaxCurrent(900) // Set maximum current the power source can provide (applies only when powered through VIN)
      .powerSourceMinVoltage(5080) // Set minimum voltage the power source can provide (applies only when powered through VIN)
      .batteryChargeCurrent(1024) // Set battery charge current
      .batteryChargeVoltage(4208) // Set battery termination voltage
      .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST); // For the cases where the device is powered through VIN
                                                                    // but the USB cable is connected to a USB host, this feature flag
                                                                    // enforces the voltage/current limits specified in the configuration
                                                                    // (where by default the device would be thinking that it's powered by the USB Host)
  System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
}


bool framTest() {
  int tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);
  if (tempVersion != FRAMversionNumber) {                 // Check to see if the memory map in the sketch matches the data on the chip
    fram.erase();                                                          // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr,FRAMversionNumber);                         // Put the right value in
  }

  int randomNumberWrote = random(10, 1000);
  int randomNumberRead;
  fram.put(FRAM::testdataAddr,randomNumberWrote);
  fram.get(FRAM::testdataAddr,randomNumberRead);

  if (randomNumberRead != randomNumberWrote) {
    snprintf(resultStr, sizeof(resultStr),"FRAM Test Failed - FRAM Read Error");
    return 0;
  } 
  else  {
    snprintf(resultStr, sizeof(resultStr),"FRAM Test Passed");
    return 1;
  }
}

bool getTemperature() {
  int reading = analogRead(tmp36Pin);   //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;        // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                    // Electron is different than the Arduino where there are only 1024 steps
  float temperatureC = (voltage - 0.5) * 100.0;  //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;  // now convert to Fahrenheit
  if (temperatureF < 60.0 || temperatureF > 85.0) {             // Reasonable range for room temperature
    snprintf(resultStr, sizeof(resultStr),"Temp seems whack: %3.1f", temperatureF);
    return 0;
  }
  else {
    snprintf(resultStr, sizeof(resultStr),"Temperature is: %3.1f", temperatureF);
    return 1;
  }
}

bool rtcClockTest() {
  time_t RTCTime;
  if (!ab1805.isRTCSet()) {
    snprintf(resultStr, sizeof(resultStr),"RTC Clock Test Failed");
    return 0;
  }
  else {
    ab1805.getRtcAsTime(RTCTime);
    snprintf(resultStr, sizeof(resultStr),"RTC Clock Passes - Time is %s GMT",(const char*)Time.timeStr(RTCTime));
    return 1;
  }
  delay(1000);
}

bool rtcAlarmTest() {                                                                 // RTC Alarm and Watchdog share access to Wake Pin via an OR gate
  waitUntil(meterParticlePublish);
  Particle.publish("Information", "Puts the device to sleep for 10 seconds", PRIVATE);
  delay (1000);
  time_t time;
  time_t sleepTime;
  bool testPassed = false;
  
  ab1805.getRtcAsTime(time);
  sleepTime = time +10;

  ab1805.interruptCountdownTimer(10,false);

  SystemSleepConfiguration config;
  
  config.mode(SystemSleepMode::STOP)
    .network(NETWORK_INTERFACE_CELLULAR, SystemSleepNetworkFlag::INACTIVE_STANDBY)
    .gpio(wakeUpPin, FALLING);
  System.sleep(config);

  waitUntil(Particle.connected);

  ab1805.getRtcAsTime(time);
  if (time >= sleepTime+1) testPassed = true;

  snprintf(resultStr, sizeof(resultStr),"Time = %li and sleepTime %li",(long int)time, (long int)sleepTime);
  waitUntil(meterParticlePublish);
  Particle.publish("RTCAlarm",resultStr,PRIVATE);

  if (testPassed) {
    snprintf(resultStr,sizeof(resultStr),"RTCAlarm Test has passed");
    return 1;
  }
  else {
    snprintf(resultStr,sizeof(resultStr),"RTCAlarm Test has failed");
    return 0;
  }
}

bool batteryChargeTest() {
  static bool initialMessage = false;
  static unsigned long lastUpdate = 0;
  int stateOfCharge = int(batteryMonitor.getSoC());
  static int initialChargeLevel = stateOfCharge;

  if (!initialMessage) {
    if (stateOfCharge <=80) {
      snprintf(resultStr, sizeof(resultStr), "Battery charge level starting at = %i", stateOfCharge);
      waitUntil(meterParticlePublish);
      Particle.publish("Update", resultStr, PRIVATE);
    }
    else {
      snprintf(resultStr, sizeof(resultStr), "Battery charge level %i no charge needed", stateOfCharge);
      return 1;
    }
    initialMessage = true;
  }

  if (millis() - lastUpdate >= 60000) {
    snprintf(resultStr, sizeof(resultStr), "Battery charge level = %i", stateOfCharge);
    waitUntil(meterParticlePublish);
    Particle.publish("Update", resultStr, PRIVATE);
    lastUpdate = millis();
    return 0;
  }
  else if ((stateOfCharge <= initialChargeLevel + 2)) return 0;
  else {
    snprintf(resultStr, sizeof(resultStr),"Battery Charge Test - Passed");
    return 1;
  }
}


// Utility Functions Area

void BlinkForever() {
  delay(1000);
  Particle.publish("Test Failed" "Reset Device to Continue", PRIVATE);
  while(1) {
    digitalWrite(blueLED,HIGH);
    delay(2000);
    digitalWrite(blueLED,LOW);
    delay(2000);
    Particle.process();
  }
}

bool meterParticlePublish(void) {                                       // Enforces Particle's limit on 1 publish a second
  static unsigned long lastPublish=0;                                   // Initialize and store value here
  if(millis() - lastPublish >= 1000) {                                  // Particle rate limits at 1 publish per second
    lastPublish = millis();
    return 1;
  }
  else return 0;
}

void getSignalStrength()
{
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};

  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();
 
  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
  snprintf(resultStr,sizeof(resultStr), "Connected: %s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}

void outOfMemoryHandler(system_event_t event, int param) {
    outOfMemory = param;
}

void rapidFlash() {
  static unsigned long lastChange = 0;
  static bool ledState = false;
  if ((millis() - lastChange) > 500) {
    ledState = !ledState;
    digitalWrite(blueLED,ledState);
    lastChange = millis();
  }
}

void slowFlash() {
  static unsigned long lastChange = 0;
  static bool ledState = false;
  if ((millis() - lastChange) > 2000) {
    ledState = !ledState;
    digitalWrite(blueLED,ledState);
    lastChange = millis();
  }
}