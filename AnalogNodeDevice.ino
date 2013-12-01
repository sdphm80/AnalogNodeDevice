#include <EEPROM.h>
#include <Settings.h>

#define BAUD_RATE 38400
#define ANALOG_SAMPLE_SIZE 500
#define ANALOG_SAMPLE_DELAY 2
#define DIGITAL_SAMPLE_DELAY 100
#define IO_PINS_TOTAL 12
#define IO_PINS_START 2
#define RESET_DEFAULTS_PIN 4

//***********************************************************************************************************
// Global variables and structures

char payload[50]; //Recieve Serial Buffer

//**************************************
//* Holds the settings for this system
//**************************************
struct IONodeDevSettings {
  boolean pinIsWrite[IO_PINS_TOTAL];
  boolean pinTriggerOn[IO_PINS_TOTAL];
  boolean pinTriggerOff[IO_PINS_TOTAL];
  
  int analogTirggerAverageAbove[6];
  int analogTirggerAverageBelow[6];
  int analogTriggerEnergyAbove[6];
  int analogTriggerEnergyBelow[6];
};

//****************************************
//* Holds the current and previous pin states
//****************************************
struct IOStatsStruct{
  boolean pinIsOne[IO_PINS_TOTAL];
  int analogAverage[6];
  long analogEnergy[6];
};

int lastValue[6];
long averageSum[6];
long analogEnergyCalc[6];


//**************************************
//* Init Global Variables
//**************************************
IONodeDevSettings mySettings;
IOStatsStruct currentState;
IOStatsStruct previousState;
unsigned long LastAnalogSampleTime = 0;
unsigned long LastDigitalSampleTime = 0;
unsigned int CurrentAnalogSample=0;


//***********************************************************************************************************
// Main System setup, loop and functions

//**************************************
//* Do system Setup
//**************************************
void setup(){
  Serial.begin(BAUD_RATE);
  
  //Check to see if pin is held for default load request
  CheckDefaultLoadReq();
  
  //Load Settings and set defaults if not set
  if (!EESettings::Load(&mySettings, sizeof(mySettings)) ){ LoadDefaultSettings(); }
  SetPinModeFromSettings();
  
  strcpy(payload, "");
  
}


//***********************************************************************************************************
void loop(){
  ReadDigital();
  ReadAnalog();
}


//**************************************
//* Performs actions when serial data is recieved
//**************************************
void serialEvent(){ 
  delay (20);  //Wait for full string to arrive hopefully
  char serialByte[2];
  serialByte[1] = '\0';

  while (Serial.available()) {
    serialByte[0] = (char) Serial.read();
    if (serialByte[0] == '\n'){
      handleRequest();
      payload[0] = '\0';
    }else{
      strncat( payload, serialByte, sizeof(payload) );
    }
  }
  
  
}


//**************************************
//Pushes the current state into the previous state and reads the new state of pins
//**************************************
void ReadDigital(){
  if (LastDigitalSampleTime + DIGITAL_SAMPLE_DELAY > millis()){ return; }

  
  //Move current settings into previous
  for (byte i=0; i < IO_PINS_TOTAL; i++){ previousState.pinIsOne[i] = currentState.pinIsOne[i]; }
  
  //Read new states for digital & send Triggers
  char valBuff[3];
  char strBuff[20];
  
  for (byte i=0; i < IO_PINS_TOTAL; i++){ 
    currentState.pinIsOne[i] = digitalRead(i + IO_PINS_START);
    
    //Check for pin state change
    if (LastDigitalSampleTime != 0 && currentState.pinIsOne[i] != previousState.pinIsOne[i]){
      itoa(i, valBuff, 10);
      
      //Check for on trigger
      if (mySettings.pinTriggerOn[i] && currentState.pinIsOne[i]){
        strcpy(strBuff, "TRIGGER:ON PIN:");
        strcat(strBuff, valBuff);
        Serial.println(strBuff);
      }
      
      //Check for off trigger
      if (mySettings.pinTriggerOff[i] && !currentState.pinIsOne[i]){
        strcpy(strBuff, "TRIGGER:OFF PIN:");
        strcat(strBuff, valBuff);
        Serial.println(strBuff);
      }
    }
  }
  
  LastDigitalSampleTime = millis();
}


void ReadAnalog(){
  //Read new states for analog
  if (LastAnalogSampleTime + ANALOG_SAMPLE_DELAY > millis()){ return; }
  LastAnalogSampleTime = millis();
  
  if (CurrentAnalogSample == 0){
    for (int pin=0; pin<6; pin++){
      averageSum[pin] = 0;
      analogEnergyCalc[pin] = 0;
    }
  }

  CurrentAnalogSample++;
  
  int thisPinValue;
  for (int pin=0; pin<6; pin++){
    thisPinValue = analogRead(pin);
    averageSum[pin] += thisPinValue;
    analogEnergyCalc[pin] += abs(thisPinValue - lastValue[pin]);
    lastValue[pin] = thisPinValue;
  }
  
  //Check to see if we have done all the samples we need
  if (CurrentAnalogSample < ANALOG_SAMPLE_SIZE){ return; }
  CurrentAnalogSample = 0;  
    
 
  // Move the current samples to the previous samples
  for (byte i=0; i<6; i++){ 
    previousState.analogAverage[i] = currentState.analogAverage[i];
    previousState.analogEnergy[i] = currentState.analogEnergy[i];
  }
  
  //Calculate the average and move the energy readings
  for (int pin=0; pin<6; pin++){
    currentState.analogAverage[pin] = averageSum[pin] / ANALOG_SAMPLE_SIZE;
    currentState.analogEnergy[pin] = analogEnergyCalc[pin];
  }
  
  CheckAnalogTriggers();
}



//**************************************
//
//**************************************
void CheckAnalogTriggers(){
  char strBuff[20];
  char valBuff[4];
  char valBuff2[8];

  for (int i=0; i<6; i++){
    itoa(i, valBuff, 10);
    
    //Check average above trigger
    if (previousState.analogAverage[i] < mySettings.analogTirggerAverageAbove[i] &&
        currentState.analogAverage[i] > mySettings.analogTirggerAverageAbove[i]){
      strcpy(strBuff, "TRIGGER:AA PIN:");
      strcat(strBuff, valBuff);
      strcat(strBuff, " V:");
      itoa(currentState.analogAverage[i], valBuff2, 10);
      strcat(strBuff, valBuff2);
      Serial.println(strBuff);     
    }
    
    //Check average below trigger
    if (previousState.analogAverage[i] > mySettings.analogTirggerAverageBelow[i] &&
        currentState.analogAverage[i] < mySettings.analogTirggerAverageBelow[i]){
      strcpy(strBuff, "TRIGGER:AB PIN:");
      strcat(strBuff, valBuff);
      strcat(strBuff, " V:");
      itoa(currentState.analogAverage[i], valBuff2, 10);
      strcat(strBuff, valBuff2);
      Serial.println(strBuff);     
    }
    
    //Check energy above trigger
    if (previousState.analogEnergy[i] < mySettings.analogTriggerEnergyAbove[i] &&
        currentState.analogEnergy[i] > mySettings.analogTriggerEnergyAbove[i]){
      strcpy(strBuff, "TRIGGER:EA PIN:");
      strcat(strBuff, valBuff);
      strcat(strBuff, " V:");
      itoa(currentState.analogEnergy[i], valBuff2, 10);
      strcat(strBuff, valBuff2);
      Serial.println(strBuff);     
    }
    
    //Check energy below trigger
    if (previousState.analogEnergy[i] > mySettings.analogTriggerEnergyBelow[i] &&
        currentState.analogEnergy[i] < mySettings.analogTriggerEnergyBelow[i]){
      strcpy(strBuff, "TRIGGER:EB PIN:");
      strcat(strBuff, valBuff);
      strcat(strBuff, " V:");
      itoa(currentState.analogEnergy[i], valBuff2, 10);
      strcat(strBuff, valBuff2);
      Serial.println(strBuff);     
    }    
  }
}


//***********************************************************************************************************

//**************************************
//* Analizes packet payload for commands
//**************************************
void handleRequest(){
  
  boolean cmdFound = false;
  
  if ( instr(payload, "type") ){ cmdFound = true; strcpy( payload, "OK:type Analog Node" ); }
  
  if ( instr(payload, "s_AA") ){ cmdFound = true; CmdSetAnalogAbove( payload ); strcpy( payload, "OK:s_AA" ); }
  if ( instr(payload, "s_AB") ){ cmdFound = true; CmdSetAnalogBelow( payload ); strcpy( payload, "OK:s_AB" ); }
  if ( instr(payload, "s_EA") ){ cmdFound = true; CmdSetEnergyAbove( payload ); strcpy( payload, "OK:s_EA" ); }
  if ( instr(payload, "s_EB") ){ cmdFound = true; CmdSetEnergyBelow( payload ); strcpy( payload, "OK:s_EB" ); }
  if ( instr(payload, "s_PM") ){ cmdFound = true; CmdSetPinMode( payload ); strcpy( payload, "OK:s_PM" ); }
  if ( instr(payload, "s_P1") ){ cmdFound = true; CmdSetPinTriggerOn( payload ); strcpy( payload, "OK:s_P1" ); }
  if ( instr(payload, "s_P0") ){ cmdFound = true; CmdSetPinTriggerOff( payload ); strcpy( payload, "OK:s_P2" ); }
  if ( instr(payload, "s_PS") ){ cmdFound = true; CmdSetPinState( payload ); strcpy( payload, "OK:s_PS" ); }
  if ( instr(payload, "g_PS") ){ cmdFound = true; CmdGetPinStatus( payload ); }
  if ( instr(payload, "g_AV") ){ cmdFound = true; CmdGetAnalogValues( payload ); }
  if ( instr(payload, "g_PM") ){ cmdFound = true; CmdGetSettingsPin( payload ); }
  if ( instr(payload, "g_SA") ){ cmdFound = true; CmdGetSettingsAnalog( payload ); }
  if ( instr(payload, "?") ){ cmdFound = true; CmdShowCommands( payload );  strcpy( payload, "OK" ); }
  
  
  if (!cmdFound){ Serial.println(payload); strcpy( payload, "Unkown Command" ); }
  Serial.println( payload );
}



//***********************************************************************************************************
// Commands


//**************************************
//* Command s_AA: SetAnalogAbove
//**************************************
void CmdSetAnalogAbove(char * payload){
  int id = getIntFromStr(payload, 4, ':', false);
  if (id < 0 || id > 5){ strcpy(payload, "ID OUT OF RANGE"); return;}
  mySettings.analogTirggerAverageAbove[id] = getIntFromStr(payload, 6, '\0', false);
  EESettings::Save(&mySettings, sizeof(mySettings) );
}

//**************************************
//* Command s_AB: SetAnalogBelow
//**************************************
void CmdSetAnalogBelow(char * payload){
  int id = getIntFromStr(payload, 4, ':', false);
  if (id < 0 || id > 5){ strcpy(payload, "ID OUT OF RANGE"); return;}
  mySettings.analogTirggerAverageBelow[id] = getIntFromStr(payload, 6, '\0', false);
  EESettings::Save(&mySettings, sizeof(mySettings) );
}

//**************************************
//* Command s_EA: SetEnergyAbove
//**************************************
void CmdSetEnergyAbove(char * payload){
  int id = getIntFromStr(payload, 4, ':', false);
  if (id < 0 || id > 5){ strcpy(payload, "ID OUT OF RANGE"); return;}
  mySettings.analogTriggerEnergyAbove[id] = getIntFromStr(payload, 6, '\0', false);
  EESettings::Save(&mySettings, sizeof(mySettings) );
}

//**************************************
//* Command s_EB: SetEnergyBelow
//**************************************
void CmdSetEnergyBelow(char * payload){
  int id = getIntFromStr(payload, 4, ':', false);
  if (id < 0 || id > 5){ strcpy(payload, "ID OUT OF RANGE"); return;}
  mySettings.analogTriggerEnergyBelow[id] = getIntFromStr(payload, 6, '\0', false);
  EESettings::Save(&mySettings, sizeof(mySettings) );
}

//**************************************
//* Command s_PM: SetPinMode
//**************************************
void CmdSetPinMode(char * payload){
  for (byte i=0; i < IO_PINS_TOTAL; i++){ 
    mySettings.pinIsWrite[i] = (payload[i+4] == '1')? true : false; 
  }
  SetPinModeFromSettings();
  EESettings::Save(&mySettings, sizeof(mySettings) );
}

//**************************************
//* Command s_P1: SetPinTriggerOn
//**************************************
void CmdSetPinTriggerOn(char * payload){
  for (byte i=0; i < IO_PINS_TOTAL; i++){ mySettings.pinTriggerOn[i] = (payload[i+4] == '1')? true : false; }
  EESettings::Save(&mySettings, sizeof(mySettings) );
}

//**************************************
//* Command s_P0: SetPinTriggerOff
//**************************************
void CmdSetPinTriggerOff(char * payload){
  for (byte i=0; i < IO_PINS_TOTAL; i++){ mySettings.pinTriggerOff[i] = (payload[i+4] == '1')? true : false; }
  EESettings::Save(&mySettings, sizeof(mySettings) );
}

//**************************************
//* Command s_PS: SetPinState
//**************************************
void CmdSetPinState(char * payload){
  for (byte i=0; i < IO_PINS_TOTAL; i++){ 
    boolean state = (payload[i+4] == '1')? true : false;
      if (mySettings.pinIsWrite[i] ){ digitalWrite (i + IO_PINS_START, state); }
  }
}

//**************************************
//* Command g_PS: GetPinStatus - returns digital values
//**************************************
void CmdGetPinStatus( char * payload ){ 
  strcpy(payload, "OK:g_PS V:");
  char buffer[2];
  buffer[1] = '\0';
  
  for (byte i=0; i < IO_PINS_TOTAL; i++){
    buffer[0] = currentState.pinIsOne[i] ? '1' : '0';
    strcat(payload, buffer);     
  }
}


//**************************************
//* Command g_AV: GetAnalogValues
//**************************************
void CmdGetAnalogValues( char * payload ){ 
  int id = getIntFromStr(payload, 4, '\0', false);
  if (id < 0 || id > 5){ strcpy(payload, "ID OUT OF RANGE"); return; }
  
  char buffer[10];
  strcpy(payload, "OK:g_AV ");
  
  strcat(payload, "PIN:");
  itoa(id, buffer, 10);
  strcat(payload, buffer);
  
  strcat(payload, " AVG:");
  itoa(currentState.analogAverage[id], buffer, 10);
  strcat(payload, buffer);
  
  strcat(payload, " ENG:");
  itoa(currentState.analogEnergy[id], buffer, 10);
  strcat(payload, buffer);
}



//**************************************
//* Command g_PM: Get Settings for Pins
//**************************************
void CmdGetSettingsPin( char * payload ){ 
  strcpy (payload, "OK:g_PM ");
  char buffer[2];
  buffer[1] = '\0';
  
  //Pin mode
  strcat(payload, "M:");
  for (byte i=0; i < IO_PINS_TOTAL; i++){
    buffer[0] = mySettings.pinIsWrite[i] ? '1': '0';
    strcat(payload, buffer );
  }
  
  //Pin on trigger
  strcat(payload, " T1:");
  for (byte i=0; i < IO_PINS_TOTAL; i++){
    buffer[0] = mySettings.pinTriggerOn[i] ? '1': '0';
    strcat(payload, buffer );
  }
  
  //Pin off trigger
  strcat(payload, " T0:");
  for (byte i=0; i < IO_PINS_TOTAL; i++){
    buffer[0] = mySettings.pinTriggerOff[i] ? '1': '0';
    strcat(payload, buffer );
  }
}


//**************************************
//* Command g_SA: Get Settings for Analog
//**************************************
void CmdGetSettingsAnalog( char * payload ){ 
  int id = getIntFromStr(payload, 4, '\0', false);
  if (id < 0 || id > 5){ strcpy(payload, "ID OUT OF RANGE"); return; }
  
  char buffer[10];
  strcpy(payload, "OK:g_SA ");
  
  strcat(payload, "PIN:");
  itoa(id, buffer, 10);
  strcat(payload, buffer);
  
  strcat(payload, " AA:");
  itoa(mySettings.analogTirggerAverageAbove[id], buffer, 10);
  strcat(payload, buffer);
  
  strcat(payload, " AB:");
  itoa(mySettings.analogTirggerAverageBelow[id], buffer, 10);
  strcat(payload, buffer);
  
  strcat(payload, " EA:");
  itoa(mySettings.analogTriggerEnergyAbove[id], buffer, 10);
  strcat(payload, buffer);
  
  strcat(payload, " EB:");
  itoa(mySettings.analogTriggerEnergyBelow[id], buffer, 10);
  strcat(payload, buffer);
  
}



//**************************************
//* Command ?: Show help list of commands
//**************************************
void CmdShowCommands( char * payload ){ 
  Serial.println( F("=== Command Help ===") );
  Serial.println( F("Items in [] represent variables and are not a litteral part of the command") );
  Serial.println( F("type - responds with device type )") );
  Serial.println("");
  Serial.println( F("s_AA[id]:[value] - Set triiger for Average Analog Above port id to value") );
  Serial.println( F("s_AB[id]:[value] - Set trigger for Average Analog Below port id to value") );
  Serial.println( F("s_EA[id]:[value] - Sets trigger for Energy Analog Above port id to value") );
  Serial.println( F("s_EB[id]:[value] - Sets tirgger for Energy Analog Below port id to value") );
  Serial.println( F("s_PM[p2][p3]...[p13] - Sets pins 2 - 13 to input or output mode (0 in, 1 out) ") );
  Serial.println( F("s_P1[p2][p3]...[p13] - enables pin On trigger for pings 2 - 13 (0 disabled, 1 enabled)") );
  Serial.println( F("s_P0[p2][p3]...[p13] - enables pin Off trigger for pins 2 - 13 (0 disabled, 1 enabled)") );
  Serial.println( F("s_PS[p2][p3]...[p13] - Turn on or off an output for pins 2 - 13 (0 off, 1 on). Ignored if pin is not in output mode") );
  Serial.println( F("") );
  Serial.println( F("g_PS - get pin status, returns pin input/output values for pins 2 - 13") );
  Serial.println( F("g_AV[id] - get analog vaues for id, returns avg:xxxx eng:xxxx") );
  Serial.println( F("g_PM - get Settings Pins, returns input or output mode for pins 2 - 13") );
  Serial.println( F("g_SA[id] - returns all analog settings for a given port") );
  Serial.println("");
  
}



//***********************************************************************************************************
// Settings Functions

//**************************************
// Sets pin mode to input our output based on settings
//**************************************
void  SetPinModeFromSettings(){
  for (byte i=0; i < IO_PINS_TOTAL; i++){
    if ( mySettings.pinIsWrite[i] ){ pinMode(i+IO_PINS_START, OUTPUT); }else{ pinMode(i+IO_PINS_START, INPUT_PULLUP); }
  }
}


//**************************************
//* Load Default Settings for this system
//**************************************
void LoadDefaultSettings(){
    for (int i=0; i < IO_PINS_TOTAL; i++){
      mySettings.pinIsWrite[i] = false; 
      mySettings.pinTriggerOn[i] = false; 
      mySettings.pinTriggerOff[i] = false;
    }
    
    for (int i=0; i<6; i++){ 
      mySettings.analogTirggerAverageAbove[6] = 32000;
      mySettings.analogTirggerAverageBelow[6] = 0;
      mySettings.analogTriggerEnergyAbove[6] = 32000;
      mySettings.analogTriggerEnergyBelow[6] = 0;
    }
    
    EESettings::Save(&mySettings, sizeof(mySettings) );
 }



//***********************************************************************************************************
// Common Functions

//**************************************
//Reads an integer or hex as int form string at at a give offset terminated by termChar
//**************************************
int  getIntFromStr(const char * inStr, int offset, byte termChar, boolean isHex){
  int num = 0;
  
  if ( isHex ){
    char HexArr[17] = "0123456789ABCDEF";
    byte b1 = inStr[offset];
    byte b2 = inStr[offset + 1];
    
    for (byte i=0; i<16; i++){
      if ( HexArr[i] == b1 ){ num += i * 16; }
      if ( HexArr[i] == b2 ){ num += i; }     
    }
    
  }else{
    char tempBuffer[12];
    int i;
    for (i=0; i<12; i++){
      if (inStr[i + offset] == termChar){ break; }
      tempBuffer[i] = inStr[i + offset];
    }
    tempBuffer[i] = '\0';
    num = atoi( tempBuffer ); 
  }

  return num;
}


//**************************************
//Returns true if needle is at the start of haystack
//**************************************
boolean instr(const char * haystack, const char * needle){
  boolean isFound = true;
  int counter=0;
  while (true){
    if (needle[counter] == '\0'){break;}
    if (haystack[counter] != needle[counter]){ isFound=false; break;}
    counter++;
  }
  return isFound;
}


//**************************************
//Checks to see if pin 2 is held low at startup
//**************************************
void CheckDefaultLoadReq(){
  pinMode(RESET_DEFAULTS_PIN, INPUT_PULLUP);
  delay(200);
  if (digitalRead(RESET_DEFAULTS_PIN) == HIGH){ return; }  //Skip setup
  
  Serial.println( F("Reseting defaults....") );
  LoadDefaultSettings();
  Serial.println( F("Enter new id in hex by sending tFFf00pN35:s_id[id] where [id] is the 2 byte hex id IE 3A") );
  
  
}
