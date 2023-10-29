/*
 * Based on 39SF040 / 39SF020A Programmer by Bob Szymanski
 * Published 9/24/2015.
 * Pinout should be used according to the pins as defined.
 * SD card pinout is as provided by Arduino, on pins 51-53.
 * 
 * This version is built using the Arduino Mega 2650.
 * 
 * Supports programing Winbond W29C011
 */
 
#include <SPI.h>
#include <SD.h>
#include "SerialCommands.h"
#include "utils.h"

#define SECTOR_SIZE   4096    /* Must be 4096 bytes for 39SF040 */
#define SST_ID        0xBF    /* SST Manufacturer's ID code   */
#define SST_39SF040   0xB7    /* SST 39SF040 device code      */

#define PAGE_SIZE     128
#define WB_ID         0xDA    /* Winbond Manufacturer's ID code   */
#define WB_29C011     0xC1    /* W29C011 device code      */

#define _WE 14
#define _OE 15 
#define _CE 16 

#define DATA_DIR DDRA
#define DATA_IN  PINA
#define DATA_OUT PORTA
#define D0 22 
#define D1 23 
#define D2 24 
#define D3 25 
#define D4 26 
#define D5 27
#define D6 28 
#define D7 29

#define ADRL_DIR DDRF
#define ADRL_IN  PINF
#define ADRL_OUT PORTF
#define ADR0 A0
#define ADR1 A1 
#define ADR2 A2 
#define ADR3 A3 
#define ADR4 A4 
#define ADR5 A5
#define ADR6 A6 
#define ADR7 A7

#define ADRM_DIR DDRK
#define ADRM_IN  PINK
#define ADRM_OUT PORTK
#define ADR8 A8 
#define ADR9 A9 
#define ADR10 A10 
#define ADR11 A11 
#define ADR12 A12 
#define ADR13 A13
#define ADR14 A14 
#define ADR15 A15

#define ADR16 46 
#define ADR17 47
#define ADR18 48

#define PROBE 21

#define SD_SS_PIN 53

bool SD_OK = true;
bool DEBUG = false;

void cmd_unrecognized(SerialCommands* sender, const char* cmd);
//void cmd_run_script(SerialCommands* sender);
void cmd_erase_sector(SerialCommands*);
void cmd_erase_chip(SerialCommands* sender);
void cmd_eeprom_to_file(SerialCommands* sender);
void cmd_file_to_eeprom(SerialCommands* sender);
void cmd_chip_id(SerialCommands* sender);
void cmd_list_dir(SerialCommands* sender);
void cmd_dump(SerialCommands* sender);
void cmd_write(SerialCommands* sender);
void cmd_debug(SerialCommands* sender);
void cmd_probe(SerialCommands* sender);
void cmd_help(SerialCommands* sender);

void signalsState(String str, bool wait = false, bool force = false);

char serial_command_buffer_[64];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\n", " ");

//SerialCommand cmd_run_script_("run", cmd_run_script);
SerialCommand cmd_erase_sector_("es", cmd_erase_sector);
SerialCommand cmd_erase_chip_("ec", cmd_erase_chip);
SerialCommand cmd_eeprom_to_file_("etf", cmd_eeprom_to_file);
SerialCommand cmd_file_to_eeprom_("fte", cmd_file_to_eeprom);
SerialCommand cmd_chip_id_("id", cmd_chip_id);
SerialCommand cmd_list_dir_("ls", cmd_list_dir);
SerialCommand cmd_dump_("d", cmd_dump);
SerialCommand cmd_write_("w", cmd_write);
SerialCommand cmd_debug_("debug", cmd_debug);
SerialCommand cmd_probe_("probe", cmd_probe);
SerialCommand cmd_help_("help", cmd_help);

void setup()
{
  Serial.begin(115200);
  delay(2);
  serial_commands_.SetDefaultHandler(cmd_unrecognized);
  //serial_commands_.AddCommand(&cmd_run_script_);
  serial_commands_.AddCommand(&cmd_erase_sector_);
  serial_commands_.AddCommand(&cmd_erase_chip_);
  serial_commands_.AddCommand(&cmd_eeprom_to_file_);
  serial_commands_.AddCommand(&cmd_file_to_eeprom_);
  serial_commands_.AddCommand(&cmd_chip_id_);
  serial_commands_.AddCommand(&cmd_list_dir_);
  serial_commands_.AddCommand(&cmd_dump_);
  serial_commands_.AddCommand(&cmd_write_);
  serial_commands_.AddCommand(&cmd_debug_);
  serial_commands_.AddCommand(&cmd_probe_);
  serial_commands_.AddCommand(&cmd_help_);
  
  Serial.println(F("\014*** SST39SF010A/020A/040 Arduino Mega Programmer ***"));

  if (!SD.begin(SD_SS_PIN))
  {
    SD_OK = false;
    Serial.println("Error initializing SD Card!");
  }
  
  pinMode(PROBE, INPUT_PULLUP);

  pinMode(_WE, OUTPUT);
  digitalWrite(_WE, HIGH);
  
  pinMode(_OE, OUTPUT);
  digitalWrite(_OE, HIGH);
  
  pinMode(_CE, OUTPUT);
  digitalWrite(_CE, HIGH);
  
  setReadMode();

  #ifdef ADRL_DDR
    ADRL_DDR = B11111111;
  #else
    pinMode(ADR0, OUTPUT);
    pinMode(ADR1, OUTPUT);
    pinMode(ADR2, OUTPUT);
    pinMode(ADR3, OUTPUT);
    pinMode(ADR4, OUTPUT);
    pinMode(ADR5, OUTPUT);
    pinMode(ADR6, OUTPUT);
    pinMode(ADR7, OUTPUT);
  #endif

  #ifdef ADRM_DDR
    ADRM_DDR = B11111111;
  #else
    pinMode(ADR8, OUTPUT);
    pinMode(ADR9, OUTPUT);
    pinMode(ADR10, OUTPUT);
    pinMode(ADR11, OUTPUT);
    pinMode(ADR12, OUTPUT);
    pinMode(ADR13, OUTPUT);
    pinMode(ADR14, OUTPUT);
    pinMode(ADR15, OUTPUT);
  #endif

  pinMode(ADR16, OUTPUT);
  pinMode(ADR17, OUTPUT);
  pinMode(ADR18, OUTPUT);
  //delay(100);
  
  setAddress(0);
  
  pinMode(SD_SS_PIN, OUTPUT); //Needed for SD to work on the Mega.
  //delay(100);
}

void loop()
{  
  serial_commands_.ReadSerial(); 
}

/* ====================================================================================== */

void cmd_help(SerialCommands* sender)
{
  sender->GetSerial()->println(F("help \t\t\t- Shows this help"));
  sender->GetSerial()->println(F("id \t\t\t- Shows Chip ID"));
  sender->GetSerial()->println(F("ls \t\t\t- Lists files on SD-card"));
  sender->GetSerial()->println(F("d arg1 arg2 \t\t- Dump to Serial EEPROM data; arg1 - start address, arg2 - data length"));
  sender->GetSerial()->println(F("w arg1 argx... \t\t- Write to Serial EEPROM data; arg1 - start address, argx - data"));
  sender->GetSerial()->println(F("debug \t\t\t- toogle debug on/off"));
  sender->GetSerial()->println(F("es arg1 \t\t- Erase Sector; param1 - sector's start address"));
  sender->GetSerial()->println(F("ec \t\t\t- Erase Chip"));
  sender->GetSerial()->println(F("etf arg1 arg2 arg3\t- Writes EEPROM data to File; arg1 - filename, arg2 - start address, arg3 - data length"));
  sender->GetSerial()->println(F("fte arg1 arg2 \t\t- Program ROM File data to EEPROM; arg1 - filename, arg2 - start destination address"));
  sender->GetSerial()->println(F("Usage: fte FIRMWARE.ROM 0x10000 - Writes file FIRMWARE.ROM to EEPROM starting from 0x10000"));
  sender->GetSerial()->println(F("       etf FIRMWARE.ROM 0x10000 0x8000 - Writes EEPROM data starting from 0x10000 and length 0x8000 to file FIRMWARE.ROM"));
  sender->GetSerial()->println(F(""));
}

void cmd_debug(SerialCommands* sender)
{
  if (DEBUG) {
    DEBUG = false;
    sender->GetSerial()->println("Debug disabled.");
  } else {
    DEBUG = true;
    sender->GetSerial()->println("Debug enabled.");
  }
}

void cmd_probe(SerialCommands* sender)
{
  signalsState("PR", false, true);
}

void cmd_list_dir(SerialCommands* sender)
{
  if (!SD_OK){
    sender->GetSerial()->println("ERROR: SD Not Ready!");
    return;
  }
  sender->GetSerial()->println("SD card listing");
  sender->GetSerial()->println("------------------------------------------");
  File root = SD.open("/");
  printDirectory(root, 0);
  sender->GetSerial()->println("------------------------------------------");
  sender->GetSerial()->println("");
}

void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

void cmd_run_script(SerialCommands* sender)
{
  char* file_str = sender->Next();
  if (file_str == NULL)
  {
    sender->GetSerial()->println("ERROR: No file name given!");
    return;
  }
  if (!SD_OK){
    sender->GetSerial()->println("ERROR: SD Not Ready!");
    return;
  }
  File configFile = SD.open(file_str);
  if (!configFile)
  {
    sender->GetSerial()->println("Error reading script file!");
  }
  else
  {
    String confString = "";
    char letter = configFile.read();
    while (letter > 0 && letter != EOF && letter != '\n')
    {
      confString += letter; 
      letter = configFile.read();
    }
    configFile.close();
    long dataAddr = getValue(confString, ':', 0).toInt();
    String romFileName = getValue(confString, ':', 1);
    String chkFileName = getValue(romFileName, '.', 0) + ".chk";
    long addressCount = fileToEeprom(romFileName, dataAddr);
    eepromToFile(chkFileName, 0, addressCount);
    compareFiles(romFileName, chkFileName);
  }
}

void cmd_erase_sector(SerialCommands* sender)
{ 
  signalsState("IDLE");

  char* addr_str = sender->Next();
  if (addr_str == NULL)
  {
    sender->GetSerial()->println("Error! Address is not set!");
    return;
  }
  unsigned long addr = strToLong(addr_str);

  if (addr % SECTOR_SIZE != 0)
  {
    sender->GetSerial()->print("Error! Sector's begining address not multiple to ");
    sender->GetSerial()->println(SECTOR_SIZE);
  }
  else
  {
    sender->GetSerial()->print("Erasing sector: 0x");
    sender->GetSerial()->println(addr, HEX);
    sectorErase(addr);
    setReadMode();
    sender->GetSerial()->println("Done!");
  }
}

void cmd_erase_chip(SerialCommands* sender)
{
  signalsState("IDLE");

  sender->GetSerial()->print("Erasing chip...");
  chipErase();
  delay(500);
  setReadMode();
  sender->GetSerial()->println("Done!");
}

void cmd_chip_id(SerialCommands* sender)
{
  signalsState("IDLE");

  sender->GetSerial()->print("EEPROM Manufacturer ID: 0x");
  sender->GetSerial()->println(getHwID(0L), HEX);
  sender->GetSerial()->print("EEPROM Chip ID: 0x");
  sender->GetSerial()->println(getHwID(1L), HEX);
}

static unsigned long start_addr = 0L;
static unsigned long data_len = 32L;

void cmd_dump(SerialCommands* sender)
{
  signalsState("IDLE");

  char* start_addr_str = sender->Next(); 
  if (start_addr_str)
    start_addr = strToLong(start_addr_str);
  
  char* data_len_str = sender->Next();
  if (data_len_str)
    data_len = strToLong(data_len_str);

  if (data_len > 0)
  {
    if (start_addr_str || start_addr == 0L) {
      sender->GetSerial()->println("EEPROM Memory dump:");
      sender->GetSerial()->print("Start address: 0x");
      sender->GetSerial()->println(start_addr, HEX);
      sender->GetSerial()->print("Data length: 0x");
      sender->GetSerial()->println(data_len, HEX);
    }
    eepromToSerial(start_addr, data_len);
    start_addr += data_len;
  }
  else
  {
    sender->GetSerial()->println("Error! Wrong data length!");
  }
}

void cmd_write(SerialCommands* sender)
{
  signalsState("IDLE");

  static unsigned long start_addr = 0L;

  char* start_addr_str = sender->Next();
  if (start_addr_str)
    start_addr = strToLong(start_addr_str);

  char* data_str;
  while ((data_str = sender->Next()) != NULL) 
  {
    byte data = strToLong(data_str);
    programByte(start_addr, data);
    start_addr++;
  }

}

void cmd_eeprom_to_file(SerialCommands* sender)
{
  signalsState("IDLE");

  if (!SD_OK){
    sender->GetSerial()->println("ERROR: SD Not Ready!");
    return;
  }
  char* file_str = sender->Next();
  if (file_str == NULL)
  {
    sender->GetSerial()->println("Error! No File name given!");
    return;
  }

  char* start_addr_str = sender->Next(); 
  char* data_len_str = sender->Next();
  if ((start_addr_str != NULL) && (data_len_str != NULL))
  {
    unsigned long start_addr = strToLong(start_addr_str);
    unsigned long data_len = strToLong(data_len_str);
    if (data_len > 0)
    {
      sender->GetSerial()->print("Reading EEPROM to file: ");
      sender->GetSerial()->println(file_str);
      eepromToFile(file_str, start_addr, data_len);
      sender->GetSerial()->println("Done!");
    }
    else
    {
      sender->GetSerial()->println("Error! Wrong data length!");
      sender->GetSerial()->print("start_addr: ");
      sender->GetSerial()->println(start_addr);
      sender->GetSerial()->print("data_len: ");
      sender->GetSerial()->println(data_len);
    }
  }
}

void cmd_file_to_eeprom(SerialCommands* sender)
{
  signalsState("IDLE");

  if (!SD_OK){
    sender->GetSerial()->println("ERROR: SD Not Ready!");
    return;
  }
  char* file_str = sender->Next();
  if (file_str == NULL)
  {
    sender->GetSerial()->println("Error! No File name given!");
    return;
  }
  char* addr_str = sender->Next();
  if (addr_str == NULL)
  {
    sender->GetSerial()->println("Error! No Address given!");
    return;
  }
  
  unsigned long addr = strToLong(addr_str);
  sender->GetSerial()->print("Writing to EEPROM from file: ");
  sender->GetSerial()->println(file_str);
  unsigned long bytesWrote = fileToEeprom(file_str, addr);
  sender->GetSerial()->print("Wrote ");
  sender->GetSerial()->print(bytesWrote);
  sender->GetSerial()->println(" bytes");
  
}

unsigned long fileToEeprom(String fileName, unsigned long address)
{ 
  signalsState("IDLE");

  byte data_arr[PAGE_SIZE];
  int datum = 0;
  unsigned long addressCount = 0;
  unsigned long bytesCount = 0;
  File sourceFile = SD.open(fileName);
  delay(500);
  if (!sourceFile)
  {
    Serial.println("Error reading ROM file!");
  }
  byte chip_id = getHwID(1L);
  Serial.print("Chip ID: 0x");
  Serial.println(chip_id, HEX);
  Serial.print("Writing to EEPROM address: 0x");
  Serial.println(address, HEX);

  if (chip_id == SST_39SF040)
  {
    while (true)
    {
      datum = sourceFile.read();
      if (datum < 0) { break; } 
      programByte(address, datum);
      addressCount++;
      address++;
      bytesCount++;
      delayMicroseconds(30);
      if (address % 1024 == 0) {
        Serial.print(".");
      }  
    }
  } else if (chip_id == WB_29C011)
  {
    while (true)
    {
      datum = sourceFile.read();
      if (datum < 0) { break; } 
      bytesCount++;
      data_arr[addressCount] = datum;
      addressCount++;
      if ((addressCount % PAGE_SIZE == 0) && (addressCount > 0)) {  
        programPage(address, &data_arr[0]);
        addressCount = 0;
        address++;
      } 
      if (bytesCount % 1024 == 0) {
        Serial.print("|");
      }  
    }
  }
  Serial.println("");
  sourceFile.close();
  return bytesCount;
}


void eepromToSerial(unsigned long offset, unsigned long dataLen)
{
  signalsState("IDLE");

  setReadMode();
  int datum;
  String datum_str;
  String offset_str = "";
  char ascii[17];
  memset(ascii,0,17);
  unsigned long count;
  unsigned long end_addr = offset + dataLen;
  int str_len = String(end_addr, HEX).length();
  int len_diff, i;
  for (count = 0; count < dataLen; count++)
  {
    offset_str = "";
    datum = readData(offset);
    if (datum < 16){
      datum_str = "0" + String(datum, HEX);  
    } else {
      datum_str = String(datum, HEX);
    }
    if ((offset % 16 == 0) || (count == 0)){
      if (count) {
        Serial.print(" : ");
        Serial.println(ascii);
        memset(ascii,0,17);
      }

      len_diff = str_len - String(offset, HEX).length();
      if (len_diff > 0){
        for (i = 0; i < len_diff; i++){
          offset_str  += "0";
        } 
      }
      offset_str += String(offset, HEX);
      Serial.print("0x" + offset_str);
      Serial.print(" : ");  
    } 
    if (datum >= 32 && datum < 128)
      ascii[count % 16] = datum;
    else
      ascii[count % 16] = '.';

    Serial.print(datum_str);
    Serial.print(" ");
    delayMicroseconds(5);
    offset++;
  }

  if (ascii[0] != 0) {
    Serial.print(" : ");
    Serial.println(ascii);
  }

  setWriteMode();
}

void eepromToFile(String fileName, unsigned long offset, unsigned long dataLen)
{
  signalsState("IDLE");

  setReadMode();
  int datum;
  unsigned long address;
  if (SD.exists(fileName)) {
    SD.remove(fileName);  
  }
  
  File copyFile = SD.open(fileName, O_CREAT | O_WRITE);
  if (!copyFile)
  {
    Serial.println("Error opening data file!");
  }
  Serial.print("Start address: 0x");
  Serial.println(offset, HEX);
  Serial.print("Data length: 0x");
  Serial.println(dataLen, HEX);
  Serial.println("Writing copy to file...");
  for (address = 0; address < dataLen; address++)
  {
    datum = readData(offset);
    copyFile.write(datum);
    delayMicroseconds(5);
    offset++;
    if (address % 1024 == 0){
      Serial.print("|");  
    }
  }
  Serial.println("");
  copyFile.flush();
  copyFile.close();
  setWriteMode(); 
}

bool compareFiles(String srcFileName, String copyFileName)
{
  long errors = 0;
  long address = 0;
  int rawData = 0;
  int datum = 0;
  File copyFile = SD.open(copyFileName); 
  File sourceFile = SD.open(srcFileName);
  if (!sourceFile || !copyFile)
  {
    Serial.println("Error reading original / verification file!");
    return false; 
  } 
  Serial.println("Verifying data...");
  while(true){
    rawData = sourceFile.read();
    if (rawData < 0) { break; }
    datum = copyFile.read();
    if (address % 1024 == 0){
      Serial.print("|");  
    }
    if (datum != rawData){
      errors++;
      Serial.print(address, HEX);
      Serial.print(": ");
      Serial.print(rawData, HEX);
      Serial.print(" <> ");
      Serial.println(datum, HEX);
    }
    address++;
  }

  sourceFile.close();
  copyFile.close();
  
  Serial.println("");
  if (errors > 0){
    Serial.println("Verification FAILED!");
    return false;
  }else{
    Serial.println("Verification OK!");
    return true;
  }
}

void setReadMode()
{
  #ifdef DATA_DIR
    DATA_DIR = B00000000;
  #else
    pinMode(D0, INPUT);
    pinMode(D1, INPUT);
    pinMode(D2, INPUT);
    pinMode(D3, INPUT);
    pinMode(D4, INPUT);
    pinMode(D5, INPUT);
    pinMode(D6, INPUT);
    pinMode(D7, INPUT);
  #endif
}

void setWriteMode()
{
  #ifdef DATA_DIR
    DATA_DIR = B11111111;
  #else
    pinMode(D0, OUTPUT);
    pinMode(D1, OUTPUT);
    pinMode(D2, OUTPUT);
    pinMode(D3, OUTPUT);
    pinMode(D4, OUTPUT);
    pinMode(D5, OUTPUT);
    pinMode(D6, OUTPUT);
    pinMode(D7, OUTPUT);
  #endif
}

void setAddress(unsigned long value)
{
  #ifdef ADRL_OUT
    ADRL_OUT = (value >> 0) & 0xff;
  #else
    digitalWrite(ADR0, (value >> 0) & 0x01);
    digitalWrite(ADR1, (value >> 1) & 0x01);
    digitalWrite(ADR2, (value >> 2) & 0x01);
    digitalWrite(ADR3, (value >> 3) & 0x01);
    digitalWrite(ADR4, (value >> 4) & 0x01);
    digitalWrite(ADR5, (value >> 5) & 0x01);
    digitalWrite(ADR6, (value >> 6) & 0x01);
    digitalWrite(ADR7, (value >> 7) & 0x01);
  #endif

  #ifdef ADRM_OUT
    ADRM_OUT = (value >> 8) & 0xff;
  #else
    digitalWrite(ADR8, (value >> 8) & 0x01);
    digitalWrite(ADR9, (value >> 9) & 0x01);
    digitalWrite(ADR10, (value >> 10) & 0x01);
    digitalWrite(ADR11, (value >> 11) & 0x01);
    digitalWrite(ADR12, (value >> 12) & 0x01);
    digitalWrite(ADR13, (value >> 13) & 0x01);
    digitalWrite(ADR14, (value >> 14) & 0x01);
    digitalWrite(ADR15, (value >> 15) & 0x01);
  #endif

  digitalWrite(ADR16, (value >> 16) & 0x01);
  digitalWrite(ADR17, (value >> 17) & 0x01);
  digitalWrite(ADR18, (value >> 18) & 0x01);
}

unsigned long getAddr()
{
  unsigned long toReturn = 0;
  #ifdef ADRL_IN
    toReturn |= ADRL_IN;
  #else
    if (digitalRead(ADR0) == HIGH) {bitSet(toReturn, 0); } 
    if (digitalRead(ADR1) == HIGH) {bitSet(toReturn, 1); } 
    if (digitalRead(ADR2) == HIGH) {bitSet(toReturn, 2); } 
    if (digitalRead(ADR3) == HIGH) {bitSet(toReturn, 3); } 
    if (digitalRead(ADR4) == HIGH) {bitSet(toReturn, 4); } 
    if (digitalRead(ADR5) == HIGH) {bitSet(toReturn, 5); } 
    if (digitalRead(ADR6) == HIGH) {bitSet(toReturn, 6); } 
    if (digitalRead(ADR7) == HIGH) {bitSet(toReturn, 7); } 
  #endif

  #ifdef ADRM_IN
    toReturn |= ADRM_IN << 8;
  #else
    if (digitalRead(ADR8) == HIGH) {bitSet(toReturn, 8); } 
    if (digitalRead(ADR9) == HIGH) {bitSet(toReturn, 9); } 
    if (digitalRead(ADR10) == HIGH) {bitSet(toReturn, 10); } 
    if (digitalRead(ADR11) == HIGH) {bitSet(toReturn, 11); } 
    if (digitalRead(ADR12) == HIGH) {bitSet(toReturn, 12); } 
    if (digitalRead(ADR13) == HIGH) {bitSet(toReturn, 13); } 
    if (digitalRead(ADR14) == HIGH) {bitSet(toReturn, 14); } 
    if (digitalRead(ADR15) == HIGH) {bitSet(toReturn, 15); } 
  #endif

  if (digitalRead(ADR16) == HIGH) {bitSet(toReturn, 16); } 
  if (digitalRead(ADR17) == HIGH) {bitSet(toReturn, 17); } 
  if (digitalRead(ADR18) == HIGH) {bitSet(toReturn, 18); } 
  return toReturn;
}

void setData(byte value)
{
  #ifdef DATA_OUT
    DATA_OUT = value;
  #else
    digitalWrite(D0, (value >> 0) & 0x01);
    digitalWrite(D1, (value >> 1) & 0x01);
    digitalWrite(D2, (value >> 2) & 0x01);
    digitalWrite(D3, (value >> 3) & 0x01);
    digitalWrite(D4, (value >> 4) & 0x01);
    digitalWrite(D5, (value >> 5) & 0x01);
    digitalWrite(D6, (value >> 6) & 0x01);
    digitalWrite(D7, (value >> 7) & 0x01);
  #endif

  //delayMicroseconds(5); 
}

byte getData()
{
  // delayMicroseconds(5);

  byte toReturn = 0;
  #ifdef DATA_IN
    toReturn = DATA_IN;
  #else
    if (digitalRead(D0) == HIGH) {bitSet(toReturn, 0); }
    if (digitalRead(D1) == HIGH) {bitSet(toReturn, 1); }
    if (digitalRead(D2) == HIGH) {bitSet(toReturn, 2); }
    if (digitalRead(D3) == HIGH) {bitSet(toReturn, 3); }
    if (digitalRead(D4) == HIGH) {bitSet(toReturn, 4); }
    if (digitalRead(D5) == HIGH) {bitSet(toReturn, 5); }
    if (digitalRead(D6) == HIGH) {bitSet(toReturn, 6); }
    if (digitalRead(D7) == HIGH) {bitSet(toReturn, 7); }
  #endif

  return toReturn;
}

byte readData(unsigned long addr)
{
  setReadMode();
  setAddress(addr);

  digitalWrite(_WE, HIGH);
  digitalWrite(_CE, LOW);
  digitalWrite(_OE, LOW);
  
  //delayMicroseconds(5);

  byte toReturn = getData();
  signalsState("RD", true);
  
  digitalWrite(_OE, HIGH);
  digitalWrite(_CE, HIGH);
  return toReturn;
}

void writeData(unsigned long addr, byte value)
{
  setWriteMode();
  setData(value);
  setAddress(addr);

  digitalWrite(_OE, HIGH);
  digitalWrite(_CE, LOW);

  //delayMicroseconds(5);

  digitalWrite(_WE, LOW);
  
  //delayMicroseconds(5);
  signalsState("WR", true); 
  
  digitalWrite(_WE, HIGH);
  digitalWrite(_CE, HIGH);

  setReadMode();
}

void softProtect()
{
  unsigned long ADDR_1 = 0x5555L;
  unsigned long ADDR_2 = 0x2AAAL;
  unsigned long ADDR_3 = 0x5555L;
  
  //perform the 3 byte program code:
  writeData(ADDR_1, 0xAA);
  
  writeData(ADDR_2, 0x55);
  
  writeData(ADDR_3, 0xA0); 
}

void programByte(unsigned long address, byte data)
{
  softProtect();
  writeData(address, data);
}

//W29C011

void programPage(unsigned long address, byte *data_arr){
  softProtect();
  int i;
  for(i = 0; i < PAGE_SIZE; i++){
    writeData(address, data_arr[i]);
    address++;
  }
  delayMicroseconds(300);
}

void setAddressLo(unsigned long value)
{
  digitalWrite(ADR0, (value >> 0) & 0x01);
  digitalWrite(ADR1, (value >> 1) & 0x01);
  digitalWrite(ADR2, (value >> 2) & 0x01);
  digitalWrite(ADR3, (value >> 3) & 0x01);
  digitalWrite(ADR4, (value >> 4) & 0x01);
  digitalWrite(ADR5, (value >> 5) & 0x01);
  digitalWrite(ADR6, (value >> 6) & 0x01);
}

void setAddressHi(unsigned long value)
{
  digitalWrite(ADR7, (value >> 0) & 0x01);
  digitalWrite(ADR8, (value >> 1) & 0x01);
  digitalWrite(ADR9, (value >> 2) & 0x01);
  digitalWrite(ADR10, (value >> 3) & 0x01);
  digitalWrite(ADR11, (value >> 4) & 0x01);
  digitalWrite(ADR12, (value >> 5) & 0x01);
  digitalWrite(ADR13, (value >> 6) & 0x01);
  digitalWrite(ADR14, (value >> 7) & 0x01);
  digitalWrite(ADR15, (value >> 8) & 0x01);
  digitalWrite(ADR16, (value >> 9) & 0x01);
}


byte getHwID(unsigned long id_type)
{
  long ADDR_1 = 0x5555L;
  long ADDR_2 = 0x2AAAL;
  long ADDR_3 = 0x5555L;
  
  //perform the 3 byte entry program code:

  writeData(ADDR_1, 0xAA);
  
  writeData(ADDR_2, 0x55);
  
  writeData(ADDR_3, 0x90); 

  byte id = readData(id_type);
  
  //perform the 3 byte exit program code:
  writeData(ADDR_1, 0xAA);
  
  writeData(ADDR_2, 0x55);
  
  writeData(ADDR_3, 0xF0); 
 
  return id;
}


/* Chip erase follows the following pattern:
 * Load 6 specific bytes at specific addresses
 * Delay shortly for erase to complete.
 */ 
void chipErase()
{
  long ADDR_1 = 0x5555L;
  long ADDR_2 = 0x2AAAL;
  long ADDR_3 = 0x5555L;
  long ADDR_4 = 0x5555L;
  long ADDR_5 = 0x2AAAL;
  long ADDR_6 = 0x5555L;
  
  byte DATA_1 = 0xAA;
  byte DATA_2 = 0x55;
  byte DATA_3 = 0x80;
  byte DATA_4 = 0xAA;
  byte DATA_5 = 0x55;
  byte DATA_6 = 0x10;
  
  //perform the 6 byte program code:
  writeData(ADDR_1, DATA_1);
  
  writeData(ADDR_2, DATA_2);
  
  writeData(ADDR_3, DATA_3);
  
  writeData(ADDR_4, DATA_4);
  
  writeData(ADDR_5, DATA_5);
  
  writeData(ADDR_6, DATA_6);
  delay(1000);
}

void sectorErase(unsigned long address)
{
  long ADDR_1 = 0x5555L;
  long ADDR_2 = 0x2AAAL;
  long ADDR_3 = 0x5555L;
  long ADDR_4 = 0x5555L;
  long ADDR_5 = 0x2AAAL;
  long ADDR_6 = address;
  
  byte DATA_1 = 0xAA;
  byte DATA_2 = 0x55;
  byte DATA_3 = 0x80;
  byte DATA_4 = 0xAA;
  byte DATA_5 = 0x55;
  byte DATA_6 = 0x30;
  
  //perform the 6 byte program code:
  writeData(ADDR_1, DATA_1);
  
  writeData(ADDR_2, DATA_2);
  
  writeData(ADDR_3, DATA_3);
  
  writeData(ADDR_4, DATA_4);
  
  writeData(ADDR_5, DATA_5);
  
  writeData(ADDR_6, DATA_6);
  delay(100);
}

//may be used for debug
void signalsState(String str, bool wait, bool force){
  if (DEBUG || force) {
    String ce = digitalRead(_CE) ? "HIGH" : "LOW";
    String we = digitalRead(_WE) ? "HIGH" : "LOW";
    String oe = digitalRead(_OE) ? "HIGH" : "LOW";
    String pr = digitalRead(PROBE) ? "HIGH" : "LOW";
    Serial.print(str + " : ");
    Serial.print("AD:");
    Serial.print(getAddr(), HEX);
    Serial.print("\tDA:");
    Serial.print(getData(), HEX);
    Serial.print("\tCE:" + ce);
    Serial.print("\tWE:" + we);
    Serial.print("\tOE:" + oe);
    Serial.println("\tPR:" + pr);

    // Wait for user
    while (wait && !Serial.available());
    Serial.read();
  }
}
