/**************************************************************************/
/*!
    Driver code for controlling the MCP4275 DAC over serial input.

    Adafruit MCP4725 breakout board: http://www.adafruit.com/products/935

    Notes:
    - Input commands must be separated by ':' and ',' without any spaces
    - Commands can be up to BUFFER_SIZE long
    - Commands should be terminated by newline character '\n' for faster perforance
    - For example: "<cmd_1>:<val1>,<cmd_2>:<val2>'\n'"
*/
/**************************************************************************/
#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

// Set this value to 9, 8, 7, 6 or 5 to adjust the resolution
#define DAC_RESOLUTION    (8)
#define BUFFER_SIZE 100

// For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
// For MCP4725A0 the address is 0x60 or 0x61
// For MCP4725A2 the address is 0x64 or 0x65
int addr = 0x62; 
int EEPROM_value = 0; // default value to use on start up before any input

String serial_in;
char serial_char[BUFFER_SIZE];
char *cmd;
int val;
char delim[] = ":,";
char cmd_t[] = "cmd_t"; // name of command for throttle control 

void setEEPROM() {
  dac.setVoltage(EEPROM_value, true);
  Serial.print("Set EEPROM to: ");
  Serial.println(EEPROM_value); 
}

void writeDACValue(int val) {
  dac.setVoltage(val, false);
}

void setup(void) {
  Serial.begin(9600);
  Serial.println("Initializing DAC connection...");

  dac.begin(addr);

  Serial.println("DAC initialized!");
}

void loop() {
  if (Serial.available() > 0) {
    serial_in = Serial.readStringUntil('\n');
    serial_in.toCharArray(serial_char, BUFFER_SIZE);
    cmd = strtok(serial_char, delim);

    while (cmd != NULL) {
      // say what you got:
      val = atoi(strtok(NULL, delim));

      if (strcmp(cmd, cmd_t) == 0) {
        writeDACValue(val);
      }
      cmd = strtok(NULL, delim); // Get the next token
    }
  }
}
