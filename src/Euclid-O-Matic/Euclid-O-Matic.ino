  /*
   *                                                                      
   *   Program:      Euclidean Rhythm Generator Kosmo/Eurorack  
   *   Author:       L J Brackney,                              
   *   Organization: Suncoast Polytechnical High School         
   *   Version:      1.0                                        
   *   Date:         11/21/2021                                 
   *                                                            
   *   Adapted by Jos Bouten, Jan ... Apr 2022              
   *   Added support for tempo tap, clock out, 4 function inputs
   *   - Added variable pattern length                        
   *   - To change the tempo the tap tempo button needs to be pressed while  
   *     the rotary encoder is rotated.                                      
   *   - Changed all channel dependent variables into arrays.
   *   - Used capitals for constants, camel case for variables and function calls
   *   - Setting the speed depends on pressing the tap tempo button.
   *   - While setting the speed, the pattern leds show a white background pattern.
   *   - Added storing, recalling and clearing of patches in EEPROM.
   *   - The channel chosen last will be recalled on startup.
   *  2022-04-27: Whenever you change the pattern length, or change the number of Euclidean Rhythm pulses,
   *     a new euclidean pattern is generated and this pattern is started from its starting position
   *     for the active pattern.
   *   
   *  Compile, upload and run the code one time with the INITIALIZE flag set.
   *  This will result in the nano writing 16 patches to its EEPROM.
   *  One patch will contain a pattern, the others are empty patches.
   *  Next comment out the line "#define INITIALIZE 1" and compile, upload 
   *  and run the code. This will program the Euclidean sequencer code into the nano 
   *  which will be run each time you start up the nano. The nano will read the patches
   *  from memory each time it starts and chooses the patch you saved last.
   *  
   *  Upon startup the sequencer is in 'Pattern Length / Tempo Mode'. When pressing the rotary encoder you can 
   *  switch to the second mode, 'Num Pulse Mode'. Press it again and the mode is 'Rotate Mode'.
   *  Press again and the mode returns to 'Pattern Length / Tempo Mode'.
   *  
   *  In 'Pattern Mode' rotating the encoder will increase or decrease the pattern length of the chosen trigger channel.
   *  Each time you change the length of the pattern a new fitting Euclidean pattern is generated depending on
   *  the number of pulses chosen (see below) for this pattern.
   *  Rotaring the dial with the 'Tap Tempo' button pressed at the same time, will allow for changing 
   *  tempo of the sequencer's internal clock. 
   *  In 'Num Pulse Mode' the encoder can be used to set the number of trigger pulses for the chosen trigger channel. 
   *  The pulses will follow a Euclidean pattern.
   *  In 'Rotate Mode' it is possible to shift the trigger pulse pattern accross the total length of the pattern.
   *  
   *  Pressing 'Tap Tempo' and the rotary switch at the same time will select the 'Program Mode'.
   *  Using the rotary encoder's knob 1 of 16 memory cells can be selected.
   *  Pressing the trigger channel D button will store the current pattern of the selected trigger channel to the corresponding memory cell.
   *  Pressing the trigger channel A button will recall the patch pattern stored in the EEPROM memory cell and store it in 
   *  the sequencer's active memory at the patch number selected with the rotary dial.
   *  Pressing the trigger channel B button will clear the patch in the eeprom the rotary encoder is set to. 
   *  The pattern already in the working memory however will not be changed. Don't forget to save it is you like it.
   *  
   *  The leds will light up red for the locations containing a patch and green for the locations that are cleared.
   *  When in 'Program Mode' pressing the rotary switch will switch to 'Pattern Length / Tempo Mode'.
   */

// Show some debug info on the serial port.
#define DEBUG 1

//#define INITIALIZE 1

// Use the switches below to make the brightness of the neopixel oscillate.

#define LED_FX 1 // Oscillate LEDs in all modes except PROGRAM_MODE.
//#define LED_FX_BLACK 1 // When set the intensity will fall back to 0, otherwise to 1.
//#define PATCH_MEMORY_LED_FX 1 // Oscillate LEDs in PROGRAM_MODE

#include <Adafruit_NeoPixel.h>    // Include Adafruit_NeoPixel library
#include <Encoder.h>              // Include rotary encoder library
#include <EEPROM.h>

/* 
    EEPROM memory layout:
    0: selectedTriggerChannel
    1: default programNumber
    2: memoryCellsInUse
    3: delayTime
    4: 16 patches:
     - pulses[5]
     - channelPattern[5]
     - patternLength[5]

*/

Encoder myEnc(3, 4);              // Attach rotary encoder to digital pins 3 and 4

#define PIXEL_PIN      2          // Attach NeoPixel ring to digital pin 2
#define PROG_PIN       5          // Attach rotary encoder switch to digital pin 5
#define CLK_PIN        6          // Attach external clock select switch to digital pin 6
#define TRIG_D_PIN     7          // Attach the four drum/voice trigger output pins to 
#define TRIG_C_PIN     8          //   digital pins 7-10.  I mixed up two wires when
#define TRIG_B_PIN     9          //   assembling the modules, and it was easier to
#define TRIG_A_PIN    10          //   swap pins 7 and 10 than to resolder things. :)
#define BUTTON_PIN    A0          // The trigger select buttons are on a voltage divider attached to A0
#define POT_PIN       A1          // The pulse width potentiometer is attached to analog pin A1
#define EXT_CLK_PIN   A2          // The external clock CV is attached to analog pin A2
#define RESET_PIN     12          // RESET_PIN CV in on D12
// If the clock switch is set to external then the tap tempo button doubles as a function key.
#define TAP_TEMPO_PIN 11          // Tap tempo / Function button on D11 
#define ON LOW
#define OFF HIGH

// Modes you can choose by pressing the rotary encoder's button.
#define PATTERN_LENGTH_MODE 1 // also used as "pulse length mode"
#define NUM_PULSE_MODE      2
#define ROTATE_MODE         3
#define PROGRAM_MODE        4 // Allows to store patterns in EEPROM or to recall them. This mode can only be reached 
                              // pressing the escape button AND the rotary encoder button.
#define MAX_MODE            3 // Note, we do not count PROGRAM_MODE here.         

#define F1_PIN A7            // Function F1 in
#define F2_PIN A6            // Function F2 in
#define F3_PIN A5            // Function F3 in
#define F4_PIN A4            // Function F4 in
#define SEQ_CLOCK_OUT_PIN A3 // Sequencer clock out (whether internal or from external source)
#define EXT_CLOCK_IN_PIN  A2 // Ext Clock in

#define HALF_DAC_RANGE 500

#define NUM_NEOP_LEDS 16     // The number of LEDs in the neopixel wheel.

#define MAX_DELAY_TIME 500   // Limit the slowest loop time to 1 sec per step.
                             // Change if you want slower tempos.          

#define NR_OF_MEMORY_CELLS NUM_NEOP_LEDS

#define EEPROM_BASE_ADDRESS 0

#define TRIG_A 3
#define TRIG_B 2
#define TRIG_C 1
#define TRIG_D 0

#define MAX_ADC_VALUE 1000 // This number was determined empirically.

#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))

char CHANNEL_NAMES[4] = {'D', 'C', 'B', 'A'};

#ifdef DEBUG
  char tmp[255];
#endif

static int step[4];               // The active step for each trigger channel.
struct Patch {
  int pulses[4];                  // The number of pulses to be generated for each of the 4 triggers.
  int patternLength[4];           // The pattern length for each trigger output.
  unsigned int channelPattern[4]; // The pattern of Euclidean distributed pulses,
};

Patch emptyPatch;
Patch currentPatch;               // The patch the sequencer is playing.

Patch patches[NR_OF_MEMORY_CELLS];
unsigned int delayTime  = 125;    // The time between steps persists in each loop.
unsigned int memoryCellsInUse = 0;
int chosenPatchNumber = 0;
int candidatePatchNumber = 0;
int selectedTriggerChannel = -1;

Adafruit_NeoPixel pixels(NUM_NEOP_LEDS, PIXEL_PIN, NEO_GRB + NEO_KHZ800); // Set up the NeoPixel ring

// Fill the dots one after the other with a color.
void colorWipe(uint32_t c, uint8_t wait, int direction = 0) {
  if (direction == 0) {
    for (uint16_t i = 0; i < pixels.numPixels(); i++) {
      pixels.setPixelColor(i, c);
      pixels.show();
      delay(wait);
    } 
  } else {
    for (uint16_t i = pixels.numPixels()-1; i > 0; i--) {
      pixels.setPixelColor(i, c);
      pixels.show();
      delay(wait);
    }
  }
}

// The Euclid function returns an unsigned integer whose bits reflect a Euclidean rhythm of
// "pulses" spread across "patternLength[]."  There are several ways to implement this, but 
// here an adapted version of algorithm described at:
// https://www.computermusicdesign.com/simplest-euclidean-rhythm-algorithm-explained/
// is used.

unsigned int euclid(Patch thisPatch, int channelNumber) {
  int bucket = 0;
  unsigned int number = 0;                                           // Number is the number of pulses that have been allocated so far.
  for (int i = 0; i < thisPatch.patternLength[channelNumber]; i++) { // Cycle through all of the possible steps in the sequence.
    bucket = bucket + thisPatch.pulses[channelNumber];               // Fill a "bucket" with the number of pulses to be allocated.
    if (bucket >= thisPatch.patternLength[channelNumber]) {          // If the bucket exceeds the maximum number of steps then "empty
      bucket = bucket - thisPatch.patternLength[channelNumber];      // the bucket" by setting the i-th bit in the sequence and
      number |= 1 << i;                                              // Refill the now empty bucket with the remainder.
    }
  }
  return number; // Return the sequence encoded as the bits set in Number.
}

// The rotateLeft function shifts the bits in an unsigned integer by one place. Any bit that falls
// off to the left (of the most significant bit) is shifted around to become the new least
// significant bit.  For our purposes, this means that we can rotate the Euclidean Rhythm clockwise
// on the ring.
void rotateLeft(Patch &thisPatch, int channelNumber) {
  int DROPPED_MSB;                        // Need to keep track of any bits dropped off the left
  DROPPED_MSB = (thisPatch.channelPattern[channelNumber] >> (thisPatch.patternLength[channelNumber] - 1)) & 1; // aka the old most significant bit
  // Shift all the bits in Pattern to the left by 1 and add back in the new least significant bit if needed.
  thisPatch.channelPattern[channelNumber] = (thisPatch.channelPattern[channelNumber] << 1) | DROPPED_MSB; 
}

// The rotateRight function shifts the bits in an unsigned integer by one place. Any bit that falls
// off to the right (of the least significant bit) is shifted around to become the new most
// significant bit.  For our purposes, this means that we can rotate the Euclidean Rhythm counter
// clockwise on the ring.
void rotateRight(Patch &thisPatch, int channelNumber) {
  int DROPPED_LSB;                        // Need to keep track of any bits dropped off the right
  DROPPED_LSB = thisPatch.channelPattern[channelNumber] & 1;              // aka the old least signficant bit.
  thisPatch.channelPattern[channelNumber] = (thisPatch.channelPattern[channelNumber] >> 1) & (~(1 << (thisPatch.patternLength[channelNumber] - 1))); // Shift all the bits in Pattern to the right by 1.
  thisPatch.channelPattern[channelNumber] = thisPatch.channelPattern[channelNumber] | (DROPPED_LSB << (thisPatch.patternLength[channelNumber] - 1)); // Tack the old LSB onto the new MSB if needed
}

// checkButtons and checkProgramButtons decode the voltage values from the button voltage divider circuit. The values here
// assume that 1k, 2.2k, 4.7k and 10k resistors are switched in line with a 5V source and dropped
// across a 10k resistor before going into the analog input channel. The ranges should be big
// enough to accommodate resistor variation, but if a button doesn't switch to the trigger as expected
// you should Serial.println(Buttons) and adjust the ranges as needed for your circuit.

int checkProgramButtons(int &selectedProgramButtonName) {
  // Note: the selectedProgramButtonName will only change if a button is pressed.
  // Read the voltage divider output.
  int buttonValue = analogRead(BUTTON_PIN); 
  //  Determine whether button A, B or D was pressed.
  if ((buttonValue > 0.40 * MAX_ADC_VALUE) && (buttonValue < 0.60 * MAX_ADC_VALUE)) { 
    selectedProgramButtonName = TRIG_D; // Button D is pressed.
    return 1;
  }
  if ((buttonValue > 0.75 * MAX_ADC_VALUE) && (buttonValue < 0.84 * MAX_ADC_VALUE)) {
    selectedProgramButtonName = TRIG_B; // Button B is pressed.
    return 1;
  }
  if (buttonValue > 0.85 * MAX_ADC_VALUE) { 
    selectedProgramButtonName = TRIG_A; // Button A is pressed.
    return 1;
  }
  selectedProgramButtonName = -1;
  return 0;
}

bool checkButtons(int &selectedChannel, int escapeButton) {
  // Note: the selectedChannel will only change if a button is pressed.
  static int previousSelectedChannel = -1;
  static bool change = false;

  // If no button was pressed, return the input parameter unchanged.
  // Pattern/trigger D will be shown in red.
  // Pattern/trigger C will be shown in green.
  // Pattern/trigger B will be shown in blue.
  // Pattern/trigger A will be shown in yellow .

  int buttonValue = analogRead(BUTTON_PIN); 
  // Read the voltage divider output and only change the selectedChannel if escapeButton is not pressed.
  if (escapeButton == 0) {       
    if ((buttonValue > 0.40 * MAX_ADC_VALUE) && (buttonValue < 0.60 * MAX_ADC_VALUE)) selectedChannel = TRIG_D; // Select pattern/trigger D if the first button is pressed
    if ((buttonValue > 0.64 * MAX_ADC_VALUE) && (buttonValue < 0.73 * MAX_ADC_VALUE)) selectedChannel = TRIG_C; // Select pattern/trigger C if the second button is pressed
    if ((buttonValue > 0.75 * MAX_ADC_VALUE) && (buttonValue < 0.84 * MAX_ADC_VALUE)) selectedChannel = TRIG_B; // Select pattern/trigger B if the third button is pressed
    if (buttonValue > 0.85 * MAX_ADC_VALUE) selectedChannel = TRIG_A;   // Select pattern/trigger A if the last button is pressed
    if (previousSelectedChannel != selectedChannel) {
      previousSelectedChannel = selectedChannel;
      change = true;
    } else {
      change = false;
    }
  }
  return change;
}

void ClearNeoPixelPattern() {
  // Erases all lights on the NeoPixel in preparation for an updated pattern to be displayed.
  for (int i = 0; i < NUM_NEOP_LEDS; i++) {         // Step through each pixel in the ring and
    pixels.setPixelColor(i, pixels.Color(0, 0, 0)); // turn each pixel off.
  }
  pixels.show();                                    // Update the pixels displayed on the ring.
}

void getCursorColor(int mode, int &R, int &G, int &B) {
  switch (mode) { 
    case PATTERN_LENGTH_MODE: R = 5; G = 5; B = 5; break; // Use the mode to determine the color.
    case NUM_PULSE_MODE:      R = 5; G = 0; B = 5; break; // White = Length and Tempo (with tap pressed)
    case ROTATE_MODE:         R = 2; G = 2; B = 0; break; // Violet = Pulses, Yellow = Rotation      
  }  
}

void updateCnt(float &cnt, float &delta, float minDelta) {
  // This function calculates the amplitude value of the oscillation 
  // used to modulate the brightness of the NeoPixels.
  #ifdef LED_FX
    cnt += delta;
    if (cnt > 30) delta = -delta;
    #ifdef LED_FX_BLACK
      if (cnt < 0) delta = -delta;
    #else
      if (cnt < minDelta) delta = -delta;
    #endif
  #endif
}

void showPatchMemory(int selectedPatch, unsigned int memoryCellsInUse) {
  // Show whether there are patches stored in the 16 memory cells or not.
  // Red = memory cell is in use.
  // Green = memory cell is empty.
  int R = 0;
  int G = 0;
  int B = 0;
  const float DELTA = 0.04F;
  static float delta = DELTA;
  static float cnt = 1.0;
  for (int i = 0; i < NR_OF_MEMORY_CELLS; i++) { // Step through each pixel in the ring.
    int cellIsInUse = (CHECK_BIT(memoryCellsInUse, i) > 0);
    if (cellIsInUse && (i == selectedPatch)) {
      R = (int) (15.0 * cnt); G = 0; B = 0;
    } else if (i == selectedPatch) { 
      // Show position of rotary encoder is on current Patch.
      R = 0; G = (int) (15.0 * cnt); B = 0;
    } else if (cellIsInUse) {
      R = 1; G = 0; B = 0;
    } else {
      // Show position of rotary encoder when in other positions.
      R = 0; G = 1; B = 0;
    }
    pixels.setPixelColor(i, pixels.Color(R, G, B));
  }
  pixels.show();  // Update the pixels displayed on the NeoPixel ring.
  #ifdef PATCH_MEMORY_LED_FX
    updateCnt(cnt, delta, DELTA);
  #endif
}

void showBitPattern(int channelNumber, unsigned int pattern, int patternLength, int mode) {
  // showBitPattern displays a Euclidean pattern associated with the bits set in Pattern. The color of the
  // pattern is dictated by channel number "channelNumber". You can change the ring colors to match the buttonValue
  // and output LEDs you use in your build here.
   
  const float DELTA = 0.05;
  static float delta = DELTA;
  static float cnt = 30.0;
  int R = 0;
  int G = 0;
  int B = 0;

  // Get the cursor colors to denote the beginning and end of the pattern.
  // We use low intensity here, to contrast with the actual pattern.
  getCursorColor(mode, R, G, B);
  // Show the pattern length by highlighting the 1st and last led.
  // Choose the color of the 'mode' for this. This makes it easier to see in 
  // what mode the sequencer is.
  pixels.setPixelColor(0, pixels.Color(R, G, B));
  pixels.setPixelColor(currentPatch.patternLength[channelNumber] - 1, pixels.Color(R, G, B)); 
  
  switch (channelNumber) {
    case 0: R = (int) cnt; G =  0; B =  0; break;  // Pattern/trigger 1 will be shown in red.
    case 1: R =  0; G = (int) cnt; B =  0; break;  // Pattern/trigger 2 will be shown in green.
    case 2: R =  0; G =  0; B = (int) cnt; break;  // Pattern/trigger 3 will be shown in blue.
    case 3: R = (int) cnt; G = (int) cnt; B =  0; break;  // Pattern/trigger 4 will be shown in yellow.
  }
  for (int i = 0; i < patternLength; i++) {  // step through each pixel in the ring
    // If the i-th bit is set, then set that pixel to the appropriate color.
    if (pattern & (1 << i)) {                
      pixels.setPixelColor(i, pixels.Color(R, G, B));
    }
  }
  pixels.show(); // Update the pixels displayed on the NeoPixel led-ring.
  updateCnt(cnt, delta, 1);
}

void writePatchesToEEPROM(Patch patches[], unsigned int memoryCellsInUse, int patchNumber, int delayTime, int selectedTriggerChannel) {
  int address = EEPROM_BASE_ADDRESS;
  // Store the currently selected trigger channel number.
  // Store the number indicating the currently loaded program.
  // Store which memory cells are in use.
  // Store all patches each containing:
  // - pulses
  // - channelPattern
  // - patternLength
  EEPROM.put(address, selectedTriggerChannel);
  address += sizeof(int);
  EEPROM.put(address, patchNumber);
  address += sizeof(int);
  EEPROM.put(address, memoryCellsInUse);
  address += sizeof(int);  
  EEPROM.put(address, delayTime); // Determines the current 'speed' of the sequencer.
  address += sizeof(int);
  for (int i = 0; i < NR_OF_MEMORY_CELLS; i++) {
    EEPROM.put(address, patches[i]);
    address += sizeof(Patch);
  }
}

void writePatchToEEPROM(Patch thisPatch, unsigned int memoryCellsInUse, int patchNumber, int delayTime, int selectedTriggerChannel) {
  int address = EEPROM_BASE_ADDRESS;
  // Store the currently selected trigger channel number.
  // Store the number indicating the currently loaded program.
  // Store which memory cells are in use.
  // Store all patches, each containing:
  // - pulses
  // - channelPattern
  // - patternLength
  EEPROM.put(address, selectedTriggerChannel);
  address += sizeof(int);
  EEPROM.put(address, patchNumber);
  address += sizeof(int);
  EEPROM.put(address, memoryCellsInUse);
  address += sizeof(int);  
  EEPROM.put(address, delayTime); // Determines the current 'speed' of the sequencer.
  address += sizeof(int);
  // Only write/replace chosen patch to/in EEPROM.
  for (int i = 0; i < NR_OF_MEMORY_CELLS; i++) {
    if (i == patchNumber) { 
      EEPROM.put(address, thisPatch);
      break; // We end here.
    }
    address += sizeof(Patch);
  }
}

void writePatchNumberToEEPROM(int patchNumber) {
  int address = EEPROM_BASE_ADDRESS + sizeof(int);
  // Store this program number as the current program number.
  EEPROM.put(address, patchNumber);
}

void writeTriggerChannelToEEPROM(int triggerChannel) {
  int address = EEPROM_BASE_ADDRESS;
  EEPROM.put(address, triggerChannel); 
}

void copyPatch(Patch srcPatch, Patch &dstPatch) {
  for (int i = 0; i < 4; i++) {
    dstPatch.pulses[i] = srcPatch.pulses[i];
    dstPatch.patternLength[i] = srcPatch.patternLength[i];
    dstPatch.channelPattern[i] = srcPatch.channelPattern[i];
  }
}

void createEmptyPatch(Patch &dstPatch) {
  for (int i = 0; i < 4; i++) {
    dstPatch.pulses[i] = 0;
    dstPatch.patternLength[i] = 0;
    dstPatch.channelPattern[i] = 0;
  }
}

void displayPatch(int patchNumber) {
  // For debug purposes.
  char tmp[255];
  sprintf(tmp, "patchNumber: %d", patchNumber);
  Serial.println(tmp);
  for (int j = 0; j < 4; j++) {
    sprintf(tmp, "pulses[%d] = %d", j, patches[patchNumber].pulses[j]);
    Serial.println(tmp);
  }
  for (int j = 0; j < 4; j++) {
    sprintf(tmp, "pattern[%d] = %d", j, patches[patchNumber].channelPattern[j]);
    Serial.println (tmp);
  } 
  for (int j = 0; j < 4; j++) {
    sprintf(tmp, "length[%d] = %d", j, patches[patchNumber].patternLength[j]);
    Serial.println(tmp);
  } Serial.print("\n");
}

void displayPatch(Patch thisPatch) {
  // For debug purposes.
  char tmp[255];
  for (int j = 0; j < 4; j++) {
    sprintf(tmp, "pulses[%d] = %d", j, thisPatch.pulses[j]);
    Serial.println(tmp);
  }
  for (int j = 0; j < 4; j++) {
    sprintf(tmp, "pattern[%d] = %d", j, thisPatch.channelPattern[j]);
    Serial.println (tmp);
  } 
  for (int j = 0; j < 4; j++) {
    sprintf(tmp, "length[%d] = %d", j, thisPatch.patternLength[j]);
    Serial.println(tmp);
  } Serial.print("\n");  
}

int readPatchesFromEEPROM(Patch patches[], unsigned int &memoryCellsInUse, unsigned int &delayTime, int &selectedTriggerChannel) {
  // Read the currently selected trigger channel number.
  // Read the patch number.
  // Read which memory cells are in use.
  // Read the delay time which determines the speed of the sequencer.
  // Read all 16 patches. Each patch contains:
  // - 4 pulse counts
  // - 4 channel patterns
  // - 4 pattern lengths
  int address = EEPROM_BASE_ADDRESS;
  int chosenPatchNumber = 0;
  EEPROM.get(address, selectedTriggerChannel);
  address += sizeof(int);
  EEPROM.get(address, chosenPatchNumber);
  address += sizeof(int);
  EEPROM.get(address, memoryCellsInUse);
  address += sizeof(int);
  EEPROM.get(address, delayTime);
  address += sizeof(int);
  // Read all patches specific variables.
  for (int i = 0; i < NR_OF_MEMORY_CELLS; i++) {
    if (memoryCellsInUse & (1 << i)) {
      EEPROM.get(address, patches[i]);
      address += sizeof(Patch);
    } else {
      // If a cell is not in use, create and load an empty patch.
      createEmptyPatch(patches[i]);
    }
  }
  return(chosenPatchNumber);
}

void readPatchFromEEPROM(Patch patches[], int chosenPatchNumber) {
  // Read one particular patch from the EEPROM.
  
  int address = EEPROM_BASE_ADDRESS;
  // Read only the content of the patch.
  // selectedTriggerChannel
  address += sizeof(int);
  // patchNumber
  address += sizeof(int);
  // memoryCellsInUse
  address += sizeof(int);  
  // delayTime
  address += sizeof(int);
  for (int i = 0; i < NR_OF_MEMORY_CELLS; i++) {
    // patches[i]
    if (i == chosenPatchNumber) {
      EEPROM.get(address, patches[i]);
      displayPatch(patches[i]);
      break;
    }
    address += sizeof(Patch);
  }
}

void resetSteps(int step[]) {
  for (int triggerChannel = 0; triggerChannel < 4; triggerChannel++) {
    step[triggerChannel] = 0;
  }
}

void testAllLeds(int delayTime) {
  int LEDS[] = { TRIG_A_PIN, TRIG_B_PIN, TRIG_C_PIN, TRIG_D_PIN, F1_PIN, F2_PIN, F3_PIN, F4_PIN, SEQ_CLOCK_OUT_PIN };
  for (int j = 0; j < 10; j++) {
    for (int i = 0; i < 9; i++) {
      digitalWrite(LEDS[i], ON);
      delay(delayTime);
      digitalWrite(LEDS[i], OFF);
      delay(delayTime);
    }
  }
}

void testNeoPixel() {
  colorWipe(pixels.Color(5, 0, 0), 25); // Red
  colorWipe(pixels.Color(0, 5, 0), 20, 1); // Green
  colorWipe(pixels.Color(0, 0, 5), 15); // Blue
}

void initializePatchInEeprom() {
  // We write one predefined patch to EEPROM. This leaves 15 free empty patch memory cells.
  unsigned int memoryCellsInUse = 0;
  int selectedTriggerChannel = 3;
  int delayTime = 95;
  char tmp[100];

  int pulses[4];             // The number of pulses to be generated for each of the 4 triggers.
  int patternLength[4];      // The pattern length for each trigger output.
  unsigned int channelPattern[4];
  int patchNr = 0;
  int initialPatchNr = 0;

  pulses[0] =  5;
  pulses[1] =  5;
  pulses[2] =  3;
  pulses[3] =  4;

  channelPattern[0] = 10532;  // Start up the generator with default bit patterns for triggers 1-4.
  channelPattern[1] = 18724;  // Each number/pattern persists between loop iterations.
  channelPattern[2] = 16912;
  channelPattern[3] = 2340;   // channelPattern[1] corresponds to trigger D, ... channelPattern[4] corresponds to trigger A

  patternLength[0] = 14;
  patternLength[1] = 15;
  patternLength[2] = 15;
  patternLength[3] = 12;

  Serial.println(tmp);
  sprintf(tmp, "sizeof patch: %d", sizeof(patches));
  Serial.println(tmp);
  for (int j = 0; j < 4; j++) {
    patches[initialPatchNr].pulses[j] = pulses[j];
    Serial.print(patches[initialPatchNr].pulses[j]);
    Serial.print(" ");
  } Serial.print("\n");
  for (int j = 0; j < 4; j++) {
    patches[initialPatchNr].channelPattern[j] = channelPattern[j];
    Serial.print(patches[initialPatchNr].channelPattern[j]);
    Serial.print(" ");
  } Serial.print("\n");
  for (int j = 0; j < 4; j++) {
    patches[patchNr].patternLength[j] = patternLength[j];
    Serial.print(patches[initialPatchNr].patternLength[j]);
    Serial.print(" ");
  } Serial.print("\n");
  memoryCellsInUse = memoryCellsInUse | (1 << initialPatchNr);
  sprintf(tmp, "patchNr %d created.", initialPatchNr);
  Serial.println(tmp);
  sprintf(tmp, "memoryCellsInUse: %d", memoryCellsInUse);
  Serial.println(tmp);
  // The rest of the 16 patterns are empty.
  for (patchNr = 1; patchNr < NR_OF_MEMORY_CELLS; patchNr++) {
      for (int i = 0; i < 4; i++) {
        patches[patchNr].pulses[i] = 0;
        patches[patchNr].channelPattern[i] = 0;
        patches[patchNr].patternLength[i] = 0;
      }
  }
  writePatchesToEEPROM(patches, memoryCellsInUse, initialPatchNr, delayTime, selectedTriggerChannel);
}

void testLedsAndPixels() {
  testAllLeds(10);            // Show that leds are working.
  testNeoPixel();             // Show that neopixels are working.
  pixels.clear();             // And clear it
  testAllLeds(5);
}
 
void setup() {
  // Set up the serial console for debugging messages
  Serial.begin(115200); 
  #ifdef INITIALIZE  
    initializePatchInEeprom();
    Serial.println("Successfuly written 16 default patches to EEPROM.");
    Serial.println("Now recompile and upload with #define INITIALIZE commented out");
    Serial.println("to run Euclid-O-Matic (a Euclidean sequencer / rhythm generator.");
  #else
  pinMode(PROG_PIN, INPUT_PULLUP);     // Assign the encoder switch which closes to ground and uses an internal pullup resistor.
  pinMode(CLK_PIN, INPUT);             // Assign the external clock selector switch.
  pinMode(TRIG_D_PIN, OUTPUT);         // Assign trigger 1 out.
  pinMode(TRIG_C_PIN, OUTPUT);         // Assign trigger 2 out.
  pinMode(TRIG_B_PIN, OUTPUT);         // Assign trigger 3 out.
  pinMode(TRIG_A_PIN, OUTPUT);         // Assign trigger 4 out.
  pinMode(F1_PIN, INPUT);              // Assign the Funct 1 input.
  pinMode(F2_PIN, INPUT);              // Assign the Funct 2 input.
  pinMode(F3_PIN, INPUT);              // Assign the Funct 3 input.
  pinMode(F4_PIN, INPUT);              // Assign the Funct 4 input.
  pinMode(SEQ_CLOCK_OUT_PIN, OUTPUT);  // Assign the clock output
  pinMode(EXT_CLOCK_IN_PIN, INPUT);    // Assign the external clock input.
  pinMode(TAP_TEMPO_PIN, INPUT);       // Assign the tap tempo button.
  pinMode(RESET_PIN, INPUT);           // Assign the external reset input.
  pixels.begin();                      // Start the NeoPixel
  pixels.clear();
  testLedsAndPixels();                 // Show that leds and pixels are working.
  // Read all 16 stored (empty or not) patches from EEPROM.
  chosenPatchNumber = readPatchesFromEEPROM(patches, memoryCellsInUse, delayTime, selectedTriggerChannel);
  candidatePatchNumber = chosenPatchNumber;
  // Load the current patch with the stored information.
  sprintf(tmp, "Reading patch info from EEPROM.");
  Serial.println(tmp);
  sprintf(tmp, "Restoring patch: %d", chosenPatchNumber);
  Serial.println(tmp);
  copyPatch(patches[chosenPatchNumber], currentPatch);
  displayPatch(currentPatch);
  sprintf(tmp, "Setting speed to: %d", delayTime);
  Serial.println(tmp);
  sprintf(tmp, "Selecting channel: %c", CHANNEL_NAMES[selectedTriggerChannel]);
  Serial.println(tmp);

#endif  
  // Initialize step count for each trigger channel.
  for (int triggerChannel = 0; triggerChannel < 4; triggerChannel++) { 
    step[triggerChannel] = 0;
  }
  createEmptyPatch(emptyPatch);
}

void loop() {
  static int mode       = 0;              // This sequencer mode will persist in each loop.
  static long mode1Pos  = 0;              // The last encoder position in mode 1 is stored for a future return to that mode.
  static int prevExtClk = 0;              // Store the previous External Clock Pulse value for transition checking.
  static unsigned long thisTime;          // Prepare to query the Nano's clock.
  static unsigned long prevTime = 0;      // Keep track of how much time elapses between loops.
  static long oldPosition;                // Keep track of the previous encoder count.
  static bool prevProg;                   // Keep track of the previous encoder switch state.
  static bool prog;                       // Prepare to read the current encoder switch state.
  static int prevF1 = 0;                  // Keep track of the value of the F1 CV input.
  static int prevF2 = 0;                  // Keep track of the value of the F2 CV input.
  static int prevF3 = 0;                  // Keep track of the value of the F3 CV input.
  static int prevF4 = 0;                  // Keep track of the value of the F4 CV input.
  static int prevResetPinIn = 0;          // Keep track of the value of the Reset in CV input.
  int deltaResetPinIn = 0;                // Keep track of the value of the Reset in CV input.

  bool triggered = false;                     // Assume that no new step is going to occur this loop.
  long newPosition = myEnc.read();            // Read the current number of encoder counts.
  int triggerWidthPotValue = analogRead(POT_PIN);       // Read the trigger pulse width potentiometer value.
  int extClkV    = analogRead(EXT_CLK_PIN);   // Read the external clock input.
  int f1In       = analogRead(F1_PIN);        // Read the Func 1 input, will return 0 if no signal present, else 255
  int f2In       = analogRead(F2_PIN);        // Read the Func 2 input
  int f3In       = analogRead(F3_PIN);        // Read the Func 3 input
  int f4In       = analogRead(F4_PIN);        // Read the Func 4 input
  int resetPinIn = digitalRead(RESET_PIN);     // Read the external RESET_PIN input
  int escapeButton = digitalRead(TAP_TEMPO_PIN);// Read the tap tempo/escape button (will be HIGH when pressed);
  int programButtonName = -1;
  int programButtonValue;
  static int prevProgramButtonValue = 0;
  boolean change = false;
  int R = 0, G = 0, B = 0;
  

  if (mode == PROGRAM_MODE) {
    programButtonValue = checkProgramButtons(programButtonName); // Find out whether Trig A, B or D was pressed.
    if (programButtonValue == 0) { // No button was pressed.
      prevProgramButtonValue = 0;
    }
  } else {
    change = checkButtons(selectedTriggerChannel, escapeButton);// Query the trigger buttons and update the active trigger if needed.
    if (change) { // Remember the channel chosen (this will be recalled when powering up).
      writeTriggerChannelToEEPROM(selectedTriggerChannel);
    }
  }

  prevProg = prog;                          // Store the last encoder switch position.
  prog     = digitalRead(PROG_PIN);         // Get the current encoder switch position.

  thisTime = millis();                      // Get the current Nano clock value in msec.

  if ((prog == true) && (prevProg == false)) { // If the encoder switch was just pressed, we need to switch modes.
    if (mode == 1) mode1Pos = newPosition;     // Store the current encoder position for a future return to that mode.
    if (escapeButton) {                        // If the escape button is pressed as well, then we start the program mode.
      mode = PROGRAM_MODE;
    } else {
      mode++;                                  // Cycle to the next mode.
      if (mode > MAX_MODE) mode = 1;           // Go back to mode 0 after mode MAX_MODE.
      if (mode == 1) {                         // Restore the encoder position when last in the current mode.
        myEnc.write(mode1Pos);
        newPosition = mode1Pos;
      }
    }
    oldPosition = newPosition;                 // The current encoder position will be the old position next loop.
  }

  ClearNeoPixelPattern();                      // Clear the NeoPixel ring in preparation for any changes to the pattern.

  // Respond to function CV inputs.
  // Let the left top Func input (Func 1) change the left most
  // trigger output (Trig A).
  if ((f1In > HALF_DAC_RANGE) && (prevF1 < HALF_DAC_RANGE)) {
    rotateLeft(currentPatch, TRIG_D);
  }
  prevF1 = f1In;

  if ((f2In > HALF_DAC_RANGE) && (prevF2 < HALF_DAC_RANGE)) {
    rotateLeft(currentPatch, TRIG_C);
  }
  prevF2 = f2In;

  if ((f3In > HALF_DAC_RANGE) && (prevF3 < HALF_DAC_RANGE)) {
      rotateLeft(currentPatch, TRIG_B);
  }
  prevF3 = f3In;

  if ((f4In > HALF_DAC_RANGE) && (prevF4 < HALF_DAC_RANGE)) {
    rotateLeft(currentPatch, TRIG_A);
  }
  prevF4 = f4In;

  // Reset on positive going edges of the reset pin.
  deltaResetPinIn = resetPinIn - prevResetPinIn;
  if (deltaResetPinIn == 1) {
    for (int triggerChannel = 0; triggerChannel < 4; triggerChannel++) {
        step[triggerChannel] = 0;
    }
    prevResetPinIn = 1;
  } 
  if (deltaResetPinIn == -1) {
    prevResetPinIn = 0;
  }

  switch (mode) {
    case PATTERN_LENGTH_MODE: // Mode 1 - Manually adjust the pattern tempo (based on the internal clock).
      if (escapeButton) {     // We only allow for changing the tempo IF the tap tempo button is pressed at the same
                              // time the rotary encoder is operated (otherwise the pattern length can be changed (see below)).
        // Increase/decrease the speed of the clock
        if (newPosition < oldPosition - 3) {
          delayTime -= 1;
          if (delayTime  < 10) delayTime  = 10;  // Limit the fastest loop time to 10 per step, which is crazy fast.
          oldPosition = newPosition;             // The current encoder position will be the old position next loop.
        } else if (newPosition > oldPosition + 3) {
          delayTime  += 1;        // The internal time delayTime  is proportional to the encoder position.
          if (delayTime  > MAX_DELAY_TIME) delayTime  = MAX_DELAY_TIME;  // Limit the slowest loop time to 1 sec per step.
          // Change if you want slower tempos.          
          oldPosition = newPosition;           // The current encoder position will be the old position next loop.          
        }      
      }
      else { // Increase/decrease the pattern length for the active pattern.
        if (newPosition < oldPosition - 3) {      // Do something if the encoder has increased by 1 detent (4 pulses for my encoder).
          currentPatch.patternLength[selectedTriggerChannel]++;// Increase the pattern length by 1 for the active pattern.
          if (currentPatch.patternLength[selectedTriggerChannel] > NUM_NEOP_LEDS) {
            currentPatch.patternLength[selectedTriggerChannel] = NUM_NEOP_LEDS;
          }
          oldPosition = newPosition;              // The current encoder position will be the old position next loop.
          // Compute a Euclidean Rhythm for the active pattern.
          currentPatch.channelPattern[selectedTriggerChannel] = euclid(currentPatch, selectedTriggerChannel);
          // Restart all patterns
          resetSteps(step);
        } else if (newPosition > oldPosition + 3) {      // Do something if the encoder has decreased by 1 detent (4 pulses for my encoder).
          currentPatch.patternLength[selectedTriggerChannel]--;// Decrease the pattern length by 1 for the active pattern.
          if (currentPatch.patternLength[selectedTriggerChannel] < 0) {
            currentPatch.patternLength[selectedTriggerChannel] = 0;
          }
          oldPosition = newPosition;              // The current encoder position will be the old position next loop.
          // Compute a Euclidean Rhythm for the active pattern.
          currentPatch.channelPattern[selectedTriggerChannel] = euclid(currentPatch, selectedTriggerChannel);
                    // Restart all patterns
          resetSteps(step);
        }
        break;
      }
      break;
    case NUM_PULSE_MODE: // Mode 2 - Change the number of Euclidean Rhythm pulses of the active pattern.
      if (newPosition < oldPosition - 3) {      // Do something if the encoder has increased by 1 detent (4 pulses for my encoder).
        currentPatch.pulses[selectedTriggerChannel]++;       // Increase the number of pulses in the active pattern.
        if (currentPatch.pulses[selectedTriggerChannel] > currentPatch.patternLength[selectedTriggerChannel]) {
          currentPatch.pulses[selectedTriggerChannel] = currentPatch.patternLength[selectedTriggerChannel];// Limit the largest number of pulses to NumSteps.
        }
        oldPosition = newPosition;              // The current encoder position will be the old position next loop.
        // Compute a Euclidean Rhythm for the active pattern.
        currentPatch.channelPattern[selectedTriggerChannel] = euclid(currentPatch, selectedTriggerChannel);
        // Restart all patterns
        resetSteps(step);
      } else if (newPosition > oldPosition + 3) { // Do something if the encoder has decreased by 1 detent (4 pulses for my encoder).
        currentPatch.pulses[selectedTriggerChannel]--;       // Decrease the number of pulses in the active pattern.
        if (currentPatch.pulses[selectedTriggerChannel] < 0) {
          currentPatch.pulses[selectedTriggerChannel] = 0;            // Limit the smallest number of pulses to zero.
        }
        oldPosition = newPosition;              // The current encoder position will be the old position next loop.
        // Compute a Euclidean Rhythm for the active pattern.
        currentPatch.channelPattern[selectedTriggerChannel] = euclid(currentPatch, selectedTriggerChannel);
        // Restart all patterns
        resetSteps(step);
      }
      break;
    case ROTATE_MODE: // Mode 3 - Rotate the Euclidean Rhythm of the active pattern.
      if (newPosition > oldPosition + 3) {       // Do something if the encoder has increased by 1 detent (4 pulses for my encoder).
        oldPosition = newPosition;               // The current encoder position will be the old position next loop.
        rotateRight(currentPatch, selectedTriggerChannel);
      } else if (newPosition < oldPosition - 3){ // Do something if the encoder has decreased by 1 detent (4 pulses for my encoder).
        oldPosition = newPosition;               // The current encoder position will be the old position next loop.
        // Rotate the active pattern one bit to the left (clockwise).
        rotateLeft(currentPatch, selectedTriggerChannel);
      }
      break;
    case PROGRAM_MODE: // Mode 4 - Store all patterns to eprom and store the current patchnumber / recall all patterns
      if (newPosition > oldPosition + 3) {      // Do something if the encoder has increased by 1 detent (4 pulses for my encoder).
        // Rotate the active pattern one bit to the right (clockwise).
        oldPosition = newPosition;              // The current encoder position will be the old position next loop.
        candidatePatchNumber -= 1;
        if (candidatePatchNumber < 0) {
          candidatePatchNumber = NR_OF_MEMORY_CELLS - 1;
        }
      } else if (newPosition < oldPosition - 3) { // Do something if the encoder has decreased by 1 detent (4 pulses for my encoder).
        // Rotate the active pattern one bit to the left (clockwise).
        oldPosition = newPosition;                // The current encoder position will be the old position next loop.
        candidatePatchNumber += 1;
        if (candidatePatchNumber > NR_OF_MEMORY_CELLS - 1) {
          candidatePatchNumber = 0;
        }
      }  
      switch (programButtonName) { 
        case TRIG_A: // If Button-A is pressed, then copy the stored pattern from EEPROM and put it in the current pattern.
          if (prevProgramButtonValue == 0) {
            prevProgramButtonValue = 1;            
            chosenPatchNumber = candidatePatchNumber;
            copyPatch(patches[chosenPatchNumber], currentPatch);
            // The pattern lengths may have changed, so we want to sync the chosen patterns and restart their step counters.
            // Therefor, we initialize the step count for each trigger channel.
            for (int triggerChannel = 0; triggerChannel < 4; triggerChannel++) { 
              step[triggerChannel] = 0;
            }
            // Write the current patch number to EEPROM.
            writePatchNumberToEEPROM(chosenPatchNumber);
            displayPatch(chosenPatchNumber);
          }
          break;
        case TRIG_B: // Wipe the memory cell chosen clean. Note, this does not affect the current patch.
          if (prevProgramButtonValue == 0) {
            prevProgramButtonValue = 1;
            memoryCellsInUse = memoryCellsInUse & ~(1 << candidatePatchNumber);            
            copyPatch(emptyPatch, patches[candidatePatchNumber]);
            writePatchToEEPROM(emptyPatch, memoryCellsInUse, candidatePatchNumber, delayTime, selectedTriggerChannel);
          }
          break;
        case TRIG_D: // If Button-D is pressed, then write the current patch to the EEPROM at the chosen patch location.
          if (prevProgramButtonValue == 0) {
            prevProgramButtonValue = 1;            
            memoryCellsInUse = memoryCellsInUse | (1 << candidatePatchNumber);
            // Write chosen patch to candidate location in EEPROM.
            writePatchToEEPROM(currentPatch, memoryCellsInUse, candidatePatchNumber, delayTime, selectedTriggerChannel);
            // Either read the memory cell back to sync patches (slow because of EEPROM) OR copy the patch directly (fast because of in memory copy).
            //readPatchFromEEPROM(patches, candidatePatchNumber);
            copyPatch(currentPatch, patches[candidatePatchNumber]);
          }
          break; 
      }
      break;
  } // end swith(mode)

  if (mode == PROGRAM_MODE) {
    // show selected memory location number.
    showPatchMemory(candidatePatchNumber, memoryCellsInUse);
  } else {
    // Display the active pattern.
    showBitPattern(selectedTriggerChannel, currentPatch.channelPattern[selectedTriggerChannel], 
                   currentPatch.patternLength[selectedTriggerChannel], mode);
  }

  if (digitalRead(CLK_PIN) == true) {             // If we're using an external clock pulse.
    if ((extClkV > 512) && (prevExtClk < 128)) {  // Check to see if the pulse just went high.
      delayTime = thisTime - prevTime;            // If so, then capture the msec delayTime associated with the external pulse.
      triggered = true;                           // Note that we've just triggered a new step in the sequence.
    }
    prevExtClk = extClkV;                         // The current external clock input will be the previous input for the next loop.
  }
  else                                            // If we're using the internal clock to generate pulses.
  {
    if ((thisTime - prevTime) > delayTime ) {     // Check to see if we've waited long enough for the next step in the sequence.
      triggered = true;                           // Note that we've triggered a new step in the sequence.
    }
  }
  unsigned int trigWidth = map(triggerWidthPotValue, 0, 1023, delayTime  / 2, delayTime  / 50); // Calculate the pulse width based on the potentiometer setting and clock.
  if ((thisTime - prevTime) > trigWidth) {        // Turn off the trigger outputs if the pulse width has elapsed.
    digitalWrite(TRIG_A_PIN, HIGH);
    digitalWrite(TRIG_B_PIN, HIGH);
    digitalWrite(TRIG_C_PIN, HIGH);
    digitalWrite(TRIG_D_PIN, HIGH);
    digitalWrite(SEQ_CLOCK_OUT_PIN, HIGH);
  }

  // For each trigger channel we keep track of at which step we area separate step.
  if (triggered) {                                // If we're triggering a new step in the sequence.
    prevTime = thisTime;                          // The current time will be the previous time in the next loop.
    for (int triggerChannel = 0; triggerChannel < 4; triggerChannel++) {
      step[triggerChannel]++;               // Increment the step counter.
      if (step[triggerChannel] > currentPatch.patternLength[triggerChannel] - 1) {
        step[triggerChannel] = 0; // Reset the step counter if we've gone through all the beats for this trigger channel.
      }
    }
    // Show a cursor by lighting up each pixel and using color to show what mode we are in.
    getCursorColor(mode, R, G, B);
    // Light up the current step pixel with a color to indicate the current mode.
    pixels.setPixelColor(step[selectedTriggerChannel], pixels.Color(4 * R, 4 * G, 4 * B));   
    // Show/refresh all pixels.
    pixels.show(); 

    // Turn on trigger output + LED A..D if the current step is in the Euclidean Rhythm.
    if (currentPatch.channelPattern[TRIG_A] & (1 << step[TRIG_A])) digitalWrite(TRIG_A_PIN, LOW);
    if (currentPatch.channelPattern[TRIG_B] & (1 << step[TRIG_B])) digitalWrite(TRIG_B_PIN, LOW);
    if (currentPatch.channelPattern[TRIG_C] & (1 << step[TRIG_C])) digitalWrite(TRIG_C_PIN, LOW);
    if (currentPatch.channelPattern[TRIG_D] & (1 << step[TRIG_D])) digitalWrite(TRIG_D_PIN, LOW);     
    digitalWrite(SEQ_CLOCK_OUT_PIN, LOW);
  }
}
