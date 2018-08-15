/*********************************************************************
Buy parts from ADAFRUIT.com - Non sponsored endorsement.
Feather MO BLE
BMP 280
TFT 2.4 Feather Shield
Remixed together using Example Skethes and some custom coding. Use for Skydiving. 
SKYDIVING IS A DANGEROUS ACTIVITY, THIS CODE AND ANYTHING DIRIVED FROM IT SHOULD NOT BE USED AS THE SOLE DEVICE FOR ALTITUDE AWARENESS DURING A SKYDIVE
USING THIS CODE AND PROJECT RELEASES THE CREATOR FROM ALL LIABILITY

GEAR CHECKS SAVE LIVES.
*/

/* FEATHER BLE */
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

/* ALTI */
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_STMPE610.h>
#include <SD.h>
#include <Fonts/FreeSans24pt7b.h>
#include <Math.h> 

// This is calibration data for the raw touch data to the screen coordinates
// #define TS_MINX 150
// #define TS_MINY 130
// #define TS_MAXX 3800
// #define TS_MAXY 4000
 
#define TFT_DC   10
#define TFT_CS   9
#define STMPE_CS 6
#define SD_CS    5
#define BUFFPIXEL 80

#define VBATPIN A7
#define SAMPLESIZE 20
Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bme(BMP_CS); // hardware SPI
//Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);

float pressureSet;
float pressureLive;
float relAlt;
float currentTemp;
float currentAlt;
float roundAlt;
int round2;
int counter = 0;
float measuredvbat;



#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
//  while (! ble.isConnected()) {
 //     delay(500);
//  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

// ALTIMETER SETUP

   Serial.println(F("BMP280 test"));
    if (!bmp.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    tft.setCursor(35, 100);
    tft.setTextColor(ILI9341_RED);  tft.setTextSize(2);
    tft.println("ERROR LOADING SENSOR");
    tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2);
    tft.setCursor(35, 130);
    tft.println("<- BMP280 MISSING ->"); 
    tft.setCursor(15, 180);
    tft.println(" FAULTY ALTI ver 1.0");
    while (1);
  }

  if (!ts.begin()) {
    Serial.println("Couldn't start touchscreen controller");
    while (1);
  }

  tft.begin();
  SD.begin(SD_CS);
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  float pressureSet = 0;
  for(int i = 0; i < SAMPLESIZE; i++)
  {
    pressureSet += bmp.readPressure();
  }
  relAlt = (pressureSet / (SAMPLESIZE * 100));
  tft.setRotation(1);
  tft.setTextColor(ILI9341_YELLOW);  tft.setTextSize(2);
  tft.setCursor(0,0);
  tft.print("ALTITUDE:");
  tft.setCursor(200, 0);
  tft.print("TIME:");
  tft.println("");
  tft.setCursor(0, 185);
  tft.print("ALT: ");
  tft.setCursor(150, 185);
  tft.print("BATT: ");
  tft.setCursor(0, 205);
  tft.print("TEMP: ");
  tft.setCursor(0, 225);
  tft.print("GPL: ");


}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);


  /* Got a packet! */
  // printHex(packetbuffer, len);

  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
        float pressureSet = 0;
        for(int i = 0; i < SAMPLESIZE; i++)
            {
              pressureSet += bmp.readPressure();
                }
      relAlt = (pressureSet / (SAMPLESIZE * 100));
   //   relAlt = (bmp.readPressure()/100);
      Serial.println(" GPL Reset");
       tft.fillScreen(ILI9341_BLACK);
       tft.setRotation(1);
        tft.setTextColor(ILI9341_YELLOW);  tft.setTextSize(6);
        tft.setCursor(0,0);
        tft.print("GROUND PRESSURE OFFSET HAS BEEN RESET");
       
        tft.fillScreen(ILI9341_BLACK);
         tft.setRotation(1);
        tft.setTextColor(ILI9341_YELLOW);  tft.setTextSize(2);
        tft.setCursor(0,0);
  tft.print("ALTITUDE:");
  tft.setCursor(200, 0);
  tft.print("TIME:");
  tft.println("");
  tft.setCursor(0, 185);
  tft.print("ALT: ");
  tft.setCursor(150, 185);
  tft.print("BATT: ");
  tft.setCursor(0, 205);
  tft.print("TEMP: ");
  tft.setCursor(0, 225);
  tft.print("GPL: ");
       
    } else {
      Serial.println(" released");
    }
  }

  // GPS Location
  if (packetbuffer[1] == 'L') {
    float lat, lon, alt;
    lat = parsefloat(packetbuffer+2);
    lon = parsefloat(packetbuffer+6);
    alt = parsefloat(packetbuffer+10);
    Serial.print("GPS Location\t");
    Serial.print("Lat: "); Serial.print(lat, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print("Lon: "); Serial.print(lon, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print(alt, 4); Serial.println(" meters");
  }

  // Accelerometer
  if (packetbuffer[1] == 'A') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Accel\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Magnetometer
  if (packetbuffer[1] == 'M') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Mag\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Gyroscope
  if (packetbuffer[1] == 'G') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Gyro\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Quaternions
  if (packetbuffer[1] == 'Q') {
    float x, y, z, w;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    w = parsefloat(packetbuffer+14);
    Serial.print("Quat\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.print('\t');
    Serial.print(w); Serial.println();
  }
/***** ALTI LOOP *******/
  if (len == 0) {
 measuredvbat = analogRead(VBATPIN);
 measuredvbat *= 2;    // we divided by 2, so multiply back
 measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
 measuredvbat /= 1024; // convert to voltage

// ===============================================================================
    currentTemp = (bmp.readTemperature()*9/5 + 32 - 13);
    
    float pressureLive = 0;
  for(int i = 0; i < SAMPLESIZE; i++)
  {
    pressureLive += (bmp.readAltitude(relAlt)*3.28084);
  }
    currentAlt = (pressureLive / (SAMPLESIZE));
  //   currentAlt = (bmp.readAltitude(relAlt)*3.28084);
    roundAlt = round(currentAlt);
    round2 = (roundAlt);
    Serial.print("Temperature = ");
    Serial.print(currentTemp);
    Serial.println("");

    Serial.print("Current BPI = ");
    Serial.print(bmp.readPressure()/100);
    Serial.println("");
   
    Serial.print("Current Altitude = ");
    Serial.print(bmp.readAltitude(relAlt)*3.28084); // this should be adjusted to your local forcase
    Serial.println("");

    Serial.print("Rounded Altitude = ");
    Serial.print(roundAlt);
    Serial.println("");

    Serial.print("Ground Pressure Level = ");
    Serial.print(relAlt); 
    Serial.println("");

    Serial.print("Time Counted = ");
    Serial.print(counter);
    Serial.println("");

  
    Serial.print("VBat = " ); Serial.print(measuredvbat); Serial.println("");
  

    counter++;
  tft.begin();
  tft.setRotation(1);
  tft.setCursor(265, 0);
  tft.print(counter);
  tft.fillRect(0,25,300,150,ILI9341_BLACK);   
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setFont(&FreeSans24pt7b);
  tft.setTextSize(3);
  tft.setCursor(0, 130);
  // tft.println(bmp.readAltitude(relAlt)*3.28084);
  tft.println(round2);
  tft.setFont();
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(65, 185);
  tft.print(currentAlt);
  tft.setCursor(215, 185);
  tft.print(measuredvbat);
  tft.setCursor(65, 205);
  tft.print(currentTemp);
  tft.print(" F");
  tft.setCursor(65, 225);
  tft.print(relAlt);
  tft.print(" Pa");
  
 
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(counter);
    dataFile.print(";");
    dataFile.print(round2);
    dataFile.print(";"); 
    dataFile.print(currentTemp);
    dataFile.println(";");
    dataFile.close();
    // print to the serial port too:
  //  Serial.println("Succesful Log to SDCARD");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }

//  delay(1000);



  } 
  
}

void bmpDraw(char *filename, int16_t x, int16_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col, x2, y2, bx1, by1;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  if((x >= tft.width()) || (y >= tft.height())) return;

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print(F("File not found"));
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.print(F("File size: ")); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print(F("Header size: ")); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        x2 = x + bmpWidth  - 1; // Lower-right corner
        y2 = y + bmpHeight - 1;
        if((x2 >= 0) && (y2 >= 0)) { // On screen?
          w = bmpWidth; // Width/height of section to load/display
          h = bmpHeight;
          bx1 = by1 = 0; // UL coordinate in BMP file
          if(x < 0) { // Clip left
            bx1 = -x;
            x   = 0;
            w   = x2 + 1;
          }
          if(y < 0) { // Clip top
            by1 = -y;
            y   = 0;
            h   = y2 + 1;
          }
          if(x2 >= tft.width())  w = tft.width()  - x; // Clip right
          if(y2 >= tft.height()) h = tft.height() - y; // Clip bottom
  
          // Set TFT address window to clipped image bounds
          tft.startWrite(); // Requires start/end transaction now
          tft.setAddrWindow(x, y, w, h);
  
          for (row=0; row<h; row++) { // For each scanline...
  
            // Seek to start of scan line.  It might seem labor-
            // intensive to be doing this on every line, but this
            // method covers a lot of gritty details like cropping
            // and scanline padding.  Also, the seek only takes
            // place if the file position actually needs to change
            // (avoids a lot of cluster math in SD library).
            if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
              pos = bmpImageoffset + (bmpHeight - 1 - (row + by1)) * rowSize;
            else     // Bitmap is stored top-to-bottom
              pos = bmpImageoffset + (row + by1) * rowSize;
            pos += bx1 * 3; // Factor in starting column (bx1)
            if(bmpFile.position() != pos) { // Need seek?
              tft.endWrite(); // End TFT transaction
              bmpFile.seek(pos);
              buffidx = sizeof(sdbuffer); // Force buffer reload
              tft.startWrite(); // Start new TFT transaction
            }
            for (col=0; col<w; col++) { // For each pixel...
              // Time to read more pixel data?
              if (buffidx >= sizeof(sdbuffer)) { // Indeed
                tft.endWrite(); // End TFT transaction
                bmpFile.read(sdbuffer, sizeof(sdbuffer));
                buffidx = 0; // Set index to beginning
                tft.startWrite(); // Start new TFT transaction
              }
              // Convert pixel from BMP to TFT format, push to display
              b = sdbuffer[buffidx++];
              g = sdbuffer[buffidx++];
              r = sdbuffer[buffidx++];
              tft.writePixel(tft.color565(r,g,b));
            } // end pixel
          } // end scanline
          tft.endWrite(); // End last TFT transaction
        } // end onscreen
        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) Serial.println(F("BMP format not recognized."));
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}





