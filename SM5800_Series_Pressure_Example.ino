/*          ********************SM5800 Series Servoflo Pressure Sensor Example For ARDUINO!!*************************

   You can use ANY SM5800 series pressure sensor by adjusting the values from line 30 - 35. Just connect pin 2 to GND, pin 7 to 5V, pin 4 to SDA, pin 5 to SCL.
Pin 1 is labeled on the pin side of the board.
   You can purchase more sensors from Servoflo.com. They carry a variety of all types of sensors (temperature, pressure, humidity, gas, and much more). Please tell them
"BillyBob sent you". A special thanks to David Ezekiel @ Servoflo for helping me find the best possable sensor for my application. 
*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "Wire.h"
//DO NOT CHANGE THESE VALUES (THESE ARE THE SENSOR ACCESS ADDRESSES FOR I2C DATA)
#define SENSOR_I2C_ADDR         0x5f //
#define SENSOR_REG_PRESSURE_LSB 0x80 //
#define SENSOR_REG_PRESSURE_MSB 0x81 //
#define SENSOR_REG_TEMP_LSB     0x82 //
#define SENSOR_REG_TEMP_MSB     0x83 //
////////////////////////////////////////////////////////////////////////////////////////////
//ADJUST THESE VALUES FOR THE TYPE OF SENSOR,THE DESIRED UNIT OF MAESURE, THE READABLE PRESSURE RANGE,AND THE ZERO VALUE. EACH INDIVIDUAL SENSOR HAS ITSS OWN ZERO VAL. MY ZERO VALUE IS 1960, BUT YOURS WILL BE DIFFERENT.
////////////////////////////////////////////////////////////////////////////////////////////
String SENSOR_TYPE             = ("DIFFERENTIAL"); // DIFFERENTIAL OR GAUGE ///////////////////////////////////
String UNIT_OF_MEASURE         = ("inWC");         // INWC, PSI, PA  "inches h2o, lbs/in^2, Pascals"               //
double SENSOR_MINIMUM          = -1.5;             // LOWEST RATED PRESSURE FOR UINT of MEASURE SELECTED                 //
uint16_t SENSOR_ZERO_READING   = 1960;             // RAW READING WHEN THE SENSOR IS READING ZERO(its different for every sensor)  //  ADJUST THESE TO THE MODEL YOU HAVE
double SENSOR_MAXIMUM          = 1.5;              // MAXIMUM RATED PRESSURE FOR UNIT SELECTEED                          //
uint16_t TIME_BETWEEN_READINGS = 1000;             // READING INTERVAL IN MILLISECONDS(time between reads)   //    //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//DO NOT ADJUST ANYTHING HERE.
uint8_t  pressLSB;
uint8_t  pressMSB;
uint16_t pressRaw;
uint8_t  tempLSB;
uint8_t  tempMSB;
uint16_t tempRaw;
double pressure;                         // ACTUAL MAESURED PRESSURE
double temperature;                      // ACTUAL MAESURED TEMPERATURE

void setup()
{
  Wire.begin();
  Serial.begin(9600);
}


void loop()

// READ THE PRESSURE
{
  Wire.beginTransmission(SENSOR_I2C_ADDR);  //Send the address of device
  Wire.write(SENSOR_REG_PRESSURE_MSB);      //send the address of the MSB
  Wire.endTransmission(false);              //Send a repeated start
  Wire.requestFrom(SENSOR_I2C_ADDR, 1);     //Request 1 byte of data

  if (Wire.available() == 1)
  {
    pressMSB = Wire.read();                      // Read the MSB register
  }
  else                                      //If more or less than 1 byte have been received something isn't right.
  {
    while (Wire.available())                //Clear the buffer
      Wire.read();
    Serial.println("I2C transaction error");
  }
  Wire.beginTransmission(SENSOR_I2C_ADDR);  // call the device address again
  Wire.write(SENSOR_REG_PRESSURE_LSB);      // call the LSB register address
  Wire.endTransmission(false);              // repeated start
  Wire.requestFrom(SENSOR_I2C_ADDR, 1);     // Get the 1 byte of data from the LSB register

  if (Wire.available() == 1)
  { // byte contains the 6 least significant bits of the LSB
    pressLSB = Wire.read();
  }
  else                                      //If more or less than 1 byte have been received something isn't right.
  {
    while (Wire.available())                //Clear the buffer
      Wire.read();
    Serial.println("I2C transaction error");
  }
  // Build the raw pressure from the two bytes received according to the release notes
  pressRaw = ((dataMSB << 8)|(dataLSB << 2)) >> 2;



// READ THE TEMPERATURE
  Wire.beginTransmission(SENSOR_I2C_ADDR);  //Send the address of device
  Wire.write(SENSOR_REG_TEMP_MSB);      //send the address of the MSB
  Wire.endTransmission(false);              //Send a repeated start
  Wire.requestFrom(SENSOR_I2C_ADDR, 1);     //Request 1 byte of data

  if (Wire.available() == 1)
  {
    tempMSB = Wire.read();                      // Read the MSB register
  }
  else                                      //If more or less than 1 byte have been received something isn't right.
  {
    while (Wire.available())                //Clear the buffer
      Wire.read();
    Serial.println("I2C transaction error");
  }

  Wire.beginTransmission(SENSOR_I2C_ADDR);  // call the device address again
  Wire.write(SENSOR_REG_TEMP_LSB);      // call the LSB register address
  Wire.endTransmission(false);              // repeated start
  Wire.requestFrom(SENSOR_I2C_ADDR, 1);     // Get the 1 byte of data from the LSB register

  if (Wire.available() == 1)
  { // byte contains the 6 least significant bits of the LSB
    tempLSB = Wire.read();
  }
  else                                      //If more or less than 1 byte have been received something isn't right.
  {
    while (Wire.available())                //Clear the buffer
      Wire.read();
    Serial.println("I2C transaction error");
  }

 tempRaw = ((tempMSB << 8)|(tempLSB << 2)) >> 2 // Bit manipulation to move the data bytes to their correct positions.
 temperature = mapfloat(tempRaw, 2770, 2100, 15, 75);


// PARSE THE DATA INTO SOMETHING MEANINGFUL (according to the user input for the sensor type)
  if (SENSOR_TYPE == "GAUGE") {
    pressure = mapfloat(pressRaw, SENSOR_ZERO_READING, 3891, 0.0, SENSOR_MAXIMUM);
  }

  if (SENSOR_TYPE == "DIFFERENTIAL") {
    if (pressRaw == SENSOR_ZERO_READING) {
      pressure = 0.0;
    }
    if (pressRaw > SENSOR_ZERO_READING) {
      pressure = mapfloat(pressRaw, SENSOR_ZERO_READING, 3891, 0.0, SENSOR_MAXIMUM);
    }
    if (pressRaw < SENSOR_ZERO_READING) {
      pressure = mapfloat(pressRaw, 205, SENSOR_ZERO_READING, SENSOR_MINIMUM, 0.0);
    }
  }

// PRINT THE READINGS TO THE SERIAL MONITOR
  Serial.print(SENSOR_TYPE);
  Serial.print(" ");
  Serial.print(pressure, 3);
  Serial.print(" ");
  Serial.println(UNIT_OF_MEASURE);
  Serial.print("Raw Pressure Value: ");
  Serial.println(pressRaw);
  Serial.print("TEMPERATURE: ");
  Serial.print(temperature);
  Serial.println("*C");
  Serial.print("Raw Temperature Value: ");
  Serial.println(tempRaw);
  Serial.println();
  Serial.println();
  delay(TIME_BETWEEN_READINGS);
}

// Below is the math I needed to get the actual reading from the pressure sensor.I MODIFIED THE "MAP" function to accept and return floating point numerals.
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

