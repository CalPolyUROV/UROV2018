/***************************************************************************
  This is a library for the BNO055 orientation sensor

  Designed specifically to work with the Adafruit BNO055 Breakout.

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by KTOWN for Adafruit Industries.

  MIT license, all text above must be included in any redistribution
 ***************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <math.h>
#include <limits.h>

#include "Adafruit_BNO055.h"

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_BNO055 class
*/
/**************************************************************************/

//-------------------------------------------------------------------------------------
/** @brief   This constructor creates an I2C driver object.
 *  @param   p_debug_port A serial port, often RS-232, for debugging text
 *                        (default: @c NULL)
 */

Adafruit_BNO055::Adafruit_BNO055(int32_t sensorID, uint8_t address)
{
  _sensorID = sensorID;
  _address = address;
    
    p_serial = p_debug_port;                // Set the debugging serial port pointer
    
    TWBR = I2C_TWBR_VALUE;                  // Set the bit rate for the I2C port
    
    // Create the mutex which will protect the I2C bus from multiple calls
    if ((mutex = xSemaphoreCreateMutex ()) == NULL)
    {
        I2C_DBG ("Error: No I2C mutex" << endl);
    }
    
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Sets up the HW
*/
/**************************************************************************/
bool Adafruit_BNO055::begin(adafruit_bno055_opmode_t mode)
{
  /* Enable I2C */
  Wire.begin();

  // BNO055 clock stretches for 500us or more!
#ifdef ESP8266
  Wire.setClockStretchLimit(1000); // Allow for 1000us of clock stretching
#endif

  /* Make sure we have the right device */
  uint8_t id = read8(BNO055_CHIP_ID_ADDR);
  if(id != BNO055_ID)
  {
    delay(1000); // hold on for boot
    id = read8(BNO055_CHIP_ID_ADDR);
    if(id != BNO055_ID) {
      return false;  // still not? ok bail
    }
  }

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);

  /* Reset */
  write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
  while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID)
  {
    delay(10);
  }
  delay(50);

  /* Set to normal power mode */
  write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  delay(10);

  write8(BNO055_PAGE_ID_ADDR, 0);

  /* Set the output units */
  /*
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  write8(BNO055_UNIT_SEL_ADDR, unitsel);
  */

  /* Configure axis mapping (see section 3.4) */
  /*
  write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
  delay(10);
  write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
  delay(10);
  */
  
  write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(mode);
  delay(20);

  return true;
}

/**************************************************************************/
/*!
    @brief  Puts the chip in the specified operating mode
*/
/**************************************************************************/
void Adafruit_BNO055::setMode(adafruit_bno055_opmode_t mode)
{
  _mode = mode;
  write8(BNO055_OPR_MODE_ADDR, _mode);
  delay(30);
}

/**************************************************************************/
/*!
    @brief  Use the external 32.768KHz crystal
*/
/**************************************************************************/
void Adafruit_BNO055::setExtCrystalUse(boolean usextal)
{
  adafruit_bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  write8(BNO055_PAGE_ID_ADDR, 0);
  if (usextal) {
    write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
  } else {
    write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
}


/**************************************************************************/
/*!
    @brief  Gets the latest system status info
*/
/**************************************************************************/
void Adafruit_BNO055::getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
  write8(BNO055_PAGE_ID_ADDR, 0);

  /* System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms */

  if (system_status != 0)
    *system_status    = read8(BNO055_SYS_STAT_ADDR);

  /* Self Test Results (see section )
     --------------------------------
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good! */

  if (self_test_result != 0)
    *self_test_result = read8(BNO055_SELFTEST_RESULT_ADDR);

  /* System Error (see section 4.3.59)
     ---------------------------------
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error */

  if (system_error != 0)
    *system_error     = read8(BNO055_SYS_ERR_ADDR);

  delay(200);
}

/**************************************************************************/
/*!
    @brief  Gets the chip revision numbers
*/
/**************************************************************************/
void Adafruit_BNO055::getRevInfo(adafruit_bno055_rev_info_t* info)
{
  uint8_t a, b;

  memset(info, 0, sizeof(adafruit_bno055_rev_info_t));

  /* Check the accelerometer revision */
  info->accel_rev = read8(BNO055_ACCEL_REV_ID_ADDR);

  /* Check the magnetometer revision */
  info->mag_rev   = read8(BNO055_MAG_REV_ID_ADDR);

  /* Check the gyroscope revision */
  info->gyro_rev  = read8(BNO055_GYRO_REV_ID_ADDR);

  /* Check the SW revision */
  info->bl_rev    = read8(BNO055_BL_REV_ID_ADDR);

  a = read8(BNO055_SW_REV_ID_LSB_ADDR);
  b = read8(BNO055_SW_REV_ID_MSB_ADDR);
  info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}

/**************************************************************************/
/*!
    @brief  Gets current calibration state.  Each value should be a uint8_t
            pointer and it will be set to 0 if not calibrated and 3 if
            fully calibrated.
*/
/**************************************************************************/
void Adafruit_BNO055::getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
  uint8_t calData = read8(BNO055_CALIB_STAT_ADDR);
  if (sys != NULL) {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL) {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL) {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL) {
    *mag = calData & 0x03;
  }
}

/**************************************************************************/
/*!
    @brief  Gets the temperature in degrees celsius
*/
/**************************************************************************/
int8_t Adafruit_BNO055::getTemp(void)
{
  int8_t temp = (int8_t)(read8(BNO055_TEMP_ADDR));
  return temp;
}

/**************************************************************************/
/*!
    @brief  Gets a vector reading from the specified source
*/
/**************************************************************************/
imu::Vector<3> Adafruit_BNO055::getVector(adafruit_vector_type_t vector_type)
{
  imu::Vector<3> xyz;
  uint8_t buffer[6];
  memset (buffer, 0, 6);

  int16_t x, y, z;
  x = y = z = 0;

  /* Read vector data (6 bytes) */
  readLen((adafruit_bno055_reg_t)vector_type, buffer, 6);

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  /* Convert the value to an appropriate range (section 3.6.4) */
  /* and assign the value to the Vector type */
  switch(vector_type)
  {
    case VECTOR_MAGNETOMETER:
      /* 1uT = 16 LSB */
      xyz[0] = ((double)x)/16.0;
      xyz[1] = ((double)y)/16.0;
      xyz[2] = ((double)z)/16.0;
      break;
    case VECTOR_GYROSCOPE:
      /* 1rps = 900 LSB */
      xyz[0] = ((double)x)/900.0;
      xyz[1] = ((double)y)/900.0;
      xyz[2] = ((double)z)/900.0;
      break;
    case VECTOR_EULER:
      /* 1 degree = 16 LSB */
      xyz[0] = ((double)x)/16.0;
      xyz[1] = ((double)y)/16.0;
      xyz[2] = ((double)z)/16.0;
      break;
    case VECTOR_ACCELEROMETER:
    case VECTOR_LINEARACCEL:
    case VECTOR_GRAVITY:
      /* 1m/s^2 = 100 LSB */
      xyz[0] = ((double)x)/100.0;
      xyz[1] = ((double)y)/100.0;
      xyz[2] = ((double)z)/100.0;
      break;
  }

  return xyz;
}

/**************************************************************************/
/*!
    @brief  Gets a quaternion reading from the specified source
*/
/**************************************************************************/
imu::Quaternion Adafruit_BNO055::getQuat(void)
{
  uint8_t buffer[8];
  memset (buffer, 0, 8);

  int16_t x, y, z, w;
  x = y = z = w = 0;

  /* Read quat data (8 bytes) */
  readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
  w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
  x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
  y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
  z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

  /* Assign to Quaternion */
  /* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
     3.6.5.5 Orientation (Quaternion)  */
  const double scale = (1.0 / (1<<14));
  imu::Quaternion quat(scale * w, scale * x, scale * y, scale * z);
  return quat;
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void Adafruit_BNO055::getSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "BNO055", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_ORIENTATION;
  sensor->min_delay   = 0;
  sensor->max_value   = 0.0F;
  sensor->min_value   = 0.0F;
  sensor->resolution  = 0.01F;
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
bool Adafruit_BNO055::getEvent(sensors_event_t *event)
{
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ORIENTATION;
  event->timestamp = millis();

  /* Get a Euler angle sample for orientation */
  imu::Vector<3> euler = getVector(Adafruit_BNO055::VECTOR_EULER);
  event->orientation.x = euler.x();
  event->orientation.y = euler.y();
  event->orientation.z = euler.z();

  return true;
}

/**************************************************************************/
/*!
@brief  Reads the sensor's offset registers into a byte array
*/
/**************************************************************************/
bool Adafruit_BNO055::getSensorOffsets(uint8_t* calibData)
{
    if (isFullyCalibrated())
    {
        adafruit_bno055_opmode_t lastMode = _mode;
        setMode(OPERATION_MODE_CONFIG);

        readLen(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS);

        setMode(lastMode);
        return true;
    }
    return false;
}

/**************************************************************************/
/*!
@brief  Reads the sensor's offset registers into an offset struct
*/
/**************************************************************************/
bool Adafruit_BNO055::getSensorOffsets(adafruit_bno055_offsets_t &offsets_type)
{
    if (isFullyCalibrated())
    {
        adafruit_bno055_opmode_t lastMode = _mode;
        setMode(OPERATION_MODE_CONFIG);
        delay(25);

        offsets_type.accel_offset_x = (read8(ACCEL_OFFSET_X_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_X_LSB_ADDR));
        offsets_type.accel_offset_y = (read8(ACCEL_OFFSET_Y_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_Y_LSB_ADDR));
        offsets_type.accel_offset_z = (read8(ACCEL_OFFSET_Z_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_Z_LSB_ADDR));

        offsets_type.gyro_offset_x = (read8(GYRO_OFFSET_X_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_X_LSB_ADDR));
        offsets_type.gyro_offset_y = (read8(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Y_LSB_ADDR));
        offsets_type.gyro_offset_z = (read8(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Z_LSB_ADDR));

        offsets_type.mag_offset_x = (read8(MAG_OFFSET_X_MSB_ADDR) << 8) | (read8(MAG_OFFSET_X_LSB_ADDR));
        offsets_type.mag_offset_y = (read8(MAG_OFFSET_Y_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Y_LSB_ADDR));
        offsets_type.mag_offset_z = (read8(MAG_OFFSET_Z_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Z_LSB_ADDR));

        offsets_type.accel_radius = (read8(ACCEL_RADIUS_MSB_ADDR) << 8) | (read8(ACCEL_RADIUS_LSB_ADDR));
        offsets_type.mag_radius = (read8(MAG_RADIUS_MSB_ADDR) << 8) | (read8(MAG_RADIUS_LSB_ADDR));

        setMode(lastMode);
        return true;
    }
    return false;
}


/**************************************************************************/
/*!
@brief  Writes an array of calibration values to the sensor's offset registers
*/
/**************************************************************************/
void Adafruit_BNO055::setSensorOffsets(const uint8_t* calibData)
{
    adafruit_bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    delay(25);

    /* A writeLen() would make this much cleaner */
    write8(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
    write8(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
    write8(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
    write8(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
    write8(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
    write8(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

    write8(GYRO_OFFSET_X_LSB_ADDR, calibData[6]);
    write8(GYRO_OFFSET_X_MSB_ADDR, calibData[7]);
    write8(GYRO_OFFSET_Y_LSB_ADDR, calibData[8]);
    write8(GYRO_OFFSET_Y_MSB_ADDR, calibData[9]);
    write8(GYRO_OFFSET_Z_LSB_ADDR, calibData[10]);
    write8(GYRO_OFFSET_Z_MSB_ADDR, calibData[11]);

    write8(MAG_OFFSET_X_LSB_ADDR, calibData[12]);
    write8(MAG_OFFSET_X_MSB_ADDR, calibData[13]);
    write8(MAG_OFFSET_Y_LSB_ADDR, calibData[14]);
    write8(MAG_OFFSET_Y_MSB_ADDR, calibData[15]);
    write8(MAG_OFFSET_Z_LSB_ADDR, calibData[16]);
    write8(MAG_OFFSET_Z_MSB_ADDR, calibData[17]);

    write8(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
    write8(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

    write8(MAG_RADIUS_LSB_ADDR, calibData[20]);
    write8(MAG_RADIUS_MSB_ADDR, calibData[21]);

    setMode(lastMode);
}

/**************************************************************************/
/*!
@brief  Writes to the sensor's offset registers from an offset struct
*/
/**************************************************************************/
void Adafruit_BNO055::setSensorOffsets(const adafruit_bno055_offsets_t &offsets_type)
{
    adafruit_bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    delay(25);

    write8(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
    write8(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
    write8(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
    write8(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
    write8(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
    write8(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

    write8(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
    write8(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
    write8(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
    write8(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
    write8(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
    write8(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

    write8(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
    write8(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
    write8(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
    write8(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
    write8(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
    write8(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

    write8(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
    write8(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

    write8(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
    write8(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

    setMode(lastMode);
}

bool Adafruit_BNO055::isFullyCalibrated(void)
{
    uint8_t system, gyro, accel, mag;
    getCalibration(&system, &gyro, &accel, &mag);
    if (system < 3 || gyro < 3 || accel < 3 || mag < 3)
        return false;
    return true;
}


/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
bool Adafruit_BNO055::write8(adafruit_bno055_reg_t reg, byte value)
{
    write (_address, reg, value)
    /*
  Wire.beginTransmission(_address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
  Wire.endTransmission();
*/
  /* ToDo: Check for error! */
  return true;
}

/**************************************************************************/
/*!
     @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
byte Adafruit_BNO055::read8(adafruit_bno055_reg_t reg )
{
  byte value = 0;

  Wire.beginTransmission(_address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();
  Wire.requestFrom(_address, (byte)1);
  #if ARDUINO >= 100
    value = Wire.read();
  #else
    value = Wire.receive();
  #endif

  return value;
}

/**************************************************************************/
/*!
    @brief  Reads the specified number of bytes over I2C
*/
/**************************************************************************/
bool Adafruit_BNO055::readLen(adafruit_bno055_reg_t reg, byte * buffer, uint8_t len)
{
  Wire.beginTransmission(_address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();
  Wire.requestFrom(_address, (byte)len);

  for (uint8_t i = 0; i < len; i++)
  {
    #if ARDUINO >= 100
      buffer[i] = Wire.read();
    #else
      buffer[i] = Wire.receive();
    #endif
  }

  /* ToDo: Check for errors! */
  return true;
}
//*************************************************************************************
/** @file Adafruit_BNO055.cpp
 *    This file contains a base class for classes that use the I2C (also known as TWI)
 *    interface on an AVR. The terms "I2C" (the two means squared) and "TWI" are
 *    essentially equivalent; Philips has trademarked the former, and Atmel doesn't pay
 *    them a license fee, so Atmel chips that meet exactly the same specification are
 *    not allowed to use the "I2C" name, even though everything works the same.
 *
 *    Note: The terms "master" and "slave" are standard terminology used in the
 *    electronics industry to describe interactions between electronic components only.
 *    The use of such terms in this documentation is made only for the purpose of
 *    usefully documenting electronic hardware and software, and such use must not be
 *    misconstrued as diminishing our revulsion at the socially diseased human behavior
 *    which is described using the same terms, nor implying any insensitivity toward
 *    people from any background who have been affected by such behavior.
 *
 *  Revised:
 *    - 12-24-2012 JRR Original file, as a standalone HMC6352 compass driver
 *    - 12-28-2012 JRR I2C driver split off into a base class for optimal reusability
 *    - 05-03-2015 JRR Added @c ping() and @c scan() methods to check for devices
 *
 *  License:
 *    This file is copyright 2012-2015 by JR Ridgely and released under the Lesser GNU
 *    Public License, version 2. It is intended for educational use only, but its use
 *    is not limited thereto. */
/*    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 *    TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 *    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//*************************************************************************************

#include "FreeRTOS.h"                       // Main header for FreeRTOS
#include "task.h"                           // Needed for the vTaskDelay() function
#include "Adafruit_BNO055.h"                // Header for this class

#define I2C_TWBR_VALUE 0b00010010
//-------------------------------------------------------------------------------------
/** @brief   This constructor creates an I2C driver object.
 *  @param   p_debug_port A serial port, often RS-232, for debugging text
 *                        (default: @c NULL)
 */



//-------------------------------------------------------------------------------------
/** @brief   Cause a start condition on the I2C bus.
 *  @details This method causes a start condition on the I2C bus. In hardware, a start
 *           condition means that the SDA line is dropped while the SCL line stays
 *           high. This gets the attention of all the other devices on the bus so that
 *           they will listen for their addresses.
 *  @return  @c true if there was an error, @c false if the I2C start was successful
 */

bool Adafruit_BNO055::start (void)
{
    // Cause the start condition to happen
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    // TWSR = set TWPS1 and TWPS0 to 0
    
    // Wait for the TWINT bit to indicate that the start condition has been completed
    for (uint8_t tntr = 0; !(TWCR & (1 << TWINT)); tntr++)
    {
        if (tntr > 1000)
        {
            return true;
        }
    }
    
    // Check that the start condition was transmitted OK
    if ((TWSR & 0b11111000) != 0x08)
    {
        return true;
    }
    return false;
}


//-------------------------------------------------------------------------------------
/** @brief   Cause a repeated start condition on the I2C bus.
 *  @details This method causes a repeated start condition on the I2C bus. This is
 *           similar to a regular start condition, except that it occurs during an
 *           already running conversation, and a different return code is expected if
 *           things go as they should.
 *  @return  @c true if there was an error, @c false if the I2C restart was successful
 */

bool Adafruit_BNO055::restart (void)
{
    // Cause the start condition to happen
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    
    // Wait for the TWINT bit to indicate that the start condition has been completed
    for (uint8_t tntr = 0; !(TWCR & (1 << TWINT)); tntr++)
    {
        if (tntr > 1000)
        {
            return true;
        }
    }
    
    // Check that the start condition was transmitted OK
    if ((TWSR & 0b11111000) != 0x10)
    {
        I2C_DBG (PMS ("I2C re-start: 0x") << hex << TWSR << PMS (" not 0x10")
                 << dec << endl);
    }
    return false;
}


//-------------------------------------------------------------------------------------
/** @brief   Send a byte to a device on the I2C bus.
 *  @details This method performs an I2C send to transfer a byte to a remote device.
 *           The expected response code varies depending on what is being sent at what
 *           time; some examples of expected responses are as follows:
 *           - @c 0x18 - When one has sent SLA+W, a slave address for a write command,
 *                       and a good ACK has been received
 *           - @c 0x40 - When one has sent SLA+R, a slave address for a read command,
 *                       and a good ACK has been received
 *           - @c 0x28 - When one has transmitted a data byte and received a good ACK
 *  @param   byte The byte which is being sent to the remote device
 *  @return  @c true if an acknowledge bit was detected, @c false if none was seen
 */

bool Adafruit_BNO055::write_byte (uint8_t byte)
{
    TWDR = byte;
    TWCR = (1 << TWINT) | (1 << TWEN);
    for (uint8_t tntr = 0; !(TWCR & (1 << TWINT)); tntr++)
    {
        if (tntr > 1000)
        {
            I2C_DBG (PMS ("I2C send timeout") << endl);
            return false;
        }
    }
    
    // Check that the byte was transmitted OK by looking at status bits 7-3
    uint8_t status = TWSR & 0b11111000;
    if (status == 0x18 || status == 0x28 || status == 0x40)
    {
        return true;
    }
    else                                    // Hopefully we got 0x20 or 0x30 for valid
    {                                       // NACK; anything else would be an error
        return false;
    }
}


//-------------------------------------------------------------------------------------
/** @brief   Receive a byte from a device on the I2C bus.
 *  @details This method receives a byte from the I2C bus. Other code must have
 *           already run the @c start() command and sent and address byte which got the
 *           other device's attention.
 *  @param   ack @c true if we are to end our data request with ACK, telling the slave
 *               that we want more data after this; false if we end our data request
 *               with NACK, telling the slave that we don't want more data after this
 *  @return  The byte which was received from the remote device or @c 0xFF for an error
 */

uint8_t Adafruit_BNO055::read_byte (bool ack)
{
    uint8_t expected_response;              // Code we expect from the AVR's I2C port
    
    if (ack)  // If we expect more data after this, send an ACK after we get the data
    {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
        expected_response = 0x50;
    }
    else      // We're not going to ask for more data; send a NACK when we're done
    {
        TWCR = (1 << TWINT) | (1 << TWEN);
        expected_response = 0x58;
    }
    
    for (uint8_t tntr = 0; !(TWCR & (1 << TWINT)); tntr++)
    {
        if (tntr > 1000)
        {
            return 0xFF;
        }
    }
    
    // Check that the address thingy was transmitted OK
    if ((TWSR & 0b11111000) != expected_response)
    {
        return 0xFF;
    }
    
    return TWDR;
}


//-------------------------------------------------------------------------------------
/** @brief   Read one byte from a slave device on the I2C bus.
 *  @details This method reads a single byte from the device on the I2C bus at the
 *           given address.
 *  @param   address The I2C address for the device. The address must already have
 *                   been shifted so that it fills the 7 @b most significant bits of
 *                   the byte.
 *  @param   reg The register address within the device from which to read
 *  @return  The byte which was read from the device
 */

uint8_t Adafruit_BNO055::read (uint8_t address, uint8_t reg)
{
    xSemaphoreTake (mutex, portMAX_DELAY);  // Take the mutex or wait for it
    
    start ();                               // Start the discussion
    if (!write_byte (address) || !write_byte (reg))
    {
        I2C_DBG ("<r:0>");
        xSemaphoreGive (mutex);
        return true;
    }
    
    stop ();
    start ();
    // 	restart ();                             // Repeated start condition
    if (!write_byte (address | 0x01))       // Address with read bit set
    {
        I2C_DBG ("<R:d>");
        xSemaphoreGive (mutex);
        return 0xFF;
    }
    uint8_t data = read_byte (false);       // Read a byte, sending a NACK
    stop ();                                // Stop the conversation
    
    xSemaphoreGive (mutex);                 // Return the mutex, as we're done
    return (data);
}


//-------------------------------------------------------------------------------------
/** @brief   Read multiple bytes from a slave device on the I2C bus.
 *  @details This method reads multiple bytes from the device on the I2C bus at the
 *           given address.
 *  @param   address The I2C address for the device. The address should already have
 *                   been shifted so that it fills the 7 @b most significant bits of
 *                   the byte.
 *  @param   reg The register address within the device from which to read
 *  @param   p_buffer A pointer to a buffer in which the received bytes will be stored
 *  @param   count The number of bytes to read from the device
 *  @return  @c true if a problem occurred during reading, @c false if things went OK
 */

bool Adafruit_BNO055::read (uint8_t address, uint8_t reg, uint8_t *p_buffer, uint8_t count)
{
    xSemaphoreTake (mutex, portMAX_DELAY);  // Take the mutex or wait for it
    
    start ();                               // Start the discussion
    
    if (!write_byte (address) || !write_byte (reg))
    {
        I2C_DBG ("<R:0>");
        xSemaphoreGive (mutex);
        return true;
    }
    
    stop ();
    start ();
    // 	restart ();                             // Repeated start condition
    
    if (!write_byte (address | 0x01))       // Address with read bit set
    {
        I2C_DBG ("<R:D>");
        xSemaphoreGive (mutex);
        return true;
    }
    for (uint8_t index = count - 1; index; index--)
    {
        *p_buffer++ = read_byte (true);     // Read bytes
    }
    *p_buffer++ = read_byte (false);        // Last byte requires acknowledgement
    stop ();
    
    xSemaphoreGive (mutex);                 // Return the mutex, as we're done
    return false;
}


//-------------------------------------------------------------------------------------
/** @brief   Write one byte to a slave device on the I2C bus.
 *  @details This method writes a single byte to the device on the I2C bus at the
 *           given address.
 *  @param   address The I2C address for the device. The address should already have
 *                   been shifted so that it fills the 7 @b most significant bits of
 *                   the byte.
 *  @param   reg The register address within the device to which to write
 *  @param   data The byte of data to be written to the device
 *  @return  @c true if there were problems or @c false if everything worked OK
 */

bool Adafruit_BNO055::write (uint8_t address, uint8_t reg, uint8_t data)
{
    xSemaphoreTake (mutex, portMAX_DELAY);  // Take the mutex or wait for it
    
    start ();                               // Start the discussion
    if (!write_byte (address) || !write_byte (reg) || !write_byte (data))
    {
        I2C_DBG ("<w:0>");
        xSemaphoreGive (mutex);
        return true;
    }
    stop ();                                // Stop the conversation
    
    xSemaphoreGive (mutex);                 // Return the mutex, as we're done
    return false;
}


//-------------------------------------------------------------------------------------
/** @brief   Write a bunch of bytes to a slave device on the I2C bus.
 *  @details This method writes a number of bytes to the device on the I2C bus at the
 *           given address.
 *  @param   address The I2C address for the device. The address should already have
 *                   been shifted so that it fills the 7 @b most significant bits of
 *                   the byte.
 *  @param   reg The register address within the device to which to write
 *  @param   p_buf Pointer to a memory address at which is found the bytes of data to
 *                 be written to the device
 *  @param   count The number of bytes to be written from the buffer to the device
 *  @return  @c true if there were problems or @c false if everything worked OK
 */

bool Adafruit_BNO055::write (uint8_t address, uint8_t reg, uint8_t* p_buf, uint8_t count)
{
    xSemaphoreTake (mutex, portMAX_DELAY);  // Take the mutex or wait for it
    
    start ();                               // Start the discussion
    if (!write_byte (address) || !write_byte (reg))
    {
        I2C_DBG ("<W:0>");
        xSemaphoreGive (mutex);
        return true;
    }
    
    for (uint8_t index = 0; index < count; index++)
    {
        if (!write_byte (*p_buf++))
        {
            I2C_DBG ("<W:" << index << '>');
            xSemaphoreGive (mutex);
            return true;
        }
    }
    stop ();
    
    xSemaphoreGive (mutex);                 // Return the mutex, as we're done
    return false;
}


//-------------------------------------------------------------------------------------
/** @brief   Check the status of the SDA line.
 *  @details This method just finds whether the SDA line is currently high or low. This
 *           method may be needed when some sensors may need to wait for the SDA line
 *           to be directly manipulated by the sensor to indicate that data is
 *           available.
 *  @return  @c true if the SDA line is high, @c false if it is low
 */

bool Adafruit_BNO055::check_SDA (void)
{
    if (I2C_PORT_SDA & (1 << I2C_PIN_SDA))
    {
        return true;
    }
    else
    {
        return false;
    }
}


//-------------------------------------------------------------------------------------
/** @brief   Check if a device is located at the given address.
 *  @details This method causes an I2C start, then sends the given address and checks
 *           for an acknowledgement. After that, it just sends a stop condition.
 *  @param   address The I2C address for the device. The address should already have
 *           been shifted so that it fills the 7 @b most significant bits of the byte.
 *  @return  @c true if a device acknowledged the given address, @c false if not
 */

bool Adafruit_BNO055::ping (uint8_t address)
{
    xSemaphoreTake (mutex, portMAX_DELAY);  // Take the mutex or wait for it
    
    start ();
    bool is_someone_there = write_byte (address);
    stop ();
    
    xSemaphoreGive (mutex);
    
    return is_someone_there;
}


//-------------------------------------------------------------------------------------
/** @brief   Scan the I2C bus, pinging each address, and print the results.
 *  @details This handy dandy utility function scans each address on the I2C bus and
 *           prints a display showing the addresses at which devices responded to a 
 *           "ping" with acknowledgement.
 *  @param   p_ser A pointer to a serial device on which the scan results are printed
 */

void Adafruit_BNO055::scan (emstream* p_ser)
{
    *p_ser << PMS ("    0 2 4 6 8 A C E") << hex << endl;
    for (uint8_t row = 0x00; row < 0x10; row++)
    {
        *p_ser << (uint8_t)row << '0';
        for (uint8_t col = 0; col < 0x10; col += 2)
        {
            p_ser->putchar (' ');
            if (ping ((row << 4) | col))
            {
                p_ser->putchar ('@');
            }
            else
            {
                p_ser->putchar ('-');
            }
        }
        *p_ser << endl;
    }
    *p_ser << dec;
}


