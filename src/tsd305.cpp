/**
 * Light Bulb, (c)CopyRight 26-May-2020
 * Contact mailID : lightbulb.ei@gmail.com
 *
 */

#include "tsd305.h"
#include <Wire.h>

//TSD305 device address
#define TSD305_ADDR 0x1e  //0b0011110
//TSD305 device commands
#define TSD305_CONVERT_ADC_COMMAND 0xaf

#define TSD305_STATUS_BUSY_MASK 0x20   							     //Sensor busy
#define TSD305_STATUS_MEMORY_ERROR_MASK 0x04						 //Memory Integrity check failed

#define TSD305_CONVERSION_TIME 100

#define ASSERT_STATUS(x)                                                       \
  status = x;                                                                  \
  if (status != tsd305_status_ok)                                              \
    return status;

enum status_code {
	STATUS_OK = 0x00,
	STATUS_ERR_OVERFLOW = 0x01,
	STATUS_ERR_TIMEOUT = 0x02,
};

tsd305::tsd305(void){
	tsd305_coeff_read = false;
}
/**
 * \brief perfom initial configuration, has to be called once
 */
void tsd305::begin(void) {
	Wire.begin();
}

/**
 * \brief Check wether TSD305 device is connected
 *
 * \return bool: status of TSD305
          - true: DEvice is present
          -false: deivce is not acknoledging I2C address
 */
bool tsd305::is_connected(void) {
	Wire.beginTransmission((uint8_t)TSD305_ADDR);
	return (Wire.endTransmission() == 0);
}

/**
 * \brief Reads the tsd305 EEPROM coefficient stored at address provided.
 *
 * \param[in] uint8_t : Address of coefficient in EEPROM
 * \param[out] uint16_t* : Value read in EEPROM
 *
 * \return tsd305_status : status of TSD305
 *       - tsd305_status_ok : I2C transfer completed successfully
 *       - tsd305_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsd305_status_busy : Sensor is busy
 *       - tsd305_status_memory_error : Sensor EEPROM memory error
 */

 enum tsd305_status tsd305::read_EE_coeff(uint8_t address, uint16_t *coeff) {
 	enum tsd305_status status;
 	enum status_code i2c_status;
 	uint8_t buffer[3];
 	uint8_t status_byte = 0;
 	uint8_t i;
    /* Read data */
 	Wire.beginTransmission((uint8_t)TSD305_ADDR);
 	Wire.write(address);
    //Send the conversion command
 	Wire.endTransmission();
 	delay(1);

 	Wire.requestFrom((uint8_t)TSD305_ADDR, 3U);
 	for (i = 0; i < 3; i++)
 		buffer[i] = Wire.read();

 	if (i2c_status == STATUS_ERR_OVERFLOW)
 		return tsd305_status_no_i2c_acknowledge;
 	if (i2c_status != STATUS_OK)
 		return tsd305_status_i2c_transfer_error;

 	status_byte = buffer[0];
 	if (status_byte & TSD305_STATUS_BUSY_MASK)
 		return tsd305_status_busy;
 	if (status_byte & TSD305_STATUS_MEMORY_ERROR_MASK)
 		return tsd305_status_memory_error;

 	*coeff = (buffer[1] << 8) | buffer[2];

 	return tsd305_status_ok;
 }
 /**
 * \brief Reads the tsd305 EEPROM coefficient stored at address provided.
 *
 * \param[in] uint8_t : Address of coefficient in EEPROM
 * \param[out] float* : IEEE-745 Value read in EEPROM
 *
 * \return tsd305_status : status of TSD305
 *       - tsd305_status_ok : I2C transfer completed successfully
 *       - tsd305_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsd305_status_busy : Sensor is busy
 *       - tsd305_status_memory_error : Sensor EEPROM memory error
 */
 enum tsd305_status tsd305::read_EE_float(uint8_t address, float *value) {
 	enum tsd305_status status;
 	uint16_t h_word, l_word;
 	uint32_t word;

 	ASSERT_STATUS(read_EE_coeff(address, &h_word));
 	ASSERT_STATUS(read_EE_coeff(address + 1, &l_word));

 	word = (uint32_t)h_word << 16 | l_word;
 	*value = *(float *)&word;

 	return tsd305_status_ok;
 }

/* enum tsd305_status tsd305::read_EE_float(uint8_t address, float *value) {
 	enum tsd305_Status status;
 	for ( i = 0; i < 2; i++ )
 		ASSERT_STATUS(read_EE_coeff(address + i, (float *)&MyUnion.iValue[i])); 
 } */
 /**
 * \brief Reads the tsd305 EEPROM coefficients to store them for computation.
 *
 * \return tsd305_status : status of TSD305
 *       - tsd305_status_ok : I2C transfer completed successfully
 *       - tsd305_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsd305_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsd305_status_busy : Sensor is busy
 *       - tsd305_status_memory_error : Sensor EEPROM memory error
 */
 enum tsd305_status tsd305::read_EE(void) {
 	enum tsd305_status status = tsd305_status_ok;

 	ASSERT_STATUS(read_EE_coeff(0x00, &EE_coeff.lot_number));
 	ASSERT_STATUS(read_EE_coeff(0x01, &EE_coeff.serial_number));
 	ASSERT_STATUS(read_EE_coeff(0x1a, (uint16_t *)&EE_coeff.min_sensor_temperature));
 	ASSERT_STATUS(read_EE_coeff(0x1b, (uint16_t *)&EE_coeff.max_sensor_temperature));
 	ASSERT_STATUS(read_EE_coeff(0x1c, (uint16_t *)&EE_coeff.min_object_temperature));
 	ASSERT_STATUS(read_EE_coeff(0x1d, (uint16_t *)&EE_coeff.max_object_temperature));
 	ASSERT_STATUS(read_EE_float(0x1e, (float *)&EE_value.temperature_coefficient));
  ASSERT_STATUS(read_EE_float(0x20, (float *)&EE_value.reference_temperature));
  ASSERT_STATUS(read_EE_float(0x22, (float *)&EE_value.k4_compansation));
  ASSERT_STATUS(read_EE_float(0x24, (float *)&EE_value.k3_compansation));
  ASSERT_STATUS(read_EE_float(0x26, (float *)&EE_value.k2_compansation));
  ASSERT_STATUS(read_EE_float(0x28, (float *)&EE_value.k1_compansation));
  ASSERT_STATUS(read_EE_float(0x2a, (float *)&EE_value.k0_compansation));
  ASSERT_STATUS(read_EE_float(0x2e, (float *)&EE_value.k4_object));
  ASSERT_STATUS(read_EE_float(0x30, (float *)&EE_value.k3_object));
  ASSERT_STATUS(read_EE_float(0x32, (float *)&EE_value.k2_object));
  ASSERT_STATUS(read_EE_float(0x34, (float *)&EE_value.k1_object));
  ASSERT_STATUS(read_EE_float(0x36, (float *)&EE_value.k0_object));

 	tsd305_coeff_read = true;

 	return status;
}
 /**
 * \brief Triggers conversion and read ADC value
 *
 * \param[in] uint8_t : Command used for conversion (will determine Temperature
 * vs Pressure and osr)
 * \param[out] uint32_t* : ADC value.
 *
 * \return tsd305_status : status of TSD305
 *       - tsd305_status_ok : I2C transfer completed successfully
 *       - tsd305_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsd305_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsd305_status_busy : Sensor is busy
 *       - tsd305_status_memory_error : Sensor EEPROM memory error
 */
 enum tsd305_status tsd305::conversion_and_read_adcs(uint32_t *adc_object, uint32_t *adc_sensor) {

 	enum tsd305_status status;
 	uint8_t i2c_status;
 	uint8_t buffer[7];
 	uint8_t status_byte = 0;
 	uint8_t i;

 	/* Read data */
 	Wire.beginTransmission((uint8_t)TSD305_ADDR);
 	Wire.write((uint8_t)TSD305_CONVERT_ADC_COMMAND);  
 	i2c_status = Wire.endTransmission();
 	delay(TSD305_CONVERSION_TIME);

 	Wire.requestFrom((uint8_t)TSD305_ADDR, 7U);
 	for (i = 0; i < 7; i++)
 		buffer[i] = Wire.read();

 	//Send the Conversion command
 	if (i2c_status == STATUS_ERR_OVERFLOW)
 		return tsd305_status_no_i2c_acknowledge;
 	if (i2c_status != STATUS_OK)
 		return tsd305_status_i2c_transfer_error;

 	status_byte = buffer[0];
 	if (status_byte & TSD305_STATUS_BUSY_MASK)			
 		return tsd305_status_busy;
 	if (status_byte & TSD305_STATUS_MEMORY_ERROR_MASK)
 		return tsd305_status_memory_error;

 	*adc_object = ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | ((uint32_t)buffer[3]);
 	*adc_sensor = ((uint32_t)buffer[4] << 16) | ((uint32_t)buffer[5] << 8) | ((uint32_t)buffer[6]);

 	return tsd305_status_ok;
 }	
 /**
 * \brief Reads the temperature and pressure ADC value and compute the
 * compensated values.
 *
 * \param[out] float* : Celsius DegrEE temperature value
 * \param[out] float* : mbar pressure value
 *
 * \return tsd305_status : status of TSD305
 *       - tsd305_status_ok : I2C transfer completed successfully
 *       - tsd305_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsd305_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsd305_status_busy : Sensor is busy
 *       - tsd305_status_memory_error : Sensor EEPROM memory error
 *       - tsd305_status_out_of_range : Sensor is out of range
 */
 enum tsd305_status tsd305::read_temperature_and_object_temperature(float *temperature, float *object_temperature) {

 	enum tsd305_status status = tsd305_status_ok;
 	int32_t adc_object, adc_sensor;
    float tc_correction_factor, temperature_compansation_offset, temperature_compansation_offset_tc;
    float adc_compansation_object, adc_compansation_object_tc;

 	if(tsd305_coeff_read == false)
 		ASSERT_STATUS(read_EE());

 	ASSERT_STATUS(conversion_and_read_adcs((uint32_t *)&adc_object, (uint32_t *)&adc_sensor));
//Calculation as per DataShEEt for Sensor Temprature
 	*temperature = (float)adc_sensor / 16777216.0f * ((float)EE_coeff.max_sensor_temperature - (float)EE_coeff.min_sensor_temperature) + (float)EE_coeff.min_sensor_temperature;
//Calculation as per DatashEEt for Object Temperature
    //TC_CORRECTION_FACTOR
    tc_correction_factor = 1 + ((*temperature - (float)EE_value.reference_temperature) * (float)EE_value.temperature_coefficient);
 	//TEMPERATURE_COMPENSATION
    temperature_compansation_offset =  ((float)(EE_value.k4_compansation) * (*temperature) * (*temperature) * (*temperature) * (*temperature)) + 
                                       ((float)(EE_value.k3_compansation) * (*temperature) * (*temperature) * (*temperature)) +
                                       ((float)(EE_value.k2_compansation) * (*temperature) * (*temperature)) +
                                       ((float)(EE_value.k1_compansation) * (*temperature)) +
                                       ((float)(EE_value.k0_compansation));
    temperature_compansation_offset_tc = ((float)(temperature_compansation_offset) * (float)tc_correction_factor);
    //Object Temperature
    adc_compansation_object = temperature_compansation_offset_tc + (float)adc_object - 8388608.0f;      //Here, Divide by 0.9 the adc_object
    adc_compansation_object_tc = adc_compansation_object / tc_correction_factor;

    *object_temperature = ((float)EE_value.k4_object * ((float)adc_compansation_object_tc) * ((float)adc_compansation_object_tc) * ((float)adc_compansation_object_tc) * ((float)adc_compansation_object_tc)) +
                          ((float)EE_value.k3_object * ((float)adc_compansation_object_tc) * ((float)adc_compansation_object_tc) * ((float)adc_compansation_object_tc)) +
                          ((float)EE_value.k2_object * ((float)adc_compansation_object_tc) * ((float)adc_compansation_object_tc)) +
                          ((float)EE_value.k1_object * ((float)adc_compansation_object_tc)) +
                          ((float)EE_value.k0_object);
    return status;
}
 







