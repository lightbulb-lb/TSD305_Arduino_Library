/**
 * Light Bulb, (c)CopyRight 26-May-2020
 * Contact mailID : lightbulb.ei@gmail.com
 *
 */

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

enum tsd305_status {
	tsd305_status_ok,
	tsd305_status_no_i2c_acknowledge,
	tsd305_status_i2c_transfer_error,
	tsd305_status_busy,
	tsd305_status_memory_error,
	tsd305_status_out_of_range
};

class tsd305 {
public:
  // Functions
	tsd305();

  /**
  *brief perform initital configuration, Has to be called once
  */
  void begin();
	/**
	* \brief Check Whether TSD305 device si connected
	*
	*\return bool : status of TSD305
		     - true : Device present
		     - false: Device is not acknowledging I2C Address
	*/
	bool is_connected(void);
	/**
	* \bried Reads the temperature and pressure ADC Value and Compute the
	*Compensated values.
	*
	* \param[out] float* : Celsius Degree temperature value
	* \parm[out] float* : mbar pressure value
	*
	* \return tsd305_status_ok : I2C transfer completed succesully
	*       - tsd305_status_i2c_transfer_error : Problem with i2c transfer
    *        - tsd305_status_no_i2c_acknowledge : I2C did not acknowledge
    *        - tsd305_status_busy : Sensor is busy
    *        - tsd305_status_memory_error : Sensor EEPROM memory error
    */
  enum tsd305_status read_temperature_and_object_temperature(float *temperature, float *object_temperature);

private:
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
	enum tsd305_status read_eeprom_coeff(uint8_t address, uint16_t *coeff);
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
	enum tsd305_status read_eeprom_float(uint8_t address, float *value);
	/**
   * \brief Triggers conversion and read ADC value
   *
   * \param[in] uint8_t : Command used for conversion (will determine
   * Temperature vs Pressure and osr)
   * \param[out] uint32_t* : ADC value.
   *
   * \return tsd305_status : status of TSD305
   *       - tsd305_status_ok : I2C transfer completed successfully
   *       - tsd305_status_i2c_transfer_error : Problem with i2c transfer
   *       - tsd305_status_no_i2c_acknowledge : I2C did not acknowledge
   *       - tsd305_status_busy : Sensor is busy
   *       - tsd305_status_memory_error : Sensor EEPROM memory error
   */
  enum tsd305_status conversion_and_read_adcs(uint32_t *adc_object, uint32_t *adc_sensor);
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
	enum tsd305_status read_eeprom(void);
   /**
   * \brief Retrieve data from 2D look up table in program memory
   *
   * \return data at given coordinates
   */
	bool tsd305_coeff_read;
	struct {
		uint16_t lot_number;
		uint16_t serial_number;

    int16_t min_sensor_temperature;
    int16_t max_sensor_temperature;
    int16_t min_object_temperature;
    int16_t max_object_temperature;
	} eeprom_coeff;

  struct 
  {
    float temperature_coefficient;
    float reference_temperature; 

    float k4_compansation;
    float k3_compansation;
    float k2_compansation;
    float k1_compansation;
    float k0_compansation;
    float k4_object;
    float k3_object;
    float k2_object;
    float k1_object;
    float k0_object;    
  } eeprom_value;
};