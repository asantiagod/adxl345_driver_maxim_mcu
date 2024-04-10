/**
 *  @file ADXL345.h
 *  @author Santiago Duque
 */

#ifndef LIB_ADXL345_ADXL345_H_
#define LIB_ADXL345_ADXL345_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stddef.h>
#include "mxc_errors.h"

/**
 *  Structure to handle reading data
 */
typedef struct
{
	double xa;       ///< x-axis acceleration
	double ya;       ///< y-axis acceleration
	double za;       ///< z-axis acceleration
	uint8_t remaining; ///< number of remaining data in FIFO memory
} acc_data_t;

/**
 * Some configuration flags
 */
enum
{
	FULL_RES = 0x08,     ///< use full resolution (4 mg/LSB)
	RANGE_PM_2g = 0x00,  ///< range of +-2g
	RANGE_PM_4g = 0x01,  ///< range of +-4g
	RANGE_PM_8g = 0x02,  ///< range of +-8g
	RANGE_PM_16g = 0x03, ///< range of +-16g

	// FIFO modes
	BYPASS = 0x00, ///< FIFO memory is not used
	FIFO = 0x40,   ///< measure process is stopped if the FIFO is full
	STREAM = 0x80, ///< when FIFO is full, the oldest data is overwritten with newer data.
	TRIGGER = 0xC0,
};

/**
 * Output data rate
 */
enum
{
	D_RATE_3200_Hz = 0b1111,
	D_RATE_1600_Hz = 0b1110,
	D_RATE_800_Hz = 0b1101,
	D_RATE_400_Hz = 0b1100,
	D_RATE_200_Hz = 0b1011,
	D_RATE_100_Hz = 0b1010,
	D_RATE_50_Hz = 0b1001,
	D_RATE_25_Hz = 0b1000,
	D_RATE_DEFAULT = D_RATE_100_Hz,
/*...*/
};

/**
 * @brief Initialize the ADXL345 hardware
 * @return Success or fail. See \ref MXC_Error_Codes
 */
int adxl345_init();

/**
 * @brief Checks communication with the ADXL345
 * @retval E_SUCCESS if the communication works
 * @retval E_NO_DEVICE if the communication fails
 */
int adxl345_isConnected();

/**
 * @param settings
 * @return See \ref MXC_Error_Codes
 */
int adxl345_setDataFormat(uint8_t settings);
uint8_t adxl345_getDataFormat();
int adxl345_setFifoCtl(uint8_t settings);
uint8_t adxl345_getFifoCtl();
int adxl345_setOffsetX(double offset);
double adxl345_getOffsetX();
int adxl345_setOffsetY(double offset);
double adxl345_getOffsetY();
int adxl345_setOffsetZ(double offset);
double adxl345_getOffsetZ();

int adxl345_startMeasure();
int adxl345_stopMeasure();
uint8_t adxl345_nFifoElements();
int adxl345_setBwRate(uint8_t outputDataRate);
uint8_t adxl345_getBwRate();
acc_data_t adxl345_readData();


#ifdef __cplusplus
}
#endif

#endif /* LIB_ADXL345_ADXL345_H_ */
