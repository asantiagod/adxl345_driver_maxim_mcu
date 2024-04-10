/**
 * @file ADXL345.C
 * @author: Santiago Duque
 */

#include "spi.h"
#include "adxl345_driver.h"

#define SPI 	MXC_SPI0
#define SPI_IRQ		SPI0_IRQn
#define SPI_SPEED	3000000
#define BUFFER_SIZE	10

/**
 * Register map
 */
enum
{
	DEVID = 0x00,
	OFSX = 0x1E,
	OFSY = 0x1F,
	OFSZ = 0x20,
	BW_RATE = 0x2C,
	POWER_CTL = 0x2D,
	DATA_FORMAT = 0x31,
	DATAX0 = 0x32, // LSB for the
	FIFO_CTL = 0x38,
	FIFO_STAT = 0x39
};

/**
 * Some bit masks
 */
enum
{
	RESOLUTION_MASK = FULL_RES, RANGE_MASK = RANGE_PM_16g,
};

/**
 * Types to handle the  behavior of the
 * SS pin after some SPI request
 */
typedef enum
{
	ASSERT = 0, ///< SS pin will be asserted after request
	DEASSERT = 1	///< SS pin will be deasserted after request
} ss_deassert_t;

/* Variable to process the SPI requests*/
static mxc_spi_req_t request;
static mxc_spi_req_t *pRequest = &request;
static uint8_t txBuffer[BUFFER_SIZE];

static union
{
	uint8_t beginning[1];
	struct
	{
		uint8_t padding; // it's necessary an additional byte to pump the
						 // address byte and perform only one transaction
		uint8_t data[BUFFER_SIZE];
	};
} rxBuffer;

// This variable will be update when the SPI
// requests finishes
static int spiReqErr;
static uint8_t dataFormat; //Value saved into Data Format register of the ADXL345

/**
 * @brief Initializes a request structure
 * @param request	Pointer to request structure
 * @param txLen		Number of bytes to send
 * @param rxLen		Number of bytes to read
 * @param deassert	handles the  behavior of the
 * 					SS pin after some SPI request
 * @retval E_NO_ERROR if the initialization was performed
 * @retval E_BAD_PARAM if there are any invalid parameters
 */
static int performRequest(mxc_spi_req_t *request, uint32_t txLen,
		uint32_t rxLen, ss_deassert_t deassert);

/**
 * @brief Writes a single byte into the ADXL345's registers.
 * @param address	Register address.
 * @param data		Data to be written.
 * @return Success or Error. See \ref MXC_Error_Codes
 */
static int writeByte(uint8_t address, uint8_t data);

/**
 * @brief Read a single byte of the some ADXL345 register.
 * @param address	Register address.
 * @return	Read value
 */
static uint8_t readByte(uint8_t address);

/**
 * @brief Read multiple bytes from the accelerometer
 * @param address	Address of the first byte.
 * @param nBytes	Number of bytes to read.
 * @return Success or Error. See \ref MXC_Error_Codes
 */
static int readMultipleBytes(uint8_t address, size_t nBytes);

/**
 * This function will be called when a SPI request completes
 */
static void spiCallback(mxc_spi_req_t *req, int error);

/**
 * @brief Setup the acceleration offset for someone axis
 * @param reg		Register address to save the offset
 * @param offset	Offset value
 * @return
 */
static int setAxisOffset(uint8_t reg, double offset);

/**
 * @brief Returns the current offset related with someone axis
 * @param reg 	Register address
 * @return Offset value
 */
static double getAxisOffset(uint8_t reg);

/**
 * @brief Converts the ADC data to g-units
 * @param adcValue	Value of the ADC
 * @return g-units
 */
static double adc2g(uint16_t adcValue);

/**
 * @brief Funtion to convert a n-bits number represented in two's complement
 *        to double;
 * @param number    Number to convert
 * @param nBits     number of bits of the integer representation
 * @return number represented in a double format
 */
static double doubleFromTwoComplement(uint16_t number, size_t nBits);

int adxl345_init()
{
	if (MXC_SPI_Init(SPI, 1, 0, 1, 0, SPI_SPEED) != E_NO_ERROR)
		return E_UNINITIALIZED;
	if (MXC_SPI_SetDataSize(SPI, 8) != E_NO_ERROR)
		return E_UNINITIALIZED;
	if (MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD) != E_NO_ERROR)
		return E_UNINITIALIZED;
	if (MXC_SPI_SetMode(SPI, SPI_MODE_3) != E_NO_ERROR)
		return E_UNINITIALIZED;

	return adxl345_isConnected();
}

int adxl345_isConnected()
{
	if (readByte(DEVID) == 0b11100101)
		return E_SUCCESS;
	else
		return E_NO_DEVICE;
}

int adxl345_setDataFormat(uint8_t settings)
{
	dataFormat = settings;
	return writeByte(DATA_FORMAT, settings);
}

uint8_t adxl345_getDataFormat()
{
	return readByte(DATA_FORMAT);
}

int adxl345_setFifoCtl(uint8_t settings)
{
	return writeByte(FIFO_CTL, settings);
}

uint8_t adxl345_getFifoCtl()
{
	return readByte(FIFO_CTL);
}

int adxl345_setOffsetX(double offset)
{
	return setAxisOffset(OFSX, offset);
}

double adxl345_getOffsetX()
{
	return getAxisOffset(OFSX);
}

int adxl345_setOffsetY(double offset)
{
	return setAxisOffset(OFSY, offset);
}

double adxl345_getOffsetY()
{
	return getAxisOffset(OFSY);
}

int adxl345_setOffsetZ(double offset)
{
	return setAxisOffset(OFSZ, offset);
}

double adxl345_getOffsetZ()
{
	return getAxisOffset(OFSZ);
}

int adxl345_startMeasure()
{
	return writeByte(POWER_CTL, 0x08);
}

int adxl345_stopMeasure()
{
	return writeByte(POWER_CTL, 0x00);
}

uint8_t adxl345_nFifoElements()
{
	return 0x3F & readByte(FIFO_STAT);
}

int adxl345_setBwRate(uint8_t outputDataRate)
{
	return writeByte(BW_RATE, 0x0F & outputDataRate);;
}

uint8_t adxl345_getBwRate()
{
	return readByte(BW_RATE);
}

acc_data_t adxl345_readData()
{
	const uint8_t N_REG = 8;
	uint8_t *data = rxBuffer.data;
	readMultipleBytes(DATAX0, N_REG);

	acc_data_t r =
	{ .xa = adc2g(data[1] << 8 | data[0]), .ya = adc2g(data[3] << 8 | data[2]),
			.za = adc2g(data[5] << 8 | data[4]), .remaining = 0x3F & data[7] };
	return r;
}

int performRequest(mxc_spi_req_t *request, uint32_t txLen, uint32_t rxLen, ss_deassert_t deassert)
{
	if (txLen + rxLen > BUFFER_SIZE)
		return E_BAD_PARAM;

	/* Number of bytes needed to complete the transfer*/
	uint32_t nBytesTransfer = txLen + rxLen;

	request->spi = SPI;
	request->txData = txBuffer;
	request->rxData = rxBuffer.beginning;
	request->txLen = nBytesTransfer;
	request->rxLen = nBytesTransfer;
	request->ssIdx = 0;
	request->ssDeassert = deassert;
	request->txCnt = 0;
	request->rxCnt = 0;
	request->completeCB = (spi_complete_cb_t) spiCallback;
	return MXC_SPI_MasterTransaction(request);
}

int writeByte(uint8_t address, uint8_t data)
{
	/* Send address to write */
	txBuffer[0] = address;
	txBuffer[1] = data;
	performRequest(pRequest, 2, 0, DEASSERT);
	return spiReqErr;
}

uint8_t readByte(uint8_t address)
{
	txBuffer[0] = address | 0x80; // read flag
	performRequest(pRequest, 1, 1, DEASSERT);
	return rxBuffer.data[0];
}

int readMultipleBytes(uint8_t address, size_t nBytes)
{
	txBuffer[0] = address | 0x80 | 0x40; // read flag + "multiple bytes" flag
	performRequest(pRequest, 1, nBytes, DEASSERT);
	return spiReqErr;
}

void spiCallback(mxc_spi_req_t *req, int error)
{
	spiReqErr = error;
}

int setAxisOffset(uint8_t reg, double offset)
{
	offset /= (double)0.0156; // 15.6 mg/LSB

	if (offset > 127)
		offset = 127;
	else if (offset < -128)
		offset = -128;

	int8_t off = (int8_t) offset;
	return writeByte(reg, off);
}

double getAxisOffset(uint8_t reg)
{
	return (0.0156 * ((int8_t) readByte(reg)));
}

double adc2g(uint16_t adcValue)
{
	uint8_t df = dataFormat; // getDataFormat();
	uint8_t nBits = 10;
	double divisor = 1024;

	if (df & FULL_RES)
	{
		switch (df & RANGE_MASK)
		{
		case RANGE_PM_4g:
			nBits = 11;
			divisor = 2048;
			break;
		case RANGE_PM_8g:
			nBits = 12;
			divisor = 4096;
			break;
		case RANGE_PM_16g:
			nBits = 13;
			divisor = 8192;
			break;
		default:
			nBits = 10;
			divisor = 1024;
			break;
		}
	}

	double multiplier;
	switch (df & RANGE_MASK)
	{
	case RANGE_PM_2g:
		multiplier = 4 / divisor;
		break;
	case RANGE_PM_4g:
		multiplier = 8 / divisor;
		break;
	case RANGE_PM_8g:
		multiplier = 16 / divisor;
		break;
	case RANGE_PM_16g:
		multiplier = 32 / divisor;
		break;
	}

	return multiplier * doubleFromTwoComplement(adcValue, nBits);
}

double doubleFromTwoComplement(uint16_t number, size_t nBits)
{
	// const uint8_t test = ~0xFE + 1;
	uint16_t bitsMask = 0xFFFF << nBits;
	uint16_t signMask = 1 << (nBits - 1);
	double sign = (number & signMask) ? -1 : 1;

	if (sign < 0)
		number = ~number + 1;
	number = ~bitsMask & number;
	return sign * (double) number;
}

