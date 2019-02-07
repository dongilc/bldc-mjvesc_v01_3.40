/*
 * app_vescuino_mpu9250.c
 *
 *  Created on: 2018. 6. 22.
 *      Author: cdi
 *      TODO - AK8963 doesn't functional at spi mode
 */

//
#include "app_vescuino_mpu9250.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

//#define USE_SW_SPI

// private variable
static IMU_DATA imu_data;
static MPU9250_VARIABLE mpu, mpu2;
MPU9250_VARIABLE *mpu_p = NULL;
int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
static uint32_t loop_cnt_imu;
static bool debug_print_flag = 0;
static uint8_t debug_print_mode = 0;
static uint16_t valid_cnt;
static uint8_t txbuf[20];
static uint8_t rxbuf[20];

// Threads
static THD_FUNCTION(mpu9250_thread, arg);
static THD_WORKING_AREA(mpu9250_thread_wa, 1024);

// I2C Functions
static msg_t status = MSG_OK;
static i2cflags_t errors = 0;
systime_t tmo = TIME_INFINITE;
//SYSTIME_T TMO = MS2ST(50);	// 170920
//SYSTIME_T TMO = MS2ST(1);

//
void app_mpu9250_start(void)
{
	chThdCreateStatic(mpu9250_thread_wa, sizeof(mpu9250_thread_wa), NORMALPRIO, mpu9250_thread, NULL);
}

// --------------- I2C Functions ---------------
/*
 * i2cMasterTransmitTimeout(I2CDriver *i2cp, i2caddr_t addr,
	                        const uint8_t *txbuf, size_t txbytes,
	                        uint8_t *rxbuf, size_t rxbytes,
	                        systime_t timeout);
 */
void status_i2c(msg_t st, uint8_t cmd_index)
{
	if (st != MSG_OK) {
		if (st == MSG_RESET) {
			errors = i2cGetErrors(&HW_I2C_DEV);
			debug_printf("\r\ni2c error - %d, at i2c cmd - %d", errors, cmd_index);
		}
		if (st == MSG_TIMEOUT) {
			debug_printf("\r\nTimeout occurred");
		}
	}
}

void writeByte_i2c(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
	uint8_t txbuf[2];

	txbuf[0] = reg;
	txbuf[1] = data;

	i2cAcquireBus(&HW_I2C_DEV);
	status = i2cMasterTransmitTimeout(&HW_I2C_DEV, dev_addr, txbuf, 2, 0, 0, tmo);
	i2cReleaseBus(&HW_I2C_DEV);
	status_i2c(status, 0);
}

uint8_t readByte_i2c(uint8_t dev_addr, uint8_t reg)
{
	uint8_t rx_reg = 0;

  	i2cAcquireBus(&HW_I2C_DEV);
  	status = i2cMasterTransmitTimeout(&HW_I2C_DEV, dev_addr, &reg, 1, &rx_reg, 1, tmo);
  	i2cReleaseBus(&HW_I2C_DEV);
  	status_i2c(status, 1);
  	return rx_reg;
}

void readBytes_i2c(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	uint8_t rxbuf[14];

	i2cAcquireBus(&HW_I2C_DEV);
	i2cMasterTransmitTimeout(&HW_I2C_DEV, address, &subAddress, 1, rxbuf, count, tmo);
	i2cReleaseBus(&HW_I2C_DEV);
	status_i2c(status, 2);

	for (uint8_t i = 0; i < count; i++)
	{
		dest[i] = rxbuf[i];
	}
}
// --------------- END of I2C Functions ---------------

// --------------- SPI SW Functions ---------------
#ifdef USE_SW_SPI
// TODO : spi-sw mpu9250 calibration malfunctional, 180620
static bool mpu_init_done = false;
static stm32_gpio_t *mpu_port_csn = MPU_PORT_CSN;
static int mpu_pin_csn = MPU_PIN_CSN;
static stm32_gpio_t *mpu_port_sck = MPU_PORT_SCK;
static int mpu_pin_sck = MPU_PIN_SCK;
static stm32_gpio_t *mpu_port_mosi = MPU_PORT_MOSI;
static int mpu_pin_mosi = MPU_PIN_MOSI;
static stm32_gpio_t *mpu_port_miso = MPU_PORT_MISO;
static int mpu_pin_miso = MPU_PIN_MISO;

static void mpu_spi_sw_delay(void) {
	for (volatile int i = 0;i < 5;i++) {
		__NOP();
	}
}

void mpu_spi_sw_begin(void) {
	palClearPad(mpu_port_csn, mpu_pin_csn);
	mpu_spi_sw_delay();
}

void mpu_spi_sw_end(void) {
	mpu_spi_sw_delay();
	palSetPad(mpu_port_csn, mpu_pin_csn);
}

void mpu_spi_sw_init(void) {
	if (!mpu_init_done) {
		palSetPadMode(mpu_port_miso, mpu_pin_miso, PAL_MODE_INPUT);
		palSetPadMode(mpu_port_csn, mpu_pin_csn, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
		palSetPadMode(mpu_port_sck, mpu_pin_sck, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
		palSetPadMode(mpu_port_mosi, mpu_pin_mosi, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

		palSetPad(mpu_port_csn, mpu_pin_csn);
		palClearPad(mpu_port_sck, mpu_pin_sck);
		mpu_init_done = true;
	}
}

void mpu_spi_sw_stop(void) {
	palSetPadMode(mpu_port_miso, mpu_pin_miso, PAL_MODE_INPUT);
	palSetPadMode(mpu_port_csn, mpu_pin_csn, PAL_MODE_INPUT);
	palSetPadMode(mpu_port_sck, mpu_pin_sck, PAL_MODE_INPUT);
	palSetPadMode(mpu_port_mosi, mpu_pin_mosi, PAL_MODE_INPUT);
	mpu_init_done = false;
}

void mpu_spi_sw_transfer(uint8_t *in_buf, const uint8_t *out_buf, int length) {
	palClearPad(mpu_port_sck, mpu_pin_sck);
	mpu_spi_sw_delay();

	for (int i = 0;i < length;i++) {
		unsigned char send = out_buf ? out_buf[i] : 0;
		unsigned char recieve = 0;

		for (int bit=0;bit < 8;bit++) {
			palWritePad(mpu_port_mosi, mpu_pin_mosi, send >> 7);
			send <<= 1;

			mpu_spi_sw_delay();

			int r1, r2, r3;
			r1 = palReadPad(mpu_port_miso, mpu_pin_miso);
			__NOP();
			r2 = palReadPad(mpu_port_miso, mpu_pin_miso);
			__NOP();
			r3 = palReadPad(mpu_port_miso, mpu_pin_miso);

			recieve <<= 1;
			if (utils_middle_of_3_int(r1, r2, r3)) {
				recieve |= 1;
			}

			palSetPad(mpu_port_sck, mpu_pin_sck);
			mpu_spi_sw_delay();
			palClearPad(mpu_port_sck, mpu_pin_sck);
		}

		if (in_buf) {
			in_buf[i] = recieve;
		}
	}
}

uint8_t writeByte_spi_sw(uint8_t reg, uint8_t value)
{
	uint8_t rxd;
	mpu_spi_sw_begin();
	mpu_spi_sw_transfer(0, &reg, 1);
	mpu_spi_sw_transfer(&rxd, &value, 1);
	mpu_spi_sw_end();
	return rxd;
}

uint8_t readByte_spi_sw(uint8_t reg)
{
	return writeByte_spi_sw(reg | 0x80, 0x00);
}

void readBytes_spi_sw(uint8_t reg, uint8_t len, uint8_t *dest)
{
	txbuf[0] = reg | 0x80;

	mpu_spi_sw_begin();
	mpu_spi_sw_transfer(0, txbuf, 1);
	mpu_spi_sw_transfer(rxbuf, 0, len);
	mpu_spi_sw_end();

	for (uint8_t i = 0; i < len; i++)
	{
		dest[i] = rxbuf[i];
	}
}
#endif
// --------------- END of SPI SW Functions ---------------

// --------------- SPI HW Functions ---------------
/*
 * SPI1 configuration structure.
 * Speed 656.25kHz, CPHA=0, CPOL=0, 8bits frames, MSb transmitted first.
 */
static const SPIConfig spi1_ch0_u1MHz = {
	SPI_MASTER,
	NULL,
	HW_SPI_PORT_NSS,
	HW_SPI_PIN_NSS,
	SPI_CR1_BR_1 | SPI_CR1_BR_2
};

static const SPIConfig spi1_ch1_u1MHz = {
	SPI_MASTER,
	NULL,
	GPIOE,
	3,
	SPI_CR1_BR_1 | SPI_CR1_BR_2
};

void mpu_spi_hw_init(void) {
	/*
	 * SPI1 I/O pins setup.
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	palSetPadMode(HW_SPI_PORT_SCK, HW_SPI_PIN_SCK, PAL_MODE_ALTERNATE(HW_SPI_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST);       /* SCK.     */
	palSetPadMode(HW_SPI_PORT_MISO, HW_SPI_PIN_MISO, PAL_MODE_ALTERNATE(HW_SPI_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST);       /* MISO.    */
	palSetPadMode(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, PAL_MODE_ALTERNATE(HW_SPI_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST);       /* MOSI.    */
	palSetPadMode(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);    /* nCS1.      */
	palSetPad(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS);
}

void spi_hw_start(uint8_t dev)
{
	// low speed
	if(dev==MPU9250_ADDRESS) 		spiStart(&SPID1, &spi1_ch0_u1MHz);
	else if(dev==MPU9250_ADDRESS+1) spiStart(&SPID1, &spi1_ch1_u1MHz);
}

uint8_t writeByte_spi_hw(uint8_t dev_addr, uint8_t reg, uint8_t value)
{
	spi_hw_start(dev_addr);
	spiSelect(&SPID1);
	spiSend(&SPID1, 1, &reg);
	spiExchange(&SPID1, 1, &value, rxbuf);
	spiUnselect(&SPID1);
	return rxbuf[0];
}

uint8_t readByte_spi_hw(uint8_t dev_addr, uint8_t reg)
{
	return writeByte_spi_hw(dev_addr, reg | 0x80, 0x00);
}

void readBytes_spi_hw(uint8_t dev_addr, uint8_t reg, uint8_t len, uint8_t *dest)
{
	txbuf[0] = reg | 0x80;

	spi_hw_start(dev_addr);
	spiSelect(&SPID1);
	spiSend(&SPID1, 1, txbuf);
	spiExchange(&SPID1, len, 0, rxbuf);
	spiUnselect(&SPID1);

	for (uint8_t i = 0; i < len; i++)
	{
		dest[i] = rxbuf[i];
	}
}
// --------------- END of SPI HW Functions ---------------

// --------------- MPU9250 Functions ---------------
uint8_t find_mpu9250(uint8_t ch)
{
	//uint8_t id = readRegister(MPU9250_ADDRESS+ch, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
	//uint8_t id = writeRegister(MPU9250_ADDRESS+ch, WHO_AM_I_MPU9250 | READ_FLAG, 0x00);
	uint8_t id = readRegister(MPU9250_ADDRESS+ch, WHO_AM_I_MPU9250);
	return id;
}

void find_ak8963(uint8_t ch)
{
	uint8_t d = readRegister(AK8963_ADDRESS+ch, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
	debug_printf("\r\nAK8963, I AM 0x%x, I SHOULD BE 0x48", d);
}

void initAK8963(float * destination, uint8_t ch)
{
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	writeRegister(AK8963_ADDRESS+ch, AK8963_CNTL, 0x00); // Power down magnetometer
	app_wait_sec(0.01);
	writeRegister(AK8963_ADDRESS+ch, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	app_wait_sec(0.01);
	readRegisters(AK8963_ADDRESS+ch, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
	destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
	writeRegister(AK8963_ADDRESS+ch, AK8963_CNTL, 0x00); // Power down magnetometer
	app_wait_sec(0.01);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeRegister(AK8963_ADDRESS+ch, AK8963_CNTL, mpu.Mscale << 4 | mpu.Mmode); // Set magnetometer data resolution and sample ODR
	app_wait_sec(0.01);

	debug_printf("\r\nAK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
}

uint8_t writeRegister(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
	uint8_t outByte = 0;

	if(dev_addr==MPU9250_ADDRESS || dev_addr==AK8963_ADDRESS) {
		if(imu_data.comm_mode[0]==1)	writeByte_i2c(dev_addr, reg, data);
		else {
#ifdef USE_SW_SPI
			outByte = writeByte_spi_sw(reg, data);
#else
			outByte = writeByte_spi_hw(dev_addr, reg, data);
#endif
		}
	}
	else if(dev_addr==MPU9250_ADDRESS+1 || dev_addr==AK8963_ADDRESS+1)
		outByte = writeByte_spi_hw(dev_addr, reg, data);

	return outByte;
}

uint8_t readRegister(uint8_t dev_addr, uint8_t reg)
{
	uint8_t outByte = 0;

	if(dev_addr==MPU9250_ADDRESS || dev_addr==AK8963_ADDRESS) {
		if(imu_data.comm_mode[0]==1) 	outByte = readByte_i2c(dev_addr, reg);
		else {
#ifdef USE_SW_SPI
			outByte = readByte_spi_sw(reg);
#else
			outByte = readByte_spi_hw(dev_addr, reg);
#endif
		}
	}
	else if(dev_addr==MPU9250_ADDRESS+1 || dev_addr==AK8963_ADDRESS+1)
		outByte = readByte_spi_hw(dev_addr, reg);

	return outByte;
}

void readRegisters(uint8_t dev_addr, uint8_t reg, uint8_t count, uint8_t * dest)
{
	if(dev_addr==MPU9250_ADDRESS || dev_addr==AK8963_ADDRESS) {
		if(imu_data.comm_mode[0]==1) 	readBytes_i2c(dev_addr, reg, count, dest);
		else {
#ifdef USE_SW_SPI
			readBytes_spi_sw(reg, count, dest);
#else
			readBytes_spi_hw(dev_addr, reg, count, dest);
#endif
		}
	}
	else if(dev_addr==MPU9250_ADDRESS+1 || dev_addr==AK8963_ADDRESS+1)
		readBytes_spi_hw(dev_addr, reg, count, dest);
}

void mpu9250_getMres(void)
{
	switch (mpu.Mscale)
	{
		// Possible magnetometer scales (and their register bit settings) are:
		// 14 bit resolution (0) and 16 bit resolution (1)
		case MFS_14BITS:
			mpu.mRes = 10.0*4912.0/8190.0; // Proper scale to return milliGauss
			break;
		case MFS_16BITS:
			mpu.mRes = 10.0*4912.0/32760.0; // Proper scale to return milliGauss
			break;
	}
}

void mpu9250_getGres(void)
{
	switch (mpu.Gscale)
	{
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
			// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case GFS_250DPS:
			mpu.gRes = 250.0/32768.0;	// 32768/250 = 131.072
			break;
		case GFS_500DPS:
			mpu.gRes = 500.0/32768.0;	// 32768/500 = 65.535
			break;
		case GFS_1000DPS:
			mpu.gRes = 1000.0/32768.0;	// 32768/1000 = 32.768
			break;
		case GFS_2000DPS:
			mpu.gRes = 2000.0/32768.0;	// 32768/2000 = 16.384
			break;
	}
}

void mpu9250_getAres(void)
{
	switch (mpu.Ascale)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
			// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case AFS_2G:
			mpu.aRes = 2.0/32768.0;		// 32768/2 = 16384
			break;
		case AFS_4G:
			mpu.aRes = 4.0/32768.0;		// 32768/4 = 8192
			break;
		case AFS_8G:
			mpu.aRes = 8.0/32768.0;		// 32768/8 = 4096
			break;
		case AFS_16G:
			mpu.aRes = 16.0/32768.0;	// 32768/16 = 2048
			break;
	}
}

void mpu9250_set_resolution(void)
{
	mpu9250_getAres();
	mpu9250_getGres();
	mpu9250_getMres();

	if(imu_data.use_mode==2)
	{
		mpu2.aRes = mpu.aRes;
		mpu2.gRes = mpu.gRes;
		mpu2.mRes = mpu.mRes;
	}
}

void initMPU9250(uint8_t ch)
{
	// Initialize MPU9250 device
	// wake up device
	writeRegister(MPU9250_ADDRESS+ch, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	app_wait_sec(0.01); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	// get stable time source
	writeRegister(MPU9250_ADDRESS+ch, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	app_wait_sec(0.02);

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	writeRegister(MPU9250_ADDRESS+ch, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeRegister(MPU9250_ADDRESS+ch, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readRegister(MPU9250_ADDRESS+ch, GYRO_CONFIG); // get current GYRO_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x02; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | mpu.Gscale << 3; // Set full scale range for the gyro
	// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	writeRegister(MPU9250_ADDRESS+ch, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

	// Set accelerometer full-scale range configuration
	c = readRegister(MPU9250_ADDRESS+ch, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | mpu.Ascale << 3; // Set full scale range for the accelerometer
	writeRegister(MPU9250_ADDRESS+ch, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readRegister(MPU9250_ADDRESS+ch, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	writeRegister(MPU9250_ADDRESS+ch, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeRegister(MPU9250_ADDRESS+ch, INT_PIN_CFG, 0x22);
	writeRegister(MPU9250_ADDRESS+ch, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	app_wait_sec(0.01);

	debug_printf("\r\nMPU9250-ch:%d initialized for active data mode....", ch); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float *destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	debug_printf("\r\nMPU9250 Self Testing...");
	app_wait_sec(0.1);

	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest_temp[6];
	int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	float factoryTrim[6];
	uint8_t FS = 0;

	writeRegister(MPU9250_ADDRESS, SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
	writeRegister(MPU9250_ADDRESS, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeRegister(MPU9250_ADDRESS, GYRO_CONFIG, FS<<3); // Set full scale range for the gyro to 250 dps
	writeRegister(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeRegister(MPU9250_ADDRESS, ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g

	for( int ii = 0; ii < 200; ii++)
	{
		// get average current values of gyro and acclerometer
		readRegisters(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		readRegisters(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	for (int ii =0; ii < 3; ii++)
	{
		// Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	writeRegister(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeRegister(MPU9250_ADDRESS, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	app_wait_sec(0.025); // Delay a while to let the device stabilize

	for( int ii = 0; ii < 200; ii++)
	{
		// get average self-test values of gyro and acclerometer
		readRegisters(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		readRegisters(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	for (int ii =0; ii < 3; ii++)
	{
		// Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	writeRegister(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
	writeRegister(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
	app_wait_sec(0.025); // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest_temp[0] = readRegister(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest_temp[1] = readRegister(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest_temp[2] = readRegister(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest_temp[3] = readRegister(MPU9250_ADDRESS, SELF_TEST_X_GYRO); // X-axis gyro self-test results
	selfTest_temp[4] = readRegister(MPU9250_ADDRESS, SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
	selfTest_temp[5] = readRegister(MPU9250_ADDRESS, SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620/1<<FS)*(powf( 1.01 , ((float)selfTest_temp[0] - 1.0) )); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(powf( 1.01 , ((float)selfTest_temp[1] - 1.0) )); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(powf( 1.01 , ((float)selfTest_temp[2] - 1.0) )); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620/1<<FS)*(powf( 1.01 , ((float)selfTest_temp[3] - 1.0) )); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(powf( 1.01 , ((float)selfTest_temp[4] - 1.0) )); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(powf( 1.01 , ((float)selfTest_temp[5] - 1.0) )); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++)
	{
		destination[i] = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.; // Report percent differences
		destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
	}

	debug_printf("\r\nSelf Test Done.");
	app_wait_sec(1);
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float *dest1, float *dest2, bool gyroBiasStore, bool accelBiasStore, uint8_t ch)
{
	debug_printf("\r\nMPU9250 Accelerometer and Gyro Calibrating...");
	app_wait_sec(0.01);

	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device, reset all registers, clear gyro and accelerometer bias registers
	writeRegister(MPU9250_ADDRESS+ch, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	app_wait_sec(0.01);

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	writeRegister(MPU9250_ADDRESS+ch, PWR_MGMT_1, 0x01);
	writeRegister(MPU9250_ADDRESS+ch, PWR_MGMT_2, 0x00);
	app_wait_sec(0.02);

	// Configure device for bias calculation
	writeRegister(MPU9250_ADDRESS+ch, INT_ENABLE, 0x00);   // Disable all interrupts
	writeRegister(MPU9250_ADDRESS+ch, FIFO_EN, 0x00);      // Disable FIFO
	writeRegister(MPU9250_ADDRESS+ch, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeRegister(MPU9250_ADDRESS+ch, I2C_MST_CTRL, 0x00); // Disable I2C master
	writeRegister(MPU9250_ADDRESS+ch, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeRegister(MPU9250_ADDRESS+ch, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	app_wait_sec(0.015);

	// Configure MPU9250 gyro and accelerometer for bias calculation
	writeRegister(MPU9250_ADDRESS+ch, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeRegister(MPU9250_ADDRESS+ch, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeRegister(MPU9250_ADDRESS+ch, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeRegister(MPU9250_ADDRESS+ch, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	// calibration is excuted at maximum sensitivity
	uint16_t  gyrosensitivity = 131.072;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeRegister(MPU9250_ADDRESS+ch, USER_CTRL, 0x40);   // Enable FIFO
	writeRegister(MPU9250_ADDRESS+ch, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
	app_wait_sec(0.04); // accumulate 40 samples in 80 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeRegister(MPU9250_ADDRESS+ch, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readRegisters(MPU9250_ADDRESS+ch, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++)
	{
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		readRegisters(MPU9250_ADDRESS+ch, FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	/// Push gyro biases to hardware registers
	if(gyroBiasStore == true)
	{
		writeRegister(MPU9250_ADDRESS+ch, XG_OFFSET_H, data[0]);
		writeRegister(MPU9250_ADDRESS+ch, XG_OFFSET_L, data[1]);
		writeRegister(MPU9250_ADDRESS+ch, YG_OFFSET_H, data[2]);
		writeRegister(MPU9250_ADDRESS+ch, YG_OFFSET_L, data[3]);
		writeRegister(MPU9250_ADDRESS+ch, ZG_OFFSET_H, data[4]);
		writeRegister(MPU9250_ADDRESS+ch, ZG_OFFSET_L, data[5]);

		debug_printf("\r\ngyro Bias stored to mpu9250");
		app_wait_sec(0.01);
	}

	// Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	// store gyro bias to eeprom
	app_mpu9250_set_imu_gyro_bias(dest1[0], dest1[1], dest1[2], ch);

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	readRegisters(MPU9250_ADDRESS+ch, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	readRegisters(MPU9250_ADDRESS+ch, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	readRegisters(MPU9250_ADDRESS+ch, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	for(ii = 0; ii < 3; ii++) {
		if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	if(accelBiasStore == true)
	{
		writeRegister(MPU9250_ADDRESS+ch, XA_OFFSET_H, data[0]);
		writeRegister(MPU9250_ADDRESS+ch, XA_OFFSET_L, data[1]);
		writeRegister(MPU9250_ADDRESS+ch, YA_OFFSET_H, data[2]);
		writeRegister(MPU9250_ADDRESS+ch, YA_OFFSET_L, data[3]);
		writeRegister(MPU9250_ADDRESS+ch, ZA_OFFSET_H, data[4]);
		writeRegister(MPU9250_ADDRESS+ch, ZA_OFFSET_L, data[5]);

		debug_printf("\r\naccel Bias stored");
		app_wait_sec(0.01);
	}

	// Output scaled accelerometer biases for manual subtraction in the main program
	dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
	dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;

	// store acc bias to eeprom
	app_mpu9250_set_imu_acc_bias(dest2[0], dest2[1], dest2[2], ch);

	debug_printf("\r\nCalibration done.");
	app_wait_sec(0.01);
}

void magcalMPU9250(float *dest1, float *dest2, uint8_t ch)
{
	if(ch==0)		mpu_p = &mpu;
	else if(ch==1) 	mpu_p = &mpu2;

	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

	debug_printf("\r\nMag Calibration - Get Ready");
	app_wait_sec(2);
	debug_printf("\r\nWave device in a figure eight until done for 15sec!");

	// shoot for ~fifteen seconds of mag data
	if(mpu_p->Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
	if(mpu_p->Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
	for(ii = 0; ii < sample_count; ii++) {
		readMagData(mag_temp, ch);  // Read the mag data
		for (int jj = 0; jj < 3; jj++) {
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		if(mpu_p->Mmode == 0x02) app_wait_sec(0.135);  // at 8 Hz ODR, new mag data is available every 125 ms
		if(mpu_p->Mmode == 0x06) app_wait_sec(0.0125);  // at 100 Hz ODR, new mag data is available every 10 ms
	}

	// Get hard iron correction
	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	dest1[0] = (float) mag_bias[0]*mpu_p->mRes*mpu_p->magCalibration[0];  // save mag biases in G for main program
	dest1[1] = (float) mag_bias[1]*mpu_p->mRes*mpu_p->magCalibration[1];
	dest1[2] = (float) mag_bias[2]*mpu_p->mRes*mpu_p->magCalibration[2];

	// Get soft iron correction estimate
	mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	dest2[0] = avg_rad/((float)mag_scale[0]);
	dest2[1] = avg_rad/((float)mag_scale[1]);
	dest2[2] = avg_rad/((float)mag_scale[2]);

	debug_printf("\r\nMag Calibration done!");
	app_wait_sec(1);
}

//void mpu9250_get_mag_bias_scale(float *magBias, float *magScale, uint8_t ch)
//{
//	debug_printf("\r\nLoad Mag. Bias and Scale...");
//
//	for(int i=0; i<3; i++)	{
//		magBias[i] = appconf->app_custom.custom_imu_mag_bias[ch][i];
//		magScale[i] = appconf->app_custom.custom_imu_mag_scale[ch][i];
//	}
//}

//void mpu9250_set_mag_bias_scale(float *magBias, float *magScale, uint8_t ch)
//{
//	debug_printf("\r\nSet Mag. Bias and Scale...");
//
//	for(int i=0; i<3; i++)	{
//		appconf->app_custom.custom_imu_mag_bias[ch][i] = magBias[i];
//		appconf->app_custom.custom_imu_mag_scale[ch][i] = magScale[i];
//	}
//
//	debug_printf("\r\nAK8963 mag biases (mG) = %f, %f, %f", (double)magBias[0], (double)magBias[1], (double)magBias[2]);
//	debug_printf("\r\nAK8963 mag scale (mG) = %f, %f, %f", (double)magScale[0], (double)magScale[1], (double)magScale[2]);
//
//	conf_general_store_app_configuration(appconf);
//
//	debug_printf("\r\nStore Mag. Bias and Scale Done.");
//}

// 1. BASIC: setup gyro&acc Calibration - gyro bias save only
void mpu9250_setup_operation(uint8_t ch)
{
	if(ch==0)		mpu_p = &mpu;
	else if(ch==1) 	mpu_p = &mpu2;

	// get sensor resolutions, only need to do this once
	mpu9250_set_resolution();

	// Calibrate gyro and accelerometers, load biases in bias registers
	calibrateMPU9250(mpu_p->gyroBias, mpu_p->accelBias, true, false, ch);		// only gyro calibrate, start steadily

	// Initialize device for active mode read of acclerometer, gyroscope, and temperature
	initMPU9250(ch);

	// Do not use in MPU Second
	// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
	//find_ak8963(ch);

	// Get magnetometer calibration from AK8963 ROM
	initAK8963(mpu_p->magCalibration, ch);	// load calibration value from mpu

	// Load Mag bias and scale
	//mpu9250_get_mag_bias_scale(mpu_p->magBias, mpu_p->magScale, ch);
}

// 2. FAST: gyro&acc Calibration - gyro and acc bias save
void mpu9250_setup_fast_calibration(uint8_t ch)
{
	if(ch==0)		mpu_p = &mpu;
	else if(ch==1) 	mpu_p = &mpu2;

	// get sensor resolutions, only need to do this once
	mpu9250_set_resolution();

	// Calibrate gyro and accelerometers, load biases in bias registers
	calibrateMPU9250(mpu_p->gyroBias, mpu_p->accelBias, true, true, ch);

	// Initialize device for active mode read of acclerometer, gyroscope, and temperature
	initMPU9250(ch);

	// Do not use in MPU Second
	// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
	//find_ak8963(ch);

	// Get magnetometer calibration from AK8963 ROM
	initAK8963(mpu_p->magCalibration, ch);	// load calibration value from mpu

	// Load Mag bias and scale
	//mpu9250_get_mag_bias_scale(mpu_p->magBias, mpu_p->magScale, ch);
}

// 3. FUll : Selftest, gyro&acc calibaration - gyro and acc bias save, mag calibration
void mpu9250_setup_full_calibration(uint8_t ch)
{
	if(ch==0)		mpu_p = &mpu;
	else if(ch==1) 	mpu_p = &mpu2;

	// 1. Start by performing self test and reporting value
	MPU9250SelfTest(mpu_p->SelfTest);

	// get sensor resolutions, only need to do this once
	mpu9250_set_resolution();

	// Calibrate gyro and accelerometers, load biases in bias registers
	calibrateMPU9250(mpu_p->gyroBias, mpu_p->accelBias, true, true, ch);

	// Initialize device for active mode read of acclerometer, gyroscope, and temperature
    initMPU9250(ch);

	// Do not use in MPU Second
	// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
	//find_ak8963(ch);

	// Get magnetometer calibration from AK8963 ROM
	initAK8963(mpu_p->magCalibration, ch);	// load calibration value from mpu

	// Magnetometer Calibration take 20sec
	magcalMPU9250(mpu_p->magBias, mpu_p->magScale, ch);
	//mpu9250_set_mag_bias_scale(mpu_p->magBias, mpu_p->magScale, ch);
}

void readMagData(int16_t * destination, uint8_t ch)
{
	uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	if(readRegister(AK8963_ADDRESS+ch, AK8963_ST1) & 0x01)
	{
		// wait for magnetometer data ready bit to be set
		readRegisters(AK8963_ADDRESS+ch, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		if(!(c & 0x08))
		{ // Check if magnetic sensor overflow set, if not then report data
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
		}
	}
}

void app_mpu9250_set_debug_print(bool print_flag, uint8_t print_mode)
{
	debug_print_flag = print_flag;
	debug_print_mode = print_mode;
}

void readMPU9250Data(int16_t * destination, uint8_t ch)
{
	uint8_t rawData[14];  // x/y/z accel register data stored here
	readRegisters(MPU9250_ADDRESS+ch, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
	destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
	destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
	destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
	destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}

void app_mpu9250_param_update(void)
{
	// Default Parameter is defined at appconf_custom.h
	//
	imu_data.use_mode = appconf->app_custom.custom_imu_use_mode;
	imu_data.comm_mode[0] = appconf->app_custom.custom_imu_comm_mode[0];
	imu_data.comm_mode[1] = appconf->app_custom.custom_imu_comm_mode[1];
	imu_data.setup_mode[0] = appconf->app_custom.custom_imu_setup_mode[0];
	imu_data.setup_mode[1] = appconf->app_custom.custom_imu_setup_mode[1];
	imu_data.dof_mode[0] = appconf->app_custom.custom_imu_dof[0];
	imu_data.dof_mode[1] = appconf->app_custom.custom_imu_dof[1];
	imu_data.quaternion_filter_mode = appconf->app_custom.custom_imu_quaternion_filter_mode;
	imu_data.rate_freq = appconf->app_custom.custom_imu_rate_freq;
	imu_data.rate_hz = imu_data.rate_freq;
	imu_data.loop_ms = (uint8_t)(1./imu_data.rate_freq*1000.);
	imu_data.dt = 1./imu_data.rate_freq;

	// 1st mpu
	mpu.Ascale = appconf->app_custom.custom_imu_ascale; // AFS_2G, AFS_4G, AFS_8G, AFS_16G
	mpu.Gscale = appconf->app_custom.custom_imu_gscale; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
	mpu.Mscale = appconf->app_custom.custom_imu_mscale; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution

	if(appconf->app_custom.custom_imu_m_mode == 0) 		  mpu.Mmode = 0x02;	// 8Hz
	else if(appconf->app_custom.custom_imu_m_mode == 1)   mpu.Mmode = 0x06;	// 100Hz

	mpu.use_acc_bias_user = appconf->app_custom.custom_imu_use_acc_bias_user[0];
	mpu.accelBias_user[0] = appconf->app_custom.custom_imu_accel_bias_user[0][0];
	mpu.accelBias_user[1] = appconf->app_custom.custom_imu_accel_bias_user[0][1];
	mpu.accelBias_user[2] = appconf->app_custom.custom_imu_accel_bias_user[0][2];

//	mpu.magBias[0] = appconf->app_custom.custom_imu_mag_bias[0][0];
//	mpu.magBias[1] = appconf->app_custom.custom_imu_mag_bias[0][1];
//	mpu.magBias[2] = appconf->app_custom.custom_imu_mag_bias[0][2];
//	mpu.magScale[0] = appconf->app_custom.custom_imu_mag_scale[0][0];
//	mpu.magScale[1] = appconf->app_custom.custom_imu_mag_scale[0][1];
//	mpu.magScale[2] = appconf->app_custom.custom_imu_mag_scale[0][2];

	// 2nd mpu
	if(imu_data.use_mode==2) {
		mpu2.Ascale = mpu.Ascale;//appconf->app_custom.custom_imu_ascale; // AFS_2G, AFS_4G, AFS_8G, AFS_16G
		mpu2.Gscale = mpu.Gscale;//appconf->app_custom.custom_imu_gscale; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
		mpu2.Mscale = mpu.Mscale;//appconf->app_custom.custom_imu_mscale; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution

		if(appconf->app_custom.custom_imu_m_mode == 0) 		  mpu2.Mmode = 0x02;	// 8Hz
		else if(appconf->app_custom.custom_imu_m_mode == 1)   mpu2.Mmode = 0x06;	// 100Hz

		mpu2.use_acc_bias_user = appconf->app_custom.custom_imu_use_acc_bias_user[1];
		mpu2.accelBias_user[0] = appconf->app_custom.custom_imu_accel_bias_user[1][0];
		mpu2.accelBias_user[1] = appconf->app_custom.custom_imu_accel_bias_user[1][1];
		mpu2.accelBias_user[2] = appconf->app_custom.custom_imu_accel_bias_user[1][2];

//		mpu2.magBias[0] = appconf->app_custom.custom_imu_mag_bias[1][0];
//		mpu2.magBias[1] = appconf->app_custom.custom_imu_mag_bias[1][1];
//		mpu2.magBias[2] = appconf->app_custom.custom_imu_mag_bias[1][2];
//		mpu2.magScale[0] = appconf->app_custom.custom_imu_mag_scale[1][0];
//		mpu2.magScale[1] = appconf->app_custom.custom_imu_mag_scale[1][1];
//		mpu2.magScale[2] = appconf->app_custom.custom_imu_mag_scale[1][2];
	}
}

void app_mpu9250_param_printf(void)
{
	app_mpu9250_param_update();

	// imu use mode
	debug_printf("\r\n------------- USE IMU Mode -------------");
	if(imu_data.use_mode==0) 		debug_printf("\r\nIMU OFF");
	else if(imu_data.use_mode==1)	debug_printf("\r\n1st IMU ON");
	else if(imu_data.use_mode==2)	debug_printf("\r\n1st, 2nd IMU ON");

	// comm mode
	debug_printf("\r\n------------- Comm Mode -------------");
	if(imu_data.comm_mode[0]==0) 		debug_printf("\r\n1st MPU-SPI Communication");
	else if(imu_data.comm_mode[0]==1)	debug_printf("\r\n1st MPU-I2C Communication");
	if(imu_data.use_mode==2) {
		if(imu_data.comm_mode[1]==0) 		debug_printf("\r\n2nd MPU-SPI Communication");
		else if(imu_data.comm_mode[1]==1)	debug_printf("\r\n2nd MPU-I2C Communication");
	}

	// Setup mode
	debug_printf("\r\n------------- Setup Mode -------------");
	if(imu_data.setup_mode[0]==0) 		debug_printf("\r\n1st MPU Basic Setup");
	else if(imu_data.setup_mode[0]==1)	debug_printf("\r\n1st MPU Fast Setup");
	else if(imu_data.setup_mode[0]==2)	debug_printf("\r\n1st MPU Full Setup");
	if(imu_data.use_mode==2) {
		if(imu_data.setup_mode[1]==0) 		debug_printf("\r\n2nd MPU Basic Setup");
		else if(imu_data.setup_mode[1]==1)	debug_printf("\r\n2nd MPU Fast Setup");
		else if(imu_data.setup_mode[1]==2)	debug_printf("\r\n2nd MPU Full Setup");
	}

	// DOF mode
	debug_printf("\r\n------------- DOF Mode -------------");
	if(imu_data.dof_mode[0]==0) 		debug_printf("\r\n1st MPU 6DOF IMU");
	else if(imu_data.dof_mode[0]==1)	debug_printf("\r\n1st MPU 9DOF AHRS");
	if(imu_data.use_mode==2) {
		if(imu_data.dof_mode[1]==0) 		debug_printf("\r\n2nd MPU 6DOF IMU");
		else if(imu_data.dof_mode[1]==1)	debug_printf("\r\n2nd MPU 9DOF AHRS");
	}

	// Quaternion
	debug_printf("\r\n------------- Quaternion Filter -------------");
	if(imu_data.quaternion_filter_mode==0) 		debug_printf("\r\nQuaternion Filter : Madgwick");
	else if(imu_data.quaternion_filter_mode==1)	debug_printf("\r\nQuaternion Filter : Mahony");
	debug_printf("\r\nIMU Rate:%.2fHz", (double)imu_data.rate_hz);	// IMU rate

	// Selftest Result
	debug_printf("\r\n------------- 1st MPU9250 -------------");
	debug_printf("\r\n------------- Self Test Result -------------");
	debug_printf("\r\nx-axis self test: acceleration trim within : %.2f %% of factory value", (double)mpu.SelfTest[0]);
	debug_printf("\r\ny-axis self test: acceleration trim within : %.2f %% of factory value", (double)mpu.SelfTest[1]);
	debug_printf("\r\nz-axis self test: acceleration trim within : %.2f %% of factory value", (double)mpu.SelfTest[2]);
	debug_printf("\r\nx-axis self test: gyration trim within     : %.2f %% of factory value", (double)mpu.SelfTest[3]);
	debug_printf("\r\ny-axis self test: gyration trim within     : %.2f %% of factory value", (double)mpu.SelfTest[4]);
	debug_printf("\r\nz-axis self test: gyration trim within     : %.2f %% of factory value", (double)mpu.SelfTest[5]);

	// Sensitivity
	debug_printf("\r\n------------- Sensitivity -------------");
	debug_printf("\r\nAccelerometer sensitivity is %f LSB/g", (double)(1.0f/mpu.aRes));
	debug_printf("\r\nGyroscope sensitivity is %f LSB/deg/s", (double)(1.0f/mpu.gRes));
	debug_printf("\r\nMagnetometer sensitivity is %f LSB/G", (double)(1.0f/mpu.mRes));

	// Calibration Result
	debug_printf("\r\n------------- Bias and Scale -------------");
	if(mpu.Ascale==0)	debug_printf("\r\nAccel Scale : AFS_2G");
	else if(mpu.Ascale==1)	debug_printf("\r\nAccel Scale : AFS_4G");
	else if(mpu.Ascale==2)	debug_printf("\r\nAccel Scale : AFS_8G");
	else if(mpu.Ascale==3)	debug_printf("\r\nAccel Scale : AFS_16G");

	if(mpu.Gscale==0)	debug_printf("\r\nGyro Scale : GFS_250DPS");
	else if(mpu.Gscale==1)	debug_printf("\r\nGyro Scale : GFS_500DPS");
	else if(mpu.Gscale==2)	debug_printf("\r\nGyro Scale : GFS_1000DPS");
	else if(mpu.Gscale==3)	debug_printf("\r\nGyro Scale : GFS_2000DPS");

	if(mpu.Mscale==0)	debug_printf("\r\nM_Mode Scale : MFS_14BITS");
	else if(mpu.Mscale==1)	debug_printf("\r\nM_Mode Scale : MFS_16BITS");

	debug_printf("\r\nx gyro bias = %f (dps)", (double)mpu.gyroBias[0]);
	debug_printf("\r\ny gyro bias = %f (dps)", (double)mpu.gyroBias[1]);
	debug_printf("\r\nz gyro bias = %f (dps)", (double)mpu.gyroBias[2]);
	debug_printf("\r\nx accel bias = %f (mg)", (double)(1000.*mpu.accelBias[0]));
	debug_printf("\r\ny accel bias = %f (mg)", (double)(1000.*mpu.accelBias[1]));
	debug_printf("\r\nz accel bias = %f (mg)", (double)(1000.*mpu.accelBias[2]));
	debug_printf("\r\nx accel bias = %f (mg) - user defined", (double)(1000.*mpu.accelBias_user[0]));
	debug_printf("\r\ny accel bias = %f (mg) - user defined", (double)(1000.*mpu.accelBias_user[1]));
	debug_printf("\r\nz accel bias = %f (mg) - user defined", (double)(1000.*mpu.accelBias_user[2]));
	debug_printf("\r\nUse user defined accel bias = %d", mpu.use_acc_bias_user);
	debug_printf("\r\nAK8963 mag sensitivity adjustment values (mG) = %f, %f, %f", (double)mpu.magCalibration[0], (double)mpu.magCalibration[1], (double)mpu.magCalibration[2]);
	debug_printf("\r\nAK8963 mag biases (mG) = %f, %f, %f", (double)mpu.magBias[0], (double)mpu.magBias[1], (double)mpu.magBias[2]);
	debug_printf("\r\nAK8963 mag scale (mG) = %f, %f, %f", (double)mpu.magScale[0], (double)mpu.magScale[1], (double)mpu.magScale[2]);

	if(imu_data.use_mode==2) {
		debug_printf("\r\n------------- 2nd MPU9250 -------------");
		debug_printf("\r\n------------- Self Test Result -------------");
		debug_printf("\r\nx-axis self test: acceleration trim within : %.2f %% of factory value", (double)mpu2.SelfTest[0]);
		debug_printf("\r\ny-axis self test: acceleration trim within : %.2f %% of factory value", (double)mpu2.SelfTest[1]);
		debug_printf("\r\nz-axis self test: acceleration trim within : %.2f %% of factory value", (double)mpu2.SelfTest[2]);
		debug_printf("\r\nx-axis self test: gyration trim within     : %.2f %% of factory value", (double)mpu2.SelfTest[3]);
		debug_printf("\r\ny-axis self test: gyration trim within     : %.2f %% of factory value", (double)mpu2.SelfTest[4]);
		debug_printf("\r\nz-axis self test: gyration trim within     : %.2f %% of factory value", (double)mpu2.SelfTest[5]);

		// Sensitivity
		debug_printf("\r\n------------- Sensitivity -------------");
		debug_printf("\r\nAccelerometer sensitivity is %f LSB/g", (double)(1.0f/mpu2.aRes));
		debug_printf("\r\nGyroscope sensitivity is %f LSB/deg/s", (double)(1.0f/mpu2.gRes));
		debug_printf("\r\nMagnetometer sensitivity is %f LSB/G", (double)(1.0f/mpu2.mRes));

		// Calibration Result
		debug_printf("\r\n------------- Bias and Scale -------------");
		if(mpu2.Ascale==0)	debug_printf("\r\nAccel Scale : AFS_2G");
		else if(mpu2.Ascale==1)	debug_printf("\r\nAccel Scale : AFS_4G");
		else if(mpu2.Ascale==2)	debug_printf("\r\nAccel Scale : AFS_8G");
		else if(mpu2.Ascale==3)	debug_printf("\r\nAccel Scale : AFS_16G");

		if(mpu2.Gscale==0)	debug_printf("\r\nGyro Scale : GFS_250DPS");
		else if(mpu2.Gscale==1)	debug_printf("\r\nGyro Scale : GFS_500DPS");
		else if(mpu2.Gscale==2)	debug_printf("\r\nGyro Scale : GFS_1000DPS");
		else if(mpu2.Gscale==3)	debug_printf("\r\nGyro Scale : GFS_2000DPS");

		if(mpu2.Mscale==0)	debug_printf("\r\nM_Mode Scale : MFS_14BITS");
		else if(mpu2.Mscale==1)	debug_printf("\r\nM_Mode Scale : MFS_16BITS");

		debug_printf("\r\nx gyro bias = %f (dps)", (double)mpu2.gyroBias[0]);
		debug_printf("\r\ny gyro bias = %f (dps)", (double)mpu2.gyroBias[1]);
		debug_printf("\r\nz gyro bias = %f (dps)", (double)mpu2.gyroBias[2]);
		debug_printf("\r\nx accel bias = %f (mg)", (double)(1000.*mpu2.accelBias[0]));
		debug_printf("\r\ny accel bias = %f (mg)", (double)(1000.*mpu2.accelBias[1]));
		debug_printf("\r\nz accel bias = %f (mg)", (double)(1000.*mpu2.accelBias[2]));
		debug_printf("\r\nx accel bias = %f (mg) - user defined", (double)(1000.*mpu2.accelBias_user[0]));
		debug_printf("\r\ny accel bias = %f (mg) - user defined", (double)(1000.*mpu2.accelBias_user[1]));
		debug_printf("\r\nz accel bias = %f (mg) - user defined", (double)(1000.*mpu2.accelBias_user[2]));
		debug_printf("\r\nUse user defined accel bias = %d", mpu2.use_acc_bias_user);
		debug_printf("\r\nAK8963 mag sensitivity adjustment values (mG) = %f, %f, %f", (double)mpu2.magCalibration[0], (double)mpu2.magCalibration[1], (double)mpu2.magCalibration[2]);
		debug_printf("\r\nAK8963 mag biases (mG) = %f, %f, %f", (double)mpu2.magBias[0], (double)mpu2.magBias[1], (double)mpu2.magBias[2]);
		debug_printf("\r\nAK8963 mag scale (mG) = %f, %f, %f", (double)mpu2.magScale[0], (double)mpu2.magScale[1], (double)mpu2.magScale[2]);
		debug_printf("\r\n--------------Parameter Updated-----------------");
	}
	debug_printf("\r\n");
}

void app_mpu9250_set_use_mode(uint8_t mode)
{
	appconf->app_custom.custom_imu_use_mode = mode;
	conf_general_store_app_configuration(appconf);
	if(mode==0) debug_printf("\r\nSet IMU USE OFF");
	else if(mode==1) debug_printf("\r\nSet IMU USE ON");
	else if(mode==2) debug_printf("\r\nSet 1st, 2nd IMU USE ON");
	debug_printf("\r\n");
}

void app_mpu9250_set_comm_mode(uint8_t mode, uint8_t ch)
{
	appconf->app_custom.custom_imu_comm_mode[ch] = mode;
	conf_general_store_app_configuration(appconf);
	if(mode==0) debug_printf("\r\nSet IMU COMM Mode - SPI, mpu %d", ch+1);
	else if(mode==1) debug_printf("\r\nSet IMU COMM Mode - I2C, mpu %d", ch+1);
	debug_printf("\r\n");
}

void app_mpu9250_set_imu_setup_mode(uint8_t mode, uint8_t ch)
{
	appconf->app_custom.custom_imu_setup_mode[ch] = mode;
	conf_general_store_app_configuration(appconf);
	if(mode==0) debug_printf("\r\nSet IMU Setup Mode - Basic, mpu %d", ch+1);
	else if(mode==1) debug_printf("\r\nSet IMU Setup Mode - Fast, mpu %d", ch+1);
	else if(mode==2) debug_printf("\r\nSet IMU Setup Mode - Full, mpu %d", ch+1);
	debug_printf("\r\n");
}

void app_mpu9250_set_imu_dof_mode(uint8_t mode, uint8_t ch)
{
	appconf->app_custom.custom_imu_dof[ch] = mode;
	conf_general_store_app_configuration(appconf);
	if(mode==0) debug_printf("\r\nSet IMU DOF Mode - 6DOF, mpu %d", ch+1);
	else if(mode==1) debug_printf("\r\nSet IMU DOF Mode - 9DOF, mpu %d", ch+1);
	debug_printf("\r\n");
}

void app_mpu9250_set_imu_gyro_bias(float bias_x, float bias_y, float bias_z, uint8_t ch)
{
	appconf->app_custom.custom_imu_gyro_bias[ch][0] = bias_x;
	appconf->app_custom.custom_imu_gyro_bias[ch][1] = bias_y;
	appconf->app_custom.custom_imu_gyro_bias[ch][2] = bias_z;
	conf_general_store_app_configuration(appconf);
	debug_printf("\r\nch:%d MPU-Store x gyro bias:%f", ch+1, (double)bias_x);
	debug_printf("\r\nch:%d MPU-Store y gyro bias:%f", ch+1, (double)bias_y);
	debug_printf("\r\nch:%d MPU-Store z gyro bias:%f", ch+1, (double)bias_z);
	debug_printf("\r\nch:%d MPU-Gyro Bias stored to stm32 internal eeprom", ch+1);
}

void app_mpu9250_set_imu_acc_bias(float bias_x, float bias_y, float bias_z, uint8_t ch)
{
	appconf->app_custom.custom_imu_accel_bias[ch][0] = bias_x;
	appconf->app_custom.custom_imu_accel_bias[ch][1] = bias_y;
	appconf->app_custom.custom_imu_accel_bias[ch][2] = bias_z;
	conf_general_store_app_configuration(appconf);
	debug_printf("\r\nch:%d MPU-Store x acc bias:%f", ch+1, (double)bias_x);
	debug_printf("\r\nch:%d MPU-Store y acc bias:%f", ch+1, (double)bias_y);
	debug_printf("\r\nch:%d MPU-Store z acc bias:%f", ch+1, (double)bias_z);
	debug_printf("\r\nch:%d MPU-Acc Bias stored to stm32 internal eeprom", ch+1);
}

void app_mpu9250_set_imu_acc_user_bias(float bias_x, float bias_y, float bias_z, uint8_t ch)
{
	appconf->app_custom.custom_imu_accel_bias_user[ch][0] = bias_x;
	appconf->app_custom.custom_imu_accel_bias_user[ch][1] = bias_y;
	appconf->app_custom.custom_imu_accel_bias_user[ch][2] = bias_z;
	conf_general_store_app_configuration(appconf);
	debug_printf("\r\nch:%d MPU-Set x accel bias:%f - user defined", ch+1, (double)bias_x);
	debug_printf("\r\nch:%d MPU-Set y accel bias:%f - user defined", ch+1, (double)bias_y);
	debug_printf("\r\nch:%d MPU-Set z accel bias:%f - user defined", ch+1, (double)bias_z);
	debug_printf("\r\n");
}

void app_mpu9250_set_imu_acc_user_bias_onoff(uint8_t mode, uint8_t ch)
{
	appconf->app_custom.custom_imu_use_acc_bias_user[ch] = mode;
	conf_general_store_app_configuration(appconf);
	if(mode==0) debug_printf("\r\nSet IMU ACC User Bias OFF, mpu %d", ch+1);
	else if(mode==1) debug_printf("\r\nSet IMU ACC User Bias ON, mpu %d", ch+1);
	debug_printf("\r\n");
}

void app_mpu9250_get_imu_data(IMU_DATA *data)
{
	//memcpy(data, &imu_data, sizeof(IMU_DATA));
	*data = imu_data;
}

uint16_t app_get_valid_cnt(void)
{
	return valid_cnt;
}

void app_set_valid_cnt_minus1(void)
{
	if(valid_cnt>0)	valid_cnt--;
}

void app_set_valid_cnt_minus2(void)
{
	if(valid_cnt>0)	valid_cnt = valid_cnt - 2;
}
// --------------- END of MPU9250 Functions ---------------

// --------------- Quaternion Filter Functions ---------------
// Madgwick
/*
 * beta :
 * The filter noise value, smaller values have smoother estimates, but have higher latency.
 * This only works for the `Madgwick` filter.
 */
float PI = (float)M_PI;
#define GyroMeasError	PI * (40.0f / 180.0f)  // gyroscope measurement error in rads/s (start at 40 deg/s)
#define GyroMeasDrift	PI * (0.0f  / 180.0f)  // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta 	(float)(sqrt(3.0f / 4.0f)) * GyroMeasError   // compute beta
//#define beta 	(float)(0.6)
//float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

// Mahony - Variable definitions
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain
volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)

//original version
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//---------------------------------------------------------------------------------------------------
// Global Variables
volatile float integralFBx_1 = 0.0f,  integralFBy_1 = 0.0f, integralFBz_1 = 0.0f;	// For Mahony, integral error terms scaled by Ki
float q0_1 = 1.0f, q1_1 = 0.0f, q2_1 = 0.0f, q3_1 = 0.0f;

// IMU algorithm update - 6DOF
void MadgwickAHRSupdateIMU(float *q_result, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1_1 * gx - q2_1 * gy - q3_1 * gz);
	qDot2 = 0.5f * (q0_1 * gx + q2_1 * gz - q3_1 * gy);
	qDot3 = 0.5f * (q0_1 * gy - q1_1 * gz + q3_1 * gx);
	qDot4 = 0.5f * (q0_1 * gz + q1_1 * gy - q2_1 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0_1;
		_2q1 = 2.0f * q1_1;
		_2q2 = 2.0f * q2_1;
		_2q3 = 2.0f * q3_1;
		_4q0 = 4.0f * q0_1;
		_4q1 = 4.0f * q1_1;
		_4q2 = 4.0f * q2_1;
		_8q1 = 8.0f * q1_1;
		_8q2 = 8.0f * q2_1;
		q0q0 = q0_1 * q0_1;
		q1q1 = q1_1 * q1_1;
		q2q2 = q2_1 * q2_1;
		q3q3 = q3_1 * q3_1;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2_1 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3_1 - _2q1 * ax + 4.0f * q2q2 * q3_1 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0_1 += qDot1 * dt;
	q1_1 += qDot2 * dt;
	q2_1 += qDot3 * dt;
	q3_1 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0_1 * q0_1 + q1_1 * q1_1 + q2_1 * q2_1 + q3_1 * q3_1);
	q0_1 *= recipNorm;
	q1_1 *= recipNorm;
	q2_1 *= recipNorm;
	q3_1 *= recipNorm;

	// result
	q_result[0] = q0_1;
	q_result[1] = q1_1;
	q_result[2] = q2_1;
	q_result[3] = q3_1;
}
// Madgwick1

void MadgwickAHRSupdate(float *q_result, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(q_result, gx, gy, gz, ax, ay, az, dt);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1_1 * gx - q2_1 * gy - q3_1 * gz);
	qDot2 = 0.5f * (q0_1 * gx + q2_1 * gz - q3_1 * gy);
	qDot3 = 0.5f * (q0_1 * gy - q1_1 * gz + q3_1 * gx);
	qDot4 = 0.5f * (q0_1 * gz + q1_1 * gy - q2_1 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0_1 * mx;
		_2q0my = 2.0f * q0_1 * my;
		_2q0mz = 2.0f * q0_1 * mz;
		_2q1mx = 2.0f * q1_1 * mx;
		_2q0 = 2.0f * q0_1;
		_2q1 = 2.0f * q1_1;
		_2q2 = 2.0f * q2_1;
		_2q3 = 2.0f * q3_1;
		_2q0q2 = 2.0f * q0_1 * q2_1;
		_2q2q3 = 2.0f * q2_1 * q3_1;
		q0q0 = q0_1 * q0_1;
		q0q1 = q0_1 * q1_1;
		q0q2 = q0_1 * q2_1;
		q0q3 = q0_1 * q3_1;
		q1q1 = q1_1 * q1_1;
		q1q2 = q1_1 * q2_1;
		q1q3 = q1_1 * q3_1;
		q2q2 = q2_1 * q2_1;
		q2q3 = q2_1 * q3_1;
		q3q3 = q3_1 * q3_1;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3_1 + _2q0mz * q2_1 + mx * q1q1 + _2q1 * my * q2_1 + _2q1 * mz * q3_1 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3_1 + my * q0q0 - _2q0mz * q1_1 + _2q1mx * q2_1 - my * q1q1 + my * q2q2 + _2q2 * mz * q3_1 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2_1 + _2q0my * q1_1 + mz * q0q0 + _2q1mx * q3_1 - mz * q1q1 + _2q2 * my * q3_1 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2_1 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3_1 + _2bz * q1_1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2_1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1_1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3_1 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2_1 + _2bz * q0_1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3_1 - _4bz * q1_1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2_1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2_1 - _2bz * q0_1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1_1 + _2bz * q3_1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0_1 - _4bz * q2_1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3_1 + _2bz * q1_1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0_1 + _2bz * q2_1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1_1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0_1 += qDot1 * dt;
	q1_1 += qDot2 * dt;
	q2_1 += qDot3 * dt;
	q3_1 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0_1 * q0_1 + q1_1 * q1_1 + q2_1 * q2_1 + q3_1 * q3_1);
	q0_1 *= recipNorm;
	q1_1 *= recipNorm;
	q2_1 *= recipNorm;
	q3_1 *= recipNorm;

	// result
	q_result[0] = q0_1;
	q_result[1] = q1_1;
	q_result[2] = q2_1;
	q_result[3] = q3_1;
} // Madgwick

// Mahony
//---------------------------------------------------------------------------------------------------
// IMU algorithm update
void MahonyAHRSupdateIMU(float *q_result, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1_1 * q3_1 - q0_1 * q2_1;
		halfvy = q0_1 * q1_1 + q2_1 * q3_1;
		halfvz = q0_1 * q0_1 - 0.5f + q3_1 * q3_1;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx_1 += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy_1 += twoKi * halfey * dt;
			integralFBz_1 += twoKi * halfez * dt;
			gx += integralFBx_1;	// apply integral feedback
			gy += integralFBy_1;
			gz += integralFBz_1;
		}
		else {
			integralFBx_1 = 0.0f;	// prevent integral windup
			integralFBy_1 = 0.0f;
			integralFBz_1 = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0_1;
	qb = q1_1;
	qc = q2_1;
	q0_1 += (-qb * gx - qc * gy - q3_1 * gz);
	q1_1 += (qa * gx + qc * gz - q3_1 * gy);
	q2_1 += (qa * gy - qb * gz + q3_1 * gx);
	q3_1 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0_1 * q0_1 + q1_1 * q1_1 + q2_1 * q2_1 + q3_1 * q3_1);
	q0_1 *= recipNorm;
	q1_1 *= recipNorm;
	q2_1 *= recipNorm;
	q3_1 *= recipNorm;

	// result
	q_result[0] = q0_1;
	q_result[1] = q1_1;
	q_result[2] = q2_1;
	q_result[3] = q3_1;
} // Mahony

void MahonyAHRSupdate(float *q_result, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(q_result, gx, gy, gz, ax, ay, az, dt);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0_1 * q0_1;
		q0q1 = q0_1 * q1_1;
		q0q2 = q0_1 * q2_1;
		q0q3 = q0_1 * q3_1;
		q1q1 = q1_1 * q1_1;
		q1q2 = q1_1 * q2_1;
		q1q3 = q1_1 * q3_1;
		q2q2 = q2_1 * q2_1;
		q2q3 = q2_1 * q3_1;
		q3q3 = q3_1 * q3_1;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx_1 += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy_1 += twoKi * halfey * dt;
			integralFBz_1 += twoKi * halfez * dt;
			gx += integralFBx_1;	// apply integral feedback
			gy += integralFBy_1;
			gz += integralFBz_1;
		}
		else {
			integralFBx_1 = 0.0f;	// prevent integral windup
			integralFBy_1 = 0.0f;
			integralFBz_1 = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0_1;
	qb = q1_1;
	qc = q2_1;
	q0_1 += (-qb * gx - qc * gy - q3_1 * gz);
	q1_1 += (qa * gx + qc * gz - q3_1 * gy);
	q2_1 += (qa * gy - qb * gz + q3_1 * gx);
	q3_1 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0_1 * q0_1 + q1_1 * q1_1 + q2_1 * q2_1 + q3_1 * q3_1);
	q0_1 *= recipNorm;
	q1_1 *= recipNorm;
	q2_1 *= recipNorm;
	q3_1 *= recipNorm;

	// result
	q_result[0] = q0_1;
	q_result[1] = q1_1;
	q_result[2] = q2_1;
	q_result[3] = q3_1;
} // Mahony

//---------------------------------------------------------------------------------------------------
// Global Variables
volatile float integralFBx_2 = 0.0f,  integralFBy_2 = 0.0f, integralFBz_2 = 0.0f;	// For Mahony, integral error terms scaled by Ki
float q0_2 = 1.0f, q1_2 = 0.0f, q2_2 = 0.0f, q3_2 = 0.0f;

// IMU algorithm update - 6DOF
void MadgwickAHRSupdateIMU_2(float *q_result, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1_2 * gx - q2_2 * gy - q3_2 * gz);
	qDot2 = 0.5f * (q0_2 * gx + q2_2 * gz - q3_2 * gy);
	qDot3 = 0.5f * (q0_2 * gy - q1_2 * gz + q3_2 * gx);
	qDot4 = 0.5f * (q0_2 * gz + q1_2 * gy - q2_2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0_2;
		_2q1 = 2.0f * q1_2;
		_2q2 = 2.0f * q2_2;
		_2q3 = 2.0f * q3_2;
		_4q0 = 4.0f * q0_2;
		_4q1 = 4.0f * q1_2;
		_4q2 = 4.0f * q2_2;
		_8q1 = 8.0f * q1_2;
		_8q2 = 8.0f * q2_2;
		q0q0 = q0_2 * q0_2;
		q1q1 = q1_2 * q1_2;
		q2q2 = q2_2 * q2_2;
		q3q3 = q3_2 * q3_2;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_2 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2_2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3_2 - _2q1 * ax + 4.0f * q2q2 * q3_2 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0_2 += qDot1 * dt;
	q1_2 += qDot2 * dt;
	q2_2 += qDot3 * dt;
	q3_2 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0_2 * q0_2 + q1_2 * q1_2 + q2_2 * q2_2 + q3_2 * q3_2);
	q0_2 *= recipNorm;
	q1_2 *= recipNorm;
	q2_2 *= recipNorm;
	q3_2 *= recipNorm;

	// result
	q_result[0] = q0_2;
	q_result[1] = q1_2;
	q_result[2] = q2_2;
	q_result[3] = q3_2;
}
//

void MadgwickAHRSupdate_2(float *q_result, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU_2(q_result, gx, gy, gz, ax, ay, az, dt);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1_2 * gx - q2_2 * gy - q3_2 * gz);
	qDot2 = 0.5f * (q0_2 * gx + q2_2 * gz - q3_2 * gy);
	qDot3 = 0.5f * (q0_2 * gy - q1_2 * gz + q3_2 * gx);
	qDot4 = 0.5f * (q0_2 * gz + q1_2 * gy - q2_2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0_2 * mx;
		_2q0my = 2.0f * q0_2 * my;
		_2q0mz = 2.0f * q0_2 * mz;
		_2q1mx = 2.0f * q1_2 * mx;
		_2q0 = 2.0f * q0_2;
		_2q1 = 2.0f * q1_2;
		_2q2 = 2.0f * q2_2;
		_2q3 = 2.0f * q3_2;
		_2q0q2 = 2.0f * q0_2 * q2_2;
		_2q2q3 = 2.0f * q2_2 * q3_2;
		q0q0 = q0_2 * q0_2;
		q0q1 = q0_2 * q1_2;
		q0q2 = q0_2 * q2_2;
		q0q3 = q0_2 * q3_2;
		q1q1 = q1_2 * q1_2;
		q1q2 = q1_2 * q2_2;
		q1q3 = q1_2 * q3_2;
		q2q2 = q2_2 * q2_2;
		q2q3 = q2_2 * q3_2;
		q3q3 = q3_2 * q3_2;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3_2 + _2q0mz * q2_2 + mx * q1q1 + _2q1 * my * q2_2 + _2q1 * mz * q3_2 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3_2 + my * q0q0 - _2q0mz * q1_2 + _2q1mx * q2_2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3_2 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2_2 + _2q0my * q1_2 + mz * q0q0 + _2q1mx * q3_2 - mz * q1q1 + _2q2 * my * q3_2 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2_2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3_2 + _2bz * q1_2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2_2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1_2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3_2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2_2 + _2bz * q0_2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3_2 - _4bz * q1_2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2_2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2_2 - _2bz * q0_2) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1_2 + _2bz * q3_2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0_2 - _4bz * q2_2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3_2 + _2bz * q1_2) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0_2 + _2bz * q2_2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1_2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0_2 += qDot1 * dt;
	q1_2 += qDot2 * dt;
	q2_2 += qDot3 * dt;
	q3_2 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0_2 * q0_2 + q1_2 * q1_2 + q2_2 * q2_2 + q3_2 * q3_2);
	q0_2 *= recipNorm;
	q1_2 *= recipNorm;
	q2_2 *= recipNorm;
	q3_2 *= recipNorm;

	// result
	q_result[0] = q0_2;
	q_result[1] = q1_2;
	q_result[2] = q2_2;
	q_result[3] = q3_2;
} // Madgwick_2

// Mahony_2
//---------------------------------------------------------------------------------------------------
// IMU algorithm update
void MahonyAHRSupdateIMU_2(float *q_result, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1_2 * q3_2 - q0_2 * q2_2;
		halfvy = q0_2 * q1_2 + q2_2 * q3_2;
		halfvz = q0_2 * q0_2 - 0.5f + q3_2 * q3_2;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx_2 += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy_2 += twoKi * halfey * dt;
			integralFBz_2 += twoKi * halfez * dt;
			gx += integralFBx_2;	// apply integral feedback
			gy += integralFBy_2;
			gz += integralFBz_2;
		}
		else {
			integralFBx_2 = 0.0f;	// prevent integral windup
			integralFBy_2 = 0.0f;
			integralFBz_2 = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0_2;
	qb = q1_2;
	qc = q2_2;
	q0_2 += (-qb * gx - qc * gy - q3_2 * gz);
	q1_2 += (qa * gx + qc * gz - q3_2 * gy);
	q2_2 += (qa * gy - qb * gz + q3_2 * gx);
	q3_2 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0_2 * q0_2 + q1_2 * q1_2 + q2_2 * q2_2 + q3_2 * q3_2);
	q0_2 *= recipNorm;
	q1_2 *= recipNorm;
	q2_2 *= recipNorm;
	q3_2 *= recipNorm;

	// result
	q_result[0] = q0_2;
	q_result[1] = q1_2;
	q_result[2] = q2_2;
	q_result[3] = q3_2;
} // Mahony_2

void MahonyAHRSupdate_2(float *q_result, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU_2(q_result, gx, gy, gz, ax, ay, az, dt);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0_2 * q0_2;
		q0q1 = q0_2 * q1_2;
		q0q2 = q0_2 * q2_2;
		q0q3 = q0_2 * q3_2;
		q1q1 = q1_2 * q1_2;
		q1q2 = q1_2 * q2_2;
		q1q3 = q1_2 * q3_2;
		q2q2 = q2_2 * q2_2;
		q2q3 = q2_2 * q3_2;
		q3q3 = q3_2 * q3_2;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx_2 += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy_2 += twoKi * halfey * dt;
			integralFBz_2 += twoKi * halfez * dt;
			gx += integralFBx_2;	// apply integral feedback
			gy += integralFBy_2;
			gz += integralFBz_2;
		}
		else {
			integralFBx_2 = 0.0f;	// prevent integral windup
			integralFBy_2 = 0.0f;
			integralFBz_2 = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0_2;
	qb = q1_2;
	qc = q2_2;
	q0_2 += (-qb * gx - qc * gy - q3_2 * gz);
	q1_2 += (qa * gx + qc * gz - q3_2 * gy);
	q2_2 += (qa * gy - qb * gz + q3_2 * gx);
	q3_2 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0_2 * q0_2 + q1_2 * q1_2 + q2_2 * q2_2 + q3_2 * q3_2);
	q0_2 *= recipNorm;
	q1_2 *= recipNorm;
	q2_2 *= recipNorm;
	q3_2 *= recipNorm;

	// result
	q_result[0] = q0_2;
	q_result[1] = q1_2;
	q_result[2] = q2_2;
	q_result[3] = q3_2;
} // Mahony_2
// --------------- End of Quaternion Filter Functions ---------------

void mpu_process(uint8_t ch) {
	if(ch==0)		mpu_p = &mpu;
	else if(ch==1) 	mpu_p = &mpu2;

	// MPU9250 Read
	readMPU9250Data(MPU9250Data, ch);

	// Gyro & Accel Bias are stored in mpu9250
	// Now we'll calculate the acceleration value into actual g's
	// acc bias offset compensation using user defined value. 20170901 cdi
	if(mpu_p->use_acc_bias_user == 1)
	{
		mpu_p->ax = (float)MPU9250Data[0]*mpu_p->aRes - mpu_p->accelBias_user[0]; //- mpu_p->accelBias[0];  // get actual g value, this depends on scale being set
		mpu_p->ay = (float)MPU9250Data[1]*mpu_p->aRes - mpu_p->accelBias_user[1]; //- mpu_p->accelBias[1];
		mpu_p->az = (float)MPU9250Data[2]*mpu_p->aRes - mpu_p->accelBias_user[2]; //- mpu_p->accelBias[2];
	}
	else
	{
		mpu_p->ax = (float)MPU9250Data[0]*mpu_p->aRes; //- mpu_p->accelBias[0];  // get actual g value, this depends on scale being set
		mpu_p->ay = (float)MPU9250Data[1]*mpu_p->aRes; //- mpu_p->accelBias[1];
		mpu_p->az = (float)MPU9250Data[2]*mpu_p->aRes; //- mpu_p->accelBias[2];
	}

	// Calculate the gyro value into actual degrees per second
	mpu_p->gx = (float)MPU9250Data[4]*mpu_p->gRes;  // get actual gyro value, this depends on scale being set
	mpu_p->gy = (float)MPU9250Data[5]*mpu_p->gRes;
	mpu_p->gz = (float)MPU9250Data[6]*mpu_p->gRes;

	if(imu_data.dof_mode[0] == 1)	// // 9dof IMU
	{
		readMagData(mpu_p->magCount, ch);  // Read the x/y/z adc values
		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		mpu_p->mx = (float)mpu_p->magCount[0]*mpu_p->mRes*mpu_p->magCalibration[0] - mpu_p->magBias[0];  // get actual magnetometer value, this depends on scale being set
		mpu_p->my = (float)mpu_p->magCount[1]*mpu_p->mRes*mpu_p->magCalibration[1] - mpu_p->magBias[1];
		mpu_p->mz = (float)mpu_p->magCount[2]*mpu_p->mRes*mpu_p->magCalibration[2] - mpu_p->magBias[2];
		mpu_p->mx *= mpu_p->magScale[0];
		mpu_p->my *= mpu_p->magScale[1];
		mpu_p->mz *= mpu_p->magScale[2];
	}

	// 6dof IMU
	if(imu_data.dof_mode == 0) {
		mpu_p->mx = 0;
		mpu_p->my = 0;
		mpu_p->mz = 0;
	}

	if(ch==0) {
		if(imu_data.quaternion_filter_mode == 0) 	    MadgwickAHRSupdate(mpu_p->q, mpu_p->gx*PI/180.0f, mpu_p->gy*PI/180.0f, mpu_p->gz*PI/180.0f, mpu_p->ax, mpu_p->ay, mpu_p->az, mpu_p->my,  mpu_p->mx, -mpu_p->mz, imu_data.dt);
		else if(imu_data.quaternion_filter_mode == 1)	MahonyAHRSupdate(mpu_p->q, mpu_p->gx*PI/180.0f, mpu_p->gy*PI/180.0f, mpu_p->gz*PI/180.0f, mpu_p->ax, mpu_p->ay, mpu_p->az, mpu_p->my,  mpu_p->mx, -mpu_p->mz, imu_data.dt);
	}
	else if(ch==1) {
		if(imu_data.quaternion_filter_mode == 0) 	    MadgwickAHRSupdate_2(mpu_p->q, mpu_p->gx*PI/180.0f, mpu_p->gy*PI/180.0f, mpu_p->gz*PI/180.0f, mpu_p->ax, mpu_p->ay, mpu_p->az, mpu_p->my,  mpu_p->mx, -mpu_p->mz, imu_data.dt);
		else if(imu_data.quaternion_filter_mode == 1)	MahonyAHRSupdate_2(mpu_p->q, mpu_p->gx*PI/180.0f, mpu_p->gy*PI/180.0f, mpu_p->gz*PI/180.0f, mpu_p->ax, mpu_p->ay, mpu_p->az, mpu_p->my,  mpu_p->mx, -mpu_p->mz, imu_data.dt);
	}

	mpu_p->yaw   = atan2(2.0f * (mpu_p->q[1] * mpu_p->q[2] + mpu_p->q[0] * mpu_p->q[3]), mpu_p->q[0] * mpu_p->q[0] + mpu_p->q[1] * mpu_p->q[1] - mpu_p->q[2] * mpu_p->q[2] - mpu_p->q[3] * mpu_p->q[3]);
	mpu_p->pitch = -asin(2.0f * (mpu_p->q[1] * mpu_p->q[3] - mpu_p->q[0] * mpu_p->q[2]));
	mpu_p->roll  = atan2(2.0f * (mpu_p->q[0] * mpu_p->q[1] + mpu_p->q[2] * mpu_p->q[3]), mpu_p->q[0] * mpu_p->q[0] - mpu_p->q[1] * mpu_p->q[1] - mpu_p->q[2] * mpu_p->q[2] + mpu_p->q[3] * mpu_p->q[3]);
	mpu_p->pitch *= 180.0f / PI;
	mpu_p->yaw   *= 180.0f / PI;
	mpu_p->yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	mpu_p->roll  *= 180.0f / PI;
}

void mpu_debug_print(uint8_t ch) {
	// debug print
	if(app_run_every_x_hz(1, loop_cnt_imu, imu_data.rate_hz) && debug_print_flag)
	{
		if(debug_print_mode == 0)
		{
			debug_printf("\r\n%d - ch:%d, MPU9250 Raw DIGIT:", loop_cnt_imu, ch+1);
			for(int i=0; i<7; i++) debug_printf("%d ", MPU9250Data[i]);
			debug_printf(", magCount:");
			for(int i=0; i<3; i++) debug_printf("%d ", mpu_p->magCount[i]);
		}
		else if(debug_print_mode == 1)
		{
			debug_printf("\r\n%d - ch:%d, ax:%.3f, ay:%.3f, az:%.3f, gx:%.3f, gy:%.3f, gz:%.3f, mx:%.3f, my:%.3f, mz:%.3f", loop_cnt_imu, ch+1,
				(double)mpu_p->ax, (double)mpu_p->ay, (double)mpu_p->az, (double)mpu_p->gx, (double)mpu_p->gy, (double)mpu_p->gz,	(double)mpu_p->mx, (double)mpu_p->my, (double)mpu_p->mz);
		}
		else if(debug_print_mode == 2)
		{
			debug_printf("\r\n%d - ch:%d roll:%.3f, pitch:%.3f, yaw:%.3f ", loop_cnt_imu, ch+1, (double)mpu_p->roll, (double)mpu_p->pitch, (double)mpu_p->yaw);
		}
		else if(debug_print_mode == 3)
		{
			debug_printf("\r\n%d - ch:%d gx:%.3f, gy:%.3f, gz:%.3f", loop_cnt_imu, ch+1, (double)mpu_p->gx, (double)mpu_p->gy, (double)mpu_p->gz);
		}
	}
}

void imu_debug_print(void)
{
	if(app_run_every_x_hz(1, loop_cnt_imu, imu_data.rate_hz) && debug_print_flag)
	{
		if(debug_print_mode == 4)
		{
			debug_printf("\r\n%d - [euler_deg] roll:%.3f, pitch:%.3f, yaw:%.3f", loop_cnt_imu, (double)imu_data.euler_x, (double)imu_data.euler_y, (double)imu_data.euler_z);
			debug_printf("\r\n%d - [gyro_dps] roll:%.3f, pitch:%.3f, yaw:%.3f", loop_cnt_imu, (double)imu_data.gyro_x, (double)imu_data.gyro_y, (double)imu_data.gyro_z);
		}
	}
}

void set_imu_data_euler_relative(MPU9250_VARIABLE *mpu_ground, MPU9250_VARIABLE *mpu_body)
{
	// ground pitch lpf
	//app_util_cdi_lowpass_filter(mpu2->roll, &mpu2->roll_lpf, 15.0, imu_data.dt);
	imu_data.euler_x = (double)(mpu_body->pitch - (mpu_ground->pitch));		//deg MPU_ROLL = SMG_PITCH
	imu_data.euler_y = (double)((-mpu_body->roll) - (-mpu_ground->roll));		//deg
	imu_data.euler_z = (double)(mpu_body->yaw - mpu_ground->yaw);			//deg
	imu_data.gyro_x = (double)(mpu_body->gy - (mpu_ground->gy));			//deg/s
	imu_data.gyro_y = (double)((-mpu_body->gx) - (-mpu_ground->gx));			//deg/s
	imu_data.gyro_z = (double)(mpu_body->gz - mpu_ground->gz);			//deg/s
}

// --------------- Thread ---------------
static THD_FUNCTION(mpu9250_thread, arg)
{
	(void)arg;
	chRegSetThreadName("mpu9250");
	app_wait_sec(1);

	// time variables
	static systime_t t_start;
	static systime_t t_last;
	//static systime_t t_end;
	static systime_t t_duration;
	static uint32_t t_duration_usec;
	//static double t_dt;

	// 1st mpu9250 init
#ifdef HW_VERSION_60_VESCUINO_V02
	if(imu_data.comm_mode[0]==0) {
		debug_printf("\r\n-----------------------");
#ifdef USE_SW_SPI
		debug_printf("\r\nStart 1st MPU9250 - SPI SW Mode");
		mpu_spi_sw_init();
		MPU9250_SPI_CS_HIGH();
		MPU9250_SPI_MODE();
#else
		mpu_spi_hw_init();	// SPI HW PAL Init
		debug_printf("\r\nStart 1st MPU9250 - SPI HW Mode");
#endif
	}
	else if(imu_data.comm_mode[0]==1)
	{
		debug_printf("\r\n-----------------------");
		debug_printf("\r\nStart 1st MPU9250 - I2C Mode");
		hw_start_i2c();

		//
		MPU9250_I2C_MODE();
	}
#else
	debug_printf("\r\nStart MPU9250 - I2C Mode");
	hw_start_i2c();
#endif

	// 1st mpu9250 calibration
	while(1)
	{
		mpu.whoami = find_mpu9250(0);
		debug_printf("\r\n1st MPU9250, I AM 0x%x, I SHOULD BE 0x71", mpu.whoami);

		if(mpu.whoami == 0x71)
		{
			debug_printf("\r\n1st MPU9250 is online...");

			if(imu_data.setup_mode[0]==0)		mpu9250_setup_operation(0);
			else if(imu_data.setup_mode[0]==1)	mpu9250_setup_fast_calibration(0);
			else if(imu_data.setup_mode[0]==2)	mpu9250_setup_full_calibration(0);
			break;
		}
		else
		{
			debug_printf("\r\nCould not connect to 1st MPU9250: 0x%x !!!", mpu.whoami);
			app_mg_set_error_flag(ERROR_IMU_WHOAMI_FIRST_MCU);
			app_wait_sec(2);
		}
	}

	// 2nd mpu9250 calibration - spi hw
	if(imu_data.use_mode==2)
	{
		debug_printf("\r\n-----------------------");
		debug_printf("\r\nStart 2nd MPU9250 - SPI HW Mode");
		palSetPadMode(GPIOE, 3, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);    /* nCS1.      */
		palSetPad(GPIOE, 3);

		while(1)
		{
			mpu.whoami = find_mpu9250(1);
			debug_printf("\r\n2nd MPU9250, I AM 0x%x, I SHOULD BE 0x71", mpu.whoami);

			if(mpu.whoami == 0x71)
			{
				debug_printf("\r\n2nd MPU9250 is online...");

				if(imu_data.setup_mode[1]==0)		mpu9250_setup_operation(1);
				else if(imu_data.setup_mode[1]==1)	mpu9250_setup_fast_calibration(1);
				else if(imu_data.setup_mode[1]==2)	mpu9250_setup_full_calibration(1);
				break;
			}
			else
			{
				debug_printf("\r\nCould not connect to 2nd MPU9250: 0x%x !!!", mpu.whoami);
				app_mg_set_error_flag(ERROR_IMU_WHOAMI_SECOND_MCU);
				app_wait_sec(2);
			}
		}
	}

	// Start Loop
	debug_printf("\r\nStart into the loop... Loop Frequency : %d (hz)", imu_data.rate_freq);
	debug_printf("\r\n----------------------------------------------------------\r\n");
	for(;;)
	{
		// Timer implementation
		t_last = t_start;
		t_start = chVTGetSystemTime();
		t_duration = t_start - t_last;
		t_duration_usec = ST2US(t_duration);
		//t_dt = (double)(t_duration_usec/1000000.);	// dt sec, filter will be disabled if there is a jitter. (Jitter happens when I2C 100kHz)
		//t_end = t_start + MS2ST(imu_data.loop_ms);	// millisecond to system time

		// error code
		if(imu_data.loop_ms!=((uint8_t)(t_duration_usec/1000)))	app_mg_set_error_flag(ERROR_IMU_I2C_RATE);
		else	app_mg_set_error_flag(ERROR_NONE);

		// 1st mpu
		mpu_process(0);
		mpu_debug_print(0);

		// 2nd mpu
		if(imu_data.use_mode==2) {
			mpu_process(1);
			mpu_debug_print(1);
		}

		set_imu_data_euler_relative(&mpu, &mpu2);
		imu_debug_print();

		valid_cnt++;
		loop_cnt_imu++;
		chThdSleepMilliseconds(1);

	} // loop
}
// --------------- End of Thread ---------------
