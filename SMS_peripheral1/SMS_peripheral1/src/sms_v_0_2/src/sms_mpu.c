/*
* sms_imu.c
*
* Created: 14.06.2016 13:27:35
*  Author: Sébastien Schiesser
*/

#include <math.h>
#include "sms_peripheral1.h"
#include "mpu9250.h"

//static struct hal_s hal = {0};
    
void sms_mpu_configure_gpio(void)
{
    struct gpio_config config_gpio_pin;

    /* Data ready interrupt from IMU */
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction = GPIO_PIN_DIR_INPUT;
    config_gpio_pin.input_pull = GPIO_PIN_PULL_DOWN;
    config_gpio_pin.aon_wakeup = true;
    if(gpio_pin_set_config(SMS_MPU_DRDY_PIN, &config_gpio_pin) != STATUS_OK) {
        DBG_LOG("[sms_imu_configure_gpio]\tProblem while setting up IMU DRDY pin");
    }

    ///* Pin output to supply IMU */
    //gpio_get_config_defaults(&config_gpio_pin);
    //config_gpio_pin.direction = GPIO_PIN_DIR_OUTPUT;
    //if(!gpio_pin_set_config(SMS_MPU_VCC_PIN, &config_gpio_pin) != STATUS_OK) {
        //DBG_LOG("[sms_imu_configure_gpio]\tProblem while setting up MPU VCC pin");
    //}
    //gpio_pin_set_output_level(SMS_MPU_VCC_PIN, true);
}

/* Register GPIO interrupt callback */
void sms_mpu_register_callbacks(void)
{
    /* MPU-9250 interrupt callback */
    gpio_register_callback(SMS_MPU_DRDY_PIN, sms_mpu_interrupt_callback, GPIO_CALLBACK_RISING);
}

/* Unregister GPIO interrupt callback */
void sms_mpu_unregister_callbacks(void)
{
    gpio_unregister_callback(SMS_MPU_DRDY_PIN, GPIO_CALLBACK_RISING);
}

/* Enable MPU DRDY interrupt */
void sms_mpu_enable_callback(void)
{
	gpio_enable_callback(SMS_MPU_DRDY_PIN);
	mpu_device.int_enabled = true;
}

/* Disable MPU DRDY interrupt */
void sms_mpu_disable_callback(void)
{
	gpio_disable_callback(SMS_MPU_DRDY_PIN);
	mpu_device.int_enabled = false;
}

/* Callback --> send interrupt message to platform */
void sms_mpu_interrupt_callback(void)
{
	if(mpu_device.int_enabled) {
		mpu_device.new_int = true;
		send_plf_int_msg_ind(SMS_MPU_DRDY_PIN, GPIO_CALLBACK_RISING, NULL, 0);
	}
}

int sms_mpu_check(void) {
	int retVal = -1;
	uint8_t c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	if(c == 0x71) {
		DBG_LOG("[sms_mpu_check]\t\tMPU-9250 is online...");
		sms_mpu_selftest(mpu_device.hal.self_test);
		DBG_LOG("[sms_mpu_check]\t\tMPU-9250 self-test passed");
		retVal = 0;
	}
	return retVal;
}

int sms_mpu_comp_check(void)
{
	int retVal = -1;
	uint8_t d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);
	if(d == 0x48) {
		DBG_LOG("[sms_mpu_comp_check]\t\tAK8963 is online...");
		retVal = 0;
	}
}

void sms_mpu_calibrate(float *dest1, float *dest2)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0};
	int32_t accel_bias[3] = {0, 0, 0};
	
	// reset device
	DBG_LOG("Reset...");
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay_ms(100);
	DBG_LOG_CONT(" done!");
	
	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	DBG_LOG("Get time source...");
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	delay_ms(200);
	DBG_LOG_CONT(" done!");

	// Configure device for bias calculation
	DBG_LOG("Configure device...");
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay_ms(15);
	DBG_LOG_CONT(" done!");
	
	// Configure MPU9250 gyro and accelerometer for bias calculation
	DBG_LOG("Configure gyro & accel...");
	writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
	DBG_LOG_CONT(" done!");
	
	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	DBG_LOG("Configure FIFO...");
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay_ms(40); // accumulate 40 samples in 40 milliseconds = 480 bytes
	DBG_LOG_CONT(" done!");

	// At end of sample accumulation, turn off FIFO sensor read
	DBG_LOG("Turn-off FIFO & read samples...");
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
	
	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
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
	DBG_LOG_CONT(" done!");
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;
	
	if(accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t) accelsensitivity;
	}  // Remove gravity from the z-axis accelerometer bias calculation
	else {
		accel_bias[2] += (int32_t) accelsensitivity;
	}
	
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;
	
	// Push gyro biases to hardware registers
	writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
	
	// Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	
	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
	
	for(ii = 0; ii < 3; ii++) {
		if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
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
	writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

	// Output scaled accelerometer biases for display in the main program
	dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
	dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void sms_mpu_initialize(void)
{
	// Initialize variables
	q[0] = 1.0;
	q[1] = 0.0;
	q[2] = 0.0;
	q[3] = 0.0;
	eInt[0] = 0.0;
	eInt[1] = 0.0;
	eInt[2] = 0.0;
	uint8_t a_scale = AFS_2G;
	uint8_t g_scale = GFS_250DPS;

	// wake up device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	delay_ms(100); // Wait for all registers to reset

	// get stable time source
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	delay_ms(200);
	
	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	// writeByte(MPU9250_ADDRESS, CONFIG, 0x03);
	writeByte(MPU9250_ADDRESS, CONFIG, 0x06);		// gyro bandwidth = 10 Hz

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	// writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  	// Use a 200 Hz rate; a rate consistent with the filter update rate
	// // determined inset in CONFIG above
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x08);  	// Use a 111 Hz rate; a rate consistent with the filter update rate
	
	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x02; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | (g_scale << 3); // Set full scale range for the gyro
	// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
	
	// Set accelerometer full-scale range configuration
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | (a_scale << 3); // Set full scale range for the accelerometer
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	// c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	c = c | 0x06;  // Set accelerometer rate to 1 kHz and bandwidth to 10 Hz
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	delay_ms(100);
}

void sms_mpu_comp_initialize(float *destination)
{
	uint8_t m_scale = MFS_16BITS;	// Choose either 14-bit or 16-bit magnetometer resolution
	uint8_t m_mode = 0x02;	// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	delay_ms(10);
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	delay_ms(10);
	readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
	destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	delay_ms(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeByte(AK8963_ADDRESS, AK8963_CNTL, m_scale << 4 | m_mode); // Set magnetometer data resolution and sample ODR
	delay_ms(10);
}

/* Extract available IMU data */
int sms_mpu_poll_data(void)
{
	
    return 0;
}

void sms_mpu_define_services(void)
{
    at_ble_status_t status;
    uint8_t init_value = 0;
    sms_ble_service_init(BLE_SERV_MPU, &mpu_device.service_handler, &init_value);
    if((status = sms_ble_primary_service_define(&mpu_device.service_handler)) != AT_BLE_SUCCESS) {
        DBG_LOG("[sms_mpu_define_services]\tServices defining failed, reason 0x%x", status);
    }
    else {
        DBG_LOG_DEV("[sms_mpu_define_services]\tServices defined, SMS MPU handle: %d", mpu_device.service_handler.serv_handle);
    }
}

void sms_mpu_selftest(float *destination)
{
	uint8_t raw_data[6] = {0};
	uint8_t stest[6];
	int32_t g_avg[3] = {0}, a_avg[3] = {0}, a_stavg[3] = {0}, g_stavg[3] = {0};
	float factory_trim[6];
	uint8_t fs = 0;
	
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);	// Set gyro sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, CONFIG, 0x02);		// Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, fs<<3);	// Set full scale range for the gyro to 250 dps
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02);	// Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, fs<<3);	// Set full scale range for the accelerometer to 2 g
	
	for(uint8_t i = 0; i < 200; i++) { // get average current values of gyro and accelerometer
		readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, raw_data);
		a_avg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
		a_avg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
		a_avg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
		
		readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, raw_data);
		g_avg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
		g_avg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
		g_avg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
	}
	for(uint8_t i = 0; i < 3; i++) { // get average of 200 values and store as average current readings
		a_avg[i] /= 200;
		g_avg[i] /= 200;
	}
	
	// Configure the accelerometer for self-test
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set (MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	delay_ms(25);  // Delay a while to let the device stabilize
	
	for(uint8_t i = 0; i < 200; i++) {  // get average self-test values of gyro and accelerometer
		readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, raw_data);  // Read the six raw data registers into data array
		a_stavg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		a_stavg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]) ;
		a_stavg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]) ;
		
		readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, raw_data);  // Read the six raw data registers sequentially into data array
		g_stavg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		g_stavg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]) ;
		g_stavg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]) ;
	}
	for(uint8_t i = 0; i < 3; i++) { // get average of 200 values and store as average self-test readings
		a_stavg[i] /= 200;
		g_stavg[i] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
	delay_ms(25);  // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	stest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	stest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	stest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	stest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
	stest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
	stest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factory_trim[0] = (float)(2620/1<<fs)*(pow( 1.01 , ((float)stest[0] - 1.0) )); // FT[Xa] factory trim calculation
	factory_trim[1] = (float)(2620/1<<fs)*(pow( 1.01 , ((float)stest[1] - 1.0) )); // FT[Ya] factory trim calculation
	factory_trim[2] = (float)(2620/1<<fs)*(pow( 1.01 , ((float)stest[2] - 1.0) )); // FT[Za] factory trim calculation
	factory_trim[3] = (float)(2620/1<<fs)*(pow( 1.01 , ((float)stest[3] - 1.0) )); // FT[Xg] factory trim calculation
	factory_trim[4] = (float)(2620/1<<fs)*(pow( 1.01 , ((float)stest[4] - 1.0) )); // FT[Yg] factory trim calculation
	factory_trim[5] = (float)(2620/1<<fs)*(pow( 1.01 , ((float)stest[5] - 1.0) )); // FT[Zg] factory trim calculation
	
	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		destination[i]   = 100.0*((float)(a_stavg[i] - a_avg[i]))/factory_trim[i] - 100.;   // Report percent differences
		destination[i+3] = 100.0*((float)(g_stavg[i] - g_avg[i]))/factory_trim[i+3] - 100.; // Report percent differences
	}

}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	sms_i2c_master_write(address, subAddress, 1, data);
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data[1];
	sms_i2c_master_read(address, subAddress, 1, data);
	return data[0];
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	sms_i2c_master_read(address, subAddress, count, &dest);
}