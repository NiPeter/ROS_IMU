#include "MPU9250.h"

//==============================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer,
//====== and temperature data
//==============================================================================

volatile bool MPU9250::_isNewData = false;

void MPU9250::updateSensors(){
        if(_isNewData) {
                _isNewData = false;
                readRawAccelData(accelCount);
                readRawGyroData(gyroCount);
                readRawMagData(magCount);

                // Scale values and add biases
                for(uint8_t i=0; i<3; i++) {
                        gyro[i]= (float)(gyroCount[i]-gyroBias[i])*gRes*DEG_TO_RAD; // rad/s
                        accel[i] = (float)(accelCount[i]-accelBias[i])*aRes; // G

                        mag[i] = (float)(magCount[i])*mRes*magCalibration[i] -magBias[i];   // mG
                        mag[i] *= magScale[i];

                }
                // // Swap axes
                float mag_temp = mag[0];
                mag[0] = mag[1];
                mag[1] = mag_temp;
                mag[2] = -mag[2];
        }
}

void MPU9250::begin(uint8_t int_pin){

        Wire.begin();
        Wire.setClock(1000000L);
        pinMode(int_pin,INPUT);
        turnOff();
        if( I2Cscan() ) {
                Serial.println();
                if(MPU9250CommunicationTest()) {
                        Serial.println();
                        MPU9250SelfTest();
                        Serial.println();
                        initMPU9250();
                        Serial.println("MPU9250 initialized for active data mode");
                        Serial.println();
                }
                if(AK8963CommunicationTest()) {
                        Serial.println();
                        initAK8963(magCalibration);
                        Serial.println("AK8963 initialized for active data mode");
                        Serial.println();
                }

                // From python_messenger calibration
                magBias[0]=86.01536560058594f;
                magBias[1]=412.04066467285156f;
                magBias[2]=-115.7061767578125f;

                magScale[0] = 1.0033856583885659f;
                magScale[1] = 0.9758402035296071f;
                magScale[2] = 1.0218509636051605f;


                // imu.gyroBias[0] = 67;
                // imu.gyroBias[1] = 390;
                // imu.gyroBias[2] = 32;
                gyroBias[0] = 14;
                gyroBias[1] = 99;
                gyroBias[2] = 1;

                // imu.accelBias[0] = 3042;
                // imu.accelBias[1] = 1438;
                // imu.accelBias[2] = -1026;
                accelBias[0] = 2979;
                accelBias[1] = 1366;
                accelBias[2] = -996;


        }
        attachInterrupt(int_pin, MPU9250::ImuInterrupt, RISING); // define interrupt for INT pin output of MPU9250
        calibrateMPU9250();
}

void MPU9250::getMres() {
        switch (Mscale)
        {
        // Possible magnetometer scales (and their register bit settings) are:
        // 14 bit resolution (0) and 16 bit resolution (1)
        case MFS_14BITS:
                mRes = 10.*4912./8190.; // Proper scale to return milliGauss
                break;
        case MFS_16BITS:
                mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
                break;
        }
}

void MPU9250::getGres() {
        switch (Gscale)
        {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case GFS_250DPS:
                gRes = 250.0/32768.0;
                break;
        case GFS_500DPS:
                gRes = 500.0/32768.0;
                break;
        case GFS_1000DPS:
                gRes = 1000.0/32768.0;
                break;
        case GFS_2000DPS:
                gRes = 2000.0/32768.0;
                break;
        }
}

void MPU9250::getAres() {
        switch (Ascale)
        {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case AFS_2G:
                aRes = 2000.0/32768.0;
                break;
        case AFS_4G:
                aRes = 4000.0/32768.0;
                break;
        case AFS_8G:
                aRes = 8000.0/32768.0;
                break;
        case AFS_16G:
                aRes = 16000.0/32768.0;
                break;
        }
}


void MPU9250::readRawAccelData(int16_t * destination)
{
        uint8_t rawData[6]; // x/y/z accel register data stored here
        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
        destination[0] = ((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
        destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}


void MPU9250::readRawGyroData(int16_t * destination)
{
        uint8_t rawData[6]; // x/y/z gyro register data stored here
        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
        destination[0] = ((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
        destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

void MPU9250::readRawMagData(int16_t * destination)
{
        // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of
        // data acquisition
        uint8_t rawData[7];
        // Wait for magnetometer data ready bit to be set
        if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)
        {
                // Read the six raw data and ST2 registers sequentially into data array
                readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);
                uint8_t c = rawData[6]; // End data read by reading ST2 register
                // Check if magnetic sensor overflow set, if not then report data
                if(!(c & 0x08))
                {
                        // Turn the MSB and LSB into a signed 16-bit value
                        destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
                        // Data stored as little Endian
                        destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
                        destination[2] =(((int16_t)rawData[5] << 8) | rawData[4]);
                }
        }
}

int16_t MPU9250::readTempData()
{
        uint8_t rawData[2]; // x/y/z gyro register data stored here
        readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]); // Read the two raw data registers sequentially into data array
        return ((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a 16-bit value
}

void MPU9250::initAK8963(float * destination)
{
        // First extract the factory calibration for each magnetometer axis
        uint8_t rawData[3]; // x/y/z gyro calibration data stored here
        writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
        delay(10);
        writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
        delay(10);
        readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]); // Read the x-, y-, and z-axis calibration values
        destination[0] =  (float)(rawData[0] - 128)/256. + 1.; // Return x-axis sensitivity adjustment values, etc.
        destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
        destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
        writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
        delay(10);
        // Configure the magnetometer for continuous read and highest resolution
        // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
        // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
        writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
        delay(10);
}

void MPU9250::turnOff(){
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, (0x01<<7)); // Reset internal registers
        delay(200); // Wait for all registers to reset


        writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
        delay(10);
}

void MPU9250::initMPU9250()
{

        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, (0x01<<7)); // Reset internal registers
        delay(200); // Wait for all registers to reset

        // wake up device
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
        delay(200); // Wait for all registers to reset


        // get stable time source
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
        delay(200);

        // Configure Gyro and Thermometer
        // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
        // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
        // be higher than 1 / 0.0059 = 170 Hz
        // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
        // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
        writeByte(MPU9250_ADDRESS, CONFIG, 0x03);
        // writeByte(MPU9250_ADDRESS, CONFIG, 0x06); // 1/0.00097 ~ 1kHz

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate
        // writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00); // Use a 1000 Hz rate; a rate consistent with the filter update rate

        // determined inset in CONFIG above

        // // determined inset in CONFIG above

        // Set gyroscope full scale range
        // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
        uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
        // c = c & ~0xE0; // Clear self-test bits [7:5]
        c = c & ~0x02; // Clear Fchoice bits [1:0]
        c = c & ~0x18; // Clear AFS bits [4:3]
        c = c | Gscale << 3; // Set full scale range for the gyro
        //c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

        // Set accelerometer full-scale range configuration
        c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
        // c = c & ~0xE0; // Clear self-test bits [7:5]
        c = c & ~0x18; // Clear AFS bits [4:3]
        c = c | Ascale << 3; // Set full scale range for the accelerometer
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

        // Set accelerometer sample rate configuration
        // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
        // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
        c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
        c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
        c = c | 0x03; // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
        // c = c | 0x06;         // Set accelerometer rate to 1 kHz and bandwidth to 5.05 Hz
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
        // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
        // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
        // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
        // can join the I2C bus and all can be controlled by the Arduino as master
        writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x12);
        writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
        delay(100);
}


void MPU9250::calibrateMPU9250(){
        if(_debug) {
                Serial.println("MPU9250 gyro and accel calibration...");
                delay(500);
        }


        // float * dest1 =  gyroBias;
        // float * dest2 =  accelBias;
        uint16_t sample_count = 0;
        int32_t accel_temp[3] = {0,0,0}, gyro_temp[3] = {0,0,0};

        uint16_t accelsensitivity = 1000.0f/aRes; // = 16384 LSB/g

        while(sample_count<1000) {
                if( _isNewData) {
                        _isNewData =false;
                        sample_count++;
                        readRawAccelData(accelCount);
                        readRawGyroData(gyroCount);
                        accel_temp[0] += (int32_t) accelCount[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
                        accel_temp[1] += (int32_t) accelCount[1];
                        accel_temp[2] += (int32_t) accelCount[2];
                        gyro_temp[0]  += (int32_t) gyroCount[0];
                        gyro_temp[1]  += (int32_t) gyroCount[1];
                        gyro_temp[2]  += (int32_t) gyroCount[2];
                }
        }

        accel_temp[0] /= (int32_t) sample_count; // Normalize sums to get average count biases
        accel_temp[1] /= (int32_t) sample_count;
        accel_temp[2] /= (int32_t) sample_count;
        gyro_temp[0]  /= (int32_t) sample_count;
        gyro_temp[1]  /= (int32_t) sample_count;
        gyro_temp[2]  /= (int32_t) sample_count;

        if(accel_temp[2] > 0L) {accel_temp[2] -= (int32_t) accelsensitivity; } // Remove gravity from the z-axis accelerometer bias calculation
        else {accel_temp[2] += (int32_t) accelsensitivity; }

        gyroBias[0] = (int16_t) gyro_temp[0]; ///(float) gyrosensitivity;
        gyroBias[1] = (int16_t) gyro_temp[1]; ///(float) gyrosensitivity;
        gyroBias[2] = (int16_t) gyro_temp[2]; ///(float) gyrosensitivity;

        accelBias[0] = (int16_t)accel_temp[0]; ///(float)accelsensitivity;
        accelBias[1] = (int16_t)accel_temp[1]; ///(float)accelsensitivity;
        accelBias[2] = (int16_t)accel_temp[2]; ///(float)accelsensitivity;


        if(_debug) {
                Serial.println("x-axis accelerometer bias:" + String(accelBias[0])+"");
                Serial.println("y-axis accelerometer bias:" + String(accelBias[1])+"");
                Serial.println("z-axis accelerometer bias:" + String(accelBias[2])+"");
                Serial.println("x-axis gyroscope bias:" + String(gyroBias[0]) + "");
                Serial.println("y-axis gyroscope bias:" + String(gyroBias[1]) + "");
                Serial.println("z-axis gyroscope bias:" + String(gyroBias[2]) + "");
                Serial.println("Done");
        }
}

void MPU9250::calibrateAK9863(){
        float * dest1 = magBias;
        float * dest2 = magScale;

        uint16_t ii = 0, sample_count = 0;
        int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
        int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

        if(_debug) {
                Serial.println("AK8963 calibration...");
                Serial.println("Wave device in a figure eight until done...");
                delay(1000);
        }


        // shoot for ~fifteen seconds of mag data
        if( Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
        if( Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
        for(ii = 0; ii < sample_count; ii++) {
                readRawMagData(mag_temp);   // Read the mag data
                for (int jj = 0; jj < 3; jj++) {
                        if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
                        if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
                }
                if( Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
                if( Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
        }


        // Get hard iron correction
        mag_bias[0]  = (mag_max[0] + mag_min[0])/2; // get average x mag bias in counts
        mag_bias[1]  = (mag_max[1] + mag_min[1])/2; // get average y mag bias in counts
        mag_bias[2]  = (mag_max[2] + mag_min[2])/2; // get average z mag bias in counts

        dest1[0] = (float) mag_bias[0]* mRes* magCalibration[0]; // save mag biases in G for main program
        dest1[1] = (float) mag_bias[1]* mRes* magCalibration[1];
        dest1[2] = (float) mag_bias[2]* mRes* magCalibration[2];


        // Get soft iron correction estimate
        mag_scale[0]  = (mag_max[0] - mag_min[0])/2; // get average x axis max chord length in counts
        mag_scale[1]  = (mag_max[1] - mag_min[1])/2; // get average y axis max chord length in counts
        mag_scale[2]  = (mag_max[2] - mag_min[2])/2; // get average z axis max chord length in counts

        float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
        avg_rad /= 3.0;

        dest2[0] = avg_rad/((float)mag_scale[0]);
        dest2[1] = avg_rad/((float)mag_scale[1]);
        dest2[2] = avg_rad/((float)mag_scale[2]);

        if(_debug) {
                Serial.print("x-axis magnetometer bias:");
                Serial.println(magBias[0],6);
                Serial.print("y-axis magnetometer bias:");
                Serial.println(magBias[1],6);
                Serial.print("z-axis magnetometer bias:");
                Serial.println(magBias[2],6);
                Serial.print("x-axis magnetometer scale:");
                Serial.println(magScale[0],6);
                Serial.print("y-axis magnetometer scale:");
                Serial.println(magScale[1],6);
                Serial.print("z-axis magnetometer scale:");
                Serial.println(magScale[2],6);
                Serial.println("Done");
        }

}

bool MPU9250::MPU9250CommunicationTest(void){
        byte c =  readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250); // Read WHO_AM_I register for MPU-9250

        if(_debug) {
                // Read the WHO_AM_I register, this is a good test of communication
                Serial.println("MPU9250 9-axis motion sensor...");
                Serial.print("MPU9250 "); Serial.print("I AM ");
                Serial.print(c, HEX);
                Serial.print(" I should be ");
                Serial.println(0x71, HEX);
        }
        return (c == 0x71);
}

bool MPU9250::AK8963CommunicationTest(void){
        byte d =  readByte(AK8963_ADDRESS, WHO_AM_I_AK8963); // Read WHO_AM_I register for AK8963

        if(_debug) {
                Serial.println("AK8963 3-axis magnetometer...");
                Serial.print("AK8963 "); Serial.print("I AM ");
                Serial.print(d, HEX); Serial.print(" I should be ");
                Serial.println(0x48, HEX);

        }
        return (d == 0x48);
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::MPU9250SelfTest(float * destination){ // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass

        uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
        uint8_t selfTest[6];
        int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
        float factoryTrim[6];
        uint8_t FS = 0;

        writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
        writeByte(MPU9250_ADDRESS, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1<<FS); // Set full scale range for the gyro to 250 dps
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

        for( int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

                readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
                aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
                aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
                aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

                readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
                gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
                gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
                gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
        }

        for (int ii =0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
                aAvg[ii] /= 200;
                gAvg[ii] /= 200;
        }

        // Configure the accelerometer for self-test
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
        delay(25); // Delay a while to let the device stabilize

        for( int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

                readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
                aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
                aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
                aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

                readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
                gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
                gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
                gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
        }

        for (int ii =0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
                aSTAvg[ii] /= 200;
                gSTAvg[ii] /= 200;
        }

        // Configure the gyro and accelerometer for normal operation
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
        delay(25); // Delay a while to let the device stabilize

        // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
        selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
        selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
        selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
        selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO); // X-axis gyro self-test results
        selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
        selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

        // Retrieve factory self-test value from self-test code reads
        factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01, ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
        factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01, ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
        factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01, ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
        factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01, ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
        factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01, ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
        factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01, ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

        // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
        // To get percent, must multiply by 100
        for (int i = 0; i < 3; i++) {
                destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i]; // Report percent differences
                destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
        }

}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::MPU9250SelfTest(void) {

        float SelfTest[6];
        MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values

        Serial.println("MPU9250 self test...");
        Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
        Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
        Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
        Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
        Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
        Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
        Serial.println("Done");


}

void MPU9250::ImuInterrupt(void){
        _isNewData = true;
}

bool MPU9250::I2Cscan(void){
        // scan for i2c devices
        byte error, address;
        int nDevices;

        Serial.println("Scanning...");

        nDevices = 0;
        for(address = 1; address < 127; address++ )
        {
                // The i2c_scanner uses the return value of
                // the Write.endTransmisstion to see if
                // a device did acknowledge to the address.
                Wire.beginTransmission(address);
                error = Wire.endTransmission();

                if (error == 0)
                {
                        Serial.print("I2C device found at address 0x");
                        if (address<16)
                                Serial.print("0");
                        Serial.print(address,HEX);
                        Serial.println("  !");

                        nDevices++;
                }
                else if (error==4)
                {
                        Serial.print("Unknow error at address 0x");
                        if (address<16)
                                Serial.print("0");
                        Serial.println(address,HEX);
                }
        }
        if (nDevices == 0) {
                Serial.println("No I2C devices found\n");
                return false;
        }
        else{
                Serial.println("Done");
                return true;
        }
}

// Wire.h read and write protocols
void MPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
        Wire.beginTransmission(address); // Initialize the Tx buffer
        Wire.write(subAddress);     // Put slave register address in Tx buffer
        Wire.write(data);           // Put data in Tx buffer
        Wire.endTransmission();     // Send the Tx buffer
}

uint8_t MPU9250::readByte(uint8_t address, uint8_t subAddress)
{
        uint8_t data; // `data` will store the register data
        Wire.beginTransmission(address);   // Initialize the Tx buffer
        Wire.write(subAddress);            // Put slave register address in Tx buffer
        Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
        Wire.requestFrom(address, (uint8_t) 1); // Read one byte from slave register address
        data = Wire.read();                // Fill Rx buffer with result
        return data;                       // Return data read from slave register
}

void MPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count,
                        uint8_t * dest)
{
        Wire.beginTransmission(address); // Initialize the Tx buffer
        Wire.write(subAddress);      // Put slave register address in Tx buffer
        Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
        uint8_t i = 0;
        Wire.requestFrom(address, count); // Read bytes from slave register address
        while (Wire.available()) {
                dest[i++] = Wire.read();
        }                              // Put read results in the Rx buffer
}
