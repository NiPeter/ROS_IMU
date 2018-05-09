#include <Arduino.h>
#include <OTA.h>

#include <Timer.h>
#include <StopWatch.h>

#include <MPU9250.h>
#include <CmdMessenger.h>

#include <DEMA.h>

#define INT_PIN D8

OTA ota;
Timer timer;
StopWatch watch(StopWatch::MICROS);

MPU9250 imu(true);
float magData[3];

#define MAG_ALPHA 0.11f
DEMAFilter MagFilter[3] = {
        DEMAFilter(MAG_ALPHA,0),
        DEMAFilter(MAG_ALPHA,0),
        DEMAFilter(MAG_ALPHA,0)
};

enum {
        get_mag,
        get_accel,
        get_gyro,
        error,
};
CmdMessenger c = CmdMessenger(Serial,',',';','/');


float elapsedTime,elapsedTimeSum;
float i = 1;


void printImuData(void);

/* Create callback functions to deal with incoming messages */

/* callback */
void sendMagData(void){
        c.sendCmdStart(get_mag);
        c.sendCmdBinArg(magData[0]);
        c.sendCmdBinArg(magData[1]);
        c.sendCmdBinArg(magData[2]);
        c.sendCmdEnd();
}

/* callback */
void on_error(void){
        c.sendCmd(error,"Command without callback.");
}

/* Attach callbacks for CmdMessenger commands */
void attach_callbacks(void) {

        c.attach(get_mag,sendMagData);
        c.attach(on_error);
}

void setup() {

        ota.begin((const char*) "TP-LINK PN",(const char*)"paiste502");
        Serial.println();

        imu.begin(INT_PIN);
        Serial.println();

        Serial.begin(115200);
        Serial.println();

        attach_callbacks();

}

void loop() {

        if(imu.isNewData()) {
                imu.updateSensors();
                for(uint8_t i=0; i<3; i++) {
                        float data = (float)(imu.getRawMag()[i])*imu.magCalibration[i];
                        data -= imu.magBias[i];
                        data *= imu.magScale[i];
                        magData[i]=MagFilter[i].filter(data);
                }
        }

        timer.update();
        c.feedinSerialData();
        ota.handle();
}

void printImuData(void){

        static auto printFun = [] (String header,const int16_t* data, String unit, uint8_t len){
                Serial.println(header);
                for (size_t i = 0; i < len; i++) {
                        Serial.print(data[i]);
                        Serial.print(", ");
                }
                Serial.println(" "+unit);
        };

        printFun("Accel :",imu.getRawAccel(),"",3);
        printFun("Gyro :",imu.getRawGyro(),"",3);
        printFun("Mag :",imu.getRawMag(),"",3);
        Serial.println("Time: " + String(millis())+ " ms");

        Serial.println();
}
