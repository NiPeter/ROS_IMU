#include <Arduino.h>
#include <OTA.h>

#include <Timer.h>
#include <StopWatch.h>

#include <MPU9250.h>
#include <CmdMessenger.h>

#include <DEMA.h>

#include <MadgwickAHRS.h>

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <WiFiHardware.h>


#define INT_PIN D8


const char* ssid = "TP-LINK PN";
const char* password = "paiste502";

OTA ota;

Timer timer;
StopWatch sw(StopWatch::MICROS);

MPU9250 imu(true);
float faccel[3] = {0,0,0},fgyro[3]={0,0,0},fmag[3]={0,0,0};
float Qt[4] = {0,0,0,0};


#define MAG_ALPHA 0.05f
DEMAFilter MagFilter[3] = {
        DEMAFilter(MAG_ALPHA,0),
        DEMAFilter(MAG_ALPHA,0),
        DEMAFilter(MAG_ALPHA,0)
};

#define ACCEL_ALPHA 0.1f
DEMAFilter AccelFilter[3] = {
        DEMAFilter(ACCEL_ALPHA,0),
        DEMAFilter(ACCEL_ALPHA,0),
        DEMAFilter(ACCEL_ALPHA,0)
};

#define GYRO_ALPHA 1.0f
DEMAFilter GyroFilter[3] = {
        DEMAFilter(GYRO_ALPHA,0),
        DEMAFilter(GYRO_ALPHA,0),
        DEMAFilter(GYRO_ALPHA,0)
};


IPAddress rosServer(192,168,1,3);
ros::NodeHandle_<WiFiHardware> nh;

sensor_msgs::Imu msgImu;
sensor_msgs::MagneticField msgMagneticField;

ros::Publisher pubImu("imu/data_raw", &msgImu);
ros::Publisher pubMag("imu/mag", &msgMagneticField);


void SerialReader(void);
void printImuData(void);
void quaternionTask(void);
void rosInit(void);
void rosTask(void);

void SerialReader(){
        if(Serial.available()) {
                String command = Serial.readString();
                command.toLowerCase();
                float param = command.substring(1).toFloat();
                switch(command[0]) {
                case 'b': {
                        beta = param;
                        Serial.print("Beta"); Serial.println(beta,6);
                        break;
                }

                case 'a': {
                        for (size_t i = 0; i < 3; i++) {
                                AccelFilter[i].setAlpha(param);
                                AccelFilter[i].setOutput(AccelFilter[i].getOutput());
                        }
                        Serial.print("Acc filter alpha"); Serial.println(AccelFilter[1].getAlpha(),6);
                        break;
                }
                case 'g': {
                        for (size_t i = 0; i < 3; i++) {
                                GyroFilter[i].setAlpha(param);
                                GyroFilter[i].setOutput(GyroFilter[i].getOutput());
                        }
                        Serial.print("Gyro filter alpha"); Serial.println(GyroFilter[1].getAlpha(),6);
                        break;
                }
                case 'm': {
                        for (size_t i = 0; i < 3; i++) {
                                MagFilter[i].setAlpha(param);
                                MagFilter[i].setOutput(MagFilter[i].getOutput());
                        }
                        Serial.print("Mag filter alpha"); Serial.println(MagFilter[1].getAlpha(),6);
                        break;
                }
                default:
                        break;
                }

        }
}


void setup() {
        Serial.begin(115200);
        Serial.println();

        ota.begin(ssid,password);
        Serial.println();

        imu.begin(INT_PIN);
        Serial.println();

        rosInit();
        Serial.println();

        timer.every(20,rosTask);
        timer.every(500,SerialReader);


}

void loop() {

        if(imu.isNewData()) {


                {
                        imu.updateSensors();
                        for(uint8_t i=0; i<3; i++) {
                                float mag = (float)(imu.getRawMag()[i])*imu.magCalibration[i];
                                mag -= imu.magBias[i];
                                mag *= imu.magScale[i];
                                fmag[i]=MagFilter[i].filter(mag);

                                float acc = imu.getAccel()[i];
                                faccel[i] = AccelFilter[i].filter(acc);

                                float gyro = imu.getGyro()[i];
                                fgyro[i] = GyroFilter[i].filter(gyro);

                        }

                        // // Swap axes
                        float mag_temp = fmag[0];
                        fmag[0] = fmag[1];
                        fmag[1] = mag_temp;
                        fmag[2] = -fmag[2];

                        quaternionTask();
                }
        }
        timer.update();
        ota.handle();
}

void quaternionTask(){

        static unsigned long pTime = 0;
        unsigned long aTime = micros();
        float dT = float(aTime-pTime)/1000000.0f;
        pTime = aTime;


        MadgwickAHRSupdate(fgyro[0], fgyro[1], fgyro[2], faccel[0], faccel[1], faccel[2], fmag[0], fmag[1], fmag[2], dT);
        // MadgwickAHRSupdate(fgyro[0], fgyro[1], fgyro[2], faccel[0], faccel[1], faccel[2], fmag[0], fmag[1], fmag[2], dT);

        Qt[0]=q0;
        Qt[1]=q1;
        Qt[2]=q2;
        Qt[3]=q3;

}

void rosInit(){
        Serial.println("ROS node setup...");
        nh.getHardware()->setConnection(rosServer);
        nh.initNode();
        nh.advertise(pubImu);
        nh.advertise(pubMag);
        Serial.println("Done");
}

void rosTask(void){

        msgMagneticField.header.stamp = msgImu.header.stamp = nh.now();
        msgMagneticField.header.frame_id = msgImu.header.frame_id = "/base_link";

        // Quaternion
        msgImu.orientation.w = Qt[0];
        msgImu.orientation.x = Qt[1];
        msgImu.orientation.y = Qt[2];
        msgImu.orientation.z = Qt[3];

        // rad per second
        msgImu.angular_velocity.x = fgyro[0];
        msgImu.angular_velocity.y = fgyro[1];
        msgImu.angular_velocity.z = fgyro[2];

        // G -  9.806m/s^2
        msgImu.linear_acceleration.x = faccel[0];
        msgImu.linear_acceleration.y = faccel[1];
        msgImu.linear_acceleration.z = faccel[2];

        msgMagneticField.magnetic_field.x = fmag[0];
        msgMagneticField.magnetic_field.y = fmag[1];
        msgMagneticField.magnetic_field.z = fmag[2];

        pubImu.publish(&msgImu);
        pubMag.publish(&msgMagneticField);

        nh.spinOnce();
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
