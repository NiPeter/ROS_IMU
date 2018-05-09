#ifndef WIFI_HARDWARE_H
#define WIFI_HARDWARE_H

#include <ros.h>
#include <ESP8266WiFi.h>

class WiFiHardware {

private:
IPAddress _server;   // ip of your ROS server
uint16_t _serverPort;
WiFiClient _client;

public:
WiFiHardware() {
};

void init() {
        // do your initialization here. this probably includes TCP server/client setup
        _client.connect(_server, _serverPort);
}

// read a byte from the serial port. -1 = failure
int read() {
        // implement this method so that it reads a byte from the TCP connection and returns it
        //  you may return -1 is there is an error; for example if the TCP connection is not open
        return _client.read();     //will return -1 when it will works
}

// write data to the connection to ROS
void write(uint8_t* data, int length) {
        // implement this so that it takes the arguments and writes or prints them to the TCP connection
        for(int i=0; i<length; i++)
                _client.write(data[i]);
}

// returns milliseconds since start of program
unsigned long time() {
        return millis(); // easy; did this one for you
}

void setConnection(IPAddress &server, int port = 11411)
{
        _server = server;
        _serverPort = port;
}

IPAddress getLocalIP()
{
        return WiFi.localIP();
}

};

wl_status_t setupWiFi(const char* ssid,const char* password)
{
        WiFi.begin(ssid, password);
        Serial.print("\nConnecting to "); Serial.println(ssid);
        delay(2000);

        wl_status_t status = WiFi.status();
        if (status != WL_CONNECTED) {
                Serial.print("Could not connect to "); Serial.println(ssid);
        } else{
                Serial.print("Ready! Use ");
                Serial.print(WiFi.localIP());
                Serial.println(" to access client");
        }
        return status;
}


#endif /* end of include guard: WIFI_HARDWARE_H */
