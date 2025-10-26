#pragma once
#include "UltraSonicSensor.hpp"
#include "ServerClient.hpp"

class SensorExecute{
  private:
    UltraSonicSensor mySensor;
    ServerClient server;
    const char* ssid;
    const char* password;
    byte actualInterval;

    // Mapea distancia (cm) a intervalos 1..3 (1=0-29, 2=30-59, 3=60+)
    static byte mapInterval(int cm) {
        if (cm < 0) return 0;            // error de medición
        if (cm < 30) return 1;
        if (cm < 60) return 2;
        return 3;
    }

  public:
    SensorExecute(uint16_t SENSOR_ID,
                  uint8_t triggerPin, uint8_t echoPin,
                  const char* serverIP, uint16_t serverPort,
                  const char* ssid, const char* password)
    : mySensor(triggerPin, echoPin),
      server(serverIP, serverPort),
      ssid(ssid),
      password(password),
      actualInterval(0)
    {
      mySensor.setSensorId(SENSOR_ID);
    }

void setup(){
    Serial.begin(115200);
    delay(100);
    Serial.println("Sensor Controller Initialized");

    server.wifiConnect(ssid, password);

    uint8_t intervalToSend[2] = { actualInterval, 0 }; // inicial (0)
    for (int i = 0; i < 3; ++i) {
        if (server.connectToServer()) {
            if (server.post(ServerClient::Resource::RES_SENSOR, mySensor.getSensorId(), intervalToSend, 1)) {
                int st = server.receiveStatus();
                Serial.print("POST init status: 0x"); Serial.println(st, HEX);
                server.disconnectFromServer();
                break;
            }
            server.disconnectFromServer();
        }
        delay(200);
    }
}

void loop(){
    int cm = mySensor.getDistanceCm();
    byte interval = (cm >= 0) ? (cm/30)+1 : 0;  // 0 si error
    if (interval > 3) interval = 3;

    uint8_t intervalToSend[2];  // <<-- aquí
    intervalToSend[1] = 0;

    Serial.print("Raw Distance (cm): "); Serial.println(cm);
    Serial.print("Interval: "); Serial.println(interval);

    if (cm >= 0) {
        if (actualInterval != interval) {
            intervalToSend[0] = interval;
            if (server.connectToServer()){
                if (server.update(ServerClient::Resource::RES_SENSOR, mySensor.getSensorId(), intervalToSend, 1)) {
                    int status = server.receiveStatus();
                    Serial.print("Update Sensor Status: 0x"); Serial.println(status, HEX);
                    actualInterval = interval; // actualizar sólo si fue OK está bien también
                }
                server.disconnectFromServer();
            }
        }
    } else {
        // error: reenvía último valor o 0
        intervalToSend[0] = actualInterval; // o 0 si prefieres marcar error
        if (server.connectToServer()){
            if (server.update(ServerClient::Resource::RES_SENSOR, mySensor.getSensorId(), intervalToSend, 1)) {
                int status = server.receiveStatus();
                Serial.print("Update Sensor Status (retry): 0x"); Serial.println(status, HEX);
            }
            server.disconnectFromServer();
        }
    }
    delay(100);
}
};