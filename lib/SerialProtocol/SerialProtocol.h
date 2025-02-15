#pragma once

#include <Arduino.h>

#include <string>

class SerialProtocol
{
public:
    SerialProtocol(HardwareSerial *serial);
    void start();
    void stop()
    {
        if (xHandle != NULL)
        {
            vTaskDelete(xHandle);
        }
    }
    void sendData(const char* msg);
    std::string* receiveData();
    void clear() { xQueueReset(sendQueue); }

private:
    static constexpr const char *tag = "SerialProtocol";
    static const unsigned long BAUD_RATE = 115200;
    const static int WAITING_START = 0;
    const static int RECEIVING_COMMAND = 1;
    const static int QUEUE_SIZE = 128;
    HardwareSerial *serial;
    TaskHandle_t xHandle = NULL;
    QueueHandle_t commandQueue;
    QueueHandle_t sendQueue;
    SemaphoreHandle_t sendSemaphore;
    static void loop(void *param);
    void sendData();
    void reset();
    void processCommand(const char *cmd);
};
