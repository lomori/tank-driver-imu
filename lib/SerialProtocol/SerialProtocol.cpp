#include "SerialProtocol.h"

#include <exception>

SerialProtocol::SerialProtocol(HardwareSerial *serial)
{
    this->serial = serial;
    this->serial->setRxBufferSize(512);
    this->serial->begin(BAUD_RATE);
    reset();
    commandQueue = xQueueCreate(QUEUE_SIZE, sizeof(std::string *));
    sendQueue = xQueueCreate(QUEUE_SIZE, sizeof(std::string *));
    this->sendSemaphore = xSemaphoreCreateBinary();
}

void SerialProtocol::reset()
{
    this->serial->clearWriteError();
    this->serial->updateBaudRate(BAUD_RATE);
    this->serial->setTimeout(100);
    this->serial->flush(false);
}

void SerialProtocol::sendData(const char *msg)
{
    std::string *cp = new std::string(msg);
    try
    {
        if (xSemaphoreTake(sendSemaphore, 500) == pdPASS)
        {
            int available = uxQueueSpacesAvailable(sendQueue);
            if (available <= 0)
            {
                // Serial.printf("Available before: %s\n", uxQueueSpacesAvailable(sendQueue));
                std::string *c = nullptr;
                xQueueReceive(sendQueue, (void *)&c, (TickType_t)0);
                esp_log_write(ESP_LOG_ERROR, tag, "TXF:%s", c->c_str());
                delete c;
                // Serial.printf("Available after: %s\n", uxQueueSpacesAvailable(sendQueue));
            }
            if (xQueueSend(sendQueue, (const void *)&cp, (TickType_t)0) != pdTRUE)
            {
                esp_log_write(ESP_LOG_ERROR, tag, "Fail:%s", msg);
                delete cp;
            }
            xSemaphoreGive(sendSemaphore);
        }
        else
        {
            Serial.println("SerialProtocol: failed to acquire semaphore");
        }
    }
    catch (const std::exception &e)
    {
        // Error handling here is critical. Logs are being normally send
        // by serial line.
        Serial.printf("Exception sending data: %s\n", e.what());
    }
}

std::string *SerialProtocol::receiveData()
{
    std::string *cp = nullptr;
    xQueueReceive(commandQueue, (void *)&cp, (TickType_t)0);
    return cp;
}

void SerialProtocol::sendData()
{
    std::string *msg;

    while (xQueueReceive(sendQueue, (void *)&msg,
                         (TickType_t)0) == pdTRUE)
    {
        // esp_log_write(ESP_LOG_INFO, tag, "TX:%s", msg->c_str());
        this->serial->write('#');
        this->serial->write(msg->c_str());
        this->serial->write('$');
        this->serial->flush();
        delete msg;
    }

    this->serial->write("#$");
    this->serial->flush();
}

void SerialProtocol::processCommand(const char *cmd)
{
    if (!strcmp("stop", cmd))
    {
        // execute almost stop immediatelly
        xQueueReset(sendQueue);
        xQueueReset(commandQueue);
    }

    std::string *cp = new std::string(cmd);
    if (xQueueSend(commandQueue, (const void *)&cp, (TickType_t)0) != pdTRUE)
    {
        esp_log_write(ESP_LOG_ERROR, tag, "Fail to enqueue");
        delete cp;
    }
}

void SerialProtocol::loop(void *param)
{
    char command[128];
    int start = 0;
    int state = WAITING_START;

    SerialProtocol *self = (SerialProtocol *)param;

    while (true)
    {
        try
        {
            int c = char(self->serial->read());

            if (c == 0xff)
            {
                vTaskDelay(10);
                continue;
            }

            if (c == '&')
            {
                self->sendData();
            }
            else if (state == WAITING_START)
            {
                if (c == '#')
                {
                    state = RECEIVING_COMMAND;
                    command[0] = 0;
                    start = 0;
                }
                else
                {
                    esp_log_write(ESP_LOG_ERROR, tag, "Inv:(%c)", c);
                    self->reset();
                }
            }
            else if (state == RECEIVING_COMMAND)
            {
                if (c == '$')
                {
                    command[start] = 0;
                    if (strlen(command))
                    {
                        self->processCommand(command);
                    }
                    else
                    {
                        esp_log_write(ESP_LOG_ERROR, tag, "Empty command");
                    }
                    state = WAITING_START;
                    start = 0;
                }
                else if (start >= (sizeof(command) - 1))
                {
                    esp_log_write(ESP_LOG_ERROR, tag, "Long:(%s)", command);
                    state = WAITING_START;
                }
                else
                {
                    command[start++] = c;
                }
            }
        }
        catch (const std::exception &e)
        {
            esp_log_write(ESP_LOG_ERROR, tag, "Exp:%s", e.what());
            self->reset();
            start = 0;
            state = WAITING_START;
        }
    }
}

void SerialProtocol::start()
{
    xSemaphoreGive(sendSemaphore);
    xTaskCreate(loop, tag, 2048, this, tskIDLE_PRIORITY, &xHandle);
    configASSERT(xHandle);
}