#pragma once

#include <Arduino.h>

class CJY901
{
public:
  CJY901(unsigned char ucAddr = 0x50);

  /* ------------ (JY901 --> Host) functions ------------ */
  void getTime();
  void getAcc();
  void getGyro();
  void getAngles();
  void getMag();
  void getDStatus();
  void getPress();
  void getLonLat();
  void getGPSV();
  void getQuaternions();

  void set6AxisAlgo();
  void set9AxisAlgo();

  /* ------------ Calibration functions ------------ */
  void autoCalibrateGyroAccel();
  void autoCalibrateMagnetic();
  void resetHeight();
  void calibrateAll();

  /* ------------ (Host --> JY901) functions ------------ */
  void saveConf(int); // save configuration
  void setIICaddr(int);
  void turnLED(int);
  /*
    void setCali(int);      // calibration mode
    void setDir(int);       // set install direction
    void enterHiber();      // enter hibernation or wake
    void changeALG(int);    // change algorithm
    void autoCaliGyro(int); // enable auto gyro calibration
    void confReport();      // configure report contents
    void setReportRate(int);
    void setBaudRate(int);

    void setAXoffset();
    void setAYoffset();
    void setAZoffset();

    void setGXoffset();
    void setGYoffset();
    void setGZoffset();

    void setHXoffset();
    void setHYoffset();
    void setHZoffset();

    void setD0mode(int);
    void setD1mode(int);
    void setD2mode(int);
    void setD3mode(int);

    void setD0PWMH();
    void setD1PWMH();
    void setD2PWMH();
    void setD3PWMH();

    void setD0PWMT();
    void setD1PWMT();
    void setD2PWMT();
    void setD3PWMT();

    void setGPSrate(int);
  */
  struct
  {
    struct
    {
      uint8_t confl;
      uint8_t confh;
    } report;

    struct
    {
      int8_t xl;
      int8_t xh;
      int8_t yl;
      int8_t yh;
      int8_t zl;
      int8_t zh;
    } aoffset;

    struct
    {
      int8_t xl;
      int8_t xh;
      int8_t yl;
      int8_t yh;
      int8_t zl;
      int8_t zh;
    } goffset;

    struct
    {
      int8_t xl;
      int8_t xh;
      int8_t yl;
      int8_t yh;
      int8_t zl;
      int8_t zh;
    } hoffset;

    struct
    {
      uint8_t d0l;
      uint8_t d0h;
      uint8_t d1l;
      uint8_t d1h;
      uint8_t d2l;
      uint8_t d2h;
      uint8_t d3l;
      uint8_t d3h;
    } pwmh;

    struct
    {
      uint8_t d0l;
      uint8_t d0h;
      uint8_t d1l;
      uint8_t d1h;
      uint8_t d2l;
      uint8_t d2h;
      uint8_t d3l;
      uint8_t d3h;
    } pwmt;

  } ctrl;

  struct
  {
    struct
    {
      uint8_t year;
      uint8_t month;
      uint8_t day;
      uint8_t hour;
      uint8_t minute;
      uint8_t second;
      uint16_t milisecond;
    } time;

    struct
    {
      int16_t x;
      int16_t y;
      int16_t z;
    } acc;

    struct
    {
      int16_t x;
      int16_t y;
      int16_t z;
    } gyro;

    struct
    {
      int16_t roll;
      int16_t pitch;
      int16_t yaw;
    } angle;

    struct
    {
      int16_t x;
      int16_t y;
      int16_t z;
    } mag;

    struct
    {
      int16_t d0;
      int16_t d1;
      int16_t d2;
      int16_t d3;
    } dStatus;

    int32_t pressure;
    int32_t altitude; // JY-901B

    int32_t lon;
    int32_t lat;
    int16_t GPSHeight;
    int16_t GPSYaw;
    int32_t GPSVelocity;

    struct
    {
      int16_t q0;
      int16_t q1;
      int16_t q2;
      int16_t q3;
    } quaternions;

    struct
    { // DOP stands for Dilution of Precision
      int16_t sn;
      int16_t pdop;
      int16_t hdop;
      int16_t vdop;
    } GPS_DOP;

  } data;

private:
  unsigned char ucDevAddr;
  void readRegisters(unsigned char deviceAddr, unsigned char addressToRead, unsigned char bytesToRead, char *dest);
  void writeRegister(unsigned char deviceAddr, unsigned char addressToWrite, unsigned char bytesToRead, char *dataToWrite);
};

extern CJY901 JY901;
