
#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include "JY901DRV.h"
#include "JY901Control.h"

CJY901::CJY901(unsigned char ucAddr)
{
  ucDevAddr = ucAddr;
}

void CJY901::readRegisters(unsigned char deviceAddr, unsigned char addressToRead, unsigned char bytesToRead, char *dest)
{
  /*
    Wire.beginTransmission(deviceAddr);
    Wire.write(addressToRead);
    Wire.endTransmission(false); //endTransmission but keep the connection active

    Wire.requestFrom(deviceAddr, bytesToRead); //Ask for bytes, once done, bus is released by default

    while(Wire.available() < bytesToRead); //Hang out until we get the # of bytes we expect

    for(int x = 0 ; x < bytesToRead ; x++)
    dest[x] = Wire.read();
  */
  Wire.beginTransmission(deviceAddr);
  Wire.write(addressToRead);
  Wire.endTransmission(false); // endTransmission but keep the connection active

  delay(10);

  Wire.requestFrom(deviceAddr, bytesToRead); // Ask for bytes, once done, bus is released by default

  unsigned char bytesRead = 0;
  for (int i = 0; i < 5; i++)
  {
    while (Wire.available())
    {
      dest[bytesRead++] = Wire.read();
      if (bytesRead >= bytesToRead)
      {
        break;
      }
    }
    if (bytesRead >= bytesToRead)
    {
      break;
    }
    else
    {
      log_w("Warning!!!!!");
      Wire.requestFrom((int)deviceAddr, (int)(bytesToRead - bytesRead));
      delay(10);
    }
  }

  if (bytesRead < bytesToRead)
  {
    log_e("Missing %d bytes out of %d", bytesToRead - bytesRead, bytesToRead);
  }

  /*
  {
    for(int j=0;j<bytesRead;j++)
    {
      Serial.printf("0x%02x,",dest[j]);
    }
    Serial.printf("\n");
  }
  */
}

void CJY901::writeRegister(unsigned char deviceAddr, unsigned char addressToWrite, unsigned char bytesToWrite, char *dataToWrite)
{
  Wire.beginTransmission(deviceAddr);
  Wire.write(addressToWrite);
  Wire.write((const uint8_t *)dataToWrite, bytesToWrite);
  // for (int i = 0; i < bytesToWrite; i++)
  //	Wire.write(dataToWrite[i]);
  Wire.endTransmission(); // Stop transmitting
}

/* Calibration */
void CJY901::autoCalibrateGyroAccel()
{
  JY901_SETCALI[3] = 1;
  writeRegister(ucDevAddr, JY901_CALSW, sizeof(JY901_SETCALI), (char *)JY901_SETCALI);
  JY901_GYROAUTO[3] = 0;
  writeRegister(ucDevAddr, JY901_CALSW, sizeof(JY901_GYROAUTO), (char *)JY901_GYROAUTO);
  vTaskDelay(3000);
  JY901_GYROAUTO[3] = 1;
  writeRegister(ucDevAddr, JY901_CALSW, sizeof(JY901_GYROAUTO), (char *)JY901_GYROAUTO);
  JY901_SETCALI[3] = 0;
  writeRegister(ucDevAddr, JY901_CALSW, sizeof(JY901_SETCALI), (char *)JY901_SETCALI);
}

void CJY901::autoCalibrateMagnetic()
{
  JY901_SETCALI[3] = 2;
  writeRegister(ucDevAddr, JY901_CALSW, sizeof(JY901_SETCALI), (char *)JY901_SETCALI);
  vTaskDelay(1000);
  JY901_SETCALI[3] = 0;
  writeRegister(ucDevAddr, JY901_CALSW, sizeof(JY901_SETCALI), (char *)JY901_SETCALI);
}

void CJY901::resetHeight()
{
  JY901_SETCALI[3] = 3;
  writeRegister(ucDevAddr, JY901_CALSW, sizeof(JY901_SETCALI), (char *)JY901_SETCALI);
  JY901_SETCALI[3] = 0;
  writeRegister(ucDevAddr, JY901_CALSW, sizeof(JY901_SETCALI), (char *)JY901_SETCALI);
}

void CJY901::set9AxisAlgo()
{
  JY901_ALGAXIS[3] = 0;
  writeRegister(ucDevAddr, JY901_CALSW, sizeof(JY901_ALGAXIS), (char *)JY901_ALGAXIS);
}

void CJY901::set6AxisAlgo()
{
  JY901_ALGAXIS[3] = 1;
  writeRegister(ucDevAddr, JY901_CALSW, sizeof(JY901_ALGAXIS), (char *)JY901_ALGAXIS);
}

void CJY901::calibrateAll()
{
  autoCalibrateGyroAccel();
  autoCalibrateMagnetic();
}

/* ------------ (JY901 --> Host) functions ------------ */
void CJY901::getTime()
{
  readRegisters(ucDevAddr, JY901_YYMM, sizeof(data.time), (char *)&data.time);
}

void CJY901::getAcc()
{
  // data.acc.x / (32768.0 / 16.0); unit: G(gravity)
  readRegisters(ucDevAddr, JY901_AX, sizeof(data.acc), (char *)&data.acc);
}

void CJY901::getGyro()
{
  // data.gyro.x / (32768.0 / 2000.0); unit: degree(s) per second
  readRegisters(ucDevAddr, JY901_GX, sizeof(data.gyro), (char *)&data.gyro);
}

/* -- Noticed that The Euler angles' order here is ---- */
/* ----------- Z-Y-X, for more please visit ----------- */
/* --- http://web.mit.edu/2.05/www/Handout/HO2.PDF ---- */

void CJY901::getAngles()
{
  // data.angle.roll / (32768.0 / 180.0); unit: degree(s)
  readRegisters(ucDevAddr, JY901_Roll, sizeof(data.angle), (char *)&data.angle);
}

void CJY901::getMag()
{
  // data.mag.x / (32768.0 / 180.0);
  readRegisters(ucDevAddr, JY901_HX, sizeof(data.mag), (char *)&data.mag);
}

/* ------ The port status output depends on its mode. ------ */
/* ----------- For more, please read the manual. ----------- */
void CJY901::getDStatus()
{
  readRegisters(ucDevAddr, JY901_D0Status, sizeof(data.dStatus), (char *)&data.dStatus);
}

/* ------------- According to NMEA8013, ------------ */
/* ------- GPS output format is dd mm.mmmmm, ------- */
/* ----- JY901 output format is ddmm(.)mmmmm, ------ */
/* --------- dd and mm can be calculated ----------- */
/* ---------- by divide(/) and modulo(%) ----------- */
/*
int32_t CJY901::getLon()
{
  return data.lon;
}
int32_t CJY901::getLat()
{
  return data.lat;
}

double CJY901::getGPSH()
{
  return data.GPSHeight / 10.0;
} // get GPS Height, unit: m(meters)

double CJY901::getGPSY()
{
  return data.GPSYaw / 10.0;
} // get GPS Yaw, unit: degree(s)

double CJY901::getGPSV()
{
  return data.GPSVelocity / 1000.0;
} // get GPS Velocity, unit: kilometers per hour
*/

void CJY901::getQuaternions()
{
  readRegisters(ucDevAddr, JY901_Q0, sizeof(data.quaternions), (char *)&data.quaternions);
}

/*
double CJY901::getDOP(const char *str)
{
  if (strcmp(str, "sn") == 0)
    return data.GPS_DOP.sn; // get number of satellites
  if (strcmp(str, "pdop") == 0)
    return data.GPS_DOP.pdop; // get PDOP
  if (strcmp(str, "hdop") == 0)
    return data.GPS_DOP.hdop; // get HDOP
  if (strcmp(str, "vdop") == 0)
    return data.GPS_DOP.vdop; // get VDOP

  return 0;
} // getDOP()
*/

/* ------------ (Host --> JY901) functions ------------ */
void CJY901::saveConf(int saveFlag)
{
  JY901_SAVECONF[3] = saveFlag;
  writeRegister(ucDevAddr, JY901_SAVE, sizeof(JY901_SAVECONF), (char *)JY901_SAVECONF);
  // Serial1.write(SAVECONF, 5);
} // save configuration

void CJY901::setIICaddr(int addrFlag)
{
  JY901_IICADDRESS[3] = addrFlag;
  writeRegister(ucDevAddr, JY901_IICADDR, sizeof(JY901_IICADDRESS), (char *)JY901_IICADDRESS);
  // Serial1.write(IICADDRESS, 5);
}

void CJY901::turnLED(int ledFlag)
{
  JY901_LED[3] = ledFlag;
  writeRegister(ucDevAddr, JY901_LEDOFF, sizeof(JY901_LED), (char *)JY901_LED);
  /*
  Serial1.write(LED, 5);
  if (ledFlag == 0)
    Serial.println("LED on");
  else if (ledFlag == 1)
    Serial.println("LED off");
  */
} // turn off LED, send again to lighten

/*
void CJY901::setCali(int caliFlag)
{
  SETCALI[3] = caliFlag;
  Serial1.write(SETCALI, 5);
} // calibration mode

void CJY901::setDir(int dirFlag)
{
  INSTALL[3] = dirFlag;
  Serial1.write(INSTALL, 5);
} // set install direction

void CJY901::enterHiber()
{
  Serial1.write(SLEEP, 5);
} // enter hibernation mode, send again to wake

void CJY901::changeALG(int algFlag)
{
  ALGAXIS[3] = algFlag;
  Serial1.write(ALGAXIS, 5);
} // change algorithm

void CJY901::autoCaliGyro(int gyroFlag)
{
  GYROAUTO[3] = gyroFlag;
  Serial1.write(GYROAUTO, 5);
} // auto gyro calibration

void CJY901::confReport()
{
  memcpy(&RPTCONF[3], &ctrl.report.confl, 2);
  Serial1.write(RPTCONF, 5);
} // need to write conf to  ctrl.report.conf first

void CJY901::setReportRate(int rateFlag)
{
  RPTRT[3] = rateFlag;
  Serial1.write(RPTRT, 5);
}

void CJY901::setBaudRate(int baudFlag)
{
  BAUDRT[3] = baudFlag;
  Serial1.write(BAUDRT, 5);
}
*/
/* ------To avoid negative value been changed, please use --------- */
/* ------------------- memcpy() to set value of ------------------- */
/* --- ctrl.aoffset ctrl.goffset ctrl.hoffset --- */
/* ----------- For more please read the example folder ------------ */
/*
void CJY901::setAXoffset()
{
  memcpy(&AXOFF[3], &ctrl.aoffset.xl, 2);
  Serial1.write(AXOFF, 5);
}
void CJY901::setAYoffset()
{
  memcpy(&AYOFF[3], &ctrl.aoffset.yl, 2);
  Serial1.write(AYOFF, 5);
}
void CJY901::setAZoffset()
{
  memcpy(&AZOFF[3], &ctrl.aoffset.zl, 2);
  Serial1.write(AZOFF, 5);
}

void CJY901::setGXoffset()
{
  memcpy(&GXOFF[3], &ctrl.goffset.xl, 2);
  Serial1.write(GXOFF, 5);
}
void CJY901::setGYoffset()
{
  memcpy(&GYOFF[3], &ctrl.goffset.yl, 2);
  Serial1.write(GYOFF, 5);
}
void CJY901::setGZoffset()
{
  memcpy(&GZOFF[3], &ctrl.goffset.zl, 2);
  Serial1.write(GZOFF, 5);
}

void CJY901::setHXoffset()
{
  memcpy(&HXOFF[3], &ctrl.hoffset.xl, 2);
  Serial1.write(HXOFF, 5);
}
void CJY901::setHYoffset()
{
  memcpy(&HYOFF[3], &ctrl.hoffset.yl, 2);
  Serial1.write(HYOFF, 5);
}
void CJY901::setHZoffset()
{
  memcpy(&HZOFF[3], &ctrl.hoffset.zl, 2);
  Serial1.write(HZOFF, 5);
}

void CJY901::setD0mode(int modeFlag)
{
  D0MODECONF[3] = modeFlag;
  Serial1.write(D0MODECONF, 5);
}
void CJY901::setD1mode(int modeFlag)
{
  D1MODECONF[3] = modeFlag;
  Serial1.write(D1MODECONF, 5);
}
void CJY901::setD2mode(int modeFlag)
{
  D2MODECONF[3] = modeFlag;
  Serial1.write(D2MODECONF, 5);
}
void CJY901::setD3mode(int modeFlag)
{
  D3MODECONF[3] = modeFlag;
  Serial1.write(D3MODECONF, 5);
}

void CJY901::setD0PWMH()
{
  memcpy(&D0PWMHCONF[3], &ctrl.pwmh.d0l, 2);
  Serial1.write(D0PWMHCONF, 5);
}
void CJY901::setD1PWMH()
{
  memcpy(&D1PWMHCONF[3], &ctrl.pwmh.d1l, 2);
  Serial1.write(D1PWMHCONF, 5);
}
void CJY901::setD2PWMH()
{
  memcpy(&D2PWMHCONF[3], &ctrl.pwmh.d2l, 2);
  Serial1.write(D2PWMHCONF, 5);
}
void CJY901::setD3PWMH()
{
  memcpy(&D3PWMHCONF[3], &ctrl.pwmh.d3l, 2);
  Serial1.write(D3PWMHCONF, 5);
}

void CJY901::setD0PWMT()
{
  memcpy(&D0PWMTCONF[3], &ctrl.pwmt.d0l, 2);
  Serial1.write(D0PWMTCONF, 5);
}
void CJY901::setD1PWMT()
{
  memcpy(&D1PWMTCONF[3], &ctrl.pwmt.d1l, 2);
  Serial1.write(D1PWMTCONF, 5);
}
void CJY901::setD2PWMT()
{
  memcpy(&D2PWMTCONF[3], &ctrl.pwmt.d2l, 2);
  Serial1.write(D2PWMTCONF, 5);
}
void CJY901::setD3PWMT()
{
  memcpy(&D3PWMTCONF[3], &ctrl.pwmt.d3l, 2);
  Serial1.write(D3PWMTCONF, 5);
}

void CJY901::setGPSrate(int gpsFlag)
{
  GPSBAUDRATE[3] = gpsFlag;
  Serial1.write(GPSBAUDRATE, 5);
}
*/
CJY901 JY901;
