#include <Arduino.h>

#include <FlexCAN_T4.h>
#include "CRC8.h"
CRC8 crc(0x85, 0x00, 0x00, false, false);
FlexCAN_T4<CAN1, RX_SIZE_512, TX_SIZE_64> can1;
FlexCAN_T4<CAN2, RX_SIZE_512, TX_SIZE_64> can2;
FlexCAN_T4<CAN3, RX_SIZE_512, TX_SIZE_64> can3;
CAN_message_t msg;

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 100;  //the value is a number of milliseconds


  int battVolt1 = 0;
  int battVolt2 = 0;
  int battVolt = 0;
  int battCur1 = 0;
  int battCur2 = 0;
  int battCur = 0;
  int soc1 = 0;
  int soc2 = 0;
  int soc = 0;
  int dislimit1 = 0;
  int dislimit2 = 0;
  int dislimit = 0;
  int chglimit1 = 0;
  int chglimit2 = 0;
  int chglimit = 0;
  int chargelimit1 = 0;
  int chargelimit2 = 0;
  int chargelimit = 0;
  int GIDS1 = 0;
  int GIDS2 = 0;
  int GIDS = 0;
  int tempSegment1 = 0;
  int tempSegment2 = 0;
  int tempSegment = 0;
  int QCfullCapacity1 = 0;
  int QCfullCapacity2 = 0;
  int QCfullCapacity = 0;
  int QCremainCapacity1 = 0;
  int QCremainCapacity2 = 0;
  int QCremainCapacity = 0;

  // int temperature1 = 0;
  // int temperature2 = 0;
  // int temperature = 0;
  // char battType1[] = "ZE0  ";
  // char battType2[] = "AZE0 ";
  // char battType[] = "ZE1";
  // char battCur1step1[6];
  // char battCur1step2[6];
  // char battCur2step1[6];
  // char battCur2step2[6];
  //char battVoltage1step1[6];
  //char battVoltage2step1[6];
  const float safetyMargin = 0.75;
  const int maxDCFC = 400;    // kW * 0.10 (400 = 40kW)

void setup(void) {
  Serial.begin(115200);
  can1.begin();              // Battery 1
  can1.setClock(CLK_60MHz);
  can1.setBaudRate(500000);  // 500kbps data rate
  can2.begin();              // Battery 2
  can2.setClock(CLK_60MHz);
  can2.setBaudRate(500000);  // 500kbps data rate
  can2.setMaxMB(64);         // Max mailboxs, 1 MailBox per can message
  can2.enableFIFO();         // Preserve order of messages.
  can3.begin();              // ZombieVerter
  can3.setClock(CLK_60MHz);
  can3.setBaudRate(500000);  // 500kbps data rate
  can3.setMaxMB(64);         // Max mailboxs, 1 MailBox per can message
  can3.enableFIFO();         // Preserve order of messages.

 startMillis = millis();  //initial start time
}

void loop() {
  if ( can1.read(msg) ) {
    const uint8_t *bytes = msg.buf;
  switch (msg.id) {
    case 0x1DB: {
      battCur1 = uint16_t(bytes[0] << 3) + uint16_t(bytes[1] >> 5);
      battVolt1 = (uint16_t(bytes[2] << 2) + uint16_t(bytes[3] >> 6));   // / 2;
      break;
    }
    case 0x1DC: {
      dislimit1 = (uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6));              // * 0.25;  // Kw discharge limit
      chglimit1 = (uint16_t((bytes[1] & 0x3F) << 4) + uint16_t(bytes[2] >> 4));     // * 0.25;  // Kw regen limit
      chargelimit1 = (uint16_t((bytes[2] & 0x0F) << 6) + uint16_t(bytes[3] >> 2));  // * 0.1;  // Kw charger limit
      break;
    }
    case 0x55B: {
      soc1 = (uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6));                   // * 0.1;
      break;
    }
    case 0x59E: {
      QCfullCapacity1 = (uint16_t(bytes[2] << 4) + uint16_t(bytes[3] >> 4));                   // * 100 Wh
      QCremainCapacity1 = (uint16_t(bytes[3] << 5) + uint16_t(bytes[4] >> 3));                   // * 100 Wh
      break;
    }
    case 0x5BC: {
      GIDS1 = (uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6));                   // GIDS
      tempSegment1 = (uint16_t(bytes[3]));                   // Temp segment for dashboard
      break;
    }
  }
  
 }
  else if ( can2.read(msg) ) {
  const uint8_t *bytes = msg.buf;
  switch (msg.id) {
    case 0x1DB: {
      battCur2 = uint16_t(bytes[0] << 3) + uint16_t(bytes[1] >> 5);
      battCur = battCur1 + battCur2;
      battCur = battCur << 5;                                                    // Shift bits 5 places to the left to stuff into CAN frame
      msg.buf[0] = highByte(battCur);
      msg.buf[1] = bytes[1] & B00011111;                                         // Clear the bits used for battCur 
      msg.buf[1] = msg.buf[1] + lowByte(battCur);                                //and add the low byte of battCur shifted 5 places to the left
      battVolt2 = (uint16_t(bytes[2] << 2) + uint16_t(bytes[3] >> 6));                     // / 2;
      //battVolt = battVolt2;
      for ( uint8_t i = 0; i < 7; i++ ) {
        crc.add(msg.buf[i]);
      }
      msg.buf[7] = crc.calc();
      crc.restart();
    break;
    } 

    case 0x1DC: {
      dislimit2 = (uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6));                    // * 0.25;  // Kw discharge limit
        if (dislimit1 == 0 || dislimit2 == 0) {
          dislimit = 0;
         }
        else {
          dislimit = dislimit1 + dislimit2;
          dislimit = dislimit * safetyMargin;
          dislimit = dislimit << 6;
        }
      chglimit2 = (uint16_t((bytes[1] & 0x3F) << 4) + uint16_t(bytes[2] >> 4));           // * 0.25;  // Kw charge limit
        if (chglimit1 == 0 || chglimit2 == 0) {
          chglimit = 0;
         }
        else {
          chglimit = chglimit1 + chglimit2;
          chglimit = chglimit * safetyMargin;
          chglimit = chglimit << 4;
        }
      chargelimit2 = (uint16_t((bytes[2] & 0x0F) << 6) + uint16_t(bytes[3] >> 2));        // * 0.1;  // Kw charger limit
        if (chargelimit1 == 0 || chargelimit2 == 0) {
          chargelimit = 0;
         }
        else {
          chargelimit = chargelimit1 + chargelimit2;
          chargelimit = chargelimit * safetyMargin;
        }
          if (chargelimit > maxDCFC) {
            chargelimit = maxDCFC;
            }
          chargelimit = chargelimit << 2;
    
      msg.buf[0] = highByte(dislimit);
      msg.buf[1] = lowByte(dislimit) + highByte(chglimit);
      msg.buf[2] = lowByte(chglimit) + highByte(chargelimit);
      msg.buf[3] = bytes[3] & B00000011;
      msg.buf[3] = msg.buf[3] + lowByte(chargelimit);

      for ( uint8_t i = 0; i < 7; i++ ) {
        crc.add(msg.buf[i]);
      }
      msg.buf[7] = crc.calc();
      crc.restart();
      break;
    }

    case 0x55B: {
      soc2 = (uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6));                           // * 0.1;
        if (soc1 > 990 || soc2 > 990) {
          soc = 1000;
         }
        else {
          soc = soc1 + soc2;
          soc = soc /2;
        }
      
      soc = soc << 6;
      msg.buf[0] = highByte(soc);
      msg.buf[1] = lowByte(soc);

      for ( uint8_t i = 0; i < 7; i++ ) {
        crc.add(msg.buf[i]);
      }
      msg.buf[7] = crc.calc();
      crc.restart();

      break;
    }
    case 0x59E: {
      QCfullCapacity2 = (uint16_t(bytes[2] << 4) + uint16_t(bytes[3] >> 4));                   // * 100 Wh
      QCremainCapacity2 = (uint16_t(bytes[3] << 5) + uint16_t(bytes[4] >> 3));                   // * 100 Wh
      QCfullCapacity = QCfullCapacity1 + QCfullCapacity2;
      QCremainCapacity = QCremainCapacity1 + QCremainCapacity2;
      QCfullCapacity = QCfullCapacity << 4;
      QCremainCapacity = QCremainCapacity << 3;
      
      msg.buf[2] = QCfullCapacity >> 4;
      msg.buf[3] = lowByte(QCfullCapacity << 4) + (QCremainCapacity >> 5);
      msg.buf[4] = lowByte(QCremainCapacity << 3);

      Serial.print("QC full capacity: ");
      Serial.print(QCfullCapacity); 
      Serial.print(" Wh, QC remaining capacity: ");
      Serial.print(QCremainCapacity);
      Serial.println(" Wh");
      Serial.print("QC full capacity1: ");
      Serial.print(QCfullCapacity1);  
      Serial.print(" Wh, QC remaining capacity1: ");
      Serial.print(QCremainCapacity1);
      Serial.println(" Wh");
      Serial.print("QC full capacity2: ");
      Serial.print(QCfullCapacity2);  
      Serial.print(" Wh, QC remaining capacity2: ");
      Serial.print(QCremainCapacity2);
      Serial.println(" Wh");
      
      break;
    }

     case 0x5BC: {
      GIDS2 = (uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6));                   // GIDS
      GIDS = GIDS1 + GIDS2;
      GIDS = GIDS << 6;
      msg.buf[0] = highByte(GIDS);
      msg.buf[1] = lowByte(GIDS);
      tempSegment2 = (uint16_t(bytes[3]));                   // Temp segment for dashboard
      if (tempSegment1 > tempSegment2) {
        msg.buf[3] = tempSegment1;
      }
      
      break;
      }
    }
    can3.write(msg);
  }
  else if ( can3.read(msg) ) {
    can1.write(msg);
    can2.write(msg);
   }
  else { 
     currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period) {  //test whether the period has elapsed
    startMillis = currentMillis; 
    
    msg.id = 0x7A1;
    msg.buf[0] = highByte(battVolt1);
    msg.buf[1] = lowByte(battVolt1);
    msg.buf[2] = highByte(battCur1);
    msg.buf[3] = lowByte(battCur1);
    msg.buf[4] = highByte(soc1);
    msg.buf[5] = lowByte(soc1);
    can3.write(msg);  

    msg.id = 0x7B1;
    msg.buf[0] = highByte(dislimit1);
    msg.buf[1] = lowByte(dislimit1);
    msg.buf[2] = highByte(chglimit1);
    msg.buf[3] = lowByte(chglimit1);
    msg.buf[4] = highByte(chargelimit1);
    msg.buf[5] = lowByte(chargelimit1);
    can3.write(msg);    

     msg.id = 0x7A2;
    msg.buf[0] = highByte(battVolt2);
    msg.buf[1] = lowByte(battVolt2);
    msg.buf[2] = highByte(battCur2);
    msg.buf[3] = lowByte(battCur2);
    msg.buf[4] = highByte(soc2);
    msg.buf[5] = lowByte(soc2);
    can3.write(msg);  

    msg.id = 0x7B2;
    msg.buf[0] = highByte(dislimit2);
    msg.buf[1] = lowByte(dislimit2);
    msg.buf[2] = highByte(chglimit2);
    msg.buf[3] = lowByte(chglimit2);
    msg.buf[4] = highByte(chargelimit2);
    msg.buf[5] = lowByte(chargelimit2);
    can3.write(msg);  
  }  
  } 
}