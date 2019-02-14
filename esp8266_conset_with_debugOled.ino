#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET LED_BUILTIN  //4
Adafruit_SSD1306 display(OLED_RESET);
#define SSD1306_LCDHEIGHT 64

#if (SSD1306_LCDHEIGHT != 64)
#error(F("Height incorrect, please fix Adafruit_SSD1306.h!"));
#endif

#include "gw.h"
#include "pltfrm.h"
#include "uart.h"
#include "dis.h"
#include "lpwmn.h"
#include "atuat.h"

#include <SoftwareSerial.h>

#include <ESP8266WiFi.h>
#include "ThingSpeak.h"
int keyIndex = 0;
WiFiClient client;
#include <WiFiClientSecure.h>
//#define tenk
#define iot

volatile long int uploadCount = 0;
unsigned long myChannelNumberUp;
const char * myWriteAPIKeyUp;
unsigned long myChannelNumber[3] = {695481, 695448, 692930};
const char * myWriteAPIKey[3] = {"VOMEG4APLLB1S72M","9DWSPW41LX3YECRQ", "4DJ7ACAOZ0M727JB"};//Freezer Tag(with ePTFE) ,Freezer Tag(withot enclosure)
bool uploadFlag= false;
double macTime = millis();
unsigned long errCnt[3] = {0,0,0};

float loadCell_m = -571; //without l.s.= -550
float loadCell_c = - 9775; //without l.s.= - 9663
float loadCell_kg = 0;

float nodeVoltage = 0;
float nodeTemp = 0;
float nodeHumidity = 0;
signed int adcVal =0;
                                 
#ifdef iot
const char* ssid = "IOT-Startups";
const char* password = "Eagleeye!18";
#endif

#ifdef tenk
const char* ssid = "BWH09";
const char* password = "6GjPAWHT";
#endif


SoftwareSerial swSer(14, 12, false, 256);

#define UART_RX_BUFF_LEN   1024

int verbose = 0;

long GW_uartRxCnt = 0;
long GW_msgTotLen = UART_FRAME_HDR_LEN;
long GW_msgPyldLen = 0;
long GW_rcvdMsgCnt = 0;

unsigned char GW_uartRxBuff[UART_RX_BUFF_LEN];
char GW_snsrDataBuff[512];

char GW_eventBuff[512];
int GW_eventBuffCnt = 0;
String cmd_String;

unsigned char GW_serTxPyldBuff[256];
unsigned int GW_pendingTxPyldLength = 0;

unsigned char GW_serTxHdrBuff[UART_FRAME_HDR_LEN];
unsigned char GW_seqNrSentToCoord = 0x57;

unsigned int GW_expHdrSeqNrFromCoord = 0x100;   // Valid range is 0x00 - 0xff


char GW_printBuff[1024];



#define GW_print(...) \
do \
{ \ 
  sprintf(GW_printBuff, __VA_ARGS__); \
  Serial.print(GW_printBuff); \
} while(0)


/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
unsigned char TLV_get(unsigned char *buff_p, unsigned char len, unsigned char type,
                      unsigned char *pyldLen_p, unsigned char **pyldBuff_pp)
{
    int buffLen = len;
    unsigned char rc = 0;

    if (buffLen < DIS_TLV_HDR_SZ)
        return 0;

    // Get the tlv type

    while (buffLen >= DIS_TLV_HDR_SZ)
    {
        unsigned char tlvPyldLen = *(buff_p + DIS_TLV_TYPE_FIELD_LEN);

        if (*buff_p == type)
        {
            *pyldLen_p = tlvPyldLen;
            *pyldBuff_pp = (buff_p + DIS_TLV_HDR_SZ);
            rc = 1;
            break;
        }
        else
        {
            buff_p += (DIS_TLV_HDR_SZ + tlvPyldLen);
            buffLen -= (DIS_TLV_HDR_SZ + tlvPyldLen);
        }
    }

    return rc;
}


float __latestVcc;
int __latestVccSet = 0;


/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
unsigned short GW_ntohs(unsigned char *buff_p)
{
   short u16Val = *buff_p;
   u16Val = (u16Val << 8) | buff_p[1];  
   return u16Val;
}


/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
void GW_htonl(unsigned char *buff_p, unsigned long val)
{
  buff_p[3] = (val >> 24) & 0xff;
  buff_p[2] = (val >> 16) & 0xff;
  buff_p[1] = (val >> 8) & 0xff;
  buff_p[0] = (val) & 0xff;
}

/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
void GW_htons(unsigned char *buff_p, unsigned short val)
{
   buff_p[0] = (val >> 8) & 0xff;
   buff_p[1] = (val) & 0xff;
}


/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
unsigned long GW_ntohl(unsigned char *buff_p)
{
   unsigned int u32Val = *buff_p;
   u32Val <<= 8;
   u32Val |= buff_p[1];
   u32Val <<= 8;
   u32Val |= buff_p[2];
   u32Val <<= 8;
   u32Val |= buff_p[3];
   return u32Val;
}



unsigned short __crc16(unsigned char *buff_p, unsigned int len)
{
   unsigned long ckSum = 0;

   while (len > 1)
   {
      unsigned short tmp = *buff_p;
      tmp = (tmp << 8) | (*(buff_p + 1));
      ckSum = ckSum + tmp;
      buff_p += 2;
      len -= 2;
   }

   if (len > 0)
       ckSum += (*buff_p);

   while (ckSum >> 16)
   {
      ckSum = (ckSum & 0xffff) + (ckSum >> 16);
   }

   return (unsigned short)(~ckSum);
}


void GW_sendDataToCloud(int snsrId,float valF, char *unit_p)
{
  if(snsrId == PLTFRM_ON_CHIP_VCC_SENSOR_DEV_ID)
   {
      nodeVoltage = valF;
      Serial.print(nodeVoltage);
      Serial.println(unit_p);
   }
   else if(snsrId == PLTFRM_CC2D33S_1_RH_DEV_ID)
   {
       nodeHumidity = valF;
       Serial.print(nodeHumidity);
       Serial.println(unit_p);
   }
   else if(snsrId == PLTFRM_CC2D33S_1_TEMP_DEV_ID)
   {
       nodeTemp = valF;
       Serial.print(nodeTemp);
       Serial.println(unit_p);
       Serial.println("Posting..");
       //GW_sendHTTPS_POST();
       return; 
   }
   else if(snsrId == PLTFRM_HIH8121_1_RH_DEV_ID)
   {
       nodeHumidity = valF;
       Serial.print(nodeHumidity);
       Serial.println(unit_p);
   }
   else if(snsrId == PLTFRM_HIH8121_1_TEMP_DEV_ID)
   {
       nodeTemp = valF;
       Serial.print(nodeTemp);
       Serial.println(unit_p);
       Serial.println("Posting..");
       //GW_sendHTTPS_POST();
   }
      
   
   else if(snsrId == PLTFRM_AD7797_1_DEV_ID)
   {
      adcVal = valF;
      loadCell_kg = ((adcVal- loadCell_c)/loadCell_m);
      Serial.print(loadCell_kg);
      Serial.println(unit_p);
      GW_sendHTTPS_POST();
   }
}

void mac_api(unsigned long a, unsigned long b, unsigned long c )
{  
      
      if ( a == 0x01   && b == 0x19 && c == 0x05) // 01:19:05
      {
        macTime = millis();
        uploadFlag = true;
        myChannelNumberUp = myChannelNumber[2];
        myWriteAPIKeyUp = myWriteAPIKey[2];
        Serial.println(F("UP FROM Load cell"));
        a = 0; b = 0; c = 0;
      }
//      else if ( a == 0x0d   && b == 0x0a && c == 0xcf) // 0d:0a:cf>
//      {
//        macTime = millis();
//        uploadFlag = true;
//        myChannelNumberUp = myChannelNumber[0];
//        myWriteAPIKeyUp = myWriteAPIKey[0];
//        Serial.println(F("UP FROM Tag 1"));
//        a = 0; b = 0; c = 0;
//      }
//      else if ( a == 0x0d   && b == 0x4a && c == 0x56) // 0d:4a:56
//      {
//        macTime = millis();
//        uploadFlag = true;
//        myChannelNumberUp = myChannelNumber[1];
//        myWriteAPIKeyUp = myWriteAPIKey[1];
//        Serial.println(F("UP FROM Tag 2"));
//        a = 0; b = 0; c = 0;
//      }
      
      else 
      {
        a = 0; b = 0; c = 0;
        uploadFlag = false;
      }

}


void GW_sendHTTPS_POST()
{
  Serial.println("In HTTPPOST");
  ThingSpeak.setField(1, nodeVoltage);
  ThingSpeak.setField(2, loadCell_kg);
//  ThingSpeak.setField(2, nodeTemp);
//  ThingSpeak.setField(3, nodeHumidity);

  if(uploadFlag)
  {
    int x = ThingSpeak.writeFields(myChannelNumberUp, myWriteAPIKeyUp);
    delay(100);
    if(x == 200){
    uploadCount++;
       if((errCnt[0]+errCnt[1]+errCnt[2])==0)
       {
        display.clearDisplay();
        display.display();
        display.setCursor(0,0);
        display.println(F("WiFi connected"));
        display.print(F("SSID:"));
        display.println(ssid);
        //display.print(F("IP:"));
        //display.println(WiFi.localIP());
        display.print(F("Packets:"));
        display.println(uploadCount);
        display.print(F("Last:"));
        display.print(nodeVoltage, 2);
        display.print(F("V,"));
        display.print(loadCell_kg, 2);
        display.println(F("kg"));
        display.display();
       }
       else
       {
       display.clearDisplay();
       display.display();
       display.setCursor(0,0);
       display.print("Wifi Err:");
       display.println(errCnt[0]);
       display.print("Mac Err:");
       display.println(errCnt[1]);
       //display.print("Tspk Err:");
       //display.println(errCnt[2]);
         if(ssid == "IOT-Startups")
         { 
          display.print("IOT-Sta.");
         }
         else
         {
         display.print(ssid);
         }
         if(WiFi.status() == WL_CONNECTED)
         {
         display.print("-OK");
         }
         else if(WiFi.status() != WL_CONNECTED)
         {
         display.print("-Flr");
         }
      display.print(":");
      display.println(uploadCount);
      display.print(F("Last:"));
      display.print(nodeVoltage, 2);
      display.print(F("V,"));
      display.print(loadCell_kg, 2);
      display.println(F("kg"));
      display.display();
      }
      Serial.println("Channel update successful!");
    }
    
//    else
//    {
//      errFun("tsp");
//    }

    uploadFlag = false; 
  } 
}
/*
 ********************************************************************
 *
 *
 *
 ********************************************************************
 */
unsigned int GW_buildSendHdr(int msgType, unsigned char *pyldBuff_p, int pyldLen)
{
   unsigned char *buff_p = GW_serTxHdrBuff, currSeqNr;
   unsigned short calcCrc16;
   //Serial.setTimeout(500);

   
   GW_htons(buff_p, msgType);
   buff_p += UART_FRAME_HDR_MSG_TYPE_FIELD_LEN;

   *buff_p = 0x0;
   buff_p += UART_FRAME_HDR_FLAGS_FIELD_LEN;

   currSeqNr = GW_seqNrSentToCoord;
   *buff_p = GW_seqNrSentToCoord ++;
   buff_p += UART_FRAME_HDR_SEQ_NR_FIELD_LEN;

   GW_htons(buff_p, pyldLen);
   buff_p += UART_FRAME_HDR_PYLD_LEN_FIELD_LEN;

   calcCrc16 = __crc16(GW_serTxHdrBuff, UART_FRAME_HDR_HDR_CRC_FIELD_OFF);
   GW_htons(buff_p, calcCrc16);  // no payload
   buff_p += UART_FRAME_HDR_HDR_CRC_FIELD_LEN;

   if (pyldLen > 0)
   {
       calcCrc16 = __crc16(pyldBuff_p, pyldLen);
       GW_htons(buff_p, calcCrc16);  // payload crc
   }
   else
       GW_htons(buff_p, 0x0);  // no payload

#if 0
   if (verbose)
   {
       int idx;

       printf("\n -------------------------- \n");

       for (idx=0; idx<UART_FRAME_HDR_LEN; idx++)
            printf(" 0x%02x ", serTxBuff[idx]);

       printf("\n -------------------------- \n");
   }
#endif

   swSer.write(GW_serTxHdrBuff, UART_FRAME_HDR_LEN);
   
   swSer.readBytes(GW_uartRxBuff, UART_FRAME_HDR_LEN);


       
   // Serial.printf("writePort() done \r\n");

   return (unsigned int)currSeqNr;
}



void GW_processNodeMsg(unsigned int msgType, unsigned char *buff_p, long msgLen)
{
   long srcShortAddr, off = 0;
   unsigned long disMsgType;
   unsigned char *extAddr_p;
   
   GW_rcvdMsgCnt ++;

   
   if (msgLen < LPWMN_MAC_SHORT_ADDR_LEN 
                + LPWMN_MAC_EXT_ADDR_LEN 
                + LPWMN_MSG_RSSI_LEN 
                + LPWMN_MSG_CORR_LQI_LEN)
       return;
       
   srcShortAddr = buff_p[off];
   srcShortAddr = (srcShortAddr << 8) | buff_p[off + 1];
   
   off += LPWMN_MAC_SHORT_ADDR_LEN;
       
   GW_print("[%lu] Received msg from node <%05lu / %02lx:%02lx:%02lx:%02lx:%02lx:%02lx:%02lx:%02lx> \r\n", 
            GW_rcvdMsgCnt,
            srcShortAddr, 
            (unsigned long)buff_p[off],
            (unsigned long)buff_p[off+1],
            (unsigned long)buff_p[off+2],
            (unsigned long)buff_p[off+3],
            (unsigned long)buff_p[off+4],
            (unsigned long)buff_p[off+5],
            (unsigned long)buff_p[off+6],
            (unsigned long)buff_p[off+7]);  
              
   mac_api(buff_p[off + 5], buff_p[off + 6], buff_p[off + 7]);
   extAddr_p = buff_p + off;
   buff_p += LPWMN_MAC_EXT_ADDR_LEN;
   
   {
      int rssi;
      unsigned int lqi_corr;
      
      rssi = (signed char)buff_p[off];
      lqi_corr = buff_p[off + 1];
      GW_print("RSSI %d dBm / LQI %u \r\n", (int)rssi, lqi_corr);
   }
         
   off += (LPWMN_MSG_RSSI_LEN + LPWMN_MSG_CORR_LQI_LEN);

   msgLen -= (LPWMN_MAC_SHORT_ADDR_LEN 
              + LPWMN_MAC_EXT_ADDR_LEN 
              + LPWMN_MSG_RSSI_LEN 
              + LPWMN_MSG_CORR_LQI_LEN);
              
   if (msgLen < 1)
       return;
       
   disMsgType = buff_p[off];

   off += DIS_MSG_TYPE_SZ;
   msgLen -= DIS_MSG_TYPE_SZ;
   buff_p += off;
  
   if (msgLen <= 0xff)
   {
       unsigned char rc, tlvLen1, *buff1_p;
  
       if (disMsgType  == DIS_MSG_TYPE_ATUAT_TAG_BCN_INFO)
       {
           // GW_processTagBcn(extAddr_p, buff_p, msgLen);
           return;
       }  
  
       
       rc = TLV_get(buff_p, msgLen, DIS_TLV_TYPE_SENSOR_OUTPUT_LIST, &tlvLen1, &buff1_p);
       if (rc == 0)
       {
           GW_print("Could not find DIS_TLV_TYPE_SENSOR_OUTPUT_LIST !! \r\n");
           return;
       }        
       else
       {
           if (verbose)
               GW_print("Found DIS_TLV_TYPE_SENSOR_OUTPUT_LIST \r\n");

           while (1)
           {
              unsigned char tlvLen2, *buff2_p;

              rc = TLV_get(buff1_p, tlvLen1, DIS_TLV_TYPE_SENSOR_OUTPUT, &tlvLen2, &buff2_p);
              if (rc == 0)
              {
                  if (verbose)
                      GW_print("Could not find another DIS_TLV_TYPE_SENSOR_OUTPUT TLV !! \r\n");
                  break;
              } 
              else
              {
                  unsigned char tlvLen3, *buff3_p;
                  int snsrId, scaleFactor = DIS_DATA_SCALE_CENTI;


                  buff1_p += (tlvLen2 + DIS_TLV_HDR_SZ);

                  if (verbose)
                      GW_print("Found DIS_TLV_TYPE_SENSOR_OUTPUT TLV .... val-fld-len<%d> \r\n", tlvLen2);
                  
                  rc = TLV_get(buff2_p, tlvLen2, DIS_TLV_TYPE_SENSOR_ID, &tlvLen3, &buff3_p);
                  if (rc == 0)
                      continue;
                  else
                  {
                      if (tlvLen3 == DIS_SENSOR_ID_FIELD_SZ) 
                      {
                          snsrId = *buff3_p;
                          if (verbose)
                              GW_print("Sensor Id <0x%x> \r\n", snsrId);
                      }
                      else
                         continue;
                  }

                  rc = TLV_get(buff2_p, tlvLen2, DIS_TLV_TYPE_DATA_SCALE_FACTOR, &tlvLen3, &buff3_p);
                  if (rc)
                  {
                      if (tlvLen3 == DIS_DATA_SCALE_FACTOR_FIELD_SZ)
                      {
                          scaleFactor = *buff3_p;
                          if (verbose)
                              GW_print("Found Scale factor <%d> \r\n", scaleFactor); 
                          if (!(scaleFactor >= DIS_DATA_SCALE_TERA && scaleFactor <= DIS_DATA_SCALE_FEMTO))
                               scaleFactor = DIS_DATA_SCALE_NONE;
                      }
                  }

                  rc = TLV_get(buff2_p, tlvLen2, DIS_TLV_TYPE_VALUE, &tlvLen3, &buff3_p);
                  if (rc == 0)
                      continue;
                  else
                  {
                      long snsrOp;
                      signed short snsrOp16;
                      char *unit_p = " ";

                      if (verbose)
                          GW_print("Found DIS_TLV_TYPE_VALUE TLV .... val-fld-len<%d> \r\n", tlvLen3);
       
                      switch(tlvLen3)
                      {
                         case 1:
                              snsrOp = (int)(*buff3_p);
                              break;
                    
                         case 2:
                              {
                                snsrOp16 = GW_ntohs(buff3_p);
                                snsrOp = snsrOp16;
                              }
                              break;

                         case 4:
                              snsrOp = (int)GW_ntohl(buff3_p);
                              break;

                         default:
                              break;
                      }
           
                          float valF = snsrOp;
                          switch (snsrId)
                      {
                         case PLTFRM_ON_CHIP_VCC_SENSOR_DEV_ID:
                         GW_print("+[Node_Voltage]=>");
                         unit_p = "Volts";
                         scaleFactor = DIS_DATA_SCALE_MILLI;
                         break;


                         case PLTFRM_LM75B_1_DEV_ID:
                         GW_print("+[Temp_LM75B]=>");
                         unit_p = "Deg C";
                         scaleFactor = DIS_DATA_SCALE_CENTI;
                         break;

                         case PLTFRM_CC2D33S_1_RH_DEV_ID:
                         GW_print("+[RH_CC2D33S]=>");
                         unit_p = "NA";
                         scaleFactor = DIS_DATA_SCALE_CENTI;
                         break;

                         case PLTFRM_CC2D33S_1_TEMP_DEV_ID:
                         GW_print("+[Temp_CC2D33S]=>");
                         unit_p = "Deg C";
                         scaleFactor = DIS_DATA_SCALE_CENTI;
                         break;

                         case PLTFRM_BMP280_1_PRESSURE_DEV_ID:
                         GW_print("+[BMP_280_Pressure]=>");
                         unit_p = "mBar";
                         scaleFactor = DIS_DATA_SCALE_CENTI;
                         break;

                         case PLTFRM_DS18B20_1_DEV_ID:
                         GW_print("+[DS18B20]=>");
                         unit_p = "Deg C";
                         scaleFactor = DIS_DATA_SCALE_TENTH_MILLI;
                         break;

                         case PLTFRM_BMP280_1_TEMPERATURE_DEV_ID:
                         GW_print("+[Temp_BMP280]=>");
                         unit_p = "Deg C";
                         scaleFactor = DIS_DATA_SCALE_CENTI;
                         break;

                         case PLTFRM_CHIRP_PWLA_1_DEV_ID:
                         GW_print("+[Soil_Moisture]=>");
                         scaleFactor = DIS_DATA_SCALE_NONE;
                         unit_p = "NA";
                         break;

                         case PLTFRM_AD7797_1_DEV_ID:
                         GW_print("+[Load Cell]=>");
                         scaleFactor = DIS_DATA_SCALE_NONE;
                         unit_p = "ADC count";
                         break;

                         case PLTFRM_HIH8121_1_RH_DEV_ID:
                         GW_print("+[HIH8121_RH  ]");
                         scaleFactor = DIS_DATA_SCALE_CENTI;
                         unit_p = "%";
                         break;
                         
                         case PLTFRM_HIH8121_1_TEMP_DEV_ID:
                         GW_print("+[HIH8121_TEMP]");
                         scaleFactor = DIS_DATA_SCALE_CENTI;
                         unit_p = "deg C";
                         break;

                         default:
                              GW_print("[Unknown]");
                              break;
                      }
                      
                          switch (scaleFactor)
                          {
                             case DIS_DATA_SCALE_MICRO:
                                  valF /= 1000;
                                  valF /= 1000;
                                  break;

                             case DIS_DATA_SCALE_MILLI:
                                  valF /= 1000;
                                  break;

                             case DIS_DATA_SCALE_CENTI:
                                  valF /= 100;
                                  break;

                             case DIS_DATA_SCALE_DECI:
                                  valF /= 10;
                                  break;

                             case DIS_DATA_SCALE_TENTH_MILLI:
                                  valF /=1000;
                                  valF /=10;
                                  break;

                             case DIS_DATA_SCALE_HECTO:
                                  valF *= 100;
                                  break;
                                  
                             case DIS_DATA_SCALE_KILO:
                                  valF *= 1000;
                                  break;
                             

                             default:
                                  break;
                          }

                      

                 
                      if (snsrId == PLTFRM_ON_CHIP_VCC_SENSOR_DEV_ID
                          || snsrId == PLTFRM_CC2D33S_1_RH_DEV_ID
                          || snsrId == PLTFRM_CC2D33S_1_TEMP_DEV_ID
                          || snsrId == PLTFRM_AD7797_1_DEV_ID
                          || snsrId == PLTFRM_HIH8121_1_RH_DEV_ID
                          || snsrId == PLTFRM_HIH8121_1_TEMP_DEV_ID)
                      {

                          Serial.print(valF);
                          GW_sendDataToCloud(snsrId, valF,unit_p);
                         
                            
                      }
                  }
              } 
          }  
       }
   }

   
   return;
}


void GW_processRcvsMsg(unsigned int msgType, unsigned char *pyld_p, long pyldLen)
{
  Serial.print("---------------------------------------------------------------------------- \r\n");

  switch (msgType)
  {
    case LPWMN_GW_MSG_TYPE_EVENT:
    case LPWMN_GW_MSG_TYPE_RELAY_FROM_NODE:
      {
        GW_processNodeMsg(msgType, pyld_p, pyldLen);
      }
      break;

    default:
      break;
  }

  Serial.print("---------------------------------------------------------------------------- \r\n");
  return;
}






void GW_procRcvdByte(unsigned char rxByte)
{   
    GW_uartRxBuff[GW_uartRxCnt] = rxByte;
    
    // GW_print("%02d:%02x\r\n", GW_uartRxCnt, rxByte);
    
    GW_uartRxCnt ++;
    
    if (GW_uartRxCnt == UART_FRAME_HDR_PYLD_CRC_FIELD_OFF)
    {
        unsigned short calcCrc16, rxdCrc16;
        
        // GW_print("Read %d bytes \r\n", GW_uartRxCnt);
        
        calcCrc16 = __crc16(GW_uartRxBuff, UART_FRAME_HDR_HDR_CRC_FIELD_OFF);
        rxdCrc16 = GW_uartRxBuff[UART_FRAME_HDR_HDR_CRC_FIELD_OFF];
        rxdCrc16 = (rxdCrc16 << 8) + GW_uartRxBuff[UART_FRAME_HDR_HDR_CRC_FIELD_OFF + 1];       
        
        if (calcCrc16 != rxdCrc16)
        {
            int idx;
            GW_print("Hdr CRC mismatch <0x%x/0x%x> !!  \r\n", rxdCrc16, calcCrc16);
            for (idx=0; idx<UART_FRAME_HDR_PYLD_CRC_FIELD_OFF-1; idx++)
                 GW_uartRxBuff[idx] = GW_uartRxBuff[idx+1];
            GW_uartRxCnt = UART_FRAME_HDR_PYLD_CRC_FIELD_OFF - 1;
        }
        else
        {
            GW_print("Hdr CRC matches <0x%x/0x%x> \r\n", rxdCrc16, calcCrc16);
            GW_msgPyldLen = GW_uartRxBuff[UART_FRAME_HDR_PYLD_LEN_FIELD_OFF];
            GW_msgPyldLen += (GW_msgPyldLen << 8) |  GW_uartRxBuff[UART_FRAME_HDR_PYLD_LEN_FIELD_OFF + 1]; 
            GW_msgTotLen = UART_FRAME_HDR_LEN + GW_msgPyldLen;
            GW_print("msg payload length<%d> / total-length<%d> \r\n", GW_msgPyldLen, GW_msgTotLen);

        }
    }
    else
    {
        if (GW_uartRxCnt == GW_msgTotLen)
        {
            unsigned short calcPyldCrc16, rxdPyldCrc16;
            unsigned int currMsgType;
            
            // Payload received ...
            GW_print("msg with total length<%d> received  ... \r\n",  GW_msgTotLen);
            if(GW_msgPyldLen > 0)
            {
              calcPyldCrc16 = __crc16(GW_uartRxBuff + UART_FRAME_HDR_LEN, GW_msgPyldLen);
              rxdPyldCrc16 = GW_uartRxBuff[UART_FRAME_HDR_PYLD_CRC_FIELD_OFF];
              rxdPyldCrc16 = (rxdPyldCrc16 << 8) + (byte)GW_uartRxBuff[UART_FRAME_HDR_PYLD_CRC_FIELD_OFF + 1];         
            }
            GW_print("<0x%x/0x%x> Payload CRC %s \r\n", 
                     rxdPyldCrc16, calcPyldCrc16, calcPyldCrc16 != rxdPyldCrc16 ? "mismatch !!" : "match");
            
            currMsgType = GW_uartRxBuff[UART_FRAME_HDR_MSG_TYPE_FIELD_OFF];
            currMsgType = (currMsgType << 8) | GW_uartRxBuff[UART_FRAME_HDR_MSG_TYPE_FIELD_OFF + 1];            


            GW_processRcvsMsg(currMsgType, GW_uartRxBuff + UART_FRAME_HDR_LEN, GW_msgPyldLen);
            GW_uartRxCnt = 0;
        }
    }
}


int Gw_procRcvdCmd()
{
   int idx, rc, off = 0;
   int shortAddr = 0, portId, pinNr, val;
   unsigned char *pyld_p = GW_serTxPyldBuff;
   unsigned char extAddr[LPWMN_MAC_EXT_ADDR_LEN];
  

   cmd_String.toCharArray(GW_eventBuff, 512);

   rc = sscanf(GW_eventBuff, "dioc %x:%x:%x:%x %d %d %d",
               &extAddr[4], &extAddr[5], &extAddr[6], &extAddr[7],
               &portId, &pinNr, &val);
   if (rc != 7)
       return -1;
   
   for (idx=4; idx<LPWMN_MAC_EXT_ADDR_LEN; idx++)
   {

       if (!((extAddr[idx] >= 0) && (extAddr[idx] <= 0xff)))
           break;
   }

   if (idx < LPWMN_MAC_EXT_ADDR_LEN)
       return -2;
   else
   {
       LPWMN_nwkNodeInfo_s *node_p = LPWMN_lookUpNode(extAddr, 4);
       if (node_p == NULL)
           return -3;
       shortAddr =  node_p->shortAddr;
   }
   
   if (portId < 0x1 || portId > 16)
       return -4;

   if (pinNr > 7)
       return -5;
   
   if (val != 0 && val != 1)
       return -6;
   GW_pendingTxPyldLength = LPWMN_MAC_SHORT_ADDR_LEN
                            + DIS_MSG_TYPE_SZ
                            + DIS_TLV_HDR_SZ   // DIS_TLV_TYPE_CTRL_DIGITAL_IO
                            + DIS_DIGITAL_IO_PORT_TLV_SZ
                            + DIS_DIGITAL_IO_PIN_TLV_SZ
                            + DIS_DIGITAL_IO_VAL_TLV_SZ;

   GW_htons(pyld_p, shortAddr);
   off = LPWMN_MAC_SHORT_ADDR_LEN;
   pyld_p[off ++] = DIS_MSG_TYPE_CTRL_DIGITAL_IO;

   pyld_p[off ++] = DIS_TLV_TYPE_DIGITAL_IO;
   pyld_p[off ++] = (DIS_DIGITAL_IO_PORT_TLV_SZ
                     + DIS_DIGITAL_IO_PIN_TLV_SZ
                     + DIS_DIGITAL_IO_VAL_TLV_SZ);

   pyld_p[off ++] = DIS_TLV_TYPE_DIGITAL_IO_PORT;
   pyld_p[off ++] = DIS_DIGITAL_IO_PORT_FIELD_SZ;
   pyld_p[off ++] = portId;

   pyld_p[off ++] = DIS_TLV_TYPE_DIGITAL_IO_PIN;
   pyld_p[off ++] = DIS_DIGITAL_IO_PIN_FIELD_SZ;
   pyld_p[off ++] = pinNr;

   pyld_p[off ++] = DIS_TLV_TYPE_DIGITAL_IO_VAL;
   pyld_p[off ++] = DIS_DIGITAL_IO_VAL_FIELD_SZ;
   pyld_p[off ++] = val;

   GW_expHdrSeqNrFromCoord = GW_buildSendHdr(LPWMN_GW_MSG_TYPE_RELAY_TO_NODE,
                                             pyld_p, GW_pendingTxPyldLength);

   swSer.write(GW_serTxPyldBuff, GW_pendingTxPyldLength);
   
  /*
    GW_eventBuff[GW_eventBuffCnt] = rxByte;

    GW_eventBuffCnt++;
    
    if(strstr(GW_eventBuff, "cfg-dpi ") != NULL)
    {
        Serial.println("command not supported");
        //rc = GW_digitalIOCtrl(LPWMN_cloudCmdBuff);
        GW_eventBuffCnt = 0;
    }
    
    */
}

void setup() 
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.display();
  display.setCursor(2,0);
  display.println(F(" Setup Feb 13 7:42pm"));
  display.display();
  delay(2000);
  display.clearDisplay();
  display.display();
  display.setTextSize(4);
  display.clearDisplay();
  display.display();
  display.setCursor(0,0);
  display.println(F("HELLO"));
  display.display();
  delay(2000);
  display.clearDisplay();
  display.display();
  display.setTextSize(2);
  display.setCursor(0,0);
  for(int i=0; i<20;i++)
  {
    display.print(F("*"));
    display.display();
  }
  delay(500);
  display.clearDisplay();
  display.display();
  display.setTextSize(2);
  display.setCursor(2,0);
  display.println(F("  WISENSE"));
  display.setTextSize(1);
  display.println(F("    www.wisense.in"));
  display.display();
  delay(3000);
  
  WiFi.mode(WIFI_STA);
  Serial.begin(38400);
  swSer.begin(38400);
  ThingSpeak.begin(client);
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println(F("    Connecting to"));
  display.print("    ");
  display.println(ssid);
  display.display();
  WiFi.begin(ssid, password);
  display.setTextSize(1);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(50);
    display.print(F(">"));
    display.display();
  }
  delay(2000);
  display.clearDisplay();
  display.display();
  display.setCursor(0,0);
  display.println(F("WiFi connected"));
  display.print(F("SSID:"));
  display.println(ssid);
  display.print(F("IP:"));
  display.println(WiFi.localIP());
  display.print(F("Packets:"));
  display.print(uploadCount);
  display.display();


  //errTimeGp = millis();
  
  //LPWMN_initNodeList();
}

void loop() 
{
  
  if(WiFi.status() != WL_CONNECTED)
  {
    errFun("wifi");
  }
  unsigned long macInterval = millis() - macTime;
  if(macInterval > 120000)
  {
    errFun("mac");
  }

  //put your main code 
  //read from port 1, send to port 0:
  if (swSer.available()) 
  {
      int inByte = swSer.read();
      // Serial.println(inByte);
      GW_procRcvdByte(inByte);
  }

  while (Serial.available())
  {
      swSer.write(Serial.read());
      //cmd_String = Serial.readString();
      //Gw_procRcvdCmd();
      //Serial.println("Recieving");
  }

}


void errFun(char *errOrig)
{
  
  if(errOrig == "wifi")
  {
    errCnt[0]++;
    if(errCnt[0]%15 == 0)
    {
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
    }
    delay(1000);
  }
  else if(errOrig == "mac")
  {
    macTime = millis();
    errCnt[1]++;
    delay(1000);
  }
  else if(errOrig == "tsp")
  {
    errCnt[2]++;
    delay(1000);
  }

    display.clearDisplay();
    display.display();
    display.setCursor(0,0);
    display.print("Wifi Err:");
    display.println(errCnt[0]);
    display.print("Mac Err:");
    display.println(errCnt[1]);
    if(ssid == "IOT-Startups")
    {
      display.print("IOT-Sta.");
    }
    else
    {
      display.print(ssid);
    }
    if(WiFi.status() == WL_CONNECTED)
    {
     display.print("-OK");
    }
    else if(WiFi.status() != WL_CONNECTED)
    {
      display.print("-Flr");
    }
    display.print(":");
    display.println(uploadCount);
    display.print(F("Last:"));
    display.print(nodeVoltage, 2);
    display.print(F("V,"));
    display.print(loadCell_kg, 2);
    display.println(F("kg"));
    display.display();
    
}
