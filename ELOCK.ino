#include <SoftwareSerial.h>
//define Pins and Data for SIM900A
#define rxPin 10
#define txPin 11
//define input of servo
#define seroIn 3
enum SMStype {UNRELATED,OPEN,CLOSE,ASK};

//蓝牙通讯接口
SoftwareSerial BTSerial(8, 9);//RX,TX for BLE
//舵机--控制开锁锁芯
#include <Servo.h>
Servo myservo;

#define GPRMC_TERM "$GPRMC,"    //定义要解析的指令，因为这条指令包含定位和时间信息

char nmeaSentence[68];
String latitude;    //纬度
String longitude;   //经度
String lndSpeed;    //速度
String gpsTime;     //UTC时间，本初子午线经度0度的时间，和北京时间差8小时
String beiJingTime;   //北京时间

//globle variable
SoftwareSerial m_serial(rxPin,txPin);
String MSG("");
String SMS("");
char myRelay=1;
char myPhone[]={"18501308512"};
char snd_tips[]={"Init finished.COMMAND[open,close,ask]"};
char snd_unrelated[]={"Error CMD"};
char snd_ask_o[]={"Opened"};
char snd_ask_c[]={"Closed"};

//function declaration
void init(SoftwareSerial &p_serial);                                            //initialize the sim900a module
void getMSG(String &p_MSG,SoftwareSerial &p_serial);                            //get the sim900a module's message
bool chkMSG(String &p_MSG);                                                     //check the message's type
void rcvSMS(String &p_SMS,SoftwareSerial &p_serial);                            //recieve the first SMS
char chkSMS(String &p_SMS);                                                     //check the SMS's type
void sndSMS(const char *phone_num,const char *content,SoftwareSerial &p_serial);//send Short Message

void init(SoftwareSerial &p_serial)
{
  p_serial.begin(9600);
  p_serial.println("AT");
  delay(500);
  p_serial.println("AT+CMGD=1, 4");
  delay(500);
  p_serial.flush();
  delay(100);
}

void setup()  //初始化内容
{
  Serial.begin(9600);     //定义波特率9600
  BTSerial.begin(9600);   //蓝牙波特率
  myservo.attach(seroIn);     //初始化舵机
  myservo.write(0);       //锁车
  init(m_serial);         //初始化sim900a
  sndSMS(myPhone,snd_tips,m_serial);//发送提示短信
}

void loop()   //主循环
{
   delay(100);
   getMSG(MSG,m_serial);
   //delay(100);
   if(!chkMSG(MSG))
     return;
   rcvSMS(SMS,m_serial);
   delay(100);
   switch(chkSMS(SMS))
   {
     case OPEN:
     {
       myservo.write(0);
       break;
     }
     case CLOSE:
     {
       myservo.write(90);
       break;
     }
     case ASK:
     {
       //sent status and GPS information
       int sg=myservo.read();
       getGPSinfo();
       if(sg==0){
         sndSMS(myPhone,snd_ask_c,m_serial);
         delay(100);
      }else{
        sndSMS(myPhone,snd_ask_o,m_serial);
        delay(100);
      }
       break;
     }
     default:
     {
       sndSMS(myPhone,snd_unrelated,m_serial);
       delay(100);
       sndSMS(myPhone,snd_tips,m_serial);
       delay(100);
     }
   }
}
void getGPSinfo(){
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)  //一秒钟内不停扫描GPS信息
  {
    BTSerial.println("waiting...");
    while (Serial.available())  //串口获取到数据开始解析
    {
      char c = Serial.read(); //读取一个字节获取的数据
      switch(c)         //判断该字节的值
      {
      case '$':         //若是$，则说明是一帧数据的开始
        Serial.readBytesUntil('*', nmeaSentence, 67);   //读取接下来的数据，存放在nmeaSentence字符数组中，最大存放67个字节
        //Serial.println(nmeaSentence);
        latitude = parseGprmcLat(nmeaSentence); //获取纬度值
        longitude = parseGprmcLon(nmeaSentence);//获取经度值
        lndSpeed = parseGprmcSpeed(nmeaSentence);//获取速度值
        gpsTime = parseGprmcTime(nmeaSentence);//获取GPS时间


        if(latitude > "")   //当不是空时候打印输出
        {
          Serial.println("------------------------------------");
          Serial.println("latitude: " + latitude);
          BTSerial.print(latitude+"|");
        }

        if(longitude > "")    //当不是空时候打印输出
        {
          Serial.println("longitude: " + longitude);
          BTSerial.println(longitude);
        }

        if(lndSpeed > "")   //当不是空时候打印输出
        {
          Serial.println("Speed (knots): " + lndSpeed);
        }

        if(gpsTime > "")    //当不是空时候打印输出
        {
          Serial.println("gpsTime: " + gpsTime);
          beiJingTime = getBeiJingTime(gpsTime);  //获取北京时间
          Serial.println("beiJingTime: " + beiJingTime);
        }
      }
    }
  }
}

String getBeiJingTime(String s)
{
  int hour = s.substring(0,2).toInt();
  int minute = s.substring(2,4).toInt();
  int second = s.substring(4,6).toInt();

  hour += 8;

  if(hour > 24)
    hour -= 24;
  s = String(hour) + String(minute) + String(second);
  return s;
}

//Parse GPRMC NMEA sentence data from String
//String must be GPRMC or no data will be parsed
//Return Latitude
String parseGprmcLat(String s)
{
  int pLoc = 0; //paramater location pointer
  int lEndLoc = 0; //lat parameter end location
  int dEndLoc = 0; //direction parameter end location
  String lat;
  /*make sure that we are parsing the GPRMC string.
   Found that setting s.substring(0,5) == "GPRMC" caused a FALSE.
   There seemed to be a 0x0D and 0x00 character at the end. */
  if(s.substring(0,4) == "GPRM")
  {
    //Serial.println(s);
    for(int i = 0; i < 5; i++)
    {
      if(i < 3)
      {
        pLoc = s.indexOf(',', pLoc+1);
        /*Serial.print("i < 3, pLoc: ");
         Serial.print(pLoc);
         Serial.print(", ");
         Serial.println(i);*/
      }
      if(i == 3)
      {
        lEndLoc = s.indexOf(',', pLoc+1);
        lat = s.substring(pLoc+1, lEndLoc);
        /*Serial.print("i = 3, pLoc: ");
         Serial.println(pLoc);
         Serial.print("lEndLoc: ");
         Serial.println(lEndLoc);*/
      }
      else
      {
        dEndLoc = s.indexOf(',', lEndLoc+1);
        lat = lat + " " + s.substring(lEndLoc+1, dEndLoc);
        /*Serial.print("i = 4, lEndLoc: ");
         Serial.println(lEndLoc);
         Serial.print("dEndLoc: ");
         Serial.println(dEndLoc);*/
      }
    }
    return lat;
  }
  //}
  //}
}

//Parse GPRMC NMEA sentence data from String
//String must be GPRMC or no data will be parsed
//Return Longitude
String parseGprmcLon(String s)
{
  int pLoc = 0; //paramater location pointer
  int lEndLoc = 0; //lat parameter end location
  int dEndLoc = 0; //direction parameter end location
  String lon;

  /*make sure that we are parsing the GPRMC string.
   Found that setting s.substring(0,5) == "GPRMC" caused a FALSE.
   There seemed to be a 0x0D and 0x00 character at the end. */
  if(s.substring(0,4) == "GPRM")
  {
    //Serial.println(s);
    for(int i = 0; i < 7; i++)
    {
      if(i < 5)
      {
        pLoc = s.indexOf(',', pLoc+1);
        /*Serial.print("i < 3, pLoc: ");
         Serial.print(pLoc);
         Serial.print(", ");
         Serial.println(i);*/
      }
      if(i == 5)
      {
        lEndLoc = s.indexOf(',', pLoc+1);
        lon = s.substring(pLoc+1, lEndLoc);
        /*Serial.print("i = 3, pLoc: ");
         Serial.println(pLoc);
         Serial.print("lEndLoc: ");
         Serial.println(lEndLoc);*/
      }
      else
      {
        dEndLoc = s.indexOf(',', lEndLoc+1);
        lon = lon + " " + s.substring(lEndLoc+1, dEndLoc);
        /*Serial.print("i = 4, lEndLoc: ");
         Serial.println(lEndLoc);
         Serial.print("dEndLoc: ");
         Serial.println(dEndLoc);*/
      }
    }
    return lon;
  }
}

//Parse GPRMC NMEA sentence data from String
//String must be GPRMC or no data will be parsed
//Return Longitude
String parseGprmcSpeed(String s)
{
  int pLoc = 0; //paramater location pointer
  int lEndLoc = 0; //lat parameter end location
  int dEndLoc = 0; //direction parameter end location
  String lndSpeed;

  /*make sure that we are parsing the GPRMC string.
   Found that setting s.substring(0,5) == "GPRMC" caused a FALSE.
   There seemed to be a 0x0D and 0x00 character at the end. */
  if(s.substring(0,4) == "GPRM")
  {
    //Serial.println(s);
    for(int i = 0; i < 8; i++)
    {
      if(i < 7)
      {
        pLoc = s.indexOf(',', pLoc+1);
        /*Serial.print("i < 8, pLoc: ");
         Serial.print(pLoc);
         Serial.print(", ");
         Serial.println(i);*/
      }
      else
      {
        lEndLoc = s.indexOf(',', pLoc+1);
        lndSpeed = s.substring(pLoc+1, lEndLoc);
        /*Serial.print("i = 8, pLoc: ");
         Serial.println(pLoc);
         Serial.print("lEndLoc: ");
         Serial.println(lEndLoc);*/
      }
    }
    return lndSpeed;
  }
}


//Parse GPRMC NMEA sentence data from String
//String must be GPRMC or no data will be parsed
//Return Longitude
String parseGprmcTime(String s)
{
  int pLoc = 0; //paramater location pointer
  int lEndLoc = 0; //lat parameter end location
  int dEndLoc = 0; //direction parameter end location
  String gpsTime;

  /*make sure that we are parsing the GPRMC string.
   Found that setting s.substring(0,5) == "GPRMC" caused a FALSE.
   There seemed to be a 0x0D and 0x00 character at the end. */
  if(s.substring(0,4) == "GPRM")
  {
    //Serial.println(s);
    for(int i = 0; i < 2; i++)
    {
      if(i < 1)
      {
        pLoc = s.indexOf(',', pLoc+1);
        /*Serial.print("i < 8, pLoc: ");
         Serial.print(pLoc);
         Serial.print(", ");
         Serial.println(i);*/
      }
      else
      {
        lEndLoc = s.indexOf(',', pLoc+1);
        gpsTime = s.substring(pLoc+1, lEndLoc);
        /*Serial.print("i = 8, pLoc: ");
         Serial.println(pLoc);
         Serial.print("lEndLoc: ");
         Serial.println(lEndLoc);*/
      }
    }
    return gpsTime;
  }
}


void sndSMS(const char *phone_num,const char *content,SoftwareSerial &p_serial)
{
  p_serial.println("AT");
  delay(500);
  p_serial.println("AT+CSCS=\"GSM\"");
  delay(500);
  p_serial.println("AT+CMGF=1");
  delay(500);
  p_serial.print("AT+CMGS=\"");
  p_serial.print(phone_num);
  p_serial.println("\"");
  for(;m_serial.read()!='>';) ;
  p_serial.print(content);
  delay(500);
  p_serial.write(0x1A);
  delay(500);
  p_serial.flush();
  delay(100);
}

void rcvSMS(String &p_SMS,SoftwareSerial &p_serial)
{
  p_SMS="";
  m_serial.println("AT");
  delay(500);
  m_serial.println("AT+CSCS=\"GSM\"");
  delay(500);
  m_serial.println("AT+CMGF=1");
  delay(500);
  m_serial.flush();
  m_serial.println("AT+CMGR=1");
  delay(500);
  while(!p_serial.available()) ;
  for(char in='\0',flag=0;p_serial.available();)
  {
    in=p_serial.read();
    if(in=='\"')
      flag++;
    if(flag==8)
    {
      p_serial.read();
      p_serial.read();
      while((in!='\r')&&(in!='\n')&&p_serial.available())
      {
        in=p_serial.read();
        p_SMS+=in;
      }
      break;
    }
  }
  p_serial.println("AT+CMGD=1, 4");
  delay(500);
  p_serial.flush();
  delay(100);
}

char chkSMS(String &p_SMS)
{
  String temp_open="open\r",temp_close="close\r",temp_ask="ask\r";

  if(p_SMS.equalsIgnoreCase(temp_open))
    return OPEN;
  if(p_SMS.equalsIgnoreCase(temp_close))
    return CLOSE;
  if(p_SMS.equalsIgnoreCase(temp_ask))
    return ASK;
  return UNRELATED;
}


void getMSG(String &p_MSG,SoftwareSerial &p_serial)
{
  p_MSG="";
  while(!p_serial.available()) ;
  for(char in='\0',flag=0;p_serial.available();)
  {
    in=p_serial.read();
    if(in=='+')
      flag=1;
    if(flag&&((in=='\r')||(in=='\n')))
      break;
    if(flag)
      p_MSG+=in;
  }
  p_MSG.trim();
  p_serial.flush();
}

bool chkMSG(String &p_MSG)
{
  for(int p_begin=0;(p_begin+5)<p_MSG.length();p_begin++)
  {
    if(p_MSG.substring(p_begin,p_begin+5)=="+CMTI")
      return true;
  }
  return false;
}

// Turn char[] array into String object
String charToString(char *c)
{

  String val = "";

  for(int i = 0; i <= sizeof(c); i++)
  {
    val = val + c[i];
  }

  return val;
}
