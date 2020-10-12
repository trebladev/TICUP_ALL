#include <WiFi.h>
#include <WiFiClient.h>

const char *ssid = "TEST";
const char *password = "12345678";

static int r_buf0_full_flag;

uint8_t r_buf0[5000];

static int data_num;

int i=0;

const IPAddress serverIP(192,168,1,102); //目标地址
uint16_t serverport = 22333;

WiFiClient client;                            //声明服务器对象

hw_timer_t *timer = NULL;

static void IRAM_ATTR Timer0_CallBack(void);

unsigned char ending[3] = {0xff,0xff,0xff};

void setup()
{
    Serial.begin(115200);
    Serial.println();

    timer = timerBegin(0,80,true);
    timerAttachInterrupt(timer,Timer0_CallBack,true);
    timerAlarmWrite(timer,20000,true);
    timerAlarmEnable(timer);
    
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                     //关闭STA模式下wifi休眠，提高响应速度
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());

    //server.begin(22333);                      //服务器启动监听端口号22333
}

void loop()
{
  Serial.println("尝试访问服务器");
  if (client.connect(serverIP, serverport))  //尝试访问目标地址
  {
    Serial.println("访问成功");

//    client.print("Hello world!");                    //向服务器发送“hello world”
    while (client.connected()) //如果处于连接状态
    {
      if(client.available())
      {
        //Serial.println("查询信息:");
        if (client.available()) //如果有数据可读取
        {
          //String line = client.readStringUntil('\n'); //读取数据到换行符
          
            client.readBytes(r_buf0+151*r_buf0_full_flag,151);
            //Serial.print("收到消息：");
            //Serial.println(sizeof(r_buf));
            //client.write(line.c_str()); //将收到的数据回发      
            //c_str()函数返回一个指向正规C字符串的指针, 内容与本string串相同  
            r_buf0_full_flag++;
        }
      }
    }
    Serial.println("关闭当前连接");
    client.stop(); //关闭客户端
  }
  else
  {
    Serial.println("访问失败");
    client.stop(); //关闭客户端
  }
}

static void IRAM_ATTR Timer0_CallBack(void)
{
  data_num++;
  Serial.printf("add 2,0,%d",r_buf0[i]);
  Serial.write(ending,3);
  if(data_num == 151)
  {
    if(r_buf0_full_flag != 0)
    {
      memcpy(r_buf0+151*r_buf0_full_flag,r_buf0,r_buf0_full_flag*151);
      r_buf0_full_flag--;
    }
  }
}
