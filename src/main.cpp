// Madgwick5
// M5Stack GRAYの9DOFセンサーの値とデバイス番号をJSON文字列化して、UDPで送信する
// multi task test for display LCD

#define M5STACK_MPU6886    // IMU

#include <Arduino.h>
#include <M5Stack.h>
#include <BMM150class.h>
#include <utility/quaternionFilters.h>
//#include <BluetoothSerial.h>
#include <WiFi.h>
#include <WiFiUDP.h>

//BluetoothSerial SerialBT;

//#define MAHONY
#define MADGWICK
#define DEVICENUM 0

const char ssid[] = "Buffalo-2088";
const char pass[] = "r6xvjbtb7ye7k";
const char *dstddr = "192.168.3.33";
const int dst_port = 50007;
const int my_port = 50008;

WiFiUDP wifiUdp;

BMM150class bmm150;
TaskHandle_t lcdTaskHandle = NULL;

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

int AVERAGENUM_GY = 256;
float init_gyroX = 0.0F;
float init_gyroY = 0.0F;
float init_gyroZ = 0.0F;

float magnetX = 0.0F;
float magnetY = 0.0F;
float magnetZ = 0.0F;

//for hard iron correction
float magoffsetX = 0.0F;
float magoffsetY = 0.0F;
float magoffsetZ = 0.0F;

//for soft iron correction
float magscaleX = 0.0F;
float magscaleY = 0.0F;
float magscaleZ = 0.0F;

float mq0 = 0.0F;
float mq1 = 0.0F;
float mq2 = 0.0F;
float mq3 = 0.0F;

float pitch = 0.0F;
float roll = 0.0F;
float yaw = 0.0F;

float temp = 0.0F;

uint32_t Now = 0;
uint32_t lastUpdate = 0;
float deltat = 0.0f;

//moved to global
float magnetX1, magnetY1, magnetZ1;
float head_dir;

// Task function
void lcdUpdateTask(void *args)
{
  while (1) // never ending
  {
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);
    M5.Lcd.setCursor(220, 18);
    M5.Lcd.print(" o/s");
    M5.Lcd.setCursor(0, 36);
    M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", accX, accY, accZ);
    M5.Lcd.setCursor(220, 54);
    M5.Lcd.print(" G");
    M5.Lcd.setCursor(0, 72);
    M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", magnetX, magnetY, magnetZ);
    M5.Lcd.setCursor(220, 90);
    M5.Lcd.print(" mT");
    M5.Lcd.setCursor(0, 108);
    M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", magnetX1, magnetY1, magnetZ1);
  
    M5.Lcd.setCursor(0, 126);
    M5.Lcd.printf("HEAD Angle : %5.2f deg ", head_dir);
    M5.Lcd.setCursor(0, 144);
    M5.Lcd.printf("   yaw   pitch   roll ");
    M5.Lcd.setCursor(0, 162);
    M5.Lcd.printf("9AX %5.2f  %5.2f  %5.2f  ", yaw, pitch, roll);
    M5.Lcd.setCursor(0, 180);
    M5.Lcd.printf("sampleFreq %5.3f Hz", 1/deltat);

    M5.Lcd.setCursor(0, 198);
    M5.Lcd.printf("Temperature : %.2f C ", temp);
//    Serial.printf(" %5.2f,  %5.2f,  %5.2f  \r\n", roll, pitch, -yaw); // to processing *yaw polarity reversed
//    SerialBT.printf(" %5.2f,  %5.2f,  %5.2f  \r\n", roll, pitch, -yaw); // to processing *yaw polarity reversed
    char txbuf[256]="";
//    sprintf(txbuf,"%02d,  %5.2f,  %5.2f,  %5.2f,  %5.2f\r\n", DEVICENUM,mq0,mq1,mq2,mq3);
      sprintf(txbuf,"[{\"dev\":%02d,\"q\":[%5.2f,%5.2f,%5.2f,%5.2f]}]\r\n",DEVICENUM,mq0,mq1,mq2,mq3);
//    Serial.printf("%02d,  %5.2f,  %5.2f,  %5.2f,  %5.2f\r\n", DEVICENUM,mq0,mq1,mq2,mq3); 
//    SerialBT.printf("%02d,  %5.2f,  %5.2f,  %5.2f,  %5.2f\r\n", DEVICENUM,mq0,mq1,mq2,mq3); 
    Serial.printf("%s",txbuf);
    wifiUdp.beginPacket(dstddr,dst_port);
    wifiUdp.write((const uint8_t*)txbuf,strlen(txbuf));
    wifiUdp.endPacket();

    M5.Lcd.setCursor(20, 220);
    M5.Lcd.printf("BTN_A:CAL ");

    M5.Lcd.setCursor(200,220);
    M5.Lcd.printf("DEVICE:%02d",DEVICENUM);

    delay(50);
  }
}

void initGyro() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("begin gyro calibration");

  for (int i = 0;i < AVERAGENUM_GY;i++) {
    M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
    init_gyroX += gyroX;
    init_gyroY += gyroY;
    init_gyroZ += gyroZ;
    delay(5);
  }
  init_gyroX /= AVERAGENUM_GY;
  init_gyroY /= AVERAGENUM_GY;
  init_gyroZ /= AVERAGENUM_GY;
}

void setup()
{
  M5.begin();
  /*
    Power chip connected to gpio21, gpio22, I2C device
    Set battery charging voltage and current
    If used battery, please call this function in your project
  */
  M5.Power.begin();
  M5.IMU.Init();
  initGyro();
  bmm150.Init();

  bmm150.getMagnetOffset(&magoffsetX, &magoffsetY, &magoffsetZ);
  bmm150.getMagnetScale(&magscaleX, &magscaleY, &magscaleZ);
  
  //USB Serial init
  Serial.begin(115200);  // 一応Serialを初期化

  //BLE Serial init
//  char btname[8];
//  sprintf(btname,"M5_%02d",DEVICENUM);
//  SerialBT.begin(btname);

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(2);

  WiFi.begin(ssid,pass);
  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    M5.Lcd.print(".");
  }
  M5.Lcd.println("WiFi connected");
  M5.Lcd.print("IP address = ");
  M5.Lcd.print(WiFi.localIP());
  wifiUdp.begin(my_port);

  delay(2000);
  M5.Lcd.fillScreen(BLACK);

  // multi-task
  xTaskCreatePinnedToCore(lcdUpdateTask, "lcdTask", 4096, NULL, 1, &lcdTaskHandle, 1);
}

void loop()
{
  // put your main code here, to run repeatedly:
  M5.update();
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  gyroX -= init_gyroX;
  gyroY -= init_gyroY;
  gyroZ -= init_gyroZ;
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  bmm150.getMagnetData(&magnetX, &magnetY, &magnetZ);

  magnetX1 = (magnetX - magoffsetX) * magscaleX;
  magnetY1 = (magnetY - magoffsetY) * magscaleY;
  magnetZ1 = (magnetZ - magoffsetZ) * magscaleZ;

  M5.IMU.getTempData(&temp);
  head_dir = atan2(magnetX1, magnetY1);
  if(head_dir < 0)
    head_dir += 2*M_PI;
  if(head_dir > 2*M_PI)
    head_dir -= 2*M_PI;
  head_dir *= RAD_TO_DEG;
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f);//0.09
  lastUpdate = Now;

#ifdef MADGWICK
  MadgwickQuaternionUpdate(accX, accY, accZ, gyroX * DEG_TO_RAD,
                           gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD,
                           -magnetX1, magnetY1, -magnetZ1, deltat);
#endif

#ifdef MAHONY
  MahonyQuaternionUpdate(accX, accY, accZ, gyroX * DEG_TO_RAD,
                           gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD,
                           -magnetX1, magnetY1, -magnetZ1, deltat);
  //delay(10); // adjust sampleFreq = 50Hz
#endif

  mq0 = *getQ();
  mq1 = *(getQ() + 1);
  mq2 = *(getQ() + 2);
  mq3 = *(getQ() + 3);
/*  
  yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                                                          *(getQ() + 3)),
              *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
  pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                                                            *(getQ() + 2)));
  roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) *
                                                     *(getQ() + 3)),
               *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
*/
  yaw = atan2(2.0f * (mq1 * mq2 + mq0 * mq3), mq0* mq0 + mq1 * mq1 - mq2 * mq2 - mq3 * mq3);
  pitch = -asin(2.0f * (mq1 * mq3 - mq0 * mq2));
  roll = atan2(2.0f * (mq0 * mq1 + mq2 * mq3),mq0 * mq0 - mq1 * mq1 - mq2 * mq2 + mq3 * mq3);
  yaw = -0.5*M_PI-yaw;
  if(yaw < 0)
    yaw += 2*M_PI;
  if(yaw > 2*M_PI)
    yaw -= 2*M_PI;
  pitch *= RAD_TO_DEG;
  yaw *= RAD_TO_DEG;
  roll *= RAD_TO_DEG;

  delay(1); 

  if(M5.BtnA.wasPressed())
  {
//    vTaskSuspend(lcdTaskHandle);
    vTaskDelete(lcdTaskHandle);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print("begin calibration in 3 seconds");
    delay(3000);
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.print("Flip + rotate core calibration");
    bmm150.bmm150_calibrate(15000);
    delay(100);

    bmm150.Init();
    bmm150.getMagnetOffset(&magoffsetX, &magoffsetY, &magoffsetZ);
    bmm150.getMagnetScale(&magscaleX, &magscaleY, &magscaleZ);

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setTextSize(2);
//    vTaskResume(lcdTaskHandle);
  // multi-task
  xTaskCreatePinnedToCore(lcdUpdateTask, "lcdTask", 4096, NULL, 1, &lcdTaskHandle, 1);
  }
}