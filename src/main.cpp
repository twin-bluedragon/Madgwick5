// Madgwick5
// M5Stack GRAYの9DOFセンサーの値とデバイス番号をJSON文字列化して、UDPで送信する
// multi task test for display LCD

#define M5STACK_MPU6886    // IMU

#include <Arduino.h>
#include <M5Stack.h>
#include <BMM150class.h>
#include <utility/quaternionFilters.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>  //不揮発メモリを操作するライブラリ 使い方：https://qiita.com/T-YOSH/items/f388b4d7cbc829829aae

//#define DEVICENUM 5
uint8_t state = 0;

uint8_t devvar;

//ESP_NOW
bool rcvmsg = false;
bool resetrequest = false;
bool master_addr_get = false;
bool restart = false;
uint8_t master_addr[] = {0,0,0,0,0,0};
uint8_t rxbuf[250];
int rxlen=0;
uint8_t txbuf[250];
int txlen=0;
unsigned int txcnt=0;
esp_now_peer_info_t peerInfo;

//#define MAHONY
#define MADGWICK

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

// マルチタスクで登録されるタスク
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
    memset(txbuf,0,250);
    sprintf((char*)txbuf,"{\"dev\":%d,\"q\":[%5.2f,%5.2f,%5.2f,%5.2f]}",devvar,mq0,mq1,mq2,mq3);
    txlen=strlen((char*)txbuf);
    //Serial.printf("%s",txbuf);

    M5.Lcd.setCursor(20, 220);
    M5.Lcd.printf("BTN_A:CAL ");

    M5.Lcd.setCursor(200,220);
    M5.Lcd.printf("DEVICE:%02d",devvar);

    delay(50);
  }
}

// ESP-NOWで受信した際に呼び出されるコールバック関数
void onReceive(const uint8_t* mac_addr, const uint8_t* data, int data_len) {
  rcvmsg = true;
  memcpy(rxbuf,data,data_len);//受信バッファにデータをコピーする
  rxlen = data_len;
  if(state==0){
    memcpy(master_addr,mac_addr,6);//statusが0だったら、そのMACアドレスをmaster_addrとする
    master_addr_get = true;
    if(*data!=0) //データが0以外だったら不測のリセットかかったとみなす
      restart=true;
    else
      restart=false;
  }

}

// ESP-NOWで受信した際に呼び出されるコールバック関数（何もしない）
void onSend(const uint8_t* mac_addr, esp_now_send_status_t status) {

}

//ESP-NOWの初期化で、マスターのアドレスをピアのリストに登録する
esp_err_t set_master_as_peer(){
    memcpy(peerInfo.peer_addr, master_addr, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    esp_err_t errorcode = esp_now_add_peer(&peerInfo);
    if(errorcode != ESP_OK)
      return errorcode;

    esp_now_register_send_cb(onSend);  
    return ESP_OK;
}

//ジャイロを初期化する
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


//M5Stack GRAY内蔵センサーから測定値を取得する
void scanM5gray_sensor()
{
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
}

//M5Stack GRAYの地磁気センサー(bmm150)のキャリブレーション
void M5sensor_calibration()
{
//    vTaskSuspend(lcdTaskHandle);
   vTaskDelete(lcdTaskHandle); //マルチタスクの削除
   M5.Lcd.fillScreen(BLACK);
   M5.Lcd.setTextColor(WHITE);
   M5.Lcd.setTextSize(1);
   M5.Lcd.setCursor(0, 0);
   M5.Lcd.print("begin calibration in 3 seconds");
   delay(3000);
   M5.Lcd.setCursor(0, 10);
   M5.Lcd.print("Flip + rotate core calibration");
   bmm150.bmm150_calibrate(15000);  //キャリブレーションルーチンの呼び出し(bmm150class.cpp)
   delay(100);
   bmm150.Init();
   bmm150.getMagnetOffset(&magoffsetX, &magoffsetY, &magoffsetZ);
   bmm150.getMagnetScale(&magscaleX, &magscaleY, &magscaleZ);
   M5.Lcd.fillScreen(BLACK);
   M5.Lcd.setTextColor(GREEN, BLACK);
   M5.Lcd.setTextSize(2);
//    vTaskResume(lcdTaskHandle);
}

//センサー起動前にESP-NOW通信を開始するので、それ用のダミーデータの作成
void set_dummydata(){
  memset(txbuf,0,250);
  sprintf((char*)txbuf,"{\"devnum\":%d,\"q\":[%5.2f,%5.2f,%5.2f,%5.2f]}",devvar,0.1,-0.2,0.3,-0.4);
  txlen=strlen((char*)txbuf);
}

void setup()
{
  Preferences mypref;

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
  
  //不揮発メモリから記録してあったデバイス番号を読みだす
  mypref.begin("mymem",false);
  mypref.getBytes("devno",&devvar,1);
  if(devvar>19){//もしデバイス番号がありえない数字だったら初期化
    mypref.clear();
    mypref.putBytes("devno",0,1);
    devvar = 0;
  }
  mypref.end();

  //USB Serial init
  Serial.begin(115200);  // Serialを初期化

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(2);

  //ESP_NOW Setting
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    M5.Lcd.setCursor(0,0);
    M5.Lcd.print("ESP NOW INIT OK");
    M5.Lcd.setCursor(0,16);
    M5.Lcd.print("Waiting for master call");
  }
  esp_now_register_recv_cb(onReceive);

  delay(2000);


  // multi-task
//  xTaskCreatePinnedToCore(lcdUpdateTask, "lcdTask", 4096, NULL, 1, &lcdTaskHandle, 1);
}

void loop()
{
  esp_err_t result;
  Preferences mypref;

  M5.update();
  scanM5gray_sensor();

  if(M5.BtnA.wasPressed()){  //キャリブレーションシーケンス
    M5sensor_calibration();
    mypref.begin("mymem",false);
    mypref.putBytes("devno",&devvar,1);
    mypref.end();
    M5.Lcd.fillScreen(BLACK);
    xTaskCreatePinnedToCore(lcdUpdateTask, "lcdTask", 4096, NULL, 1, &lcdTaskHandle, 1);//マルチタスク再開
  }else if(M5.BtnC.wasPressed()){
    if(devvar < 19)
      devvar++;
  }else if(M5.BtnB.wasPressed()){
    if(devvar > 0)
      devvar--;
  }  else{
    switch(state){
      case 0: //マスターからのブロードキャスト通信を待つ
        if(rcvmsg){
          rcvmsg=false;
          if(master_addr_get)
            state=1;  //ブロードキャスト通信を受信したらstate=1に進む
        }
        break;
      case 1:
        if(set_master_as_peer()==ESP_OK) //マスターからのデータが問題なければ、通信元のMACアドレスをpeerに登録する
          state = 2;  // 成功したらstate=2に進む
        else
          state = 0;  // 失敗したらstate=0に戻る
        break;
      case 2: //自分のアドレスをマスターに知らせるために、適当なタイミングでデータを送信する
        set_dummydata();
        if(!restart){
          delay(10 * devvar);  //衝突回避のインターバル
          result = esp_now_send(master_addr,txbuf,txlen);          
        }else
          result = ESP_OK;  //restart==trueならチェックしないでESP_OKにする

        if(result == ESP_OK){
          M5.Lcd.fillScreen(BLACK);
          xTaskCreatePinnedToCore(lcdUpdateTask, "lcdTask", 4096, NULL, 1, &lcdTaskHandle, 1);// マルチタスクの開始
          state = 3;
        }else
          state = 5;
        break;
      case 3: //定常状態
        if(rcvmsg){ //自局あてのメッセージを受信したら即データを送信する
          rcvmsg=false;
          result = esp_now_send(master_addr,txbuf,txlen);
          state = 4;
        }
        break;
      case 4://次の送信開始までの最小インターバル時間空ける
//        M5.Lcd.setCursor(0,48);
//        M5.Lcd.printf("tx done:%d",txcnt++);
        delay(10);
        state = 3;
        break;      
      case 5: // first tx failed
        M5.Lcd.setCursor(0,32);
        M5.Lcd.printf("first tx fail:%d\n%02X:%02X:%02X:%02X:%02X:%02X",ESP_OK,
          (unsigned char)master_addr[0],
          (unsigned char)master_addr[1],
          (unsigned char)master_addr[2],
          (unsigned char)master_addr[3],
          (unsigned char)master_addr[4],
          (unsigned char)master_addr[5]);
        for(;;);//halt
        break;
      default:
        break;   
    }
  }
  delay(1); 
}