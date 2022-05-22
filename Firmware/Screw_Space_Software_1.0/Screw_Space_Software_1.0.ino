#include <Wire.h>
#include <MPU6050_light.h>
#include "BluetoothSerial.h"
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <Tone32.h>
#include <EEPROM.h>
#include <Adafruit_BMP280.h>


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif



//MICRO SD:
#define CS 5
int n_sd = 0;
String text = "";
unsigned long data_logging_timer = 0;
#define DATA_LOGGING_TIMER 30

File myFile;


//EEPROM:
#define EEPROM_SIZE 18

#define actual_pid_constants_address 6
int pid_constants_address = actual_pid_constants_address;

#define x_zero_position_address 0
#define x_max_position_address 1
#define x_min_position_address 2

#define y_zero_position_address 3
#define y_max_position_address 4
#define y_min_position_address 5


//BLUETHOOTH:
BluetoothSerial SerialBT;
String message = "";
char incomingChar;
bool bt_imu_print = 0;
bool bt_pid_print = 0;
bool bt_bmp_print = 0;
bool bt_mos1_enable = 0;
unsigned long bt_print_timer;


//MPU:
#define MPU_REFRESH_DELAY 5
MPU6050 mpu(Wire);
unsigned long mpu_read_timer = 0;
float pitch, roll, yaw;
float f_pitch, f_roll, f_yaw;
float Acc_pitch, Acc_roll;
float AccX, AccY, AccZ;
float mpu_temperature;


//BMP280:
Adafruit_BMP280 bmp;
#define BMP_REFRESH_DELAY 5
unsigned long bmp_read_timer = 0;

float bmp_temperature, pressure, actual_altitude, offset_bmp, altitude_with_offset; 
float last_altitude_with_offset = 0;
float max_altitude_with_offset = 0;


//PID:
#define x_integrative_limit 15
#define y_integrative_limit 15
float setpoint_x = 0; float setpoint_y = 0;
float kp = 0.50; float kd = 0.05; float ki = 0.10;
float p_x, i_x, d_x, pid_x;
float p_y, i_y, d_y, pid_y;
float error_x, error_y;
float last_error_x, last_error_y;
unsigned long last_time_dx, last_time_dy;
unsigned long pid_timer;


//SERVO: 
#define TVC_REFRESH_DELAY 15 //bps: 25Hz, altre cose: 400Hz

Servo x_axis;
#define x_axis_pin 4
int x_zero_position = 115;  
int x_max_position = 175;   
int x_min_position = 0;     
int x_action = 0;

Servo y_axis;
#define y_axis_pin 15
int y_zero_position = 80;   
int y_max_position = 100;   
int y_min_position = 35;   
int y_action = 0;

bool tvc_enable = 0;


//LED & BUZZER:
#define STANDARD_OFF_DELAY 3000

#define LED_RED 27
#define LED_GREEN 26
#define LED_BLUE 25
bool led_state = 0;
bool led_enable = 1;
unsigned long last_switch = 0;
int led_color, led_t_on, led_t_off;

#define BUZZER_PIN 17
#define canale 9
#define STANDARD_FREQUENCY 2750

bool buzzer_state = 0;
bool buzzer_enable = 1;
unsigned long buzzer_last_switch = 0;
int buzzer_t_on, buzzer_t_off, frequency;


//MOSFET:
#define MOS1 33
bool mos1_state = 0;


//VARIABILI & COSTANTI DURANTE IL VOLO:
int stato_razzo = 0;  //0 = A TERRA, 1 = PARTITO, 2 = DISCESA.

//Partenza:
#define SOGLIA_ACCELERAZIONE_PARTENZA 15
bool tilt_angles_ok = 0;

#define numero_di_test_soglia 5
float valori_media_acc[numero_di_test_soglia] = {};
float media = 0;
int numero_test = 0;

//Discesa:
unsigned long timer_monitorazione_altezza = 0;
#define PERCENTUALE_ALTEZZA_APERTURA_PARACADUTE 0.8

//Landing:
#define TEMPO_ACCENSIONE_MOSFET_PARACADUTE 1000
#define SOGLIA_ALTEZZA_LANDING 10
#define TEMPO_DI_CONFERMA_LANDING 3000
unsigned long timer_razzo_a_terra = 0;


//OTHER: 
#define DEG_TO_RAD 0.01745329


 
void setup(){
  Serial.begin(115200);                         
  delay(250);
  Serial.println();


  //BUZZER & LED Setup:
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  
  //SD Setup:
  pinMode(CS, OUTPUT);
  if(!SD.begin(CS)){
    Serial.println("Inizializzazione SD Fallita");
    Serial.println();
    while(1) led_control(1, 1000, 2000), buzzer_control(1000, 2000, STANDARD_FREQUENCY);  
  }
  Serial.println("SD Inizializzata");
  Serial.println();
  
  while(1){
    text = "/data" + (String)n_sd + ".txt";
    if(!SD.exists(text)){
      Serial.println();
      Serial.print(text);
      Serial.print(" ");
      Serial.println("Non Esiste");
      Serial.println("Creo il file...");
      myFile = SD.open(text, FILE_WRITE);
      if(myFile){
        myFile.println("Pitch, Roll, Yaw, BMP Temperature, MPU Temperature, Pressure, Altitude, Altitude With Offset, AccX, AccY, AccZ, P_X, I_X, D_X, X_Action, P_Y, I_Y, D_Y, Y_Action");
        myFile.close();
        Serial.println("Scrittura Iniziale Eseguita");
        Serial.println();
      }
      else{
        Serial.print("Errore Apertura ");
        Serial.println(text);
        while(1) led_control(1, 25, 500), buzzer_control(2000, 1000, STANDARD_FREQUENCY); 
      }
      break;
    }
    else n_sd++;
  }


  //EEPROM Setup:
  EEPROM_setup();


  //Print Setup:
  print_taratura_servo();
  print_pid_constants();

  
  //Servo Setup:
  x_axis.attach(x_axis_pin);
  y_axis.attach(y_axis_pin);
  //servo_setup_animation();
  //delay(1000);


  //BT Setup:
  SerialBT.begin("Screw Space Flight Computer"); //Screw Space Flight Computer
  Serial.println();
  Serial.println("Bluethooth Inizializzato");
  Serial.println();


  //BMP280 Setup:
  if(!bmp.begin()){
    Serial.println();
    Serial.println("Inizializzazione Barometro Fallita");
    while(1) led_control(1, 1000, 2000), buzzer_control(1000, 2000, STANDARD_FREQUENCY);     
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
                  
  float calc_offsets_bmp = 0;
  for(int i = 0; i < 10; i++) calc_offsets_bmp += bmp.readAltitude(1013.25);
  offset_bmp = calc_offsets_bmp/10;
  
  Serial.println();
  Serial.print("Barometro Inizializzato - ");
  Serial.print("Offset: ");
  Serial.println(offset_bmp);  
  Serial.println();      


  //MOSFET:
  pinMode(MOS1, OUTPUT);
  digitalWrite(MOS1, LOW);
  //digitalWrite(MOS1, LOW);digitalWrite(MOS1, HIGH);delay(1000);digitalWrite(MOS1, LOW);


  //IMU Setup:
  Wire.begin();
  if(mpu.begin()){
    Serial.println();
    Serial.println("Inizializzazione Giroscopio Fallita");
    while(1) led_control(1, 1000, 2000), buzzer_control(1000, 2000, STANDARD_FREQUENCY);      
  }
  
  mpu.setFilterGyroCoef(0.98); 
  mpu.calcOffsets(1, 1);

  //mpu.calcGyroOffsets(); 
  //mpu.setAccOffsets((float)x_acc_offset, (float)y_acc_offset, (float)z_acc_offset);
  //mpu.setGyroOffsets((float)x_gyro_offset, (float)y_gyro_offset, (float)z_gyro_offset);
  
  Serial.println("Gyro Offsets: ");
  Serial.println(mpu.getGyroXoffset(), 4);
  Serial.println(mpu.getGyroYoffset(), 4);
  Serial.println(mpu.getGyroZoffset(), 4);
  
  Serial.println();
  
  Serial.println("Acc Offsets: ");
  Serial.println(mpu.getAccXoffset(), 4);
  Serial.println(mpu.getAccYoffset(), 4);
  Serial.println(mpu.getAccZoffset(), 4);
  
  Serial.println();
  Serial.println("Giroscopio Inizializzato");  
  Serial.println();                          


  //LED & BUZZER: 
  led_color = 2;
  led_t_on = 25;
  led_t_off = STANDARD_OFF_DELAY;

  buzzer_t_on = 25;
  buzzer_t_off = STANDARD_OFF_DELAY;
  frequency = STANDARD_FREQUENCY;
}

void loop() {
  mpu.update();  
  
  mpu_refresh();
  bmp_refresh();
  

  led_control(led_color, led_t_on, led_t_off);
  buzzer_control(buzzer_t_on, buzzer_t_off, frequency);


  if(tvc_enable){
    tvc_refresh();
    data_logging();
  }
  else tvc_reset();


  //Verifica se Angolo è ok
  if(!tvc_enable){
    if(!stato_razzo && media <= 10.5){
      //Riconoscimento se angolo di partenza è ok
      if((pitch - 0.3) < 0 && (pitch + 0.3) > 0 && (roll - 0.3) < 0 && (roll + 0.3) > 0){
        tilt_angles_ok = 1;
        led_color = 2;
      }
      else{
        led_color = 1;
        tilt_angles_ok = 0;
      }
    }
    else tilt_angles_ok = 1;
  }
        
  //Partenza Rilevata & Attivazione TVC  
  if((media >= SOGLIA_ACCELERAZIONE_PARTENZA) && !stato_razzo) Serial.println("Partenza Rilevata"), stato_razzo = 1, media = 0, tvc_enable = 1, tvc_state_change();

  //Monitoraggio Altezza
  else if(stato_razzo == 1){
    if((millis() - timer_monitorazione_altezza) >= 50){
      //Verifica Altezza Massima
      if(altitude_with_offset > max_altitude_with_offset) max_altitude_with_offset = altitude_with_offset, Serial.print("Altezza Max: "), Serial.println(max_altitude_with_offset);
      //Verifica se si è in fase di discesa
      if((max_altitude_with_offset - altitude_with_offset) >= 1){ //altitude_with_offset < last_altitude_with_offset 
        //Spegni TVC
        if(tvc_enable){
          Serial.println("Si Scende");
          tvc_enable = 0, tvc_state_change();          
        }
        //Controlla se il razzo ha raggiunto la percentuale di altezza per aprire il paracadute
        if(altitude_with_offset <= (PERCENTUALE_ALTEZZA_APERTURA_PARACADUTE * max_altitude_with_offset)){
          Serial.print("Attivazione Paracadute, ");Serial.print("Altezza da terra: ");Serial.print(altitude_with_offset);Serial.print(", Altezza massima da terra: ");Serial.println(max_altitude_with_offset);
          mos1_state = 1;
          digitalWrite(MOS1, mos1_state);
          stato_razzo = 2; 
        }
      }

      last_altitude_with_offset = altitude_with_offset;
      timer_monitorazione_altezza = millis();
    }
  }
  //Discesa con paracadute
  else if(stato_razzo == 2){
    //Spegnimento mosfet dopo tempo predefinito
    if((millis() - timer_monitorazione_altezza) >= TEMPO_ACCENSIONE_MOSFET_PARACADUTE && mos1_state) mos1_state = 0, digitalWrite(MOS1, mos1_state), Serial.println("Mosfet Spento");
    //Controlla se il razzo è a terra
    if(altitude_with_offset <= SOGLIA_ALTEZZA_LANDING){
      if(!timer_razzo_a_terra) timer_razzo_a_terra = millis();
      else if((millis() - timer_razzo_a_terra) > TEMPO_DI_CONFERMA_LANDING){
        Serial.println("Razzo Tornato a Terra");
        stato_razzo = 0;
        myFile.close();
      }
    }
    else timer_razzo_a_terra = 0;
  }


  //Bluethooth
  if(!stato_razzo){
    bt_read();

    if((millis() - bt_print_timer) > 500){
      if(bt_imu_print) bt_print_imu();
      if(bt_bmp_print){
        bt_print_bmp();
        if(bt_imu_print && !bt_pid_print) SerialBT.println();
      }
      if(bt_pid_print){
        bt_print_pid();
        if(!bt_imu_print && !bt_bmp_print){}
        else SerialBT.println();
      }
      
      bt_print_timer = millis();
    }
    if(bt_mos1_enable) digitalWrite(MOS1, mos1_state);
  }

}
