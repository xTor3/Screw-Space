#include <Tone32.h>
#include "BluetoothSerial.h"

//BUZZER
#define BUZZER_PIN 17
#define BUZZER_CHANNEL 9
#define STANDARD_FREQUENCY 2750


//BLUETOOTH
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
String message = "";
char incomingChar;


//RELE
#define RELE_NUMBER 3
const unsigned int led_rele_state[RELE_NUMBER] = {2, 27, 12}; 
const unsigned int led_connection_state[RELE_NUMBER] = {4, 26, 14}; 
const unsigned int rele[RELE_NUMBER] = {15, 19, 13};
bool rele_state[RELE_NUMBER] = {};
const unsigned int analog_input[RELE_NUMBER] = {33, 32, 25};


//COUNTDOWN
#define SECONDI_COUNTDOWN 10
bool countdown = 0;
int canale = 0;
bool ignition = 0;
int secondi_passati = 1;

unsigned long timer_countdown = 0;


//IGNITION
#define IGNITION_DELAY 4000
unsigned long timer_ignition = 0;



void setup() {
  Serial.begin(115200);

  //BLUETHOOTH
  SerialBT.begin("Screw Space - Launch Base"); //Bluetooth device name
  
  //RELE
  for(int i = 0; i < RELE_NUMBER; i++){
    pinMode(rele[i], OUTPUT);
    digitalWrite(rele[i], LOW);
    
    pinMode(led_rele_state[i], OUTPUT);
    digitalWrite(led_rele_state[i], LOW);

    pinMode(led_connection_state[i], OUTPUT);
    digitalWrite(led_connection_state[i], LOW);

    pinMode(analog_input[i], INPUT);

    rele_state[i] = 0;
  }

}

void loop() {
  for(int i = 0; i < RELE_NUMBER; i++){
    if(analogRead(analog_input[i]) > 0) digitalWrite(led_connection_state[i], HIGH);
    else digitalWrite(led_connection_state[i], LOW);

    digitalWrite(rele[i], rele_state[i]);
    digitalWrite(led_rele_state[i], rele_state[i]);
  }

  if(countdown && !ignition) partenza_razzo();
  else timer_countdown = millis(), secondi_passati = 1;

  if(ignition) rele_state[canale] = 1, ignition = 0, timer_ignition = millis();
  else if((millis() - timer_ignition) >= IGNITION_DELAY) rele_state[canale] = 0;
  


  //BT
  if (SerialBT.available()){
    char incomingChar = SerialBT.read();
    if (incomingChar != '\n') message += String(incomingChar);
    else message = "";
  }

  if(message == "stopcountdown") countdown = 0;
  
  String utility = "";
  for(int i = 0; i < RELE_NUMBER; i++){
    utility = String(i+1) + "countdown" ;
    if(message == utility){
      countdown = 1;
      canale = i;
      Serial.print("Countdown di "); Serial.print(SECONDI_COUNTDOWN); Serial.print("s Avviato,"); Serial.print(" Canale: "); Serial.println(canale+1);
    }
    
    utility = "channel" + String(i+1) + "on";
    if(message == utility) rele_state[i] = 1;
    utility = "channel" + String(i+1) + "off";
    if(message == utility) rele_state[i] = 0;
  }
}



void partenza_razzo(void){
  if(secondi_passati > SECONDI_COUNTDOWN - 1){
    for(int i = 0; i < 40; i++){
      delayMicroseconds(12500);
      tone_esp(BUZZER_PIN, STANDARD_FREQUENCY, 0, BUZZER_CHANNEL);
      delayMicroseconds(12500);
      noTone_esp(BUZZER_PIN, BUZZER_CHANNEL);
    }
    Serial.println("Ignition");
    countdown = 0;
    ignition = 1;
  }
  else if((millis() - timer_countdown) >= 1000){
    tone_esp(BUZZER_PIN, STANDARD_FREQUENCY, 0, BUZZER_CHANNEL);
    delay(25);
    noTone_esp(BUZZER_PIN, BUZZER_CHANNEL);
    Serial.print("Secondi Passati: ");
    Serial.println(secondi_passati);
    
    secondi_passati++;
    timer_countdown = millis();
  }
}
