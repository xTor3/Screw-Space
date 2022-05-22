void led_control(int color, int t_on, int t_off){

  if(!led_enable || t_on == 0) led_state = 0;
  else if (t_off == 0) led_state = 1;
  else if(led_state && (millis() - last_switch) >= t_on){
    led_state = 0;
    last_switch = millis();
  }
  else if(!led_state && (millis() - last_switch) >= t_off){
    led_state = 1;
    last_switch = millis();
  }
  
  switch(color){
    case 1:
      digitalWrite(LED_RED, led_state);
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_BLUE, LOW);
      break;
    case 2:
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, led_state);
      digitalWrite(LED_BLUE, LOW);
      break;
    case 3:
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_BLUE, led_state);
      break;
  }
}


bool ok_on = 0;
bool ok_off = 0;

void buzzer_control(int t_on, int t_off, int freq){
  if(!buzzer_enable || t_on == 0) buzzer_state = 0;
  else if (t_off == 0) buzzer_state = 1;
  else if(buzzer_state && (millis() - buzzer_last_switch) >= t_on){
    buzzer_state = 0;
    buzzer_last_switch = millis();
  }
  else if(!buzzer_state && (millis() - buzzer_last_switch) >= t_off){
    buzzer_state = 1;
    buzzer_last_switch = millis();
  }

  if(buzzer_state && !ok_on) tone_esp(BUZZER_PIN, freq, 0, canale), ok_on = 1, ok_off = 0;
  else if(!buzzer_state && !ok_off) noTone_esp(BUZZER_PIN, canale), ok_on = 0, ok_off = 1;

}
