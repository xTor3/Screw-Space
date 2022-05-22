void tvc_refresh(void){
  if((millis() - pid_timer) > TVC_REFRESH_DELAY){
    x_action = (x_zero_position - calc_pid_x(roll));
    x_axis.write(x_action);
    y_action = (y_zero_position - calc_pid_y(pitch));
    y_axis.write(y_action);

    pid_timer = millis();
  }
}


void tvc_state_change(void){
  if(tvc_enable){
    myFile = SD.open(text, FILE_APPEND);
    led_color = 3;
    led_t_on = 25;
    led_t_off = 0;
  
    buzzer_enable = 0;
  }
  else{
    if(!stato_razzo) myFile.close();
    led_color = 2;
    led_t_on = 25;
    led_t_off = STANDARD_OFF_DELAY;
  
    buzzer_enable = 1, buzzer_last_switch = 0;
    led_state = 0, last_switch = 0;
  }
}



int calc_pid_x(float pitch_value){
  error_x = setpoint_x - pitch_value;


  p_x = error_x * kp;
  
  //if(error_x < 10 && error_x > -10) i_x += (error_x * ki);
  i_x += (error_x * ki);
  if(i_x > x_integrative_limit) i_x = x_integrative_limit;
  else if(i_x < -(x_integrative_limit)) i_x = -(x_integrative_limit);
  
  d_x = ((error_x - last_error_x) /(float)((millis() - last_time_dx)/(float)1000)) * kd;
  last_time_dx = millis();
  last_error_x = error_x;
  

  pid_x = p_x + d_x + i_x;

  //Limitazione PID Asse X:
  if(pid_x > (x_max_position - x_zero_position)) pid_x = (x_max_position - x_zero_position);
  else if(pid_x < -(x_zero_position - x_min_position)) pid_x = -(x_zero_position - x_min_position);

  return (int)pid_x;
}

int calc_pid_y(float roll_value){
  error_y = setpoint_y - roll_value;


  p_y = error_y * kp;
  
  //if(error_y < 10 && error_y > -10) i_y += (error_y * ki);
  i_y += (error_y * ki);
  if(i_y > y_integrative_limit) i_y = y_integrative_limit;
  else if(i_y < -(y_integrative_limit)) i_y = -(y_integrative_limit);
  
  d_y = ((error_y - last_error_y) /(float)((millis() - last_time_dy)/(float)1000)) * kd;
  last_time_dy = millis();
  last_error_y = error_y;
  

  pid_y = p_y + d_y + i_y;

  //Limitazione PID Asse Y:
  if(pid_y > (y_max_position - y_zero_position)) pid_y = (y_max_position - y_zero_position);
  else if(pid_y < -(y_zero_position - y_min_position)) pid_y = -(y_zero_position - y_min_position);

  return (int)pid_y;
}

void reset_pid(void){
  pid_y = 0;
  p_y = 0;
  d_y = 0;
  i_y = 0;

  pid_x = 0;
  p_x = 0;
  d_x = 0;
  i_x = 0;
}

void tvc_reset(void){
  x_axis.write(x_zero_position);
  y_axis.write(y_zero_position);
  reset_pid();
}



void modifica_costanti_pid(void){
  led_control(1,0,0);
  print_pid_constants();

  float pid_modify;
  for(int i = 0; i < 3; i++){
    switch(i){
      case 0: 
        SerialBT.println("Modifica Kp");
        Serial.println("Modifica Kp");
        pid_modify = kp;
        break;
      case 1:
        Serial.println();
        SerialBT.println();
        SerialBT.println("Modifica Ki");
        Serial.println("Modifica Ki");
        pid_modify = ki;
        break;
      case 2:
        Serial.println();
        SerialBT.println();
        SerialBT.println("Modifica Kd");
        Serial.println("Modifica Kd");
        pid_modify = kd;
        break;
    }

    int count = 0;
    while(1){
      if(SerialBT.available()){
        bt_receive();
        count++;
      }
      if(message.toFloat() > 0.00 && count > 5){
        pid_modify = message.toFloat();
        SerialBT.println();
        SerialBT.println(pid_modify);
        Serial.println(pid_modify);
        Serial.println();
        break;
      }
    }

    switch(i){
      case 0:
        kp = pid_modify;
        break;
      case 1:
        ki = pid_modify;
        break;
      case 2:
        kd = pid_modify;
        break;
    }
  }
  
  print_pid_constants();

  EEPROM.writeFloat(pid_constants_address, kp);
  pid_constants_address += sizeof(float);
  EEPROM.writeFloat(pid_constants_address, ki);
  pid_constants_address += sizeof(float);
  EEPROM.writeFloat(pid_constants_address, kd);
  
  pid_constants_address = actual_pid_constants_address;
  
  EEPROM.commit();
  
  Serial.println("Resetta il flight computer per salvare i cambiamenti");
  SerialBT.println("Resetta il flight computer per salvare i cambiamenti");

  while(1) delay(1000);
}



void taratura_servo(void){
  led_control(1,0,0);
  print_taratura_servo();
  
  int servo_val = 0;
  for(int i = 0; i < 6; i++){
    switch(i){
      case 0:
        SerialBT.println("Modifica X Min");
        Serial.println("Modifica X Min");
        servo_val = x_min_position;
        break;
      case 1:
        Serial.println();
        SerialBT.println();
        SerialBT.println("Modifica X Max");
        Serial.println("Modifica X Max");
        servo_val = x_max_position;
        break;
      case 2:
        Serial.println();
        SerialBT.println();
        SerialBT.println("Modifica X Zero");
        Serial.println("Modifica X Zero");
        servo_val = x_zero_position;
        break;
      case 3:
        Serial.println();
        SerialBT.println();
        SerialBT.println("Modifica Y Min");
        Serial.println("Modifica Y Min");
        servo_val = y_min_position;
        break;
      case 4:
        Serial.println();
        SerialBT.println();
        SerialBT.println("Modifica Y Max");
        Serial.println("Modifica Y Max");
        servo_val = y_max_position;
        break;
      case 5:
        Serial.println();
        SerialBT.println();
        SerialBT.println("Modifica Y Zero");
        Serial.println("Modifica Y Zero");
        servo_val = y_zero_position;
        break;
    }
    while(1){
      bt_receive();
      if(message == "+"){
        servo_val++;
        if(servo_val > 180) servo_val = 180;
        SerialBT.print("Valore Attuale: ");
        SerialBT.println(servo_val);
        Serial.print("Valore Attuale: ");
        Serial.println(servo_val);
      }
      else if(message == "-"){
        servo_val--;
        if(servo_val < 0) servo_val = 0;
        SerialBT.print("Valore Attuale: ");
        SerialBT.println(servo_val);
        Serial.print("Valore Attuale: ");
        Serial.println(servo_val);
      }
      else if(message == "calibrazione servo") break;

      if(i > 2) y_axis.write(servo_val);
      else x_axis.write(servo_val);
    }

    switch(i){
      case 0:
        x_min_position = servo_val;
        SerialBT.print("X Min: ");
        SerialBT.println(x_min_position);
        Serial.print("X Min: ");
        Serial.println(x_min_position);
        break;
      case 1:
        x_max_position = servo_val;
        SerialBT.print("X Max: ");
        SerialBT.println(x_max_position);
        Serial.print("X Max: ");
        Serial.println(x_max_position);
        break;
      case 2:
        x_zero_position = servo_val;
        SerialBT.print("X Zero: ");
        SerialBT.println(x_zero_position);
        Serial.print("X Zero: ");
        Serial.println(x_zero_position);
        break;
      case 3:
        y_min_position = servo_val;
        SerialBT.print("Y Min: ");
        SerialBT.println(y_min_position);
        Serial.print("Y Min: ");
        Serial.println(y_min_position);
        break;
      case 4:
        y_max_position = servo_val;
        SerialBT.print("Y Max: ");
        SerialBT.println(y_max_position);
        Serial.print("Y Max: ");
        Serial.println(y_max_position);
        break;
      case 5:
        y_zero_position = servo_val;
        SerialBT.print("Y Zero: ");
        SerialBT.println(y_zero_position);
        Serial.print("Y Zero: ");
        Serial.println(y_zero_position);
        break;
    }
  }

  EEPROM.write(x_zero_position_address, x_zero_position);
  EEPROM.write(x_min_position_address, x_min_position);
  EEPROM.write(x_max_position_address, x_max_position);
  EEPROM.write(y_zero_position_address, y_zero_position);
  EEPROM.write(y_min_position_address, y_min_position);
  EEPROM.write(y_max_position_address, y_max_position);
  
  EEPROM.commit();

  print_taratura_servo();
  
  Serial.println("Resetta il flight computer per salvare i cambiamenti");
  SerialBT.println("Resetta il flight computer per salvare i cambiamenti");

  while(1) delay(1000);
}

void print_taratura_servo(void){
  Serial.println();
  Serial.println("Dati Taratura Servo:");
  Serial.print("x_min: ");Serial.print(x_min_position);Serial.print("; x_max: ");Serial.print(x_max_position);Serial.print("; x_zero: ");Serial.print(x_zero_position);Serial.println(".");
  Serial.print("y_min: ");Serial.print(y_min_position);Serial.print("; y_max: ");Serial.print(y_max_position);Serial.print("; y_zero: ");Serial.print(y_zero_position);Serial.println(".");
  Serial.println();
  SerialBT.println();
  SerialBT.println("Dati Taratura: ");
  SerialBT.print("X Min: ");SerialBT.print(x_min_position);SerialBT.print("; X Max: ");SerialBT.print(x_max_position);SerialBT.print("; X Zero: ");SerialBT.print(x_zero_position);SerialBT.println(".");
  SerialBT.print("Y Min: ");SerialBT.print(y_min_position);SerialBT.print("; Y Max: ");SerialBT.print(y_max_position);SerialBT.print("; Y_Zero: ");SerialBT.print(y_zero_position);SerialBT.println(".");
  SerialBT.println();
}

void print_pid_constants(void){
  Serial.println();
  Serial.println("Costanti P.I.D:");
  Serial.print("Kp: ");Serial.print(kp);Serial.print("; Ki: ");Serial.print(ki);Serial.print("; Kd: ");Serial.print(kd);Serial.println(".");
  Serial.println();
  SerialBT.println();
  SerialBT.println("Costanti P.I.D:");
  SerialBT.print("Kp: ");SerialBT.print(kp);SerialBT.print("; Ki: ");SerialBT.print(ki);SerialBT.print("; Kd: ");SerialBT.print(kd);SerialBT.println(".");
  SerialBT.println();
}
