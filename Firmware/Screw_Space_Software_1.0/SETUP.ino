void servo_setup_animation(void){
  for(int i = 0; i <= 360; i++){
    float x = cos((float)i * DEG_TO_RAD) * (float)180;
    float y = sin((float)i * DEG_TO_RAD) * (float)180;

    x_axis.write(map((int)x, -180, 180, x_min_position, x_max_position));
    y_axis.write(map((int)y, -180, 180, y_min_position, y_max_position));
    delay(6);
  }
  
  x_axis.write(x_zero_position);
  y_axis.write(y_zero_position);
}


void EEPROM_setup(void){
  EEPROM.begin(EEPROM_SIZE);
  if(EEPROM.read(x_zero_position_address) != 255) x_zero_position = EEPROM.read(x_zero_position_address);
  if(EEPROM.read(x_min_position_address) != 255) x_min_position = EEPROM.read(x_min_position_address);
  if(EEPROM.read(x_max_position_address) != 255) x_max_position = EEPROM.read(x_max_position_address);
  if(EEPROM.read(y_zero_position_address) != 255) y_zero_position = EEPROM.read(y_zero_position_address);
  if(EEPROM.read(y_min_position_address) != 255) y_min_position = EEPROM.read(y_min_position_address);
  if(EEPROM.read(y_max_position_address) != 255) y_max_position = EEPROM.read(y_max_position_address);

  if(!isnan(EEPROM.readFloat(pid_constants_address))) kp = EEPROM.readFloat(pid_constants_address);
  pid_constants_address += sizeof(float);
  if(!isnan(EEPROM.readFloat(pid_constants_address))) ki = EEPROM.readFloat(pid_constants_address);
  pid_constants_address += sizeof(float);
  if(!isnan(EEPROM.readFloat(pid_constants_address))) kd = EEPROM.readFloat(pid_constants_address);
  
  pid_constants_address = actual_pid_constants_address;
}


/*
for(int a = 0; a < 2; a++){
  for(int i = x_zero_position; i < x_max_position; i++){
    x_axis.write(i);
    delay(2);
  }
  for(int i = x_max_position; i > x_zero_position; i--){
    x_axis.write(i);
    delay(2);
  }
  for(int i = x_zero_position; i > x_min_position; i--){
    x_axis.write(i);
    delay(2);
  }
  for(int i = x_min_position; i <= x_zero_position; i++){
    x_axis.write(i);
    delay(2);
  }
}

for(int a = 0; a < 3; a++){
  for(int i = y_zero_position; i < y_max_position; i++){
    y_axis.write(i);
    delay(2);
  }
  for(int i = y_max_position; i > y_zero_position; i--){
    y_axis.write(i);
    delay(2);
  }
  for(int i = y_zero_position; i > y_min_position; i--){
    y_axis.write(i);
    delay(2);
  }
  for(int i = y_min_position; i <= y_zero_position; i++){
    y_axis.write(i);
    delay(2);
  }
}
*/
