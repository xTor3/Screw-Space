void bt_read(void){

  bt_receive();
  
  //BT ACTIONS:
  if(message == "imu print") bt_imu_print = !bt_imu_print;
  else if(message == "bmp print") bt_bmp_print = !bt_bmp_print;
  else if(message == "pid print") bt_pid_print = !bt_pid_print;
  
  else if(message == "led") led_enable = !led_enable;
  else if(message == "buzzer") buzzer_enable = !buzzer_enable;

  else if(message == "mos1"){
    bt_mos1_enable = !bt_mos1_enable; 
    if(!bt_mos1_enable) digitalWrite(MOS1, LOW);
  }
  else if(message == "test_mos1") mos1_state = !mos1_state;
  
  else if(message == "tvc enable"){
    tvc_enable = !tvc_enable;
    tvc_state_change();
  }
  
  else if(message == "calibrazione servo") taratura_servo();
  else if(message == "costanti pid") modifica_costanti_pid();
}

void bt_receive(void){
  if(SerialBT.available()){
    char incomingChar = SerialBT.read();
    if (incomingChar != '\n') message += String(incomingChar);
    else message = "";
    //Serial.write(incomingChar);  
  }
}

void bt_print_imu(void){
    SerialBT.print("Pitch: "); SerialBT.print(pitch); SerialBT.print(" ");
    SerialBT.print("Roll: "); SerialBT.print(roll); SerialBT.print(" ");
    SerialBT.print("Yaw: "); SerialBT.print(yaw); SerialBT.println();

    SerialBT.print("Acc_Pitch: "); SerialBT.print(Acc_pitch); SerialBT.print(" ");
    SerialBT.print("Acc_Roll: "); SerialBT.print(Acc_roll); SerialBT.println();

    SerialBT.print("AccX: "); SerialBT.print(AccX); SerialBT.print(" ");
    SerialBT.print("AccY: "); SerialBT.print(AccY); SerialBT.print(" ");
    SerialBT.print("AccZ: "); SerialBT.print(AccZ); SerialBT.println();

    SerialBT.print("MPU Temp: "); SerialBT.print(mpu_temperature); SerialBT.println();

    SerialBT.println();
}

void bt_print_pid(void){
    SerialBT.print("X_p: "); SerialBT.print(p_x); SerialBT.print(" ");
    SerialBT.print("X_i: "); SerialBT.print(i_x); SerialBT.print(" ");
    SerialBT.print("X_d: "); SerialBT.print(d_x); SerialBT.println();

    SerialBT.print("Y_p: "); SerialBT.print(p_y); SerialBT.print(" ");
    SerialBT.print("Y_i: "); SerialBT.print(i_y); SerialBT.print(" ");
    SerialBT.print("Y_d: "); SerialBT.print(d_y); SerialBT.println();

    SerialBT.println();
}

void bt_print_bmp(void){
    SerialBT.print("Temp.: "); SerialBT.print(bmp_temperature); SerialBT.print(" ");
    SerialBT.print("Pressure: "); SerialBT.print(pressure); SerialBT.println();
    SerialBT.print("Altitude: "); SerialBT.print(actual_altitude); SerialBT.print(" ");
    SerialBT.print("Alt. with offset: "); SerialBT.print(altitude_with_offset); SerialBT.println();

    SerialBT.println();
}
