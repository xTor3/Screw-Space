void data_logging(void){
  if((millis() - data_logging_timer) > DATA_LOGGING_TIMER){ 
    
    String dataString = "";
    dataString = String(pitch) + "," + String(roll) + "," + String(yaw) + "," + String(bmp_temperature) + "," + String(mpu_temperature) + "," + String(pressure) + "," + String(actual_altitude) + "," + String(altitude_with_offset) +  "," + String(AccX) + "," + String(AccY) + "," + String(AccZ) + "," + String(p_x) + "," + String(i_x) + "," + String(d_x) + "," + String(x_action) + "," + String(p_y) + "," + String(i_y) + "," + String(d_y) + "," + String(y_action);
    
    if(myFile) myFile.println(dataString); 
    else{
      Serial.print("Errore Apertura ");
      Serial.println(text);
    }
    
    data_logging_timer = millis();
  }
}
