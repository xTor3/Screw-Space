void bmp_refresh(void){
  if((millis() - bmp_read_timer) >= BMP_REFRESH_DELAY){
    bmp_temperature = bmp.readTemperature();
    pressure = bmp.readPressure();
    actual_altitude = bmp.readAltitude(1013.25);
    altitude_with_offset = actual_altitude - offset_bmp;
    
    bmp_read_timer = millis();
  }
}
