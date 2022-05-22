unsigned long print_mpu_timer_1 = 0;

void mpu_refresh(void){
  if((millis() - mpu_read_timer) >= MPU_REFRESH_DELAY){ 

    //Calcolo Angoli con Giroscopio
    f_pitch = mpu.getAngleX();
    f_roll = mpu.getAngleY();  
    f_yaw = mpu.getAngleZ();

    //Calcolo Angoli con Accelerometro
    Acc_pitch = mpu.getAccAngleX();
    Acc_roll = mpu.getAccAngleY();

    if((f_pitch-Acc_pitch) >= 5.00 || (f_pitch-Acc_pitch) <= -5.00){
      mpu.setFilterGyroCoef(0.9998); 
    }
    else mpu.setFilterGyroCoef(0.98);

    //Filtraggio
    pitch = f_pitch * 1 + Acc_pitch * 0; //0.0002
    roll = f_roll * 1 + Acc_roll * 0;
    

    if((millis() - print_mpu_timer_1) > 50){
      Serial.print(pitch);
      Serial.print(",");
      Serial.print(roll);
      Serial.print(",");
      Serial.print(yaw);
      Serial.print(",");
      Serial.print(Acc_pitch);
      Serial.print(",");
      Serial.println(Acc_roll);
      
      print_mpu_timer_1 = millis();
    }


    AccX = 10 * mpu.getAccX();
    AccY = 10 * mpu.getAccY();
    AccZ = 10 * mpu.getAccZ();

    mpu_temperature = mpu.getTemp();


    //CALCOLO MEDIA VALORI ACCELERAZIONE:
    if(stato_razzo == 10){
      if(numero_test < numero_di_test_soglia){
        valori_media_acc[numero_test] = AccZ;
        numero_test++;
        //Serial.print(valori_media_acc[numero_test]); Serial.print(" ");
      }
      else{
        media = 0;
        for(int i = 0; i < numero_di_test_soglia; i++){
          media += valori_media_acc[i];
        }
        media /= numero_di_test_soglia;
        numero_test = 0;
  
        //Serial.println(); Serial.println(media); Serial.println();
      }
    }
    else numero_test = 0;

    //Serial.print(pitch);Serial.print(", ");Serial.println(roll);

    mpu_read_timer = millis();  
  }
}
