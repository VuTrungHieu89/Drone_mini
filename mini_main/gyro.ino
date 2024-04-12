
void gyro_setup(void) {
  HWire.beginTransmission(gyro_address);                      
  HWire.write(0x6B);                                           
  HWire.write(0x00);                                           
  HWire.endTransmission();                                    

  HWire.beginTransmission(gyro_address);                      
  HWire.write(0x1B);                                         
  HWire.write(0x08);                                         
  HWire.endTransmission();                                    
  HWire.beginTransmission(gyro_address);                       
  HWire.write(0x1C);                                          
  HWire.write(0x10);                                            
  HWire.endTransmission();                                

  HWire.beginTransmission(gyro_address);                       
  HWire.write(0x1A);                                          
  HWire.write(0x03);                                          
  HWire.endTransmission();                                  

  acc_pitch_cal_value  = EEPROM.read(0x16);
  acc_roll_cal_value  = EEPROM.read(0x17);
}


void calibrate_gyro(void) {
  cal_int = 0;                                                                      
  if (cal_int != 2000) {
   
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                
      if (cal_int % 25 == 0) digitalWrite(PB4, !digitalRead(PB4));                 
      gyro_signalen();                                                       
      gyro_roll_cal += gyro_roll;                                                
      gyro_pitch_cal += gyro_pitch;                                                 
      gyro_yaw_cal += gyro_yaw;                                                    
      delay(4);                                                                     
    }
    red_led(HIGH);                                                                    

    gyro_roll_cal /= 2000;                                                        
    gyro_pitch_cal /= 2000;                                                        
    gyro_yaw_cal /= 2000;                                                          
  }
}


void gyro_signalen(void) {
  HWire.beginTransmission(gyro_address);                       
  HWire.write(0x3B);                                         
  HWire.endTransmission();                                    
  HWire.requestFrom(gyro_address, 14);                       
  acc_y = HWire.read() << 8 | HWire.read();                  
  acc_x = HWire.read() << 8 | HWire.read();                 
  acc_z = HWire.read() << 8 | HWire.read();                  
  temperature = HWire.read() << 8 | HWire.read();          
  gyro_roll = HWire.read() << 8 | HWire.read();             
  gyro_pitch = HWire.read() << 8 | HWire.read();         
  gyro_yaw = HWire.read() << 8 | HWire.read();             
  gyro_pitch *= -1;                                      
  gyro_yaw *= -1;                                         

  if (level_calibration_on == 0) {
    acc_y -= acc_pitch_cal_value;                         
    acc_x -= acc_roll_cal_value;                          
  }
  if (cal_int >= 2000) {
    gyro_roll -= gyro_roll_cal;                                  
    gyro_pitch -= gyro_pitch_cal;                           
    gyro_yaw -= gyro_yaw_cal;                                
  }
}
