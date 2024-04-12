
void red_led(int8_t level) {
  if (flip32)digitalWrite(PB4, !level);  
  else digitalWrite(PB4, level);          
}
void green_led(int8_t level) {
  if (flip32)digitalWrite(PB3, !level);    
  else digitalWrite(PB3, level);          
}


void error_signal(void) {
  if (error >= 100) red_led(HIGH);                                                     
  else if (error_timer < millis()) {                                                    
    error_timer = millis() + 250;                                                         
    if (error > 0 && error_counter > error + 3) error_counter = 0;                      
    if (error_counter < error && error_led == 0 && error > 0) {                           
      red_led(HIGH);                                                                  
      error_led = 1;                                                                      
    }
    else {                                                                              
      red_led(LOW);                                                                 
      error_counter++;                                                                   
      error_led = 0;                                                                     
    }
  }
}
