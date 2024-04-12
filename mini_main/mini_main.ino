
#include <EEPROM.h>
#include <Wire.h>                         
TwoWire HWire (2, I2C_FAST_MODE);        


float pid_p_gain_roll = 1.3;              
float pid_i_gain_roll = 0.04;            
float pid_d_gain_roll = 18.0;             
int pid_max_roll = 400;                   
float pid_p_gain_pitch = pid_p_gain_roll; 
float pid_i_gain_pitch = pid_i_gain_roll;  
float pid_d_gain_pitch = pid_d_gain_roll;  
int pid_max_pitch = pid_max_roll;         

float pid_p_gain_yaw = 4.0;                
float pid_i_gain_yaw = 0.02;             
float pid_d_gain_yaw = 0.0;                
int pid_max_yaw = 400;                    


float battery_compensation = 40.0;

int16_t manual_takeoff_throttle = 1505;   
int16_t motor_idle_speed = 1100;          

uint8_t gyro_address = 0x68;              
float battery_voltage_calibration = 0.0;  
float low_battery_warning = 3.4;       


uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
uint8_t check_byte, flip32, start;
uint8_t error, error_counter, error_led;
uint8_t flight_mode, flight_mode_counter, flight_mode_led;
uint8_t takeoff_detected, manual_altitude_change;
uint8_t channel_select_counter;
uint8_t level_calibration_on;

int16_t esc_1, esc_2, esc_3, esc_4;
int16_t manual_throttle;
int16_t throttle, takeoff_throttle, cal_int;
int16_t temperature, count_var;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;

int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t measured_time, measured_time_start, receiver_watchdog;
int32_t acc_total_vector, acc_total_vector_at_start;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
int16_t acc_pitch_cal_value;
int16_t acc_roll_cal_value;

uint32_t loop_timer, error_timer, flight_mode_timer;
uint32_t delay_micros_timer;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
float battery_voltage, dummy_float;


void setup() {
  pinMode(4, INPUT_ANALOG);                                   

  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);               
  flip32 = 0;

  pinMode(PB3, OUTPUT);                                      
  pinMode(PB4, OUTPUT);                                      

  green_led(LOW);                                              
  red_led(HIGH);                                             

  pinMode(PB0, OUTPUT);                                        


  EEPROM.PageBase0 = 0x801F000;
  EEPROM.PageBase1 = 0x801F800;
  EEPROM.PageSize  = 0x400;

  //Serial.begin(57600);                                 
  //delay(250);                                              

  timer_setup();                                             
  delay(50);                                                


  //Check if the MPU-6050 is responding.
  HWire.begin();                                               
  HWire.beginTransmission(gyro_address);                       
  error = HWire.endTransmission();                             
  while (error != 0) {                                         
    error = 1;                                                 
    error_signal();                                          
    delay(4);                                                 
  }

  gyro_setup();                                               


  for (count_var = 0; count_var < 1250; count_var++) {         
    if (count_var % 125 == 0) {                               
      digitalWrite(PB4, !digitalRead(PB4));                    
    }
    delay(4);                                                  
  }
  count_var = 0;                                            
  calibrate_gyro();                                           


  while (channel_1 < 990 || channel_2 < 990 || channel_3 < 990 || channel_4 < 990)  {
    error = 4;                                                 
    error_signal();                                          
    delay(4);                                                 
  }
  error = 0;                                                  



  red_led(LOW);                                                

  battery_voltage = (float)analogRead(4) / 112.81;
  
  start = 0;

  if (motor_idle_speed < 1000)motor_idle_speed = 1000;          //Limit the minimum idle motor speed to 1000us.
  if (motor_idle_speed > 1200)motor_idle_speed = 1200;          //Limit the maximum idle motor speed to 1200us.

  loop_timer = micros();                                        //Set the timer for the first loop.
}

void loop() {
  if (receiver_watchdog < 750)receiver_watchdog ++;
  if (receiver_watchdog == 750 && start == 2) {
    channel_1 = 1500;
    channel_2 = 1500;
    channel_3 = 1500;
    channel_4 = 1500;
    error = 8;
 
  }
 
  if (start == 0) {
   
    //Level calibration move both sticks to the top left.
    if (channel_1 < 1100 && channel_2 < 1100 && channel_3 > 1900 && channel_4 < 1100)calibrate_level();
  
  }

  flight_mode = 1;                                                               
  green_led(HIGH);     


  error_signal();                                                              
  gyro_signalen();                                                               


  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);    





  angle_pitch += (float)gyro_pitch * 0.0000611;                                   
  angle_roll += (float)gyro_roll * 0.0000611;                                      
 

  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);              
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);              

  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));  

  if (abs(acc_y) < acc_total_vector) {                                            
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              
  }
  if (abs(acc_x) < acc_total_vector) {                                           
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;            
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                  
  pitch_level_adjust = angle_pitch * 15;                                          
  roll_level_adjust = angle_roll * 15;                                           

  calculate_pid();                                                              
    if (channel_3 < 1100 && channel_4 < 1100) {
        start = 1;
    }
     if (start == 1 && channel_3 < 1100 && channel_4 > 1445) {
        start = 2;
        green_led(LOW);  
                                                          
        angle_pitch = angle_pitch_acc;                                        
        angle_roll = angle_roll_acc;                                          

        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
    }

    if (start == 2 && channel_3 < 1100 &&  channel_4 > 1900) {
        start = 0;
        green_led(HIGH);                                                    
    }


  battery_voltage = (battery_voltage * 0.92) + ((((float)analogRead(4) / 112.81) + battery_voltage_calibration) * 0.08);


  if (battery_voltage > 3.3 && battery_voltage < low_battery_warning && error == 0)error = 1;

  throttle = channel_3;



  if (start == 2) {                                                             
    if (throttle > 1800) throttle = 1800;                                        
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;      
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;      
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;      
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;      

    if (battery_voltage < 3.60 && battery_voltage > 3.4) {                       
      esc_1 += (3.60 - battery_voltage) * battery_compensation;                   
      esc_2 += (3.60 - battery_voltage) * battery_compensation;                  
      esc_3 += (3.60 - battery_voltage) * battery_compensation;                  
      esc_4 += (3.60 - battery_voltage) * battery_compensation;
    }

    if (esc_1 < motor_idle_speed) esc_1 = motor_idle_speed;                      
    if (esc_2 < motor_idle_speed) esc_2 = motor_idle_speed;                       
    if (esc_3 < motor_idle_speed) esc_3 = motor_idle_speed;                       
    if (esc_4 < motor_idle_speed) esc_4 = motor_idle_speed;                        

    if (esc_1 > 2000)esc_1 = 2000;                                              
    if (esc_2 > 2000)esc_2 = 2000;                                               
    if (esc_3 > 2000)esc_3 = 2000;                                             
    if (esc_4 > 2000)esc_4 = 2000;                                                
  }

  else {
    esc_1 = 1000;                                                                 
    esc_2 = 1000;                                                                 
    esc_3 = 1000;                                                                 
    esc_4 = 1000;                                                                 
  }


  TIMER4_BASE->CCR1 = (esc_1 - 1000)*3;                                                      
  TIMER4_BASE->CCR2 = (esc_2 - 1000)*3;                                                      
  TIMER4_BASE->CCR3 = (esc_3 - 1000)*3;                                                       
  TIMER4_BASE->CCR4 = (esc_4 - 1000)*3;                                                      
  TIMER4_BASE->CNT = 5000;                                                        



  if (micros() - loop_timer > 4050)error = 2;                                     
  while (micros() - loop_timer < 4000);                                         
  loop_timer = micros();                                                         
}
