#include<Wire.h>
#include<Servo.h>

#define gyro_address 0x68

long gyro_x, gyro_y, gyro_z;
float acc_x, acc_y, acc_z, acc_total_vector, temp;
int gyro_x_cal, gyro_y_cal, gyro_z_cal;
float angle_pitch, angle_roll, angle_yaw;
float angle_roll_acc, angle_pitch_acc;

float pi = acos(-1);
float to_deg = 1.0 / (250*65.5);
float to_deg_to_rad = to_deg * pi / 180;
float rad_to_deg = 1.0*180 / pi;
long loop_timer;
int start;
long last_time;
int last_val;

int last_yaw, last_roll, last_pitch;

Servo s1, s2, s3, s4;

void setup() {
  
  //Serial.begin(115200);
  
  s1.attach(3, 1000, 2000);
  s2.attach(4, 1000, 2000);
  s3.attach(5, 1000, 2000);
  s4.attach(6, 1000, 2000);
  s1.write(90);
  s2.write(90);
  s3.write(90);
  s4.write(0);

  last_yaw = 90;
  last_roll = 90;
  last_pitch = 90;
  
  
  Wire.begin();        
  TWBR = 12;
  
  pinMode(13, OUTPUT);
  pinMode(A0, INPUT);
  
  digitalWrite(13, HIGH);
  setup_mpu();
  delay(2000);                                      // wait 5 seconds
  calibrate_gyro();
  last_time = 0;
  last_val = 0;
  digitalWrite(13, LOW);
  loop_timer = micros(); 
}

void loop() {
  // put your main code here, to run repeatedly:

  calculate_angle();

  int pitch = angle_pitch;
  int roll = angle_roll;
  int yaw = angle_yaw;
  
  constrain(pitch, -90, 90);
  constrain(roll, -90, 90);
  constrain(yaw, -90, 90);

  pitch += 90;
  roll += 90;
  yaw += 90;

  /*
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  */
  
  roll = 180 - roll;
  pitch = 180 - pitch;
  
  last_yaw = 0.5 * last_yaw + 0.5 * yaw;
  last_pitch = 0.5 * last_pitch + 0.5 * pitch;
  last_roll = 0.5 * last_roll + 0.5 * roll;
  
  s1.write(last_yaw);
  s2.write(last_roll);
  s3.write(last_pitch);
  
  if (analogRead(A0) < 600) last_time = micros();
  else if (micros() - last_time > 1000000) {
      last_time = micros();
      last_val = !last_val;
  }
   
  s4.write(last_val*60);

 
  //Serial.println(last_val*40);
 
  while(loop_timer + 4000 > micros());                                      //Start the pulse after 4000 micro seconds i.e. 250hz loop.
  loop_timer = micros(); 
 
}

void calculate_angle() {
  
  read_mpu();                                                     
  apply_calibration();
  
  angle_pitch += gyro_x * to_deg;                                    
  angle_roll += gyro_y * to_deg;                                    
  angle_yaw += gyro_z * to_deg;

  float tmp_pitch = angle_pitch;
  angle_pitch += angle_roll * sin(gyro_z * to_deg_to_rad);                 
  angle_roll -= tmp_pitch * sin(gyro_z * to_deg_to_rad);                 
  
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       
  
  if(abs(acc_y) < acc_total_vector){                                        
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* rad_to_deg;          
  }
  if(abs(acc_x) < acc_total_vector){                                       
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -rad_to_deg;         
  }
  
  if(start){                                                           
    angle_pitch = angle_pitch * 0.96 + angle_pitch_acc * 0.04;          
    angle_roll = angle_roll * 0.96 + angle_roll_acc * 0.04;           
  }
  else{                                                                    
    angle_pitch = angle_pitch_acc;                                         
    angle_roll = angle_roll_acc;                                          
    start = true;                                                         
  }
  
}

void calibrate_gyro() {
  long gyro_x_cal_tmp = 0, gyro_y_cal_tmp = 0, gyro_z_cal_tmp = 0;
  for (int i = 0; i < 2000; i++){                                    
    read_mpu();                                                      
    gyro_x_cal_tmp += gyro_x;                                            
    gyro_y_cal_tmp += gyro_y;                                              
    gyro_z_cal_tmp += gyro_z;                                            
    delay(2);
  }  
  gyro_x_cal_tmp /= 2000;                                                 
  gyro_y_cal_tmp /= 2000;                                                 
  gyro_z_cal_tmp /= 2000;                                              
  gyro_x_cal = gyro_x_cal_tmp;
  gyro_y_cal = gyro_y_cal_tmp;
  gyro_z_cal = gyro_z_cal_tmp;
}

void apply_calibration() {
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                               
  gyro_z -= gyro_z_cal;                                               
}


void read_mpu(){                                                       
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x3B);                                               
  Wire.endTransmission();                                             
  Wire.requestFrom(0x68,14);                                         
  while(Wire.available() < 14);                                       
  acc_x = Wire.read()<<8|Wire.read();                               
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                
  temp = Wire.read()<<8|Wire.read();                                  
  gyro_x = Wire.read()<<8|Wire.read();                                
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                 
}

void setup_mpu(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                    
  Wire.endTransmission();                                              
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1C);                                                   
  Wire.write(0x10);                                                    
  Wire.endTransmission();                                              
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1B);                                                    
  Wire.write(0x08);                                                    
  Wire.endTransmission();        
  //DLPF to 42 hz
  Wire.beginTransmission(gyro_address);                        
  Wire.write(0x1A);                                            
  Wire.write(0x00);                                            
  Wire.endTransmission();                                                                         
}
