// =========================================================================
// 專案：模組化狀態機循跡車 (ESP32 - 完美分層終極修正版)
// 架構：同學修改版腳位 -> 輸入層 -> 大腦層(狀態機) -> 輸出層(硬體抽象)
// =========================================================================

// ==========================================
// 0. 腳位與參數定義 (同學修改版)
// ==========================================
const int LED_R = 17;
const int LED_G = 23; 
const int LED_B = 18;

const int SENSOR_L2 = 16; 
const int SENSOR_L1 = 4;  
const int SENSOR_M  = 21; 
const int SENSOR_R1 = 22; 
const int SENSOR_R2 = 34; 

const int MOTOR_PWMA = 32;
const int MOTOR_AIN1 = 25;
const int MOTOR_AIN2 = 33;
const int MOTOR_PWMB = 19; 
const int MOTOR_BIN1 = 27;
const int MOTOR_BIN2 = 14;
const int MOTOR_STBY = 26;

const int PWM_FREQ = 5000;      
const int PWM_RESOLUTION = 8;   

const int BASE_SPEED = 150;     
const int TURN_SPEED = 140;     
float Kp = 35.0;                
float Kd = 10.0;                
float Ki = 0.0;

int last_error = 0;             
int last_valid_error = 0; 
unsigned long lost_start_time = 0; 
bool is_lost_counting = false;      

// ==========================================
// 1. 定義系統狀態 (System States)
// ==========================================
enum CarState {
  STATE_NORMAL_PID,  
  STATE_LEFT_90,     
  STATE_RIGHT_90,    
  STATE_LOST         
};

CarState current_state = STATE_NORMAL_PID; 

// ==========================================
// 函式前方宣告 (新增 robotReverse 宣告)
// ==========================================
void setLED(int r, int g, int b);
void robotForward(int leftSpeed, int rightSpeed);
void robotReverse(int leftSpeed, int rightSpeed); // 新增倒車抽象層
void robotSpinLeft(int speed);
void robotSpinRight(int speed);
void robotBrake();
CarState updateState();
int calculate_error();
void run_PID_tracking();
void handle_left_90();
void handle_right_90();
void handle_lost_track();

// ==========================================
// 2. 初始化設定 (Setup)
// ==========================================
void setup() {
  Serial.begin(115200);

  pinMode(SENSOR_L2, INPUT);
  pinMode(SENSOR_L1, INPUT);
  pinMode(SENSOR_M,  INPUT);
  pinMode(SENSOR_R1, INPUT);
  pinMode(SENSOR_R2, INPUT);

  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  pinMode(MOTOR_BIN1, OUTPUT);
  pinMode(MOTOR_BIN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH); 

  ledcAttach(MOTOR_PWMA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_PWMB, PWM_FREQ, PWM_RESOLUTION);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  
  setLED(1, 1, 1); delay(1000); setLED(0, 0, 0);
}

// ==========================================
// 3. 主程式大腦層 (Control Layer - 狀態機)
// ==========================================
void loop() {
  current_state = updateState();

  switch (current_state) {
    case STATE_NORMAL_PID:
      setLED(0, 0, 1);       
      run_PID_tracking();    
      break;

    case STATE_LEFT_90:
      setLED(1, 0, 0);       
      handle_left_90();      
      break;

    case STATE_RIGHT_90:
      setLED(0, 1, 0);       
      handle_right_90();     
      break;

    case STATE_LOST:
      setLED(1, 1, 0);       
      handle_lost_track();   
      break;
  }
}

// ==========================================
// 4. 軟體輸入層函式 (Input Layer)
// ==========================================
int calculate_error() {
  int l2 = !digitalRead(SENSOR_L2);
  int l1 = !digitalRead(SENSOR_L1);
  int m  = !digitalRead(SENSOR_M);
  int r1 = !digitalRead(SENSOR_R1);
  int r2 = !digitalRead(SENSOR_R2);

       if (l2 == 1 && l1 == 0 && m == 0 && r1 == 0 && r2 == 0) return -4;
  else if (l2 == 1 && l1 == 1 && m == 0 && r1 == 0 && r2 == 0) return -3;
  else if (l2 == 0 && l1 == 1 && m == 0 && r1 == 0 && r2 == 0) return -2;
  else if (l2 == 0 && l1 == 1 && m == 1 && r1 == 0 && r2 == 0) return -1;
  else if (l2 == 0 && l1 == 0 && m == 1 && r1 == 0 && r2 == 0) return  0; 
  else if (l2 == 0 && l1 == 0 && m == 1 && r1 == 1 && r2 == 0) return  1;
  else if (l2 == 0 && l1 == 0 && m == 0 && r1 == 1 && r2 == 0) return  2;
  else if (l2 == 0 && l1 == 0 && m == 0 && r1 == 1 && r2 == 1) return  3;
  else if (l2 == 0 && l1 == 0 && m == 0 && r1 == 0 && r2 == 1) return  4;
  
  return last_error; 
}

CarState updateState() {
  int l2 = !digitalRead(SENSOR_L2);
  int l1 = !digitalRead(SENSOR_L1);
  int m  = !digitalRead(SENSOR_M);
  int r1 = !digitalRead(SENSOR_R1);
  int r2 = !digitalRead(SENSOR_R2);

  if (l2 == 0 && l1 == 0 && m == 0 && r1 == 0 && r2 == 0) {
    return STATE_LOST;
  }
  else if (l2 == 1 && r2 == 0) {
    return STATE_LEFT_90;
  }
  else if (r2 == 1 && l2 == 0) {
    return STATE_RIGHT_90;
  }
  else {
    return STATE_NORMAL_PID;
  }
}

// ==========================================
// 5. 大腦演算法處理函式 (Control Algorithms)
// ==========================================
void run_PID_tracking() {
  is_lost_counting = false; 
  
  int error = calculate_error();
  if (error != 0) {
    last_valid_error = error;
  }

  int P = error;
  int D = error - last_error;
  int output = (Kp * P) + (Kd * D);
  last_error = error;

  int left_speed  = BASE_SPEED + output;
  int right_speed = BASE_SPEED - output;

  robotForward(left_speed, right_speed);
}

void handle_left_90() {
  robotBrake();
  delay(40); 

  robotSpinLeft(TURN_SPEED);
  delay(40); 

  while (digitalRead(SENSOR_L1) == HIGH && 
         digitalRead(SENSOR_M)  == HIGH && 
         digitalRead(SENSOR_R1) == HIGH) {
    robotSpinLeft(TURN_SPEED);
    delay(1); 
  }

  robotBrake();
  delay(20);
  last_error = 0; 
}

void handle_right_90() {
  robotBrake();
  delay(40); 

  robotSpinRight(TURN_SPEED);
  delay(40); 

  while (digitalRead(SENSOR_L1) == HIGH && 
         digitalRead(SENSOR_M)  == HIGH && 
         digitalRead(SENSOR_R1) == HIGH) {
    robotSpinRight(TURN_SPEED);
    delay(1);
  }

  robotBrake();
  delay(20);
  last_error = 0;
}

void handle_lost_track() {
  if (!is_lost_counting) {
    lost_start_time = millis(); 
    is_lost_counting = true;
  }

  unsigned long duration = millis() - lost_start_time;

  if (duration > 5000) {
    robotBrake();
    setLED(1, 0, 0); 
    while(1);        
  }

  int reverse_speed_fast = 130;
  int reverse_speed_slow = 40;

  // 【優化】這裡不再直接寫硬體腳位，100% 呼叫第六區的語意化 API
  if (last_valid_error < 0) {
    robotReverse(reverse_speed_fast, reverse_speed_slow);
  } else {
    robotReverse(reverse_speed_slow, reverse_speed_fast);
  }
}

// ==========================================
// 6. 軟體輸出層：硬體抽象化 API (第六區隆重歸位)
// ==========================================

void robotForward(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, LOW);
  ledcWrite(MOTOR_PWMA, leftSpeed); 

  digitalWrite(MOTOR_BIN1, HIGH);
  digitalWrite(MOTOR_BIN2, LOW);
  ledcWrite(MOTOR_PWMB, rightSpeed); 
}

void robotReverse(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // 倒車：兩輪皆為 LOW / HIGH
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, HIGH);
  ledcWrite(MOTOR_PWMA, leftSpeed); 

  digitalWrite(MOTOR_BIN1, LOW);
  digitalWrite(MOTOR_BIN2, HIGH);
  ledcWrite(MOTOR_PWMB, rightSpeed); 
}

void robotSpinLeft(int speed) {
  speed = constrain(speed, 0, 255);

  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, HIGH);
  ledcWrite(MOTOR_PWMA, speed);

  digitalWrite(MOTOR_BIN1, HIGH);
  digitalWrite(MOTOR_BIN2, LOW);
  ledcWrite(MOTOR_PWMB, speed);
}

void robotSpinRight(int speed) {
  speed = constrain(speed, 0, 255);

  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, LOW);
  ledcWrite(MOTOR_PWMA, speed);

  digitalWrite(MOTOR_BIN1, LOW);
  digitalWrite(MOTOR_BIN2, HIGH);
  ledcWrite(MOTOR_PWMB, speed);
}

void robotBrake() {
  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, HIGH);
  digitalWrite(MOTOR_BIN1, HIGH);
  digitalWrite(MOTOR_BIN2, HIGH);
  ledcWrite(MOTOR_PWMA, 0);
  ledcWrite(MOTOR_PWMB, 0);
}

void setLED(int r, int g, int b) {
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
}