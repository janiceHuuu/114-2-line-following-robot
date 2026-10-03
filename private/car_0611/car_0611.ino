// =========================================================================
// 專案：模組化狀態機循跡車 (ESP32)
// 架構：硬體配置 -> 輸入層 -> 大腦層(狀態機) -> 輸出層(硬體抽象)
// =========================================================================

// ==========================================
// 0. 腳位與參數定義 (Pin Definitions)
// ==========================================
// 輸入層：五路感測器 (從左到右：OUT1 ~ OUT5)
const int SENSOR_L2 = 16;  // OUT1 (最左)
const int SENSOR_L1 = 4;   // OUT2
const int SENSOR_M = 0;    // OUT3 (正中)
const int SENSOR_R1 = 2;   // OUT4
const int SENSOR_R2 = 15;  // OUT5 (最右)

// 輸出層：馬達驅動模組 (TB6612FNG)
const int MOTOR_PWMA = 32;  // A馬達速度
const int MOTOR_AIN1 = 25;  // A馬達方向
const int MOTOR_AIN2 = 33;  // A馬達方向
const int MOTOR_STBY = 26;  // 驅動晶片致能 (高電位工作)
const int MOTOR_BIN1 = 27;  // B馬達方向
const int MOTOR_BIN2 = 14;  // B馬達方向
const int MOTOR_PWMB = 12;  // B馬達速度

// 輸出層：RGB LED
const int LED_R = 17;
const int LED_G = 5;
const int LED_B = 18;

// ESP32 特有 PWM 參數設定
const int PWM_FREQ = 5000;     // 5kHz
const int PWM_RESOLUTION = 8;  // 0 ~ 255
const int CH_A = 0;            // 左/右馬達通道
const int CH_B = 1;            // 右/左馬達通道

// 速度與 PID 核心參數 (可根據現場賽道調整)
const int BASE_SPEED = 160;  // 直線基礎速度
const int TURN_SPEED = 180;  // 直角彎盲轉速度
float Kp = 45.0;             // P 參數：決定修正靈敏度
float Kd = 25.0;             // D 參數：決定煞車阻尼（消除蛇行）
float Ki = 0.0;              // I 參數：通常設為 0

// 全域控制變數
int last_error = 0;        // 紀錄上一次的誤差
int last_valid_error = 0;  // 紀錄最後一次有效的誤差（用於迷路搜線）

// ==========================================
// 1. 定義系統狀態 (System States)
// ==========================================
enum CarState {
  STATE_NORMAL_PID,  // 藍燈：常規 PID 循跡（直線與弧線）
  STATE_LEFT_90,     // 紅燈：觸發左直角彎
  STATE_RIGHT_90,    // 綠燈：觸發右直角彎
  STATE_LOST         // 黃燈：衝出賽道迷路
};

CarState current_state = STATE_NORMAL_PID;  // 初始狀態
// ==========================================
// 函式前方宣告 (Forward Declarations)
// ==========================================
void setLED(int r, int g, int b);
void robotForward(int leftSpeed, int rightSpeed);
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

  // 初始化輸入層 (感測器)
  pinMode(SENSOR_L2, INPUT);
  pinMode(SENSOR_L1, INPUT);
  pinMode(SENSOR_M, INPUT);
  pinMode(SENSOR_R1, INPUT);
  pinMode(SENSOR_R2, INPUT);

  // 初始化輸出層 (馬達)
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  pinMode(MOTOR_BIN1, OUTPUT);
  pinMode(MOTOR_BIN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);  // 啟動馬達驅動晶片

  // ESP32 新版 (3.0+) PWM 配置
  // 直接綁定腳位、頻率、解析度，不需要再手動指定 channel 0 或 1 了
  ledcAttach(MOTOR_PWMA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_PWMB, PWM_FREQ, PWM_RESOLUTION);

  // 初始化指示燈
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  // 開機動畫：白燈亮 1 秒，代表系統正常
  setLED(1, 1, 1);
  delay(1000);
  setLED(0, 0, 0);

}
// ==========================================
// 3. 主程式大腦層 (Control Layer - 狀態機)
// ==========================================
  void loop() {

  // 【一、輸入層】更新目前的系統狀態
  current_state = updateState();

  // 【二、大腦與輸出層】依據不同狀態分流執行，互不干擾
  switch (current_state) {
    
    case STATE_NORMAL_PID:
      setLED(0, 0, 1);       // 藍燈
      run_PID_tracking();    // 執行 PID 弧線/直線循跡
      break;

    case STATE_LEFT_90:
      setLED(1, 0, 0);       // 紅燈
      handle_left_90();      // 執行左直角處理邏輯
      break;

    case STATE_RIGHT_90:
      setLED(0, 1, 0);       // 綠燈
      handle_right_90();     // 執行右直角處理邏輯
      break;

    case STATE_LOST:
      setLED(1, 1, 0);       // 黃燈 (紅+綠)
      handle_lost_track();   // 執行衝出賽道搜線邏輯
      break;
  }
}

// ==========================================
// 4. 軟體輸入層函式 (Input Layer)
// ==========================================

// 計算環境誤差值 (Error)
 // 假設：讀到黑線為 HIGH (1)，白底為 LOW (0)
int calculate_error() {
  int l2 = digitalRead(SENSOR_L2);
  int l1 = digitalRead(SENSOR_L1);
  int m  = digitalRead(SENSOR_M);
  int r1 = digitalRead(SENSOR_R1);
  int r2 = digitalRead(SENSOR_R2);

  // 權重計算法轉換誤差
       if (l2 == 1 && l1 == 0 && m == 0 && r1 == 0 && r2 == 0) return -4;
  else if (l2 == 1 && l1 == 1 && m == 0 && r1 == 0 && r2 == 0) return -3;
  else if (l2 == 0 && l1 == 1 && m == 0 && r1 == 0 && r2 == 0) return -2;
  else if (l2 == 0 && l1 == 1 && m == 1 && r1 == 0 && r2 == 0) return -1;
  else if (l2 == 0 && l1 == 0 && m == 1 && r1 == 0 && r2 == 0) return  0; // 正中央
  else if (l2 == 0 && l1 == 0 && m == 1 && r1 == 1 && r2 == 0) return  1;
  else if (l2 == 0 && l1 == 0 && m == 0 && r1 == 1 && r2 == 0) return  2;
  else if (l2 == 0 && l1 == 0 && m == 0 && r1 == 1 && r2 == 1) return  3;
  else if (l2 == 0 && l1 == 0 && m == 0 && r1 == 0 && r2 == 1) return  4;
  
  // 如果是全開空 (0,0,0,0,0)，回傳上一次的有效誤差，不更新
  return last_error; 
}

//狀態識別器
CarState updateState() {
  int l2 = digitalRead(SENSOR_L2);
  int l1 = digitalRead(SENSOR_L1);
  int m  = digitalRead(SENSOR_M);
  int r1 = digitalRead(SENSOR_R1);
  int r2 = digitalRead(SENSOR_R2);

  // 1. 判斷是否迷路 (全開空)
  if (l2 == 0 && l1 == 0 && m == 0 && r1 == 0 && r2 == 0) {
    return STATE_LOST;
  }
  // 2. 判斷左直角彎特判 (最左邊 L2 踩死，且最右邊沒踩)
  else if (l2 == 1 && r2 == 0) {
    return STATE_LEFT_90;
  }
  // 3. 判斷右直角彎特判 (最右邊 R2 踩死，且最左邊沒踩)
  else if (r2 == 1 && l2 == 0) {
    return STATE_RIGHT_90;
  }
  // 4. 其餘情況皆屬於常規循跡路段
  else {
    return STATE_NORMAL_PID;
  }
}

// ==========================================
// 5. 大腦演算法處理函式 (Control Algorithms)
// ==========================================

//常規 PID 循跡 (走直線與弧線彎道)
void run_PID_tracking() {
  int error = calculate_error();
  
  // 如果誤差有效（不為0），就紀錄為最後有效誤差，留給迷路狀態使用
  if (error != 0) {
    last_valid_error = error;
  }

  // PID 核心公式
  int P = error;
  int D = error - last_error;
  // 常規循跡中，通常不使用 I 或是限制其範圍，這裡設為 0
  int I = 0; 

  int output = (Kp * P) + (Kd * D);
  last_error = error; // 紀錄誤差

  // 根據 PID 輸出修正左右輪速度 ( output > 0 代表車子偏左，需向右修正 )
  int left_speed  = BASE_SPEED + output;
  int right_speed = BASE_SPEED - output;

  // 呼叫語意化輸出
  robotForward(left_speed, right_speed);
}

//左直角彎特殊處理 (強制盲轉直到回正)
 
void handle_left_90() {
  // 步驟一：先執行短暫的強制原地左旋轉，強制讓感測器擺脫當前的交叉黑線干擾
  robotSpinLeft(TURN_SPEED);
  delay(80); // 盲轉延時 (毫秒)，可依車速調整

  // 步驟二：持續左轉，直到中間感測器（M）重新接觸到黑線
  while (digitalRead(SENSOR_M) == LOW) {
    robotSpinLeft(TURN_SPEED);
    delay(1); // 避免無窮迴圈鎖死 CPU
  }

  // 步驟三：抓回黑線了，急煞車穩定姿態，重設 PID 記憶，準備無縫切回 PID 模式
  robotBrake();
  delay(30);
  last_error = 0; 
}

//右直角彎特殊處理
 
void handle_right_90() {
  robotSpinRight(TURN_SPEED);
  delay(80); 

  while (digitalRead(SENSOR_M) == LOW) {
    robotSpinRight(TURN_SPEED);
    delay(1);
  }

  robotBrake();
  delay(30);
  last_error = 0;
}


 //衝出賽道處理 (憑記憶搜線)
 
void handle_lost_track() {
  // 讀取最後一次有效誤差的方向
  if (last_valid_error < 0) {
    // 最後記憶是偏右，代表黑線在左邊 -> 執行原地左轉搜線
    robotSpinLeft(BASE_SPEED);
  } else {
    // 最後記憶是偏左，代表黑線在右邊 -> 執行原地右轉搜線
    robotSpinRight(BASE_SPEED);
  }
}

// ==========================================
// 6. 軟體輸出層：硬體抽象化 API (新版 ESP32 3.0 專用)
// ==========================================

void robotForward(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, LOW);
  ledcWrite(MOTOR_PWMA, leftSpeed); // 新版直接寫腳位 MOTOR_PWMA

  digitalWrite(MOTOR_BIN1, HIGH);
  digitalWrite(MOTOR_BIN2, LOW);
  ledcWrite(MOTOR_PWMB, rightSpeed); // 新版直接寫腳位 MOTOR_PWMB
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

// 語意：指示燈控制 (補回實體內容)
 
void setLED(int r, int g, int b) {
  // 這裡假設是共陰極 LED (HIGH會亮)
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
}
