#include <WiFi.h>
#include <WebServer.h>

// =========================================================================
// 專案：模組化狀態機循跡車 (ESP32 - 完美分層終極修正版)
// 架構：同學修改版腳位 -> 輸入層 -> 大腦層(狀態機) -> 輸出層(硬體抽象)
// =========================================================================

// ==========================================
// HTTP EBS 與 VLS 設定
// ==========================================

const char *EBS_WIFI_SSID = "ESP32-EBS";
const char *EBS_WIFI_PASSWORD = "12345678";

WebServer server(80);

// 開機預設為安全停止。
// 手機網頁按下 ARM 後，直接視為 VLS 起跑觸發，立即開始自主循跡。
volatile bool ebsLatched = true;
volatile bool vlsStarted = false;

// 本版本將 HTTP ARM 直接作為 VLS 起跑觸發。
// 不需要另外接 GPIO13 實體 VLS 開關。
// 保留原有 VLS 程式碼，但固定停用實體 VLS。
const bool USE_VLS = false;
const int VLS_PIN = 13;
const int VLS_ACTIVE_LEVEL = LOW;
const unsigned long VLS_DEBOUNCE_MS = 30;
unsigned long vlsActiveStartMs = 0;

enum AslMode {
  ASL_SAFE,
  ASL_AUTONOMOUS,
  ASL_OTHER
};

volatile AslMode aslMode = ASL_OTHER;

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

// HTTP EBS、VLS 與 ASL 輔助函式
void setAslMode(AslMode newMode);
void forceSafeStop();
void startHttpEbsServer();
void ebsHttpTask(void *parameter);
void updateVlsStart();

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

  ledcAttach(MOTOR_PWMA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_PWMB, PWM_FREQ, PWM_RESOLUTION);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  if (USE_VLS) {
    pinMode(VLS_PIN, INPUT_PULLUP);
  }

  // 初始化期間：藍燈。
  setAslMode(ASL_OTHER);

  // 開機時先強制停止馬達。
  forceSafeStop();

  // HTTP EBS 使用獨立 task 處理請求。
  // 即使原本循跡函式暫時卡在 while 迴圈，仍可接收 STOP。
  startHttpEbsServer();

  // 初始化完成後進入安全狀態，等待手機 ARM。
  setAslMode(ASL_SAFE);

  Serial.println("Line follower ready. Press ARM on HTTP page to start autonomous navigation.");
}

// ==========================================
// 3. 主程式大腦層 (Control Layer - 狀態機)
// ==========================================
void loop() {
  // EBS STOP 優先於所有循跡邏輯。
  if (ebsLatched) {
    forceSafeStop();
    setAslMode(ASL_SAFE);
    delay(2);
    return;
  }

  // HTTP ARM 直接視為 VLS 起跑觸發；本版本不等待實體開關。
  updateVlsStart();

  if (!vlsStarted) {
    forceSafeStop();
    setAslMode(ASL_SAFE);
    delay(2);
    return;
  }

  // ARM 且 VLS 已觸發：自主導航狀態。
  digitalWrite(MOTOR_STBY, HIGH);
  setAslMode(ASL_AUTONOMOUS);

  // 以下保留原本循跡狀態機，不修改其判斷邏輯。
  current_state = updateState();

  switch (current_state) {
    case STATE_NORMAL_PID:
      run_PID_tracking();
      break;

    case STATE_LEFT_90:
      handle_left_90();
      break;

    case STATE_RIGHT_90:
      handle_right_90();
      break;

    case STATE_LOST:
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
  // 保留原本循跡函式中的 setLED() 呼叫，但 ASL 顏色只由車輛模式決定。
  (void)r;
  (void)g;
  (void)b;

  digitalWrite(LED_R, aslMode == ASL_AUTONOMOUS ? HIGH : LOW);
  digitalWrite(LED_G, aslMode == ASL_SAFE       ? HIGH : LOW);
  digitalWrite(LED_B, aslMode == ASL_OTHER      ? HIGH : LOW);
}

void setAslMode(AslMode newMode) {
  aslMode = newMode;
  setLED(0, 0, 0);
}

// ==========================================
// 7. HTTP EBS、VLS 與 ASL 控制
// ==========================================

void forceSafeStop() {
  ledcWrite(MOTOR_PWMA, 0);
  ledcWrite(MOTOR_PWMB, 0);

  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, LOW);
  digitalWrite(MOTOR_BIN1, LOW);
  digitalWrite(MOTOR_BIN2, LOW);
  digitalWrite(MOTOR_STBY, LOW);
}

void updateVlsStart() {
  if (!USE_VLS) {
    vlsStarted = true;
    return;
  }

  if (vlsStarted) {
    return;
  }

  if (digitalRead(VLS_PIN) == VLS_ACTIVE_LEVEL) {
    if (vlsActiveStartMs == 0) {
      vlsActiveStartMs = millis();
    }

    if (millis() - vlsActiveStartMs >= VLS_DEBOUNCE_MS) {
      vlsStarted = true;
      digitalWrite(MOTOR_STBY, HIGH);
      setAslMode(ASL_AUTONOMOUS);
      Serial.println("VLS triggered: autonomous navigation started.");
    }
  } else {
    vlsActiveStartMs = 0;
  }
}

String buildEbsHtml() {
  String html = "";
  html += "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:sans-serif;text-align:center;margin-top:36px;}";
  html += "button{width:270px;padding:22px;margin:12px;font-size:26px;border:none;border-radius:12px;}";
  html += ".stop{background:#d32f2f;color:white;}";
  html += ".arm{background:#388e3c;color:white;}</style></head><body>";
  html += "<h1>Line Follower HTTP EBS</h1>";

  if (ebsLatched) {
    html += "<h2>Status: SAFE STOP</h2>";
    html += "<p>Motor output is disabled. ASL should be GREEN.</p>";
  } else if (!vlsStarted) {
    html += "<h2>Status: ARMED - WAITING FOR VLS</h2>";
    html += "<p>EBS is armed. Motor output remains disabled until VLS is triggered.</p>";
  } else {
    html += "<h2>Status: AUTONOMOUS NAVIGATION</h2>";
    html += "<p>Line-following program is running. ASL should be RED.</p>";
  }

  html += "<p><a href='/stop'><button class='stop'>STOP - Emergency Stop</button></a></p>";
  html += "<p><a href='/arm'><button class='arm'>ARM / VLS START - Enable Vehicle</button></a></p>";
  html += "</body></html>";
  return html;
}

void handleEbsRoot() {
  server.send(200, "text/html; charset=UTF-8", buildEbsHtml());
}

void handleEbsStop() {
  ebsLatched = true;
  vlsStarted = false;
  vlsActiveStartMs = 0;
  forceSafeStop();
  setAslMode(ASL_SAFE);
  Serial.println("HTTP EBS STOP: motor output disabled.");
  server.send(200, "text/html; charset=UTF-8", buildEbsHtml());
}

void handleEbsArm() {
  ebsLatched = false;
  vlsStarted = !USE_VLS;
  vlsActiveStartMs = 0;

  if (vlsStarted) {
    digitalWrite(MOTOR_STBY, HIGH);
    setAslMode(ASL_AUTONOMOUS);
    Serial.println("HTTP EBS ARMED: autonomous navigation enabled.");
  } else {
    forceSafeStop();
    setAslMode(ASL_SAFE);
    Serial.println("HTTP EBS ARMED: waiting for VLS trigger.");
  }

  server.send(200, "text/html; charset=UTF-8", buildEbsHtml());
}

void handleEbsNotFound() {
  server.send(404, "text/plain; charset=UTF-8", "404: page not found");
}

void ebsHttpTask(void *parameter) {
  (void)parameter;

  for (;;) {
    server.handleClient();

    // 不修改原本 LOST 邏輯；僅在其 5 秒安全停止條件成立時同步 ASL 與 STBY。
    if (is_lost_counting && millis() - lost_start_time > 5000) {
      ebsLatched = true;
      vlsStarted = false;
      forceSafeStop();
      setAslMode(ASL_SAFE);
    }

    delay(2);
  }
}

void startHttpEbsServer() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(EBS_WIFI_SSID, EBS_WIFI_PASSWORD);

  server.on("/", handleEbsRoot);
  server.on("/stop", handleEbsStop);
  server.on("/arm", handleEbsArm);
  server.onNotFound(handleEbsNotFound);
  server.begin();

  xTaskCreatePinnedToCore(
    ebsHttpTask,
    "EBS_HTTP_Task",
    4096,
    nullptr,
    1,
    nullptr,
    0
  );

  Serial.println();
  Serial.println("HTTP EBS server started.");
  Serial.print("Wi-Fi SSID: ");
  Serial.println(EBS_WIFI_SSID);
  Serial.print("Open browser at: http://");
  Serial.print(WiFi.softAPIP());
  Serial.println("/");
}
