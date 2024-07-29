#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <PID_v1.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>

#define PIN_EN_M_LEFT 22
#define PIN_1_M_LEFT 17
#define PIN_2_M_LEFT 5

#define PIN_EN_M_RIGHT 23
#define PIN_1_M_RIGHT 18
#define PIN_2_M_RIGHT 19

#define PIN_ENCODER_LEFT 15
#define PIN_ENCODER_RIGHT 16

#define PIN_LINE_LEFT 4
#define PIN_LINE_RIGHT 2
#define PIN_CROSS 21

#define BASE_SERVO_PIN 25
#define SHOULDER_SERVO_PIN 26
#define ELBOW_SERVO_PIN 27
#define GRIPPER_SERVO_PIN 14

#define L1 110.0  // Lungimea primului segment (mm)
#define L2 150.0  // Lungimea celui de-al doilea segment (mm)

// Inițializarea obiectelor Servo
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo gripperServo;

// Configurați Access Point
const char *ssid = "ESP32-AP";
const char *password = "12345678";

// Inițializați serverul pe portul 80
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

int X = 0;
int Y = 0;
// Variabile pentru a stoca valorile primite
int targetX = 0;
int targetY = 0;
int carry_targetX = 0;
int carry_targetY = 0;
int togetherX = 0;
int togetherY = 0;
// Variabile pentru a stoca valorile anterioare
int prev_targetX = 0;
int prev_targetY = 0;
int prev_togetherX = 0;
int prev_togetherY = 0;

bool use_target = true;


// Variabile pentru brațul robotic
int arm_x = 0;
int arm_y = 1;
int arm_z = 0;
int gripper = 0;

int remoteX = 0;
int remoteY = 0;
String remoteDirection = "E";
bool remoteConnected = false;

enum direction { N,
                 E,
                 S,
                 W };
enum direction current_direction = N;
enum direction last_direction = N;
bool last_reached = false;
bool reached_destination = false;


int speed_forward = 40;

volatile long pulses_left = 0;
volatile long pulses_right = 0;

const unsigned long cross_debounce_delay = 200;

unsigned long last_left_pulses_time = 0;
unsigned long last_right_pulses_time = 0;
unsigned long debounceDelay = 2000;  // Debounce delay in microseconds

bool stop = false;

bool sensor_left = false;
bool sensor_right = false;
bool sensor_cross = false;

bool last_detection_on_left = false;
bool last_detection_on_right = false;
bool previous_state_left = false;
bool previous_state_right = false;
bool previous_state_cross = false;

bool turn_left = false;
bool turn_right = false;

// Define PID variables
double SetpointLeft = 0, InputLeft, OutputLeft;
double SetpointRight = 0, InputRight, OutputRight;
double Kp = 2.5, Ki = 0.5, Kd = 0;

PID leftPID(&InputLeft, &OutputLeft, &SetpointLeft, Kp, Ki, Kd, DIRECT);
PID rightPID(&InputRight, &OutputRight, &SetpointRight, Kp, Ki, Kd, DIRECT);

unsigned long lastTime = 0;
const long interval = 100;  // interval in milliseconds

float baseAngle, shoulderAngle, elbowAngle;

TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t controlTaskHandle = NULL;

void checkTaskStackUsage(TaskHandle_t taskHandle, const char *taskName) {
  UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
  Serial.printf("Task [%s] - stack high water mark: %d\n", taskName, stackHighWaterMark);
}

void IRAM_ATTR encoder_left_ISR() {
  unsigned long currentTime = micros(); 
  // implementare debounce software data: 2ms
  if (currentTime - last_left_pulses_time > debounceDelay) {
    pulses_left++;
    last_left_pulses_time = currentTime;
  }
}

void IRAM_ATTR encoder_right_ISR() {
  unsigned long currentTime = micros();
  if (currentTime - last_right_pulses_time > debounceDelay) {  // implementare debounce software
    pulses_right++;
    last_right_pulses_time = currentTime;
  }
}

void IRAM_ATTR line_left_ISR() {
  sensor_left = digitalRead(PIN_LINE_LEFT);
  if (sensor_left == true && previous_state_left == false && turn_left == true) {
    turn_left = false;
    if (current_direction == N) {
      current_direction = W;
    } else if (current_direction == E) {
      current_direction = N;
    } else if (current_direction == S) {
      current_direction = E;
    } else if (current_direction == W) {
      current_direction = S;
    }
  }
}

void IRAM_ATTR line_right_ISR() {
  sensor_right = digitalRead(PIN_LINE_RIGHT);
  if (sensor_right == true && previous_state_right == false && turn_right == true) {
    turn_right = false;
    if (current_direction == N) {
      current_direction = E;
    } else if (current_direction == E) {
      current_direction = S;
    } else if (current_direction == S) {
      current_direction = W;
    } else if (current_direction == W) {
      current_direction = N;
    }
  }
}

void sendUpdateToSlave(direction current_direction, bool reached_destination) {
  WiFiClient client;
  if (client.connect("192.168.4.10", 80)) {  // IP-ul static al celui de-al doilea robot ESP32
    String directionStr = directionToString(current_direction);
    String reachedStr = reached_destination ? "true" : "false";
    client.print(String("GET /updateDirection?current_direction=") + directionStr + "&reached_destination=" + reachedStr + " HTTP/1.1\r\n" + "Host: 192.168.4.3\r\n" + "Connection: close\r\n\r\n");
  } else {
    Serial.println("Failed to connect to secondary ESP32");
  }
}


void handleRoot() {
  String html = "<html>"
                "<head>"
                "<style>"
                "body {"
                "  font-family: Arial, sans-serif;"
                "  background-color: #f4f4f9;"
                "  color: #333;"
                "  margin: 0;"
                "  padding: 0;"
                "  display: flex;"
                "  justify-content: center;"
                "  align-items: flex-start;"
                "  height: 100vh;"
                "}"
                "h1, h2 {"
                "  color: #444;"
                "  font-size: 24px;"
                "}"
                ".container {"
                "  display: flex;"
                "  justify-content: space-between;"
                "  width: 80%;"
                "  margin-top: 20px;"
                "}"
                ".left, .right {"
                "  width: 48%;"
                "}"
                ".left {"
                "  display: flex;"
                "  flex-direction: column;"
                "}"
                "form {"
                "  background: #fff;"
                "  padding: 20px;"
                "  margin: 10px 0;"
                "  border-radius: 8px;"
                "  box-shadow: 0 0 10px rgba(0,0,0,0.1);"
                "  flex: 1;"
                "}"
                "label {"
                "  display: block;"
                "  margin-bottom: 8px;"
                "  font-weight: bold;"
                "}"
                "input[type='text'] {"
                "  width: calc(100% - 20px);"
                "  padding: 6px;"
                "  margin-bottom: 10px;"
                "  border: 1px solid #ccc;"
                "  border-radius: 4px;"
                "  font-size: 14px;"
                "}"
                "input[type='button'] {"
                "  background: #5c6bc0;"
                "  color: #fff;"
                "  border: none;"
                "  padding: 10px 20px;"
                "  border-radius: 4px;"
                "  cursor: pointer;"
                "}"
                "input[type='button']:hover {"
                "  background: #3f51b5;"
                "}"
                "#data, #remoteData {"
                "  margin: 10px 0;"
                "  padding: 10px;"
                "  background: #fff;"
                "  border-radius: 8px;"
                "  box-shadow: 0 0 10px rgba(0,0,0,0.1);"
                "}"
                "</style>"
                "</head>"
                "<body>"
                "<div class='container'>"
                "<div class='left'>"
                "<h1>Coordonate de destinatie pentru cei 2 roboti</h1>"
                "<form id=\"mainForm\">"
                "<label for=\"targetX\">MainBot X:</label>"
                "<input type=\"text\" id=\"targetX\" name=\"targetX\"><br><br>"
                "<label for=\"targetY\">MainBot Y:</label>"
                "<input type=\"text\" id=\"targetY\" name=\"targetY\"><br><br>"
                "<label for=\"carry_targetX\">CarryBot X:</label>"
                "<input type=\"text\" id=\"carry_targetX\" name=\"carry_targetX\"><br><br>"
                "<label for=\"carry_targetY\">CarryBot Y:</label>"
                "<input type=\"text\" id=\"carry_targetY\" name=\"carry_targetY\"><br><br>"
                "<label for=\"togetherX\">Colaborare X:</label>"
                "<input type=\"text\" id=\"togetherX\" name=\"togetherX\"><br><br>"
                "<label for=\"togetherY\">Colaborare Y:</label>"
                "<input type=\"text\" id=\"togetherY\" name=\"togetherY\"><br><br>"
                "<input type=\"button\" value=\"Trimite\" onclick=\"sendMainForm()\">"
                "</form>"
                "<h2>Coordonatele si orientarea actuala MainBot</h2>"
                "<div id=\"data\">X: 0, Y: 0, Direction: N</div>"
                "<h2>Coordonatele si orientarea actuala CarryBot</h2>"
                "<div id=\"remoteData\">X: 0, Y: 0, Direction: N, Connected: No</div>"
                "</div>"
                "<div class='right'>"
                "<h2>Control Brat Robotic</h2>"
                "<form id=\"armForm\">"
                "<label for=\"arm_x\">X:</label>"
                "<input type=\"text\" id=\"arm_x\" name=\"arm_x\"><br><br>"
                "<label for=\"arm_y\">Y:</label>"
                "<input type=\"text\" id=\"arm_y\" name=\"arm_y\"><br><br>"
                "<label for=\"arm_z\">Z:</label>"
                "<input type=\"text\" id=\"arm_z\" name=\"arm_z\"><br><br>"
                "<label for=\"gripper\">Gripper:</label>"
                "<input type=\"text\" id=\"gripper\" name=\"gripper\"><br><br>"
                "<input type=\"button\" value=\"Trimite\" onclick=\"sendArmForm()\">"
                "</form>"
                "</div>"
                "</div>"
                "<script>"
                "var webSocket = new WebSocket('ws://' + window.location.hostname + ':81');"
                "webSocket.onmessage = function(event) {"
                "  var data = JSON.parse(event.data);"
                "  document.getElementById('data').innerText = 'X: ' + data.X + ', Y: ' + data.Y + ', Direction: ' + data.Direction;"
                "  document.getElementById('remoteData').innerText = 'X: ' + data.remoteX + ', Y: ' + data.remoteY + ', Direction: ' + data.remoteDirection + ', Connected: ' + (data.remoteConnected ? 'Yes' : 'No');"
                "};"
                "function sendMainForm() {"
                "  var form = document.getElementById('mainForm');"
                "  var formData = new FormData(form);"
                "  fetch('/set', { method: 'POST', body: formData })"
                "    .then(response => response.text())"
                "    .then(data => {"
                "      console.log(data);"
                "    });"
                "}"
                "function sendArmForm() {"
                "  var form = document.getElementById('armForm');"
                "  var formData = new FormData(form);"
                "  fetch('/setArm', { method: 'POST', body: formData })"
                "    .then(response => response.text())"
                "    .then(data => {"
                "      console.log(data);"
                "    });"
                "}"
                "</script>"
                "</body>"
                "</html>";
  server.send(200, "text/html", html);
}







void handleSet() {

  prev_targetX = targetX;
  prev_targetY = targetY;
  prev_togetherX = togetherX;
  prev_togetherY = togetherY;

  targetX = server.arg("targetX").toInt();
  targetY = server.arg("targetY").toInt();
  carry_targetX = server.arg("carry_targetX").toInt();
  carry_targetY = server.arg("carry_targetY").toInt();
  togetherX = server.arg("togetherX").toInt();
  togetherY = server.arg("togetherY").toInt();

  Serial.println("Received POST /set");
  Serial.print("targetX: ");
  Serial.println(targetX);
  Serial.print("targetY: ");
  Serial.println(targetY);
  Serial.print("carry_targetX: ");
  Serial.println(carry_targetX);
  Serial.print("carry_targetY: ");
  Serial.println(carry_targetY);
  Serial.print("togetherX: ");
  Serial.println(togetherX);
  Serial.print("togetherY: ");
  Serial.println(togetherY);

  // Verifică care set de coordonate a fost modificat
  if (targetX != prev_targetX || targetY != prev_targetY) {
    use_target = true;

  } else if (togetherX != prev_togetherX || togetherY != prev_togetherY) {
    use_target = false;
    prev_togetherX = togetherX;
    prev_togetherY = togetherY;
  }

  server.send(200, "text/plain", "Valori primite: \nTarget X: " + String(targetX) + "\nTarget Y: " + String(targetY) + "\nCarry Target X: " + String(carry_targetX) + "\nCarry Target Y: " + String(carry_targetY) + "\nTogether X: " + String(togetherX) + "\nTogether Y: " + String(togetherY));

  // Trimitere valori carry_targetX, carry_targetY, togetherX, togetherY către a doua placă ESP32
  WiFiClient client;
  if (client.connect("192.168.4.10", 80)) {  // IP-ul static al celei de-a doua plăci ESP32
    client.print(String("GET /update?carry_targetX=") + String(carry_targetX) + "&carry_targetY=" + String(carry_targetY) + "&togetherX=" + String(togetherX) + "&togetherY=" + String(togetherY) + " HTTP/1.1\r\n" + "Host: 192.168.4.10\r\n" + "Connection: close\r\n\r\n");
  } else {
    Serial.println("Failed to connect to secondary ESP32");
  }
}


void handleSetArm() {
  arm_x = server.arg("arm_x").toInt();
  arm_y = server.arg("arm_y").toInt();
  arm_z = server.arg("arm_z").toInt();
  gripper = server.arg("gripper").toInt();

  Serial.println("Received POST /setArm");
  Serial.print("arm_x: ");
  Serial.println(arm_x);
  Serial.print("arm_y: ");
  Serial.println(arm_y);
  Serial.print("arm_z: ");
  Serial.println(arm_z);
  Serial.print("gripper: ");
  Serial.println(gripper);

  server.send(200, "text/plain", "Valori primite: \nArm X: " + String(arm_x) + "\nArm Y: " + String(arm_y) + "\nArm Z: " + String(arm_z) + "\nGripper: " + String(gripper));
}

void notifyClients() {
  String json = "{\"X\":" + String(X) + ",\"Y\":" + String(Y) + ",\"Direction\":\"" + directionToString(current_direction) + "\",\"remoteX\":" + String(remoteX) + ",\"remoteY\":" + String(remoteY) + ",\"remoteDirection\":\"" + remoteDirection + "\",\"remoteConnected\":" + String(remoteConnected ? "true" : "false") + "}";
  Serial.print("Sending WebSocket message: ");
  Serial.println(json);
  webSocket.broadcastTXT(json);
}


void handleData() {
  String data = "X: " + String(X) + ", Y: " + String(Y) + ", Direction: " + directionToString(current_direction);
  Serial.println("Sending /data response: " + data);
  server.send(200, "text/plain", data);
}

void handleRemoteData() {
  if (getRemoteData()) {
    String data = "X: " + String(remoteX) + ", Y: " + String(remoteY) + ", Direction: " + remoteDirection + ", Connected: Yes";
    Serial.println("Sending /remoteData response: " + data);
    server.send(200, "text/plain", data);
  } else {
    String data = "X: 0, Y: 0, Direction: N, Connected: No";
    Serial.println("Sending /remoteData response: " + data);
    server.send(200, "text/plain", data);
  }
}

bool getRemoteData() {
  HTTPClient http;
  http.begin("http://192.168.4.10/data");
  int httpCode = http.GET();

  if (httpCode > 0) {
    String payload = http.getString();
    Serial.println("Received remote data: " + payload);

    // Parsing the payload to get remote values
    int xIndex = payload.indexOf("X: ");
    int yIndex = payload.indexOf("Y: ");
    int dirIndex = payload.indexOf("Direction: ");

    if (xIndex != -1 && yIndex != -1 && dirIndex != -1) {
      remoteX = payload.substring(xIndex + 3, payload.indexOf(",", xIndex)).toInt();
      remoteY = payload.substring(yIndex + 3, payload.indexOf(",", yIndex)).toInt();
      remoteDirection = payload.substring(dirIndex + 11).c_str();
      remoteConnected = true;
      Serial.print("Parsed remoteX: ");
      Serial.println(remoteX);
      Serial.print("Parsed remoteY: ");
      Serial.println(remoteY);
      Serial.print("Parsed remoteDirection: ");
      Serial.println(remoteDirection);
    } else {
      remoteConnected = false;
      Serial.println("Failed to parse remote data");
    }
  } else {
    Serial.println("Failed to get remote data, error: " + String(httpCode));
    remoteConnected = false;
  }

  http.end();
  return remoteConnected;
}



void checkMemory() {
  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("Free PSRAM: ");
  Serial.println(ESP.getFreePsram());
}

void WiFiTask(void *pvParameters) {
  WiFi.softAP(ssid, password, 1);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  server.on("/", handleRoot);
  server.on("/set", HTTP_POST, handleSet);
  server.on("/setArm", HTTP_POST, handleSetArm);
  server.on("/data", handleData);
  server.on("/remoteData", handleRemoteData);
  server.begin();
  Serial.println("HTTP server started");

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  for (;;) {

    // Adăugați apelul funcției getRemoteData
    if (getRemoteData()) {
      notifyClients();  // Notifică clienții cu noile date
    }

    // Detectează schimbările și trimite actualizările
    if (reached_destination != last_reached) {
      sendUpdateToSlave(current_direction, reached_destination);
      last_direction = current_direction;
      last_reached = reached_destination;
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS);  // Verifică datele de la robotul slave la fiecare 5 secunde
  }
}


void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("WebSocket client #%u connected from %s\n", num, webSocket.remoteIP(num).toString().c_str());
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("WebSocket client #%u disconnected\n", num);
  }
}

void ControlTask(void *pvParameters) {
  for (;;) {
    server.handleClient();  // Procesare cereri de la clienți
    webSocket.loop();       // Loop WebSocket

    sensor_left = digitalRead(PIN_LINE_LEFT);  // logica pozitiva: 1-detecteaza; 0-nu
    sensor_right = digitalRead(PIN_LINE_RIGHT);
    sensor_cross = digitalRead(PIN_CROSS);

    if (sensor_cross == true && previous_state_cross == false) {  // && turn_right == false && turn_left == false
      if (current_direction == N) {
        Y++;
      } else if (current_direction == S) {
        Y--;
      } else if (current_direction == E) {
        X++;
      } else if (current_direction == W) {
        X--;
      }
      notifyClients();  // Notify clients when position changes
    }


    // checkTaskStackUsage(controlTaskHandle, "Control Task");
    // UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(controlTaskHandle);
    // Serial.printf("Task Control Task - stack high water mark: %d\n", stackHighWaterMark);

    // Navigare spre destinație

    if(use_target == true){
    navigate_to(targetX, targetY);
    }else{
      navigate_to(togetherX, togetherY);
    }

    if (turn_right == false && turn_left == false && stop == false) {
      follow_line_forward();
    } else if (turn_right == true && stop == false) {
      func_turn_right();
    } else if (turn_left == true && stop == false) {
      func_turn_left();
    } else if (stop == true) {
      motor_stop();
    }

    inverseKinematics(arm_x, arm_y, arm_z, &baseAngle, &shoulderAngle, &elbowAngle);
    setServoAngles(baseAngle, shoulderAngle, elbowAngle, gripper);

    previous_state_left = sensor_left;
    previous_state_right = sensor_right;
    previous_state_cross = sensor_cross;

    // checkMemory();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  stop = true;
  turn_left = false;
  turn_right = false;

  Serial.begin(115200);

  // Creăm task-urile și le atribuim pe nuclee diferite
  xTaskCreatePinnedToCore(
    WiFiTask,     // Funcția de task
    "WiFi Task",  // Numele task-ului
    4096,         // Dimensiunea stack-ului
    NULL,         // Parametru pentru funcția de task
    1,            // Prioritatea task-ului
    NULL,         // Task handle
    0             // Nucleul pe care rulează task-ul
  );

  xTaskCreatePinnedToCore(
    ControlTask,     // Funcția de task
    "Control Task",  // Numele task-ului
    4096,            // Dimensiunea stack-ului
    NULL,            // Parametru pentru funcția de task
    1,               // Prioritatea task-ului
    NULL,            // Task handle
    1                // Nucleul pe care rulează task-ul
  );

  //Config pini senzori linie
  pinMode(PIN_LINE_LEFT, INPUT);
  pinMode(PIN_LINE_RIGHT, INPUT);
  pinMode(PIN_CROSS, INPUT);

  // Configurare pini motor stanga
  pinMode(PIN_EN_M_LEFT, OUTPUT);
  pinMode(PIN_1_M_LEFT, OUTPUT);
  pinMode(PIN_2_M_LEFT, OUTPUT);

  // Configurare pini motor dreapta
  pinMode(PIN_EN_M_RIGHT, OUTPUT);
  pinMode(PIN_1_M_RIGHT, OUTPUT);
  pinMode(PIN_2_M_RIGHT, OUTPUT);

  pinMode(PIN_ENCODER_LEFT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT), encoder_left_ISR, RISING);

  // Configurare pin encoder 2
  pinMode(PIN_ENCODER_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), encoder_right_ISR, RISING);

  attachInterrupt(digitalPinToInterrupt(PIN_LINE_RIGHT), line_right_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_LINE_LEFT), line_left_ISR, RISING);

  // Initialize PID controllers
  SetpointLeft = 0;  // 0 pulses per second
  SetpointRight = 0;
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(-255, 255);
  rightPID.SetOutputLimits(-255, 255);

  // Atașarea servomotoarelor la pini
  baseServo.attach(BASE_SERVO_PIN);
  shoulderServo.attach(SHOULDER_SERVO_PIN);
  elbowServo.attach(ELBOW_SERVO_PIN);
  gripperServo.attach(GRIPPER_SERVO_PIN);
}

void loop() {
  // Nu facem nimic aici, task-urile rulează pe nucleele lor
}

void func_turn_right() {
  while (turn_right == true) {
    OutputRight = 0;
    SetpointRight = 0;
    previous_state_left = sensor_left;
    previous_state_right = sensor_right;
    control_motors_PWM(170, -145);
    delay(200);
    sensor_left = digitalRead(PIN_LINE_LEFT);
    sensor_right = digitalRead(PIN_LINE_RIGHT);
  }
}

void func_turn_left() {
  while (turn_left == true) {
    OutputLeft = 0;
    SetpointLeft = 0;
    previous_state_left = sensor_left;
    previous_state_right = sensor_right;
    control_motors_PWM(-155, 165);
    delay(200);
    sensor_left = digitalRead(PIN_LINE_LEFT);
    sensor_right = digitalRead(PIN_LINE_RIGHT);
  }
}

void follow_line_forward() {
  bool sensor_left = digitalRead(PIN_LINE_LEFT);  // logica pozitiva: 1-detecteaza; 0-nu
  bool sensor_right = digitalRead(PIN_LINE_RIGHT);

  // Logica pentru urmărirea liniei
  if (sensor_left == 0 && sensor_right == 0) {  // nicio linie detectata
    if (last_detection_on_left) {               // ultimul senzor care a detectat linia a fost cel din stanga
      control_motors_PWM(0, 155);               // intoarcere spre dreapta
    } else if (last_detection_on_right) {       // ultimul senzor care a detectat linia a fost cel din dreapta
      control_motors_PWM(155, 0);               // intoarcere spre stanga
    } else {
      // control_motors(50, 50);  // continua drept daca nu avem nicio referinta
      control_motors_PWM(145, 145);
    }
  } else if (sensor_left == 0 && sensor_right == 1) {  // senzorul din dreapta detecteaza linie -> turn left
    control_motors_PWM(155, 100);
    last_detection_on_left = false;
    last_detection_on_right = true;
  } else if (sensor_left == 1 && sensor_right == 0) {  // senzorul din stanga detecteaza linie -> turn right
    control_motors_PWM(100, 155);
    last_detection_on_left = true;
    last_detection_on_right = false;
  } else {
    // control_motors(50, 50);
    control_motors_PWM(145, 145);
  }
}

void control_motors(double newSetpointLeft, double newSetpointRight) {
  // Calculate current speed in pulses per second
  unsigned long currentTime = millis();
  SetpointLeft = newSetpointLeft;
  SetpointRight = newSetpointRight;
  //calculeaza noile valori pentru output
  if (currentTime - lastTime >= interval) {
    InputLeft = pulses_left * (1000.0 / interval);  // pulses per second
    leftPID.Compute();
    InputRight = pulses_right * (1000.0 / interval);  // pulses per second
    rightPID.Compute();
    pulses_left = 0;   // reset pulse count
    pulses_right = 0;  // reset pulse count
    lastTime = currentTime;
  }

  control_motors_PWM(OutputLeft, OutputRight);  // Call motor control with PID outputs
}

void control_motors_PWM(int speed_left, int speed_right) {
  // control motor stanga
  if (speed_left >= 0) {
    digitalWrite(PIN_1_M_LEFT, LOW);
    digitalWrite(PIN_2_M_LEFT, HIGH);
  } else {
    digitalWrite(PIN_1_M_LEFT, HIGH);
    digitalWrite(PIN_2_M_LEFT, LOW);
    speed_left = -speed_left;
  }
  analogWrite(PIN_EN_M_LEFT, speed_left);

  // control motor dreapta
  if (speed_right >= 0) {
    digitalWrite(PIN_1_M_RIGHT, LOW);
    digitalWrite(PIN_2_M_RIGHT, HIGH);
  } else {
    digitalWrite(PIN_1_M_RIGHT, HIGH);
    digitalWrite(PIN_2_M_RIGHT, LOW);
    speed_right = -speed_right;
  }
  analogWrite(PIN_EN_M_RIGHT, speed_right);
}

void motor_stop() {
  digitalWrite(PIN_1_M_LEFT, LOW);
  digitalWrite(PIN_2_M_LEFT, LOW);
  digitalWrite(PIN_1_M_RIGHT, LOW);
  digitalWrite(PIN_2_M_RIGHT, LOW);
  stop = true;
  SetpointLeft = 0;
  SetpointRight = 0;
}

const char *directionToString(enum direction dir) {
  switch (dir) {
    case N: return "N";
    case E: return "E";
    case S: return "S";
    case W: return "W";
    default: return "Unknown";
  }
}

// Funcție pentru a seta unghiurile servomotoarelor
void setServoAngles(float baseAngle, float shoulderAngle, float elbowAngle, float gripper) {
  baseServo.write(baseAngle);
  shoulderServo.write(shoulderAngle-20);
  elbowServo.write(elbowAngle + 80);
  gripperServo.write(gripper);
}

// Funcție pentru cinemataică inversă
void inverseKinematics(float x, float y, float z, float *baseAngle, float *shoulderAngle, float *elbowAngle) {
  // Calcularea unghiului pentru servomotorul de bază (S4)
  *baseAngle = atan2(y, x) * 180.0 / M_PI;

  // Calcularea planului 2D pentru segmentul brațului
  float r = sqrt(x * x + y * y);
  float D = (r * r + z * z - L1 * L1 - L2 * L2) / (2 * L1 * L2);

  // Asigurarea că D este în intervalul [-1, 1] pentru a evita eroarea acos
  if (D > 1) D = 1;
  if (D < -1) D = -1;

  *elbowAngle = acos(D) * 180.0 / M_PI;

  float phi1 = atan2(z, r) * 180.0 / M_PI;
  float phi2 = acos((r * r + z * z + L1 * L1 - L2 * L2) / (2 * L1 * sqrt(r * r + z * z))) * 180.0 / M_PI;

  *shoulderAngle = phi1 + phi2;
}

// X SI Y COMBINAT:
void navigate_to(int targetX, int targetY) {
  // Resetare variabile de direcție
  turn_left = false;
  turn_right = false;

  // Calcularea diferenței dintre coordonatele curente și cele țintă
  int deltaX = targetX - X;
  int deltaY = targetY - Y;

  // Dacă robotul este deja la destinație
  if (deltaX == 0 && deltaY == 0) {
    stop = true;
    reached_destination = true;
    //return;
  } else {
    stop = false;
    reached_destination = false;
  }


  // Logica de navigație în funcție de direcția curentă și destinație
  if (deltaX == 0) {                             // Aceeași coloană
    if (deltaY > 0 && current_direction != N) {  // Trebuie să se îndrepte spre Nord
      if (current_direction == S) {
        turn_right = true;
      } else if (current_direction == E) {
        turn_left = true;
      } else if (current_direction == W) {
        turn_right = true;
      }
    } else if (deltaY < 0 && current_direction != S) {  // Trebuie să se îndrepte spre Sud
      if (current_direction == N) {
        turn_right = true;
      } else if (current_direction == E) {
        turn_right = true;
      } else if (current_direction == W) {
        turn_left = true;
      }
    }
  } else if (deltaY == 0) {                      // Aceeași linie
    if (deltaX > 0 && current_direction != E) {  // Trebuie să se îndrepte spre Est
      if (current_direction == W) {
        turn_right = true;
      } else if (current_direction == N) {
        turn_right = true;
      } else if (current_direction == S) {
        turn_left = true;
      }
    } else if (deltaX < 0 && current_direction != W) {  // Trebuie să se îndrepte spre Vest
      if (current_direction == E) {
        turn_right = true;
      } else if (current_direction == N) {
        turn_left = true;
      } else if (current_direction == S) {
        turn_right = true;
      }
    }
  } else if (deltaX != 0 && deltaY != 0) {  // Trebuie să se deplaseze atât pe X cât și pe Y
    if (deltaY != Y) {
      if (deltaY > 0 && current_direction != N) {  // Trebuie să se îndrepte spre Nord
        if (current_direction == S) {
          turn_right = true;
        } else if (current_direction == E) {
          turn_left = true;
        } else if (current_direction == W) {
          turn_right = true;
        }
      } else if (deltaY < 0 && current_direction != S) {  // Trebuie să se îndrepte spre Sud
        if (current_direction == N) {
          turn_right = true;
        } else if (current_direction == E) {
          turn_right = true;
        } else if (current_direction == W) {
          turn_left = true;
        }
      }
    } else if (deltaX != X) {
      if (deltaX > 0 && current_direction != E) {  // Trebuie să se îndrepte spre Est
        if (current_direction == W) {
          turn_right = true;
        } else if (current_direction == N) {
          turn_right = true;
        } else if (current_direction == S) {
          turn_left = true;
        }
      } else if (deltaX < 0 && current_direction != W) {  // Trebuie să se îndrepte spre Vest
        if (current_direction == E) {
          turn_right = true;
        } else if (current_direction == N) {
          turn_left = true;
        } else if (current_direction == S) {
          turn_right = true;
        }
      }
    }
  }
}
