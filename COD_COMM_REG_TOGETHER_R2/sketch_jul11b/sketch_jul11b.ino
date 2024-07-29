#include <WiFi.h>
#include <WebServer.h>
#include <PID_v1.h>

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

#define PIN_TRIG 25
#define PIN_ECHO 26

// Configurați SSID și parola pentru rețeaua AP
const char* ssid = "ESP32-AP";
const char* password = "12345678";

// Configurați IP-ul static
IPAddress local_IP(192, 168, 4, 10);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// Inițializați serverul pe portul 80
WebServer server(80);

int X = 1;
int Y = 0;
// Variabile pentru a stoca valorile primite
int carry_targetX = 1;
int carry_targetY = 0;
int togetherX = 0;
int togetherY = 0;
// Variabile pentru a stoca valorile anterioare
int prev_carry_targetX = 1;
int prev_carry_targetY = 0;
int prev_togetherX = 0;
int prev_togetherY = 0;

bool use_carry_target = true;


int deltaX = 0;
int deltaY = 0;


enum direction { N,
                 E,
                 S,
                 W };
enum direction current_direction = N;
enum direction main_current_direction = N;
bool main_reached_destination = false;
bool last_main_reached_destination = false;


int PWM_motor_left = 90;
int PWM_motor_right = 90;

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

bool rotate_left = false;
bool rotate_right = false;



// Define PID variables
double SetpointLeft = 0, InputLeft, OutputLeft;
double SetpointRight = 0, InputRight, OutputRight;
double Kp = 1.5, Ki = 5, Kd = 0;
PID leftPID(&InputLeft, &OutputLeft, &SetpointLeft, Kp, Ki, Kd, DIRECT);
PID rightPID(&InputRight, &OutputRight, &SetpointRight, Kp, Ki, Kd, DIRECT);

unsigned long lastTime = 0;
const long interval = 100;  // interval in milliseconds

long duration;
int distance;

void IRAM_ATTR encoder_left_ISR() {
  unsigned long currentTime = micros();
  if (currentTime - last_left_pulses_time > debounceDelay) {  // implementare debounce software
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
  if (sensor_left == true && previous_state_left == false && (turn_left == true || rotate_left == true)) {
    turn_left = false;
    rotate_left = false;
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
  if (sensor_right == true && previous_state_right == false && (turn_right == true || rotate_right == true)) {
    turn_right = false;
    rotate_right = false;
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
void IRAM_ATTR line_cross_ISR() {
  sensor_cross = digitalRead(PIN_CROSS);
  if (sensor_cross == true && previous_state_cross == false && turn_right == false && turn_left == false) {  // && turn_right == false && turn_left == false
    if (current_direction == N) {
      Y++;
    } else if (current_direction == S) {
      Y--;
    } else if (current_direction == E) {
      X++;
    } else if (current_direction == W) {
      X--;
    }
  }
}

void handleUpdateDirection() {
  String directionStr = server.arg("current_direction");
  // main_reached_destination = server.arg("reached_destination");
  main_reached_destination = server.arg("reached_destination") == "true";


  if (directionStr == "N") {
    main_current_direction = N;
  } else if (directionStr == "E") {
    main_current_direction = E;
  } else if (directionStr == "S") {
    main_current_direction = S;
  } else if (directionStr == "W") {
    main_current_direction = W;
  }

  server.send(200, "text/plain", "Direction and destination status updated");
  Serial.println("Direction and destination status updated");
  Serial.print("Main current direction: ");
  Serial.println(directionToString(main_current_direction));
  Serial.print("Reached destination: ");
  Serial.println(main_reached_destination ? "true" : "false");
}


void handleUpdate() {
  prev_carry_targetX = carry_targetX;
  prev_carry_targetY = carry_targetY;
  prev_togetherX = togetherX;
  prev_togetherY = togetherY;

  carry_targetX = server.arg("carry_targetX").toInt();
  carry_targetY = server.arg("carry_targetY").toInt();
  togetherX = server.arg("togetherX").toInt();
  togetherY = server.arg("togetherY").toInt();

  // Verificăm care set de variabile a fost modificat
  if (carry_targetX != prev_carry_targetX || carry_targetY != prev_carry_targetY) {
    use_carry_target = true;
  } else if (togetherX != prev_togetherX || togetherY != prev_togetherY) {
    use_carry_target = false;
  }

  server.send(200, "text/plain", "Valori actualizate: \nCarry Target X: " + String(carry_targetX) + "\nCarry Target Y: " + String(carry_targetY) + "\nTogether X: " + String(togetherX) + "\nTogether Y: " + String(togetherY));
  Serial.println("Carry Target X primită: " + String(carry_targetX));
  Serial.println("Carry Target Y primită: " + String(carry_targetY));
  Serial.println("Together X primită: " + String(togetherX));
  Serial.println("Together Y primită: " + String(togetherY));
}

void handleData() {
  String data = "X: " + String(X) + ", Y: " + String(Y) + ", Direction: " + directionToString(current_direction);
  server.send(200, "text/plain", data);
}

void setup() {
  stop = true;
  turn_left = false;
  turn_right = false;

  Serial.begin(115200);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectare la WiFi...");
  }

  Serial.print("Conectat, IP: ");
  Serial.println(WiFi.localIP());

  server.on("/update", handleUpdate);
  server.on("/data", handleData);
  server.on("/updateDirection", handleUpdateDirection);
  server.begin();
  Serial.println("HTTP server started");

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
  attachInterrupt(digitalPinToInterrupt(PIN_CROSS), line_cross_ISR, RISING);


  // Initialize PID controllers
  SetpointLeft = 0;  // 0 pulses per second
  SetpointRight = 0;
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(-255, 255);
  rightPID.SetOutputLimits(-255, 255);

  // Configurare pini pentru senzorul ultrasonic
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
}

void loop() {
  last_main_reached_destination = main_reached_destination;
  server.handleClient();  // Procesare cereri de la clienți


  sensor_left = digitalRead(PIN_LINE_LEFT);  // logica pozitiva: 1-detecteaza; 0-nu
  sensor_right = digitalRead(PIN_LINE_RIGHT);
  sensor_cross = digitalRead(PIN_CROSS);

  // if (sensor_cross == true && previous_state_cross == false && turn_right == false && turn_left == false) {  // && turn_right == false && turn_left == false
  //   if (current_direction == N) {
  //     Y++;
  //   } else if (current_direction == S) {
  //     Y--;
  //   } else if (current_direction == E) {
  //     X++;
  //   } else if (current_direction == W) {
  //     X--;
  //   }
  // }

  // Apelăm navigate_to cu setul de variabile corespunzător
  if (use_carry_target) {
    navigate_to(carry_targetX, carry_targetY);
    Serial.print("Acum foloseste carry_target: ");
    Serial.print(carry_targetX);
    Serial.println(carry_targetY);
  } else {
    if (main_reached_destination == true && last_main_reached_destination == false) {
      if (main_current_direction == N) {
        navigate_to(togetherX, togetherY + 1);
      } else if (main_current_direction == E) {
        navigate_to(togetherX + 1, togetherY);
      } else if (main_current_direction == S) {
        navigate_to(togetherX, togetherY - 1);
      } else if (main_current_direction == W) {
        navigate_to(togetherX - 1, togetherY + 1);
      }
    }
  }


  //   // if(deltaX == 0 && deltaY == 0){
  //   //     switch(main_current_direction){
  //   //       case N:
  //   //       switch(current_direction){
  //   //         case N:return;
  //   //         case E:rotate_left == true;
  //   //         case S:return;
  //   //         case W:rotate_right == true;
  //   //       }
  //   //       case E:
  //   //       switch(current_direction){
  //   //         case N:rotate_left == true;
  //   //         case E:return;
  //   //         case S:rotate_right == true;
  //   //         case W:return;
  //   //       }
  //   //       case S:
  //   //       switch(current_direction){
  //   //         case N:return;
  //   //         case E:rotate_left == true;
  //   //         case S:return;
  //   //         case W:rotate_right == true;
  //   //       }
  //   //       case W:
  //   //       switch(current_direction){
  //   //         case N:rotate_right == true;
  //   //         case E:return;
  //   //         case S:rotate_left == true;
  //   //         case W:return;
  //   //       }
  //   //     }
  //   // }

  // }

  if (turn_right == false && turn_left == false && stop == false) {
    follow_line_forward();
  } else if (turn_right == true && stop == false) {
    func_turn_right();
  } else if (turn_left == true && stop == false) {
    func_turn_left();
  } else if (stop == true) {
    motor_stop();
  }

  //  int distance = readUltrasonicDistance();
  //   Serial.print("Distanta: ");
  //   Serial.print(distance);
  //   Serial.println(" cm");

  previous_state_left = sensor_left;
  previous_state_right = sensor_right;
  previous_state_cross = sensor_cross;
}

void func_turn_right() {
  while (turn_right == true) {
    previous_state_left = sensor_left;
    previous_state_right = sensor_right;
    control_motors_PWM(100, 0);
    delay(200);
    sensor_left = digitalRead(PIN_LINE_LEFT);
    sensor_right = digitalRead(PIN_LINE_RIGHT);
  }
}

void func_turn_left() {
  while (turn_left == true) {
    previous_state_left = sensor_left;
    previous_state_right = sensor_right;
    control_motors_PWM(0, 100);
    delay(200);
    sensor_left = digitalRead(PIN_LINE_LEFT);
    sensor_right = digitalRead(PIN_LINE_RIGHT);
  }
}

void func_rotate_left() {
  while (rotate_left) {
    previous_state_left = sensor_left;
    previous_state_right = sensor_right;
    control_motors_PWM(-100, 100);
    delay(200);
    sensor_left = digitalRead(PIN_LINE_LEFT);
    sensor_right = digitalRead(PIN_LINE_RIGHT);
  }
}

void func_rotate_right() {
  while (rotate_right) {
    previous_state_left = sensor_left;
    previous_state_right = sensor_right;
    control_motors_PWM(100, -100);
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
      control_motors_PWM(0, 130);             // intoarcere spre dreapta
      // control_motors(0, 30);

    } else if (last_detection_on_right) {  // ultimul senzor care a detectat linia a fost cel din dreapta
      control_motors_PWM(130, 0);        // intoarcere spre stanga
      // control_motors(30, 0);

    } else {
      //control_motors(30, 30);  // continua drept daca nu avem nicio referinta
    }
  } else if (sensor_left == 0 && sensor_right == 1) {  // senzorul din dreapta detecteaza linie -> turn left
    control_motors_PWM(125, 110);
    // control_motors(40, 20);
    last_detection_on_left = false;
    last_detection_on_right = true;
  } else if (sensor_left == 1 && sensor_right == 0) {  // senzorul din stanga detecteaza linie -> turn right
    control_motors_PWM(110, 125);
    // control_motors(20, 40);
    last_detection_on_left = true;
    last_detection_on_right = false;
  } else {
    // control_motors(40, 40);
    control_motors_PWM(120, 115);
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

const char* directionToString(enum direction dir) {
  switch (dir) {
    case N: return "N";
    case E: return "E";
    case S: return "S";
    case W: return "W";
    default: return "Unknown";
  }
}

// X SI Y COMBINAT:
void navigate_to(int targetX, int targetY) {
  // Resetare variabile de direcție
  turn_left = false;
  turn_right = false;

  // Calcularea diferenței dintre coordonatele curente și cele țintă
  deltaX = targetX - X;
  deltaY = targetY - Y;

  // Dacă robotul este deja la destinație
  if (deltaX == 0 && deltaY == 0) {
    stop = true;
    // main_reached_destination = false;
    return;
  } else {
    stop = false;
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

int readUltrasonicDistance() {
  // Trig pin LOW pentru 2 microsecunde
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);

  // Trig pin HIGH pentru 10 microsecunde
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Măsoară durata impulsului pe ECHO pin
  duration = pulseIn(PIN_ECHO, HIGH);

  // Calculează distanța în cm
  distance = duration * 0.034 / 2;

  return distance;
}
