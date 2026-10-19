#define BLYNK_TEMPLATE_ID "TMPL2RBERiOJ1"
#define BLYNK_TEMPLATE_NAME "robotic arm"
#define BLYNK_AUTH_TOKEN "tGV2IKYUvGa53EVdFgGFxin7pUAtSwyo"
#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <vector> 
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define SDA_PIN 21
#define SCL_PIN 22
#define OLED_RESET   -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define JOY1_X_PIN 35  // Base
#define JOY1_Y_PIN 32  // Shoulder
#define JOY2_Y_PIN 34  // Elbow

#define ENCODER_CLK 18
#define ENCODER_DT  19
#define ENCODER_SW  5

int lastCLKState;
unsigned long lastEncoderDebounce = 0;

struct ServoPins
{
  Servo servo;
  int servoPin;
  String servoName;
  int initialPosition; 
  float currentPos;    
  float targetPos;     
  bool isAttached; 
};

std::vector<ServoPins> servoPins = 
{
  { Servo(), 13 , "Base", 90, 90.0, 90.0, false},
  { Servo(), 12 , "Shoulder", 90, 90.0, 90.0, false},
  { Servo(), 14 , "Elbow", 90, 90.0, 90.0, false},
  { Servo(), 27 , "Gripper", 90, 90.0, 90.0, false},
};

struct RecordedStep
{
  int servoIndex;
  int value;
  int delayInStep;
};

std::vector<RecordedStep> recordedSteps;

bool recordSteps = false;
bool playRecordedSteps = false;

unsigned long previousTimeInMilli = millis();
unsigned long lastUpdatePID = 0;
unsigned long lastDisplayUpdate = 0;

unsigned long lastActivityTime = 0;
bool isMoving = false;
bool isDisplayOn = true;
const unsigned long IDLE_SERVO_TIMEOUT  = 2000; 
const unsigned long IDLE_OLED_TIMEOUT   = 8000; 

const float SMOOTH_ALPHA = 0.08; 

const char* ssid     = "Galaxy A36 5G 97F6";
const char* password = "12345678";

void registerActivity() {
  lastActivityTime = millis();
  
  if (!isMoving) {
    isMoving = true;
    setCpuFrequencyMhz(240); 
  }

  if (!isDisplayOn) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    isDisplayOn = true;
  }
}

void attachServoIfNeeded(int i) {
  if (!servoPins[i].isAttached) {
    servoPins[i].servo.attach(servoPins[i].servoPin, 500, 2400);
    servoPins[i].isAttached = true;
  }
}

void readRotaryEncoder()
{
  if (playRecordedSteps) return;

  int currentCLKState = digitalRead(ENCODER_CLK);

  if (currentCLKState != lastCLKState && currentCLKState == LOW)
  {
    if (millis() - lastEncoderDebounce > 5)
    {
      registerActivity();
      attachServoIfNeeded(3);

      if (digitalRead(ENCODER_DT) != currentCLKState)
      {
        servoPins[3].targetPos = constrain(servoPins[3].targetPos + 4.0, 0, 180);
      }
      else
      {
        servoPins[3].targetPos = constrain(servoPins[3].targetPos - 4.0, 0, 180);
      }

      if (recordSteps) {
        writeServoValues(3, (int)servoPins[3].targetPos);
      }
      lastEncoderDebounce = millis();
    }
  }
  lastCLKState = currentCLKState;
}

void updateOLEDDisplay()
{
  if (!isDisplayOn) return;

  if (millis() - lastDisplayUpdate >= 80)
  {
    lastDisplayUpdate = millis();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    display.setCursor(15, 0);
    display.println(F("--- ROBOT ARM ---"));
    display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

    display.setCursor(0, 14);
    display.printf("Base    (13): %d deg", (int)round(servoPins[0].currentPos));
    
    display.setCursor(0, 26);
    display.printf("Shoulder(12): %d deg", (int)round(servoPins[1].currentPos));
    
    display.setCursor(0, 38);
    display.printf("Elbow   (14): %d deg", (int)round(servoPins[2].currentPos));
    
    display.setCursor(0, 50);
    display.printf("Gripper (27): %d deg", (int)round(servoPins[3].currentPos));

    display.display();
  }
}

void readJoysticks()
{
  if (playRecordedSteps) return;

  int joy1X = analogRead(JOY1_X_PIN);
  int joy1Y = analogRead(JOY1_Y_PIN);
  int joy2Y = analogRead(JOY2_Y_PIN);

  if (abs(joy1X - 2048) > 500) {
    registerActivity();
    attachServoIfNeeded(0);
    float speed = (joy1X - 2048) / 2048.0 * 1.5;
    servoPins[0].targetPos = constrain(servoPins[0].targetPos + speed, 0, 180);
    if (recordSteps) writeServoValues(0, (int)servoPins[0].targetPos);
  }

  if (abs(joy1Y - 2048) > 500) {
    registerActivity();
    attachServoIfNeeded(1);
    float speed = (joy1Y - 2048) / 2048.0 * 1.5;
    servoPins[1].targetPos = constrain(servoPins[1].targetPos + speed, 0, 180);
    if (recordSteps) writeServoValues(1, (int)servoPins[1].targetPos);
  }

  if (abs(joy2Y - 2048) > 500) {
    registerActivity();
    attachServoIfNeeded(2);
    float speed = (joy2Y - 2048) / 2048.0 * 1.5;
    servoPins[2].targetPos = constrain(servoPins[2].targetPos + speed, 0, 180);
    if (recordSteps) writeServoValues(2, (int)servoPins[2].targetPos);
  }
}

void updateSmoothServos()
{
  if (millis() - lastUpdatePID >= 15)
  {
    lastUpdatePID = millis();
    readJoysticks();

    for (int i = 0; i < servoPins.size(); i++)
    {
      float diff = servoPins[i].targetPos - servoPins[i].currentPos;
      
      if (abs(diff) > 0.1) 
      {
        attachServoIfNeeded(i);
        servoPins[i].currentPos += diff * SMOOTH_ALPHA;
        servoPins[i].servo.write((int)round(servoPins[i].currentPos));
      }
    }
  }
}

void writeServoValues(int servoIndex, int value)
{
  registerActivity();
  attachServoIfNeeded(servoIndex);

  if (recordSteps)
  {
    RecordedStep recordedStep;       
    if (recordedSteps.size() == 0)
    {
      for (int i = 0; i < servoPins.size(); i++)
      {
        recordedStep.servoIndex = i; 
        recordedStep.value = (int)servoPins[i].targetPos; 
        recordedStep.delayInStep = 0;
        recordedSteps.push_back(recordedStep);        
      }      
    }
    unsigned long currentTime = millis();
    recordedStep.servoIndex = servoIndex; 
    recordedStep.value = value; 
    recordedStep.delayInStep = currentTime - previousTimeInMilli;
    recordedSteps.push_back(recordedStep);  
    previousTimeInMilli = currentTime;        
  }

  servoPins[servoIndex].targetPos = value;   
}

void playRecordedRobotArmSteps()
{
  if (recordedSteps.size() == 0)
  {
    playRecordedSteps = false;
    Blynk.virtualWrite(V5, 0);
    return;
  }

  registerActivity();
  for (int i = 0; i < servoPins.size(); i++) attachServoIfNeeded(i);
  
  for (int i = 0; i < 4 && playRecordedSteps; i++)
  {
    RecordedStep &recordedStep = recordedSteps[i];
    servoPins[recordedStep.servoIndex].targetPos = recordedStep.value;
    
    while (abs(servoPins[recordedStep.servoIndex].currentPos - recordedStep.value) > 1.0 && playRecordedSteps)  
    {
      updateSmoothServos();
      updateOLEDDisplay();
      Blynk.run();
      delay(10);
    }
  }
  
  unsigned long startDelay = millis();
  while (millis() - startDelay < 1000 && playRecordedSteps) {
    updateSmoothServos();
    updateOLEDDisplay();
    Blynk.run();
    delay(10);
  }
  
  for (int i = 4; i < recordedSteps.size() && playRecordedSteps; i++)
  {
    RecordedStep &recordedStep = recordedSteps[i];
    
    unsigned long stepStart = millis();
    while (millis() - stepStart < recordedStep.delayInStep && playRecordedSteps) {
      updateSmoothServos();
      updateOLEDDisplay();
      Blynk.run();
      delay(10);
    }
    
    if (playRecordedSteps) {
      servoPins[recordedStep.servoIndex].targetPos = recordedStep.value;
    }
  }

  playRecordedSteps = false;
  Blynk.virtualWrite(V5, 0);
}

void setUpPinModes()
{
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  lastCLKState = digitalRead(ENCODER_CLK);

  Wire.begin(SDA_PIN, SCL_PIN);

  bool status = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!status) {
    status = display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  }

  if (status) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(15, 25);
    display.println(F("System Ready..."));
    display.display();
    delay(1000);
  }

  pinMode(JOY1_X_PIN, INPUT);
  pinMode(JOY1_Y_PIN, INPUT);
  pinMode(JOY2_Y_PIN, INPUT);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < servoPins.size(); i++)
  {
    servoPins[i].servo.setPeriodHertz(50);
    attachServoIfNeeded(i);
    
    servoPins[i].currentPos = servoPins[i].initialPosition;
    servoPins[i].targetPos = servoPins[i].initialPosition;
    servoPins[i].servo.write(servoPins[i].initialPosition);    
  }
  lastActivityTime = millis();
}

BLYNK_WRITE(V0) { if (!playRecordedSteps) writeServoValues(0, param.asInt()); } 
BLYNK_WRITE(V1) { if (!playRecordedSteps) writeServoValues(1, param.asInt()); } 
BLYNK_WRITE(V2) { if (!playRecordedSteps) writeServoValues(2, param.asInt()); } 
BLYNK_WRITE(V3) { if (!playRecordedSteps) writeServoValues(3, param.asInt()); } 

BLYNK_WRITE(V4) { 
  registerActivity();
  recordSteps = param.asInt();
  if (recordSteps)
  {
    recordedSteps.clear();
    previousTimeInMilli = millis();
  }
}

BLYNK_WRITE(V5) { 
  registerActivity();
  playRecordedSteps = param.asInt();
}

void setup(void) 
{
  Serial.begin(115200);
  setUpPinModes();

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

  Serial.print("Connected to WiFi. IP address: ");
  Serial.println(WiFi.localIP());
}

void managePowerInIdle()
{
  unsigned long now = millis();

  if (isMoving && (now - lastActivityTime > IDLE_SERVO_TIMEOUT))
  {
    isMoving = false;
    for (int i = 0; i < servoPins.size(); i++) {
      if (servoPins[i].isAttached) {
        servoPins[i].servo.detach();
        servoPins[i].isAttached = false;
      }
    }
    setCpuFrequencyMhz(80); 
  }

  if (isDisplayOn && (now - lastActivityTime > IDLE_OLED_TIMEOUT))
  {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    isDisplayOn = false;
  }

  if (!isMoving && !playRecordedSteps)
  {
    esp_sleep_enable_timer_wakeup(15 * 1000); 
    esp_light_sleep_start();
  }
}

void loop() 
{
  if (!isMoving || playRecordedSteps) {
    Blynk.run();
  }
  
  readRotaryEncoder(); 
  updateSmoothServos();
  updateOLEDDisplay();

  if (playRecordedSteps)
  { 
    playRecordedRobotArmSteps();
  }

  managePowerInIdle();
}اشرح كويس