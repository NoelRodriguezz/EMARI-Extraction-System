/**
 *                Instituto Politécnico Nacional
 * Unidad Profesional Interdisciplinaria de Ingeniería Campus Zacatecas (UPIIZ)
 *       
 *      Infrared Radiation-Assisted Metabolite Extractor (EMARI)  
 *
 *                 Systems control and HMI code
 *               
 *                            Author:
 *                  Noel Francisco Rodríguez
 *
 *                            Advisors:
 *                  Dr. Hans Christian Correa-Aguado
 *                    Dr. Teodoro Ibarra Pérez
 *                M. en C. Ramón Jaramillo Martínez
 *
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MAX31865.h>
#include <dimmable_light.h> 

// ==========================================
//      PIN CONFIGURATION (ESP32 S3)
// ==========================================

// ---  EMERGENCY STOP BUTTON ---
#define PIN_EMERGENCY 1    

// ---  DIMMERS ---
#define D1_PIN        48 //Regula Dimmer 1
#define D2_PIN        21 //Regula Dimmer 2
#define D3_PIN        38 //Regula Dimmer 3
#define SYNC_PIN_ZC   47   

// ---  MOTORS ---
#define M1_PWM_PIN    36 
#define M1_GND_PIN    35 
#define M2_PWM_PIN    0  
#define M2_GND_PIN    45 

// ---  MOTOR ENCODERS ---
#define ENC_M1_PIN    7   
#define ENC_M2_PIN    17  

// ---  PT100 Sensor (SPI) ---
#define CS_PIN        10
#define SPI_MOSI      11
#define SPI_MISO      13
#define SPI_SCK       12

// ---  ROTARY ENCODER (HMI Knob) ---
#define CLK           4    
#define DT            5    
#define SW            6 

// ---  20x4 DISPLAY (I2C) ---
#define I2C_SDA       46
#define I2C_SCL       9

// ---  BUZZER ---
#define BUZZER_PIN    8    

// ---  LM35 SENSORS  ---
#define PIN_LM35_1    15   
#define PIN_LM35_2    16   
#define PIN_LM35_PCB  18  

// --- 4-WIRE PWM FANS ---
#define FAN1_PWM_PIN  39
#define FAN1_TAC_PIN  40 
#define FAN2_PWM_PIN  41
#define FAN2_TAC_PIN  42 


// ==========================================
//      PARAMETERS
// ==========================================
#define PWM_RES       8      
#define MOT_FREQ      5000 
#define M1_CHANNEL    0  
#define M2_CHANNEL    1  
#define FAN_FREQ      25000 
#define FAN1_CHANNEL  2
#define FAN2_CHANNEL  3

#define SAMPLE_TIME   50
#define STARTUP_PWM   60  

// ==========================================
//      OBJECTS AND VARIABLES
// ==========================================

LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_MAX31865 thermo = Adafruit_MAX31865(CS_PIN, SPI_MOSI, SPI_MISO, SPI_SCK);

DimmableLight dimmer1(D1_PIN);
DimmableLight dimmer2(D2_PIN);
DimmableLight dimmer3(D3_PIN);

#define RREF      430.0
#define RNOMINAL  100.0

enum SystemState { 
  MAIN_MENU, CONFIG_MENU, ADV_CONFIG_MENU, 
  CALIB_MENU, EDIT_PARAM, EDIT_ADV_PARAM, EDIT_CALIB_PARAM, 
  CALIB_MOT_MENU, CALIB_MOT_MANUAL, CALIB_MOT_RESULT, 
  PPR_EDIT_MENU, PPR_EDIT_VAL, 
  TEST_ACT, TEST_EDIT,
  RUNNING, STOPPING, MONITOR, SHUTDOWN, EMERGENCY_STOP 
};

enum AgitationType { MAGNETICA, ROTATORIA };
enum SoundType { SND_NONE, SND_START, SND_FINISH, SND_ALARM, SND_CLICK };
enum ControlSensor { SENS_PT100, SENS_LM35_AVG };
enum TempUnit { UNIT_C, UNIT_F };
enum BuzzerMode { BUZZ_OFF, BUZZ_ON, BUZZ_ALERTS }; 

struct ConfigParams {
  AgitationType agitation;
  int targetTemp;      
  int agitationSpeed;  
  int operationTime;
  ControlSensor controlSensor;
  BuzzerMode buzzerMode; 
  int pcbMaxTemp;      
  TempUnit tempUnit;
  float pt100Offset; 
  float lm35_1_Offset; 
  float lm35_2_Offset; 
  float lm35_pcb_Offset; 
};

SystemState currentState = MAIN_MENU;
ConfigParams params = {
  MAGNETICA, 40, 50, 5, SENS_PT100, BUZZ_ON, 50, UNIT_C, 0.0, 0.0, 0.0, 0.0};

// --- TEMPERATURE PID VARIABLES ---
float Kp = 17.0;       
float Ki = 0.5;        
float Kd = 190;       
float pidIntegral = 0;
float pidLastError = 0;

// Control Variables
unsigned long processStartTime = 0;
unsigned long lastScreenUpdate = 0;
unsigned long lastControlTime = 0; 
unsigned long lastCalibUpdate = 0; 

// --- MOTOR VARIABLES ---
volatile long pulseCount1 = 0;
volatile long pulseCount2 = 0;
float currentPPR_M1 = 2960.0; 
float currentPPR_M2 = 540.0; 

float currentRPM_M1 = 0; 
float currentRPM_M2 = 0; 
float currentPWM_M1 = 0; 
float currentPWM_M2 = 0;

// Hardware State
int lastSentDimmers = -99; 
int lastSentFan = -1; 
int currentHeatPowerPct = 0;
float currentPcbTemp = 0.0;

// Buzzer
SoundType currentSound = SND_NONE;
unsigned long buzzTimer = 0;
int buzzStep = 0;
bool buzzState = false;
bool pcbAlarmMuted = false;

// Navigation
int currentSelection = 0;
int configSelection = 0;
int advConfigSelection = 0; 
int calibSelection = 0; 
int calibMotSelection = 0; 
int pprSelection = 0;      
bool menuRedrawRequired = true;

bool hasReachedTarget = false;
float initialProcessTemp = 0.0; 

const char* mainMenuItems[] = { 
  "1.Iniciar Proceso", 
  "2.Configuracion", 
  "3.Monitor Sensores", 
  "4.Prueba Actuador", 
  "5.Apagar Sistema" };

const int mainMenuSize = 5;
const char* configLabels[] = { 
  "Agita:", 
  "Temp.Obj.:", 
  "Velocidad:", 
  "T.Operac.:", 
  "Config. Avanz. >", 
  "[ SALIR ]" };

const int advMenuSize = 5; 
const char* advConfigLabels[] = { 
  "Buzzer:", 
  "PCB Max:", 
  "Unidad:", 
  "Calib. Sens. >", 
  "[ SALIR ]" };

const char* calibLabels[] = { 
  "PT100", 
  "S.Amb 1", 
  "S.Amb 2", 
  "S. PCB", 
  "[ SALIR ]" };

// --- UPDATED TEST LABELS ---
const char* testLabels[] = { 
  "Motor:",       // 0
  "Vel.Motor:",   // 1 
  "Fuentes RI:",  // 2 
  "Vent 1:",      // 3
  "Vent 2:",      // 4
  "Estado:",      // 5
  "[ SALIR ]"     // 6
};
const int testMenuSize = 7; 

// Test Variables
int testSelection = 0;
int testSpeed = 0; 
int testHeat = 0;  
int testFan1 = 0; 
int testFan2 = 0; 
bool testRunning = false;
AgitationType testMotor = MAGNETICA;

// Encoder
int lastClk = HIGH;
unsigned long lastButtonPress = 0;
const int debounceDelay = 200; 

// Motor Calibration Aux Variables
long calibMotPulses = 0;
int motorToCalib = 0;

// INTERRUPTS
volatile unsigned long lastDebounceM1 = 0;
volatile unsigned long lastDebounceM2 = 0;

// Motor 1 (Rotary)
void IRAM_ATTR onPulseM1() { 
  unsigned long now = micros();
  if (now - lastDebounceM1 > 1000) { 
    pulseCount1++; 
    lastDebounceM1 = now;
  }
}

// Motor 2 (Magnetic)
void IRAM_ATTR onPulseM2() { 
  unsigned long now = micros();
  if (now - lastDebounceM2 > 200) {
    pulseCount2++; 
    lastDebounceM2 = now;
  }
}

// --- PROTOTYPES ---
void lcdPrint(const char* text);
void drawInterface();
void drawTestMenu(); 
void drawConfigMenu();
void drawAdvConfigMenu(); 
void drawCalibMenu(); 
void updateCalibValuesOnly(); 
void drawListMenu(const char* title, const char* items[], int size, int selection);
void readEncoder();
void handleRotation(int dir);
void handleButton();
void executeMainMenuAction();
void modifyParam(int dir);
void modifyAdvParam(int dir); 
void modifyCalibParam(int dir); 
void modifyTestParam(int dir);
void runSystemLogic(); 
void runStoppingLogic(); 
void runMonitorLogic();
void runCalibrationLogic(); 
float readLM35(int pin);
float getRawPT100(); 
float getControlTemperature(); 
float convertTemp(float celsius); 
void controlSystemMotors(bool active); 
void calculateRPM(); 
void controlTemperature(bool active);
void controlPCBCooling(bool active); 
void stopAllActuators(); 
void triggerSound(SoundType type);
void runBuzzerLogic();
void checkEmergencyButton(); 
void runSafetyCheck(); 

void drawMotorCalibMenu(); 
void drawMotorCalibManual(); 
void drawMotorCalibResult();
void drawPPREditMenu();     
void modifyPPRVal(int dir); 

// ==========================================
//                SETUP
// ==========================================

void setup() {
  Serial.begin(115200);

  // --- PIN CONFIGURATION ---
  pinMode(CLK, INPUT); 
  pinMode(DT, INPUT); 
  pinMode(SW, INPUT_PULLUP);
  pinMode(PIN_LM35_1, INPUT); 
  pinMode(PIN_LM35_2, INPUT);
  pinMode(PIN_LM35_PCB, INPUT);
  pinMode(FAN1_TAC_PIN, INPUT_PULLUP); 
  pinMode(FAN2_TAC_PIN, INPUT_PULLUP);
  pinMode(PIN_EMERGENCY, INPUT_PULLUP); 

  pinMode(ENC_M1_PIN, INPUT_PULLUP); 
  pinMode(ENC_M2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_M1_PIN), onPulseM1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_M2_PIN), onPulseM2, RISING);

  pinMode(M1_GND_PIN, OUTPUT); 
  pinMode(M2_GND_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT); 
  digitalWrite(BUZZER_PIN, LOW);

  // --- PWM CONFIGURATION ---
  ledcSetup(M1_CHANNEL, MOT_FREQ, PWM_RES);
  ledcSetup(M2_CHANNEL, MOT_FREQ, PWM_RES);
  ledcAttachPin(M1_PWM_PIN, M1_CHANNEL);
  ledcAttachPin(M2_PWM_PIN, M2_CHANNEL);

  ledcSetup(FAN1_CHANNEL, FAN_FREQ, PWM_RES);
  ledcSetup(FAN2_CHANNEL, FAN_FREQ, PWM_RES);
  ledcAttachPin(FAN1_PWM_PIN, FAN1_CHANNEL);
  ledcAttachPin(FAN2_PWM_PIN, FAN2_CHANNEL);
  
  ledcWrite(FAN1_CHANNEL, 0); ledcWrite(FAN2_CHANNEL, 0);

  // --- SCREEN AND PERIPHERALS INIT ---
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(25000); 
  Wire.setTimeOut(10);

  lcd.init(); 
  lcd.backlight(); 
  lcd.clear();

  DimmableLight::setSyncPin(SYNC_PIN_ZC); 
  DimmableLight::begin(); 
  stopAllActuators(); 

  thermo.begin(MAX31865_3WIRE); 
  
  // --- SPLASH SCREEN ---
  
  lcd.setCursor(2, 0); 
  lcdPrint("EXT. METABOLITOS");   
  
  lcd.setCursor(0, 1); 
  lcdPrint("ASISTIDO POR RAD. IR"); 
  
  lcd.setCursor(0, 2); 
  lcdPrint("____________________"); 
  
  lcd.setCursor(3, 3); 
  lcdPrint("Noel Rodriguez");     
  
  triggerSound(SND_START);
  delay(3000); 
  
  lcd.clear();
  menuRedrawRequired = true;
}
// ==========================================
//                LOOP
// ==========================================
void loop() {
  checkEmergencyButton(); 
  
  unsigned long now = millis();
  if (now - lastControlTime >= SAMPLE_TIME) {
      if(currentState != CALIB_MOT_MANUAL) calculateRPM(); 
      runSafetyCheck(); 
      
      if (currentState == RUNNING) runSystemLogic(); 
      else if (currentState == STOPPING) runStoppingLogic();
      else if (currentState == MONITOR) runMonitorLogic();
      else if (currentState == CALIB_MOT_MANUAL) runCalibrationLogic();
      
      // TEST MODE
      else if (currentState == TEST_ACT || currentState == TEST_EDIT) {
        if (testRunning) {
          controlSystemMotors(true); 
          
          int dimVal = map(testHeat, 0, 100, 0, 255);
          if (dimVal != lastSentDimmers) {
            dimmer1.setBrightness(dimVal); 
            dimmer2.setBrightness(dimVal); 
            dimmer3.setBrightness(dimVal);
            lastSentDimmers = dimVal;
          }
          int f1PWM = (testFan1 > 0) ? map(testFan1, 0, 100, 75, 255) : 0;
          int f2PWM = (testFan2 > 0) ? map(testFan2, 0, 100, 75, 255) : 0;
          ledcWrite(FAN1_CHANNEL, f1PWM); ledcWrite(FAN2_CHANNEL, f2PWM);
        } else {
            controlSystemMotors(false);
            ledcWrite(FAN1_CHANNEL, 0); ledcWrite(FAN2_CHANNEL, 0);
            if(lastSentDimmers != 0) { 
              dimmer1.setBrightness(0); 
              dimmer2.setBrightness(0); 
              dimmer3.setBrightness(0); 
              lastSentDimmers=0; 
            }
        }
      }
      lastControlTime = now;
  }
  
  readEncoder(); 
  runBuzzerLogic(); 
  
  if (millis() - lastScreenUpdate >= 300) {
      if (menuRedrawRequired) {
        drawInterface();
        if (currentState == CALIB_MENU || currentState == EDIT_CALIB_PARAM) {
            drawCalibMenu(); updateCalibValuesOnly();
        }
        menuRedrawRequired = false;
      }
      lastScreenUpdate = millis();
  }
}

// ==========================================
//      MOTOR CONTROL 
// ==========================================

void calculateRPM() {
    float ppr1 = (currentPPR_M1 > 0) ? currentPPR_M1 : 1.0;
    float rawRPM1=((float)pulseCount1*(1000.0/SAMPLE_TIME)*60.0)/ppr1;
    pulseCount1 = 0; 
    currentRPM_M1 = (currentRPM_M1 * 0.5) + (rawRPM1 * 0.5); 
    
    float ppr2 = (currentPPR_M2 > 0) ? currentPPR_M2 : 1.0;
    float rawRPM2=((float)pulseCount2*(1000.0/SAMPLE_TIME)*60.0)/ppr2;
    pulseCount2 = 0;
    currentRPM_M2 = (currentRPM_M2 * 0.7) + (rawRPM2 * 0.3); 
}
void controlSystemMotors(bool active) {
  if (!active) { 
      stopAllActuators(); 
      currentPWM_M1 = 0; currentPWM_M2 = 0; 
      return; 
  }
  
  float targetRPM = 0;
  bool controlM1 = false; 

  // --- MOTOR SELECTION ---
  if (currentState == TEST_ACT || currentState == TEST_EDIT) {
      float speedPct = (float)testSpeed;
      targetRPM = map(speedPct, 0, 100, 0, 200); 
      if (testMotor == ROTATORIA) controlM1 = true; 
      else controlM1 = false;
  } else {
      targetRPM = (float)params.agitationSpeed;
      if (params.agitation == ROTATORIA) controlM1 = true; 
      else controlM1 = false;
  }
  
  static unsigned long lastKickM1 = 0;
  static unsigned long lastKickM2 = 0;
  unsigned long now = millis();

  if (controlM1) {
      // ==========================================
      //      MOTOR 1: ROTARY 
      // ==========================================
      ledcWrite(M2_CHANNEL, 0); currentPWM_M2 = 0; 
      
      if (currentRPM_M1 > 1000.0) { 
          pulseCount1 = 0;    
          if (currentPWM_M1 > 60) {  
          } 
          else {
             currentRPM_M1 = 0;
          }
      }

      if (targetRPM == 0) {
          currentPWM_M1 = 0;
      } else {
          float error = targetRPM - currentRPM_M1;
          float absError = abs(error);
          float step = 0; float tolerance = 0.5;
          
          if (absError > tolerance) {
              if (absError > 10.0) step = 5.0;       
              else if (absError > 3.0) step = 2.0;   
              else step = 0.5;                       
              
              if (error > 0) {
                  if (currentPWM_M1 < STARTUP_PWM && targetRPM > 0){
                    currentPWM_M1 = STARTUP_PWM; 
                  }
                  else currentPWM_M1 += step; 
              } else {
                  currentPWM_M1 -= step; 
              }
          }
      }
      currentPWM_M1 = constrain(currentPWM_M1, 0, 255);
      ledcWrite(M1_CHANNEL, (int)currentPWM_M1);
      digitalWrite(M1_GND_PIN, LOW);
      
  } else {
      // ==========================================
      //      MOTOR 2: MAGNETIC 
      // ==========================================
      ledcWrite(M1_CHANNEL, 0); currentPWM_M1 = 0; 
      
      if (currentRPM_M2 > 1500.0) {
          pulseCount2 = 0;    
    
          if (currentPWM_M2 > 60) {
          }
          else {
              currentRPM_M2 = 0;
          }
      }

      if (targetRPM == 0) {
          currentPWM_M2 = 0;
      } else {
          float error = targetRPM - currentRPM_M2;
          float absError = abs(error);
          float step = 0.5;
          
          if (absError > 2.0) { 
              if (absError > 10.0) step = 2.0; 
              if (error > 0) {
                  if (currentPWM_M2 < 40) currentPWM_M2 = 40;
                  else currentPWM_M2 += step; 
              } else {
                  currentPWM_M2 -= step;
              }
          }
      }
      currentPWM_M2 = constrain(currentPWM_M2, 0, 255);
      ledcWrite(M2_CHANNEL, (int)currentPWM_M2);
      digitalWrite(M2_GND_PIN, LOW);
  }
}
// ==========================================
//      TEMPERATURE CONTROL
// ==========================================
void controlTemperature(bool active) {
  float target = (float)params.targetTemp;
  
  if (!active) { 
    if (lastSentDimmers != 0) { 
      dimmer1.setBrightness(0); 
      dimmer2.setBrightness(0); 
      dimmer3.setBrightness(0); 
      lastSentDimmers = 0; 
      currentHeatPowerPct = 0; }
    pidIntegral = 0; 
    pidLastError = 0;
    return; 
  }
  
  static int tempCounter = 0;
  tempCounter++;
  
  if (tempCounter > 4) { 
    float currentVal = getControlTemperature(); 
    float error = target - currentVal; 
    float P = error * Kp;
    
    if (abs(error) < 3.0) {
        pidIntegral += error;
        pidIntegral = constrain(pidIntegral, -50.0, 50.0);
    } else {
        pidIntegral = 0; 
    }
    float I = pidIntegral * Ki;
    float D = (error - pidLastError) * Kd;
    pidLastError = error;

    float output = P + I + D;
    
    if (error < -0.5) output = 0; 

    int powerPct = constrain((int)output, 0, 100); 

    float areaTemp1 = readLM35(PIN_LM35_1) + params.lm35_1_Offset; 
    if (areaTemp1 > 135.0) powerPct = 0; 
    
    currentHeatPowerPct = powerPct; 
    int dimVal = map(powerPct, 0, 100, 0, 255);
    
    if (dimVal != lastSentDimmers) { 
        dimmer1.setBrightness(dimVal); 
        dimmer2.setBrightness(dimVal); 
        dimmer3.setBrightness(dimVal); 
        lastSentDimmers = dimVal; 
    }
    tempCounter = 0;
  }
}
void controlPCBCooling(bool active) {
  if (!active) { 
    if (lastSentFan != 0) { 
      ledcWrite(FAN1_CHANNEL, 0); 
      ledcWrite(FAN2_CHANNEL, 0); 
      lastSentFan = 0; 
      } 
    return;
  }
  int fanPWM = 0; 
  if (currentPcbTemp < 30.0) fanPWM = 0; 
  else { 
    fanPWM = map((int)currentPcbTemp, 30, 50, 160, 255); 
    fanPWM = constrain(fanPWM, 0, 255); 
  }
  if (abs(fanPWM - lastSentFan) > 2) { 
    ledcWrite(FAN1_CHANNEL, fanPWM); 
    ledcWrite(FAN2_CHANNEL, fanPWM); 
    lastSentFan = fanPWM; 
  }
}

void stopAllActuators() {
  ledcWrite(M1_CHANNEL, 0);
  digitalWrite(M1_GND_PIN, LOW); 
  ledcWrite(M2_CHANNEL, 0); 
  digitalWrite(M2_GND_PIN, LOW);
  currentPWM_M1 = 0; currentPWM_M2 = 0; 
  dimmer1.setBrightness(0); 
  dimmer2.setBrightness(0); 
  dimmer3.setBrightness(0); 
  lastSentDimmers = 0;
  ledcWrite(FAN1_CHANNEL, 0); 
  ledcWrite(FAN2_CHANNEL, 0); 
  lastSentFan = 0;
}

void checkEmergencyButton() {
  
  if (digitalRead(PIN_EMERGENCY) == LOW) {
    unsigned long pressStart = millis();
    
    while (digitalRead(PIN_EMERGENCY) == LOW) {
      
      if (millis() - pressStart > 500) {
          stopAllActuators(); 
          lcd.clear();
          
          bool inEmergency = true;
          bool alarmSilenced = false; 
          bool counting = false;
          unsigned long countStart = 0;
          
          // Blocking Loop
          while (inEmergency) {
            stopAllActuators(); 
            
            // PHASE 1: AUDIO AND VISUAL ALARM
            if (!alarmSilenced) {
               lcd.setCursor(3, 1); lcdPrint("! PARO TOTAL !");
               lcd.setCursor(2, 2); lcdPrint("SISTEMA BLOQUEADO");
               lcd.setCursor(0, 3); lcdPrint("Click para Silenciar");
               
               if ((millis() / 200) % 2 == 0){
                digitalWrite(BUZZER_PIN, HIGH);
               }
               else digitalWrite(BUZZER_PIN, LOW);
               
               // Mute with encoder button
               if (digitalRead(SW) == LOW) {
                   digitalWrite(BUZZER_PIN, LOW); 
                   alarmSilenced = true; 
                   lcd.clear();
                   delay(200); 
                   while(digitalRead(SW) == LOW); 
               }
            } 
            // PHASE 2: SAFE REBOOT MENU
            else {
               if (digitalRead(SW) == LOW) {
                   if (!counting) { 
                       counting = true; 
                       countStart = millis(); 
                       lcd.clear(); 
                   }
                   
                   unsigned long elapsed = millis() - countStart;
                   int remaining = 5 - (elapsed / 1000);
                   if (remaining < 0) remaining = 0;

                   lcd.setCursor(0, 0); 
                   lcdPrint("  REINICIANDO...    ");
                   lcd.setCursor(0, 1); 
                   lcdPrint(" MANTENGA PRESIONADO");
                   lcd.setCursor(0, 2); 
                   lcd.print("REINICIO EN: "); 
                   lcd.print(remaining); 
                   lcd.print(" s   ");

                   lcd.setCursor(0, 3);
                   int bars = map(elapsed, 0, 5000, 0, 20);
                   for(int i=0; i<20; i++) { 
                    if(i < bars) lcd.print(">"); else lcd.print(" "); 
                  }
                   
                   if (elapsed >= 5000) {
                       lcd.clear();
                       lcd.setCursor(6, 1); 
                       lcdPrint("SISTEMA");
                       lcd.setCursor(5, 2); 
                       lcdPrint("REINICIADO");
                       triggerSound(SND_START);
                       delay(1500);
                       testRunning = false;
                       currentState = MAIN_MENU;
                       menuRedrawRequired = true;
                       inEmergency = false; 
                   }
               } 
               else {
                   counting = false; 
                   lcd.setCursor(0, 1); 
                   lcdPrint("ALARMA SILENCIADA ");
                   lcd.setCursor(0, 2); 
                   lcdPrint("Presione 5s para  ");
                   lcd.setCursor(0, 3); 
                   lcdPrint("REINICIAR SISTEMA ");
               }
            }
            delay(50); 
          }
          return; 
      }
      delay(10); 
    }
  }
}

void runSystemLogic() {
  controlSystemMotors(true); 
  controlTemperature(true); 
  controlPCBCooling(true); 

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - processStartTime;
  unsigned long totalTimeMillis=(unsigned long)params.operationTime*60000;
  
  // Check for timeout
  if (elapsedTime >= totalTimeMillis) { 
      currentState = STOPPING; 
      lcd.clear(); 
      lcd.setCursor(3, 1); 
      lcdPrint("FINALIZANDO..."); 
      return; 
  }

  // Update Screen
  if (currentTime - lastScreenUpdate > 300) { 
    
    // --- TIME CALCULATION ---
    long remainingMillis = totalTimeMillis - elapsedTime;
    if (remainingMillis < 0) remainingMillis = 0; 
    int minutes = (remainingMillis / 1000) / 60;
    int seconds = (remainingMillis / 1000) % 60;
    
    // --- UNIFIED VARIABLES ---
    float currentTemp = getControlTemperature(); 
    int targetRPM = params.agitationSpeed; 
    float displayTemp = currentTemp;
    float targetT = (float)params.targetTemp;

    // --- DISPLAYING ON SCREEN ---
    
    // Row 1: Temperature
    lcd.setCursor(0, 1); 
    lcd.print("Temperatura: "); 
    lcd.print(convertTemp(displayTemp), 1); 
    lcd.print(params.tempUnit == UNIT_F ? "F" : "C");
    lcd.print("  "); 
    
    // Row 2: Remaining Time
    lcd.setCursor(0, 2); 
    lcd.print("T. restante: "); 
    if (minutes < 10) lcd.print("0"); lcd.print(minutes); 
    lcd.print(":"); 
    if (seconds < 10) lcd.print("0"); lcd.print(seconds);
    lcd.print("  "); 

    // Row 3: RPM (Setpoint) and Button
    lcd.setCursor(0, 3);
    lcd.print("RPM:"); 
    lcd.print(targetRPM); 
    lcd.print("      ");  
    
    lcd.setCursor(13, 3); 
    lcd.print("[Parar]"); 
    
    lastScreenUpdate = currentTime; 
  }
}

void runMonitorLogic() {
    
    static unsigned long lastMonUpdate = 0;
    if (millis() - lastMonUpdate < 300) return; 
    lastMonUpdate = millis();

    float t1 = readLM35(PIN_LM35_1) + params.lm35_1_Offset;
    float t2 = readLM35(PIN_LM35_2) + params.lm35_2_Offset;
    float tPCB = readLM35(PIN_LM35_PCB) + params.lm35_pcb_Offset;
    float tPt100 = getRawPT100() + params.pt100Offset;
    
    // Determine unit letter
    const char* unitStr = (params.tempUnit == UNIT_F) ? "F" : "C";

    // 1. SAMPLE (PT100)
    lcd.setCursor(10, 1); 
    lcd.print(convertTemp(tPt100), 1); 
    lcd.print(" "); lcd.print(unitStr); // Ej: "37.5 C"

    // 2. CHAMBER (LM35s)
    lcd.setCursor(10, 2); 
    lcd.print(convertTemp(t1), 0); 
    lcd.print(unitStr);
    lcd.print("/"); 
    lcd.print(convertTemp(t2), 0); 
    lcd.print(unitStr); // Ej: "28C/29C"

    // 3. PCB
    lcd.setCursor(10, 3); 
    lcd.print(convertTemp(tPCB), 0); 
    lcd.print(unitStr); 
    
    if(tPCB < 100 && tPCB > -10) { 
    }
    
    lcd.setCursor(13, 3); 
    lcd.print("[SALIR]"); 
}

void runStoppingLogic() {
  controlTemperature(false); 
  controlPCBCooling(true); 
  controlSystemMotors(false); 
  if (currentRPM_M1 <= 2.0 && currentRPM_M2 <= 2.0) { 
    stopAllActuators(); 
    triggerSound(SND_FINISH); 
    currentState = MAIN_MENU; 
    menuRedrawRequired = true; 
  }
}

float getRawPT100() { return thermo.temperature(RNOMINAL, RREF); }

float getControlTemperature() {
  float temp = 0.0;
  
  if (params.agitation == MAGNETICA) { 
      temp = getRawPT100() + params.pt100Offset;
  } 
  else { 
    float t1 = readLM35(PIN_LM35_1) + params.lm35_1_Offset; 
    float t2 = readLM35(PIN_LM35_2) + params.lm35_2_Offset; 
    temp = (t1 + t2+3*getRawPT100()) / 5.0; 
  } 
  
  if (isnan(temp)) return 0.0;
  return temp;
}

float convertTemp(float celsius) { 
  if (params.tempUnit == UNIT_F) { 
    return (celsius * 1.8) + 32.0; 
  } 
  return celsius; 
}

void triggerSound(SoundType type) { 
  if (params.buzzerMode == BUZZ_OFF && type != SND_ALARM) return; 
  if (params.buzzerMode == BUZZ_ALERTS && type == SND_CLICK) return; 
  if (type == SND_ALARM && pcbAlarmMuted) return;
  currentSound = type; buzzStep = 0; 
  buzzTimer = millis(); 
  digitalWrite(BUZZER_PIN, LOW);
}

void runBuzzerLogic() {
  if (currentSound == SND_NONE) return; 
  unsigned long currentMillis = millis();
  switch (currentSound) {
    case SND_START: 
      if (buzzStep == 0) {
        digitalWrite(BUZZER_PIN, HIGH); 
        buzzTimer = currentMillis; 
        buzzStep = 1; 
      } else if (buzzStep == 1 && (currentMillis - buzzTimer > 800)) { 
        digitalWrite(BUZZER_PIN, LOW); 
        currentSound = SND_NONE; 
      } 
      break;
    case SND_FINISH: 
      if (buzzStep == 0) { 
        digitalWrite(BUZZER_PIN, HIGH); 
        buzzTimer = currentMillis; 
        buzzStep++; 
        } else if (buzzStep==1&&(currentMillis-buzzTimer>150)){ 
          digitalWrite(BUZZER_PIN, LOW); 
          buzzTimer = currentMillis; 
          buzzStep++; 
        } else if (buzzStep==2&&(currentMillis-buzzTimer>100)){ 
          digitalWrite(BUZZER_PIN, HIGH); 
          buzzTimer = currentMillis; 
          buzzStep++; 
        } else if (buzzStep==3&&(currentMillis-buzzTimer>150)){ 
          digitalWrite(BUZZER_PIN, LOW);
          buzzTimer = currentMillis; 
          buzzStep++; 
        } else if (buzzStep==4&&(currentMillis-buzzTimer>100)){ 
          digitalWrite(BUZZER_PIN, HIGH); 
          buzzTimer = currentMillis; 
          buzzStep++; 
        } else if (buzzStep==5&&(currentMillis-buzzTimer>150)){ 
          digitalWrite(BUZZER_PIN, LOW); 
          currentSound = SND_NONE; 
        } 
        break;
    case SND_ALARM: 
      if (currentMillis - buzzTimer > 100) { 
        buzzState = !buzzState; 
        digitalWrite(BUZZER_PIN, buzzState ? HIGH : LOW); 
        buzzTimer = currentMillis; 
      } 
      break;
    case SND_CLICK: 
      if (buzzStep == 0) { 
        digitalWrite(BUZZER_PIN, HIGH); 
        buzzTimer = currentMillis; 
        buzzStep = 1; 
      } else if (buzzStep == 1 && (currentMillis - buzzTimer > 30)) { 
        digitalWrite(BUZZER_PIN, LOW); 
        currentSound = SND_NONE; 
      } 
      break;
    default: break;
  }
}
float readLM35(int pin) { 
  long sum = 0; 
  for(int i=0; i<5; i++){
    sum += analogReadMilliVolts(pin); 
    delayMicroseconds(50); 
  } 
  float val = (sum / 5.0) / 10.0;
  return val; 
}
void runSafetyCheck() {
  currentPcbTemp = readLM35(PIN_LM35_PCB) + params.lm35_pcb_Offset; 
  if (currentPcbTemp > params.pcbMaxTemp) {
     if (!pcbAlarmMuted && currentSound != SND_ALARM) {
        triggerSound(SND_ALARM);
     }
  } else {
     pcbAlarmMuted = false;
     if (currentSound == SND_ALARM) triggerSound(SND_NONE);
  }
}

// ==========================================
//      INPUTS AND MENU 
// ==========================================
void readEncoder() {
  int newClk = digitalRead(CLK);
  if (newClk != lastClk && newClk == LOW) {
    int dtValue = digitalRead(DT);
    int direction = (dtValue != newClk) ? 1 : -1;
    handleRotation(direction);
    menuRedrawRequired = true; 
  }
  lastClk = newClk;
  if (digitalRead(SW) == LOW) {
    if (millis() - lastButtonPress > debounceDelay) {
      handleButton(); 
      lastButtonPress = millis();
      menuRedrawRequired = true;
    }
  }
}

void handleRotation(int dir) { 
  bool T1 = (currentState==RUNNING);
  bool T2 = (currentState==MONITOR);
  if(T1||T2||currentState==STOPPING||currentState==CALIB_MOT_MANUAL){
    return;
  }  
  switch (currentState) { 
    case MAIN_MENU: 
      currentSelection += dir; 
      if (currentSelection < 0) currentSelection = mainMenuSize - 1; 
      if (currentSelection > mainMenuSize - 1) currentSelection = 0; 
      break; 
    case CONFIG_MENU: 
      configSelection += dir; 
      if (configSelection < 0) configSelection = 5; 
      if (configSelection > 5) configSelection = 0; 
      break; 
    case ADV_CONFIG_MENU: 
      advConfigSelection += dir; 
      if (advConfigSelection < 0) advConfigSelection = 4; 
      if (advConfigSelection > 4) advConfigSelection = 0; 
      break;
    case CALIB_MENU: 
      calibSelection += dir; 
      if (calibSelection < 0) calibSelection = 4; 
      if (calibSelection > 4) calibSelection = 0; 
      break;
    case CALIB_MOT_MENU: 
      calibMotSelection += dir; 
      if(calibMotSelection < 0) calibMotSelection = 2; 
      if(calibMotSelection > 2) calibMotSelection = 0; 
      break;
    
    // --- ROTATION LOGIC FOR EDIT PPR MENU ---
    case PPR_EDIT_MENU: 
      pprSelection += dir; 
      if(pprSelection < 0) pprSelection = 2; 
      if(pprSelection > 2) pprSelection = 0; 
      break;
    case PPR_EDIT_VAL: 
      modifyPPRVal(dir); 
      break;
        
    case EDIT_PARAM: 
      modifyParam(dir); 
      break; 
    case EDIT_ADV_PARAM: 
      modifyAdvParam(dir); 
      break; 
    case EDIT_CALIB_PARAM: 
      modifyCalibParam(dir); 
      break; 
    case TEST_ACT: 
      testSelection += dir; 
      if (testSelection < 0) testSelection = testMenuSize - 1; 
      if (testSelection > testMenuSize - 1) testSelection = 0; 
      break; 
    case TEST_EDIT: 
      modifyTestParam(dir); 
      break; 
  } 
}

void handleButton() { 
  triggerSound(SND_CLICK); 
  switch (currentState) { 
    case MAIN_MENU: 
      executeMainMenuAction(); 
      break; 
    case CONFIG_MENU: 
      if (configSelection == 5) currentState = MAIN_MENU; 
      else if (configSelection == 4) currentState = ADV_CONFIG_MENU; 
      else currentState = EDIT_PARAM; 
      break; 
    case ADV_CONFIG_MENU: 
      if (advConfigSelection == 4) currentState = CONFIG_MENU;     
      else if (advConfigSelection == 3) currentState = CALIB_MENU; 
      else currentState = EDIT_ADV_PARAM;                          
      break;
    case CALIB_MENU:
      if (calibSelection == 4) { currentState = ADV_CONFIG_MENU; } 
      else currentState = EDIT_CALIB_PARAM;
      break;
    case CALIB_MOT_MENU:
        if (calibMotSelection == 2) currentState = ADV_CONFIG_MENU; 
        else { 
          motorToCalib = calibMotSelection; 
          pulseCount1 = 0; 
          pulseCount2 = 0; 
          calibMotPulses = 0; 
          currentState = CALIB_MOT_MANUAL; 
        }
        break;
    case CALIB_MOT_MANUAL:
        stopAllActuators();
        long totalPulses; 
        if(motorToCalib == 0) totalPulses = pulseCount1; 
        else totalPulses = pulseCount2;
        calibMotPulses = totalPulses;
        if (totalPulses > 10) { 
          if(motorToCalib == 0) currentPPR_M1 = (float)totalPulses; 
          else currentPPR_M2 = (float)totalPulses; 
        }
        currentState = CALIB_MOT_RESULT;
        break;
    case CALIB_MOT_RESULT: currentState = CALIB_MOT_MENU; break;
    
    // --- BUTTONS FOR PPR MENU ---
    case PPR_EDIT_MENU:
        if (pprSelection == 2) currentState = ADV_CONFIG_MENU; // Exit
        else currentState = PPR_EDIT_VAL; // Enter edit value
        break;
    case PPR_EDIT_VAL:
        currentState = PPR_EDIT_MENU; // Confirm and return
        break;
        
    case EDIT_CALIB_PARAM: 
      currentState = CALIB_MENU; 
      break;
    case EDIT_PARAM: 
      currentState = CONFIG_MENU; 
      break; 
    case EDIT_ADV_PARAM: 
      currentState = ADV_CONFIG_MENU; 
      break; 
    case TEST_ACT: 
      if (testSelection < 5) currentState = TEST_EDIT; 
      else if (testSelection == 5) { 
        testRunning = !testRunning; 
        if (!testRunning) stopAllActuators(); } 
      else if (testSelection == 6) { 
        stopAllActuators(); 
        testRunning = false; 
        currentState = MAIN_MENU; 
      } 
      break; 
    case TEST_EDIT: 
      currentState = TEST_ACT; 
      break; 
    case RUNNING: 
      if (currentSound == SND_ALARM) { 
        pcbAlarmMuted = true; 
        triggerSound(SND_NONE); 
      } 
      else { 
        currentState = STOPPING; 
        lcd.clear(); 
        lcd.setCursor(3, 1); 
        lcdPrint("DETENIENDO..."); 
      }
      break; 
    case MONITOR: 
      currentState = MAIN_MENU; 
      break; 
  } 
}
void executeMainMenuAction() { 
  switch (currentSelection) { 
    case 0: // START PROCESS
      lcd.clear(); 
      currentState = RUNNING; 
      processStartTime = millis(); 
      lastScreenUpdate = 0; 
      
      // RESET PID
      pidIntegral = 0; 
      pidLastError = 0;
      
      hasReachedTarget = false; 
      initialProcessTemp = getControlTemperature(); 
      
      triggerSound(SND_START); 
      break;
      
    case 1: // CONFIGURATION
      currentState = CONFIG_MENU; 
      configSelection = 0; 
      break; 
      
    case 2: // MONITOR
      lcd.clear(); 
      currentState = MONITOR; 
      break; 
      
    case 3: // ACTUATOR TEST
      currentState = TEST_ACT; 
      testSelection = 0; 
      testSpeed=0; testHeat=0; testFan1=0; testFan2=0; 
      testRunning=false; 
      stopAllActuators(); 
      break; 
      
    case 4: // SHUT DOWN
      currentState = SHUTDOWN; 
      stopAllActuators(); 
      break; 
  } 
}
void modifyParam(int dir) { 
  switch (configSelection) { 
    case 0: 
        params.agitation = (params.agitation == MAGNETICA) ? ROTATORIA : MAGNETICA; 
        if(params.agitation == ROTATORIA && params.agitationSpeed > 15){
          params.agitationSpeed = 15;
        }
        if(params.agitation == MAGNETICA && params.agitationSpeed < 40){
          params.agitationSpeed = 40;
        } 
        break; 
        
    case 1: 
        params.targetTemp = constrain(params.targetTemp + dir, 40, 100); 
        break; 
        
    case 2: 
        if (params.agitation == MAGNETICA) { 
            params.agitationSpeed = constrain(params.agitationSpeed+(dir*10),40,160); 
        } else { 
            params.agitationSpeed = constrain(params.agitationSpeed+(dir*1),4,15); 
        } 
        break;
    case 3: params.operationTime = max(1, params.operationTime + dir); break; 
  } 
}

void modifyAdvParam(int dir) { 
  switch (advConfigSelection) { 
    
    case 0: 
      {int mode = (int)params.buzzerMode + dir; 
      if (mode > 2) mode = 0; 
      if (mode < 0) mode = 2; 
      params.buzzerMode = (BuzzerMode)mode; 
      break;} 
    case 1: 
      params.pcbMaxTemp = constrain(params.pcbMaxTemp + dir, 30, 80); 
      break; 
    case 2: 
      params.tempUnit = (params.tempUnit == UNIT_C) ? UNIT_F : UNIT_C; 
      break; 
  } 
}

void modifyCalibParam(int dir) {
  switch (calibSelection) {
    case 0: 
      params.pt100Offset += (dir * 0.1); 
      if (params.pt100Offset > 15.0) params.pt100Offset = 15.0; 
      if (params.pt100Offset < -15.0) params.pt100Offset = -15.0; 
      break;
    case 1: 
      params.lm35_1_Offset += (dir * 0.1); 
      if (params.lm35_1_Offset > 15.0) params.lm35_1_Offset = 15.0; 
      if (params.lm35_1_Offset < -15.0) params.lm35_1_Offset = -15.0; 
      break;
    case 2: params.lm35_2_Offset += (dir * 0.1); 
      if (params.lm35_2_Offset > 15.0) params.lm35_2_Offset = 15.0; 
      if (params.lm35_2_Offset < -15.0) params.lm35_2_Offset = -15.0; 
      break;
    case 3: params.lm35_pcb_Offset += (dir * 0.1); 
      if (params.lm35_pcb_Offset > 15.0) params.lm35_pcb_Offset = 15.0; 
      if (params.lm35_pcb_Offset < -15.0) params.lm35_pcb_Offset = -15.0; 
      break;
  }
}

void modifyTestParam(int dir) { 
  switch (testSelection) { 
    case 0: testMotor = (testMotor == MAGNETICA) ? ROTATORIA : MAGNETICA; break; 
    case 1: testSpeed = constrain(testSpeed + (dir * 5), 0, 100); break; 
    case 2: testHeat  = constrain(testHeat + (dir * 5), 0, 100); break; 
    case 3: testFan1  = constrain(testFan1 + (dir * 5), 0, 100); break; 
    case 4: testFan2  = constrain(testFan2 + (dir * 5), 0, 100); break; 
  } 
}

// --- MODIFY PPR MANUALLY ---
void modifyPPRVal(int dir) {
    if(pprSelection == 0) { 
        currentPPR_M1 += (dir * 10); 
        if(currentPPR_M1 < 10) currentPPR_M1 = 10; 
    } 
    else { 
        currentPPR_M2 += (dir * 10); 
        if(currentPPR_M2 < 10) currentPPR_M2 = 10; 
    }
}

void lcdPrint(const char* text) { lcd.print(text); }

void drawInterface() {
  if(currentState != RUNNING && currentState != MONITOR) lcd.clear(); 
  switch (currentState) {
    case MAIN_MENU: 
      drawListMenu("MENU PRINCIPAL", mainMenuItems, mainMenuSize, currentSelection); 
      break;
    case CONFIG_MENU: 
    case EDIT_PARAM: 
      drawConfigMenu(); 
      break;
    case ADV_CONFIG_MENU: 
    case EDIT_ADV_PARAM: 
      drawAdvConfigMenu(); 
      break;
    case CALIB_MENU: 
    case EDIT_CALIB_PARAM: 
      drawCalibMenu(); 
      break; 
    case CALIB_MOT_MENU: 
      drawMotorCalibMenu(); 
      break;
    case CALIB_MOT_MANUAL: 
      drawMotorCalibManual(); 
      break;
    case CALIB_MOT_RESULT: 
      drawMotorCalibResult(); 
      break;
    case PPR_EDIT_MENU: 
    case PPR_EDIT_VAL: 
      drawPPREditMenu(); 
      break;
    case TEST_ACT: 
    case TEST_EDIT: 
      drawTestMenu(); 
      break;
    case STOPPING: 
      lcd.setCursor(3, 1); 
      lcdPrint("DETENIENDO..."); 
      lcd.setCursor(5, 2); 
      lcdPrint("SUAVE"); 
      break;
    case RUNNING: 
      lcd.setCursor(2, 0); lcdPrint(">> EXTRACCION <<"); 
      lcd.setCursor(0, 1); lcdPrint("Temperatura: ...."); 
      lcd.setCursor(0, 2); lcdPrint("T. restante: ...."); 
      
      break;
    case MONITOR: 
      lcd.setCursor(1, 0); 
      lcdPrint("ESTADO DE SENSORES"); 
      lcd.setCursor(0, 1); 
      lcdPrint("Muestra :"); 
      lcd.setCursor(0, 2); 
      lcdPrint("Camara  :"); 
      lcd.setCursor(0, 3); 
      lcdPrint("PCB     :"); 
      lcd.setCursor(13, 3); 
      lcdPrint("[SALIR]"); 
      lastScreenUpdate = 0; 
      break;
    case SHUTDOWN: 
      lcd.setCursor(4, 1); 
      lcdPrint("APAGANDO..."); 
      break;
  }
}
void drawListMenu(const char* title, const char* items[], int size, int selection) {
  lcd.setCursor(1, 0); 
  lcdPrint(title); 
  int startIdx = 0; 
  if (selection > 1) startIdx = selection - 1; 
  if (startIdx > size - 3) startIdx = size - 3; 
  if (startIdx < 0) startIdx = 0;
  for (int i = 0; i < 3; i++) { 
    int itemIdx = startIdx + i; 
    if (itemIdx >= size) break; 
    lcd.setCursor(0, i + 1); 
    if (itemIdx == selection) lcd.print(">"); 
    else lcd.print(" "); 
    lcdPrint(items[itemIdx]); }
}
void drawConfigMenu() {
  lcd.setCursor(2, 0); 
  lcdPrint("CONFIG BASICA"); 
  int startIdx = 0; 
  if (configSelection > 1) startIdx = configSelection - 1; 
  if (startIdx > 6 - 3) startIdx = 6 - 3; 
  if (startIdx < 0) startIdx = 0;

  for (int i = 0; i < 3; i++) { 
    int itemIdx = startIdx + i; 
    
    if (itemIdx >= 6) break; 
    lcd.setCursor(0, i + 1); 
    
    if (itemIdx == configSelection) lcd.print(">"); 
    else lcd.print(" ");

    if (itemIdx == 4 || itemIdx == 5) { lcdPrint(configLabels[itemIdx]); } 
    else { 
      lcdPrint(configLabels[itemIdx]); 
      lcd.print(" "); 
      bool editingThis = (currentState == EDIT_PARAM && itemIdx == configSelection); 
      if (editingThis) lcd.print("[");

      switch(itemIdx) { 
        case 0: 
          lcdPrint(params.agitation == MAGNETICA ? "MAG" : "ROT"); 
          break; 
        case 1: 
          lcd.print(convertTemp(params.targetTemp)); 
          lcd.print(params.tempUnit == UNIT_F ? "F" : "C"); 
          break; 
        case 2: 
          lcd.print(params.agitationSpeed); 
          lcd.print(" RPM"); 
          break; 
        case 3: 
          lcd.print(params.operationTime); 
          lcd.print("m"); 
          break; 
      }
      if (editingThis) lcd.print("]"); 
    } 
  }
}
void drawAdvConfigMenu() {
  lcd.setCursor(1, 0); lcdPrint("CONFIG AVANZADA"); 
  
  int size = advMenuSize; 
  int startIdx = 0; 
  
  // Scroll Logic
  if (advConfigSelection > 1) startIdx = advConfigSelection - 1; 
  if (startIdx > size - 3) startIdx = size - 3; 
  if (startIdx < 0) startIdx = 0;
  
  for (int i = 0; i < 3; i++) { 
    int itemIdx = startIdx + i; 
    if (itemIdx >= size) break; 
    
    lcd.setCursor(0, i + 1); 
    if (itemIdx == advConfigSelection) lcd.print(">"); else lcd.print(" ");
    
    // Items 3 and 4 are navigation (no editable values)
    if (itemIdx >= 3) { 
        lcdPrint(advConfigLabels[itemIdx]); 
    } 
    else { 
        // Items 0, 1, 2 are editable
        lcdPrint(advConfigLabels[itemIdx]); lcd.print(" "); 
        bool editingThis=(currentState==EDIT_ADV_PARAM&&itemIdx==advConfigSelection); 
        if (editingThis) lcd.print("[");
        
        switch(itemIdx) { 
          case 0: // Buzzer
            if(params.buzzerMode == BUZZ_OFF) lcdPrint("OFF"); 
            else if(params.buzzerMode == BUZZ_ON) lcdPrint("TODO"); 
            else lcdPrint("ALERTA"); 
            break; 
          case 1: // PCB Max
            lcd.print(convertTemp(params.pcbMaxTemp), 0); 
            lcd.print(params.tempUnit == UNIT_F ? "F" : "C"); 
            break; 
          case 2: // Unit
            lcdPrint(params.tempUnit == UNIT_C ? "CELSIUS" : "FAHREN"); 
            break; 
        } 
        if (editingThis) lcd.print("]"); 
    } 
  }
}
void drawCalibMenu() {
  if (currentState == CALIB_MENU) { 
    lcd.setCursor(1, 0); 
    lcdPrint("MENU CALIBRACION"); 
    int startIdx = 0; 

    if (calibSelection > 1) startIdx = calibSelection - 1; 
    if (startIdx > 5 - 3) startIdx = 5 - 3; 
    if (startIdx < 0) startIdx = 0; 

    for (int i = 0; i < 3; i++) { 
      int itemIdx = startIdx + i; 
      if (itemIdx >= 5) break; 
      lcd.setCursor(0, i + 1); 

      if (itemIdx == calibSelection) lcd.print(">"); 
      else lcd.print(" "); 
      lcdPrint(calibLabels[itemIdx]); 
    } 
  } 
  else { 
    lcd.setCursor(0, 0); 
    if(calibSelection == 0) lcdPrint("CALIBRANDO PT100"); 
    else if(calibSelection == 1) lcdPrint("CALIBRANDO AMB 1"); 
    else if(calibSelection == 2) lcdPrint("CALIBRANDO AMB 2"); 
    else if(calibSelection == 3) lcdPrint("CALIBRANDO PCB"); 

    lcd.setCursor(0, 1); 
    lcdPrint("Valor: "); 
    lcd.setCursor(0, 2); 
    lcdPrint("Offset:"); 
    }
}
void updateCalibValuesOnly() {
  if (currentState != EDIT_CALIB_PARAM) return;
  float currentVal = 0.0; float offsetVal = 0.0;
  
  if (calibSelection == 0) {
    currentVal = getRawPT100() + params.pt100Offset; 
    offsetVal = params.pt100Offset; 
  } else if (calibSelection == 1) { 
    currentVal = readLM35(PIN_LM35_1) + params.lm35_1_Offset; 
    offsetVal = params.lm35_1_Offset; 
  } else if (calibSelection == 2) { 
    currentVal = readLM35(PIN_LM35_2) + params.lm35_2_Offset; 
    offsetVal = params.lm35_2_Offset; 
  } else if (calibSelection == 3) { 
    currentVal = readLM35(PIN_LM35_PCB) + params.lm35_pcb_Offset; 
    offsetVal = params.lm35_pcb_Offset;
  }

  lcd.setCursor(7, 1); 
  lcd.print(convertTemp(currentVal), 1); 
  lcd.print(params.tempUnit==UNIT_F ? "F " : "C "); 
  lcd.print("   "); 
  lcd.setCursor(8, 2); 
  lcd.print("["); 
  if(offsetVal >= 0) lcd.print("+"); 
  lcd.print(offsetVal, 1); 
  lcd.print("]   ");
}
void drawTestMenu() {
  lcd.setCursor(1, 0); 
  lcdPrint("TEST COMPONENTES"); 
  int size = testMenuSize; 
  int startIdx = 0; 
  if (testSelection > 1) startIdx = testSelection - 1; 
  if (startIdx > size - 3) startIdx = size - 3; 
  if (startIdx < 0) startIdx = 0;
  for (int i = 0; i < 3; i++) { 
    int itemIdx = startIdx + i; 
    
    if (itemIdx >= size) break;
    lcd.setCursor(0, i + 1); 
    
    if (itemIdx == testSelection) lcd.print(">"); 
    else lcd.print(" "); 
    
    lcdPrint(testLabels[itemIdx]); 
    lcd.print(" "); 
    bool editingThis=(currentState==TEST_EDIT&&itemIdx==testSelection); 
    if (editingThis) lcd.print("[");
    switch(itemIdx) { 
      case 0: 
        lcdPrint(testMotor == MAGNETICA ? "MAG" : "ROT"); 
        break; 
      case 1: 
        lcd.print(testSpeed); 
        lcd.print("%"); 
        break; 
      case 2: 
        lcd.print(testHeat); 
        lcd.print("%"); 
        break; 
      case 3: 
        lcd.print(testFan1); 
        lcd.print("%"); 
        break; 
      case 4: 
        lcd.print(testFan2); 
        lcd.print("%"); 
        break; 
      case 5: 
        lcdPrint(testRunning ? "ON " : "OFF"); 
        break; 
      } 
      if (editingThis) lcd.print("]"); }
}
void drawMotorCalibMenu() { 
  lcd.setCursor(1, 0);
  lcdPrint("CALIBRAR MOTOR"); 
  lcd.setCursor(0, 1);
  if(calibMotSelection == 0) lcd.print(">");
  else lcd.print(" "); lcdPrint("M1: ROTATORIO"); 
  lcd.setCursor(0, 2); 
  if(calibMotSelection == 1) lcd.print(">");
  else lcd.print(" "); lcdPrint("M2: MAGNETICO"); 
  
  lcd.setCursor(0, 3); 
  if(calibMotSelection == 2) lcd.print(">"); 
  else lcd.print(" "); lcdPrint("[ SALIR ]   "); 
}
void drawMotorCalibManual() { 
  lcd.setCursor(0, 0); 
  lcdPrint("GIRE 1 VUELTA"); 
  lcd.setCursor(0, 1); 
  lcdPrint("Contador: "); 
  long current = (motorToCalib == 0) ? pulseCount1 : pulseCount2; 
  lcd.print(current); 
  lcd.setCursor(0, 3); 
  lcdPrint("[Click para Guardar]"); 
}

void drawMotorCalibResult() { 
  lcd.setCursor(0, 0); 
  lcdPrint("NUEVO VALOR PPR:"); 
  lcd.setCursor(0, 1); 
  lcd.print((long)calibMotPulses); 
  lcd.setCursor(0, 3); 
  lcdPrint("[ GUARDADO OK ]"); 
}


void drawPPREditMenu() {
    lcd.setCursor(0, 0); 
    lcdPrint("EDITAR PPR MOTOR");
    lcd.setCursor(0, 1); 
    if(pprSelection==0) lcd.print(">"); 
    else lcd.print(" "); lcdPrint("M1: "); lcd.print((int)currentPPR_M1);
    if(pprSelection==0 && currentState==PPR_EDIT_VAL) lcd.print(" <");
    
    lcd.setCursor(0, 2); 
    if(pprSelection==1) lcd.print(">"); 
    else lcd.print(" "); lcdPrint("M2: "); lcd.print((int)currentPPR_M2);
    if(pprSelection==1 && currentState==PPR_EDIT_VAL) lcd.print(" <");
    
    lcd.setCursor(0, 3); 
    if(pprSelection==2) lcd.print(">"); 
    else lcd.print(" "); lcdPrint("[ SALIR ]   ");
}

void runCalibrationLogic() {
    stopAllActuators();
    static long lastDisp = -1;
    long current = (motorToCalib == 0) ? pulseCount1 : pulseCount2;
    if (current != lastDisp) { 
      lcd.setCursor(10, 1); 
      lcd.print(current); 
      lcd.print("   "); 
      lastDisp = current; 
    }
}
