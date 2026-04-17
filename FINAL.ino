#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <avr/interrupt.h> 
#include <stdio.h> 
#include <stdlib.h> // Cho hàm atoi

int getFreeRam() {
  extern int __heap_start, *__brkval;
  int heap_top = (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
  
  // RAMEND là địa chỉ vật lý cuối cùng của RAM Uno (0x08FF)
  // Trừ đi đỉnh của Heap, và trừ hao thêm ~100 bytes cho System Interrupt Stack
  int free_ram = (int)RAMEND - heap_top - 100; 
  
  return (free_ram < 0) ? 0 : free_ram;
}

// =========================
// PIN CONFIG & MACROS 
// =========================
#define TRIG_PIN      9
#define ECHO_PIN      2  
#define BUZZER_PIN    8

#define BTN_UP        3  
#define BTN_DOWN      4  
#define BTN_MODE      5  

#define RED_LED_PIN   6
#define GREEN_LED_PIN 7

// MACRO Truy cập thanh ghi trực tiếp 
#define READ_ECHO()   (PIND & (1 << PIND2))    
#define TRIG_HIGH()   (PORTB |= (1 << PORTB1)) 
#define TRIG_LOW()    (PORTB &= ~(1 << PORTB1))

// =========================
// LCD & CONFIG
// =========================
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int EEPROM_ADDR_THRESHOLD = 0;

const uint16_t MIN_THRESHOLD = 2;
const uint16_t MAX_THRESHOLD = 400;
const uint16_t DEFAULT_THRESHOLD = 10;

const TickType_t DEBOUNCE_TICKS = pdMS_TO_TICKS(30);
const TickType_t LONG_PRESS_TICKS = pdMS_TO_TICKS(2000);
const TickType_t REPEAT_TICKS = pdMS_TO_TICKS(200);
const TickType_t EEPROM_SAVE_DELAY_TICKS = pdMS_TO_TICKS(1500);

// =========================
// STRUCTURES & SHARED STATE
// =========================
enum Screen { SCREEN_DISTANCE = 0, SCREEN_STATUS = 1, SCREEN_STATS = 2 };

// Đóng gói Shared State 
struct SystemState {
  uint16_t currentDistance; // Đã lọc (EMA)
  uint16_t rawDistance;     // Giá trị gốc chưa lọc
  uint16_t minDistance;
  uint16_t maxDistance;
  uint16_t alarmThreshold;
  Screen currentScreen;
  bool inSettingMode;
  bool manualLed;           // Bật đèn thủ công từ UI
  bool manualBuzzer;        // Bật còi thủ công từ UI
  
  // Nạp chồng toán tử == để update LCD
  bool operator!=(const SystemState &other) const {
    return currentDistance != other.currentDistance ||
           minDistance != other.minDistance ||
           maxDistance != other.maxDistance ||
           alarmThreshold != other.alarmThreshold ||
           currentScreen != other.currentScreen ||
           inSettingMode != other.inSettingMode ||
           manualLed != other.manualLed ||
           manualBuzzer != other.manualBuzzer;
  }
};

SystemState sysState = {0, 0, 999, 0, DEFAULT_THRESHOLD, SCREEN_DISTANCE, false, false, false};

// Biến quản lý EEPROM
volatile bool thresholdDirty = false;
volatile bool forceEepromSave = false; 
volatile TickType_t lastThresholdChangeTick = 0;

// =========================
// ISR VARIABLES & TASK HANDLES
// =========================
volatile uint32_t echoStartTime = 0;
volatile uint32_t echoEndTime = 0;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t uiTaskHandle = NULL;     
TaskHandle_t uartTaskHandle = NULL; 

// =========================
// BUTTON STRUCT
// =========================
struct Button {
  uint8_t pin; 
  bool lastReading; 
  bool stableState;
  bool longPressHandled;
  TickType_t lastDebounceTime; 
  TickType_t pressStartTime;
};

Button btnMode = {BTN_MODE, HIGH, HIGH, false, 0, 0};
Button btnUp   = {BTN_UP,   HIGH, HIGH, false, 0, 0};
Button btnDown = {BTN_DOWN, HIGH, HIGH, false, 0, 0};
TickType_t lastRepeatUp = 0;
TickType_t lastRepeatDown = 0;

// =========================
// FUNCTION DECLARATIONS
// =========================
void TaskSensorAndAlert(void *pvParameters);
void Task_UI(void *pvParameters); 
void Task_UART_Comms(void *pvParameters); 
void triggerSensor();
void printPadded(uint16_t val, uint8_t width);
void handleButton(Button &btn, uint8_t btnId, TickType_t currentTick);
void handleHeldRepeat(TickType_t currentTick);

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(9600);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();
  lcd.print(F("Booting RTOS..."));
  
  Serial.println(F("{\"log\":\"SYSTEM_BOOT\"}"));

  // Đọc EEPROM
  uint16_t loadedThreshold = 0;
  EEPROM.get(EEPROM_ADDR_THRESHOLD, loadedThreshold);
  if (loadedThreshold >= MIN_THRESHOLD && loadedThreshold <= MAX_THRESHOLD) {
    sysState.alarmThreshold = loadedThreshold;
  }

  // Khởi tạo Tasks
  xTaskCreate(TaskSensorAndAlert, "SensAlrt", 160, NULL, 2, &sensorTaskHandle); 
  xTaskCreate(Task_UI, "UI_Task",  180, NULL, 1, &uiTaskHandle);
  xTaskCreate(Task_UART_Comms, "UART_Task", 180, NULL, 1, &uartTaskHandle); 

  // Cấu hình Ngắt
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoInterrupt, CHANGE);
  PCICR |= (1 << PCIE2); 
  PCMSK2 |= (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21); 
}

void loop() {}

// =========================
// ISRs
// =========================
void echoInterrupt() {
  BaseType_t xWoken = pdFALSE;
  if (READ_ECHO()) { 
    echoStartTime = micros();
  } else {
    echoEndTime = micros();   
    if (sensorTaskHandle) vTaskNotifyGiveFromISR(sensorTaskHandle, &xWoken);
  }
  if (xWoken) portYIELD_FROM_ISR();
}

ISR(PCINT2_vect) {
  BaseType_t xWoken = pdFALSE;
  if (uiTaskHandle) vTaskNotifyGiveFromISR(uiTaskHandle, &xWoken);
  if (xWoken) portYIELD_FROM_ISR();
}

// =========================
// TASK 1: SENSOR & ALERT
// =========================
void TaskSensorAndAlert(void *pvParameters) {
  TickType_t lastToggleTick = xTaskGetTickCount();
  bool toggleState = false;
  uint16_t filteredDist = 0; 

  for (;;) {
    triggerSensor();
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
    uint16_t dist = 400; 
    uint16_t rawDist = 400;
    
    if (ulNotificationValue > 0) {
      uint32_t startT, endT;
      taskENTER_CRITICAL();
      startT = echoStartTime;
      endT = echoEndTime;
      taskEXIT_CRITICAL();

      if (endT > startT) {
        uint32_t duration = endT - startT;
        if (duration <= 30000) { 
          rawDist = duration / 58;
          if (rawDist < 2) rawDist = 2;
          if (rawDist > 400) rawDist = 400;

          // Bộ lọc EMA 
          if (filteredDist == 0) filteredDist = rawDist;
          else filteredDist = (filteredDist * 3 + rawDist) / 4;
          
          dist = filteredDist;
        }
      }
    }

    // Cập nhật State 
    taskENTER_CRITICAL();
    sysState.rawDistance = rawDist;
    sysState.currentDistance = dist;
    if (dist < sysState.minDistance) sysState.minDistance = dist;
    if (dist > sysState.maxDistance) sysState.maxDistance = dist;

    bool settingMode = sysState.inSettingMode;
    uint16_t thr = sysState.alarmThreshold;
    bool manLed = sysState.manualLed;
    bool manBuzzer = sysState.manualBuzzer;
    taskEXIT_CRITICAL();

    // --- Logic Cảnh báo & Điều khiển thủ công ---
    digitalWrite(GREEN_LED_PIN, settingMode ? HIGH : LOW);

    if (settingMode) {
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      toggleState = false;
    } else {
      bool danger = (dist < thr);
      
      // Đèn Đỏ: Bật nếu nguy hiểm HOẶC bị bật thủ công từ UI
      if (danger || manLed) {
        if (manLed && !danger) {
           digitalWrite(RED_LED_PIN, HIGH); // Bật cứng nếu chỉ do thủ công
        } else {
           // Nhấp nháy nếu đang nguy hiểm
           TickType_t interval = pdMS_TO_TICKS(500);
           if (dist <= 5) interval = pdMS_TO_TICKS(100);
           else if (dist < (thr / 2)) interval = pdMS_TO_TICKS(250);

           TickType_t currentTick = xTaskGetTickCount();
           if ((currentTick - lastToggleTick) >= interval) {
             toggleState = !toggleState;
             digitalWrite(RED_LED_PIN, toggleState ? HIGH : LOW);
             lastToggleTick = currentTick;
           }
        }
      } else {
        digitalWrite(RED_LED_PIN, LOW);
        toggleState = false;
      }

      // Còi báo: Bật nếu nguy hiểm HOẶC bị bật thủ công từ UI
      if (danger || manBuzzer) {
        if (manBuzzer && !danger) {
           digitalWrite(BUZZER_PIN, HIGH); 
        } else {
           digitalWrite(BUZZER_PIN, toggleState ? HIGH : LOW); // Đồng bộ với đèn nhấp nháy
        }
      } else {
        digitalWrite(BUZZER_PIN, LOW);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void triggerSensor() {
  TRIG_LOW();  delayMicroseconds(2);
  TRIG_HIGH(); delayMicroseconds(10); 
  TRIG_LOW();
}

// =========================
// TASK 2: UI (LCD & EEPROM)
// =========================
void Task_UI(void *pvParameters) {
  SystemState lastDrawnState;
  lastDrawnState.alarmThreshold = 999;
  
  for (;;) {
    TickType_t currentTick = xTaskGetTickCount();

    handleButton(btnMode, 1, currentTick);
    handleButton(btnUp,   2, currentTick);
    handleButton(btnDown, 3, currentTick);
    handleHeldRepeat(currentTick);

    SystemState localState;
    taskENTER_CRITICAL();
    localState = sysState;
    bool dirty = thresholdDirty;
    bool forceSave = forceEepromSave;
    TickType_t changedAt = lastThresholdChangeTick;
    taskEXIT_CRITICAL();

    // Logic EEPROM có gửi Log báo cáo lên UI
    if (dirty) {
      if (forceSave || (currentTick - changedAt >= EEPROM_SAVE_DELAY_TICKS)) {
        EEPROM.put(EEPROM_ADDR_THRESHOLD, localState.alarmThreshold);
        taskENTER_CRITICAL();
        thresholdDirty = false;
        forceEepromSave = false;
        taskEXIT_CRITICAL();
        
        Serial.println(F("{\"log\":\"EEPROM_SAVED\"}"));
      }
    }

    if (localState != lastDrawnState) { 
      lastDrawnState = localState;
      if (localState.inSettingMode) {
        lcd.setCursor(0, 0); lcd.print(F("SETTING MODE    "));
        lcd.setCursor(0, 1); lcd.print(F("Thr:")); printPadded(localState.alarmThreshold, 3); lcd.print(F("cm       "));
      } else {
        switch (localState.currentScreen) {
          case SCREEN_DISTANCE:
            lcd.setCursor(0, 0); lcd.print(F("D:")); printPadded(localState.currentDistance, 3); lcd.print(F("cm         "));
            lcd.setCursor(0, 1); lcd.print(F("Thr:")); printPadded(localState.alarmThreshold, 3); lcd.print(F("cm       "));
            break;
          case SCREEN_STATUS:
            lcd.setCursor(0, 0); lcd.print(localState.currentDistance < localState.alarmThreshold ? F("Status:WARN     ") : F("Status:SAFE     "));
            lcd.setCursor(0, 1); lcd.print(F("Thr:")); printPadded(localState.alarmThreshold, 3); 
            lcd.print(F(" D:")); printPadded(localState.currentDistance, 3); lcd.print(F("   "));
            break;
          case SCREEN_STATS:
            lcd.setCursor(0, 0); lcd.print(F("Min:")); printPadded(localState.minDistance, 3); 
            lcd.print(F(" M:")); printPadded(localState.maxDistance, 3); lcd.print(F("   "));
            lcd.setCursor(0, 1); lcd.print(F("Thr:")); printPadded(localState.alarmThreshold, 3); lcd.print(F(" HoldM   "));
            break;
        }
      }
    }

    TickType_t sleepTime = pdMS_TO_TICKS(200);
    if (btnMode.stableState == LOW || btnUp.stableState == LOW || btnDown.stableState == LOW ||
       (currentTick - btnMode.lastDebounceTime < pdMS_TO_TICKS(50)) || 
       (currentTick - btnUp.lastDebounceTime < pdMS_TO_TICKS(50)) || 
       (currentTick - btnDown.lastDebounceTime < pdMS_TO_TICKS(50))) {
        sleepTime = pdMS_TO_TICKS(20);
    }
    ulTaskNotifyTake(pdTRUE, sleepTime);
  }
}

void printPadded(uint16_t val, uint8_t width) {
  lcd.print(val);
  if(val < 10 && width >= 2) lcd.print(F(" "));
  if(val < 100 && width >= 3) lcd.print(F(" "));
}

// =========================
// BUTTON LOGIC ENGINE
// =========================
void handleButton(Button &btn, uint8_t btnId, TickType_t currentTick) {
  bool reading = digitalRead(btn.pin);
  if (reading != btn.lastReading) btn.lastDebounceTime = currentTick;

  if ((currentTick - btn.lastDebounceTime) > DEBOUNCE_TICKS) {
    if (reading != btn.stableState) {
      btn.stableState = reading;
      if (btn.stableState == LOW) {
        btn.pressStartTime = currentTick; 
        btn.longPressHandled = false;
      } else {
        if (!btn.longPressHandled && (currentTick - btn.pressStartTime) < LONG_PRESS_TICKS) {
          taskENTER_CRITICAL();
          if (btnId == 1) { // MODE
            if (!sysState.inSettingMode) sysState.currentScreen = (Screen)((sysState.currentScreen + 1) % 3);
          } else if (btnId == 2 && sysState.inSettingMode) { // UP
            if (sysState.alarmThreshold < MAX_THRESHOLD) sysState.alarmThreshold++;
            thresholdDirty = true; lastThresholdChangeTick = currentTick;
          } else if (btnId == 3 && sysState.inSettingMode) { // DOWN
            if (sysState.alarmThreshold > MIN_THRESHOLD) sysState.alarmThreshold--;
            thresholdDirty = true; lastThresholdChangeTick = currentTick;
          }
          taskEXIT_CRITICAL();
        }
      }
    }
  }
  
  if (btn.stableState == LOW && !btn.longPressHandled) {
    if (currentTick - btn.pressStartTime >= LONG_PRESS_TICKS) {
      btn.longPressHandled = true;
      taskENTER_CRITICAL();
      if (btnId == 1) {
        if (!sysState.inSettingMode && sysState.currentScreen == SCREEN_STATS) {
          sysState.minDistance = sysState.currentDistance;
          sysState.maxDistance = sysState.currentDistance;
        } else {
          sysState.inSettingMode = !sysState.inSettingMode;
          if (!sysState.inSettingMode && thresholdDirty) forceEepromSave = true;
        }
      }
      taskEXIT_CRITICAL();
    }
  }
  btn.lastReading = reading;
}

void handleHeldRepeat(TickType_t currentTick) {
  if (btnUp.stableState == LOW && btnUp.longPressHandled) {
    if (currentTick - lastRepeatUp >= REPEAT_TICKS) {
      lastRepeatUp = currentTick;
      taskENTER_CRITICAL();
      if (sysState.inSettingMode) {
        sysState.alarmThreshold += 5;
        if (sysState.alarmThreshold > MAX_THRESHOLD) sysState.alarmThreshold = MAX_THRESHOLD;
        thresholdDirty = true; lastThresholdChangeTick = currentTick;
      }
      taskEXIT_CRITICAL();
    }
  }
  if (btnDown.stableState == LOW && btnDown.longPressHandled) {
    if (currentTick - lastRepeatDown >= REPEAT_TICKS) {
      lastRepeatDown = currentTick;
      taskENTER_CRITICAL();
      if (sysState.inSettingMode) {
        if (sysState.alarmThreshold >= MIN_THRESHOLD + 5) sysState.alarmThreshold -= 5;
        else sysState.alarmThreshold = MIN_THRESHOLD;
        thresholdDirty = true; lastThresholdChangeTick = currentTick;
      }
      taskEXIT_CRITICAL();
    }
  }
}

// =========================
// TASK 3: UART COMMS (ESP32 <-> UNO)
// =========================
void Task_UART_Comms(void *pvParameters) {
  char rxBuffer[16]; 
  uint8_t rxIndex = 0;
  TickType_t lastTxTick = xTaskGetTickCount();

  for (;;) {
    // --- 1. NHẬN LỆNH TỪ ESP32 & GỬI ACK ---
    while (Serial.available() > 0) {
      char c = Serial.read();
      
      if (c == '\n' || c == '>') { 
        rxBuffer[rxIndex] = '\0';

        
        // --- Xử lý lệnh Threshold (T) ---
        if (rxBuffer[0] == 'T') {
          uint16_t oldThr = sysState.alarmThreshold;
          taskENTER_CRITICAL();
          if (rxBuffer[2] == '+') { // Đã lùi index từ 3 về 2
            if (sysState.alarmThreshold < MAX_THRESHOLD) sysState.alarmThreshold++;
          } else if (rxBuffer[2] == '-') {
            if (sysState.alarmThreshold > MIN_THRESHOLD) sysState.alarmThreshold--;
          } else {
            uint16_t val = atoi(&rxBuffer[2]);
            if (val >= MIN_THRESHOLD && val <= MAX_THRESHOLD) sysState.alarmThreshold = val;
          }
          thresholdDirty = true; forceEepromSave = true; lastThresholdChangeTick = xTaskGetTickCount();
          uint16_t newThr = sysState.alarmThreshold;
          taskEXIT_CRITICAL();
          
          // Gửi ACK Threshold
          Serial.print(F("{\"ack\":\"T\",\"old\":")); Serial.print(oldThr);
          Serial.print(F(",\"new\":")); Serial.print(newThr); Serial.println(F("}"));
        }
        
        // --- Xử lý lệnh Mode (M) ---
        else if (rxBuffer[0] == 'M') {
          taskENTER_CRITICAL();
          if (!sysState.inSettingMode) sysState.currentScreen = (Screen)((sysState.currentScreen + 1) % 3);
          taskEXIT_CRITICAL();
          Serial.println(F("{\"ack\":\"M\"}"));
        }
        
        // --- Xử lý lệnh Manual LED (L) ---
        else if (rxBuffer[0] == 'L') {
          taskENTER_CRITICAL();
          sysState.manualLed = (rxBuffer[2] == '1');
          taskEXIT_CRITICAL();
          Serial.print(F("{\"ack\":\"L\",\"st\":")); Serial.print(sysState.manualLed); Serial.println(F("}"));
        }
        
        // --- Xử lý lệnh Manual Buzzer (B) ---
        else if (rxBuffer[0] == 'B') {
          taskENTER_CRITICAL();
          sysState.manualBuzzer = (rxBuffer[2] == '1');
          taskEXIT_CRITICAL();
          Serial.print(F("{\"ack\":\"B\",\"st\":")); Serial.print(sysState.manualBuzzer); Serial.println(F("}"));
        }

        rxIndex = 0; 
      } else if (c == '<') {
        rxIndex = 0; 
      } else if (rxIndex < sizeof(rxBuffer) - 1) {
        rxBuffer[rxIndex++] = c; 
      }
    }

    // --- 2. GỬI TELEMETRY DATA (Khoảng 1 giây/lần) ---
    TickType_t currentTick = xTaskGetTickCount();
    if (currentTick - lastTxTick >= pdMS_TO_TICKS(1000)) {
      SystemState localState;
      
      taskENTER_CRITICAL();
      localState = sysState;
      taskEXIT_CRITICAL();

      // Đóng gói JSON kèm khoảng cách lọc (d) và gốc (r)
      char txString[80];
      int freeRam = getFreeRam();
      snprintf(txString, sizeof(txString), "{\"d\":%u,\"r\":%u,\"t\":%u,\"min\":%u,\"max\":%u,\"ram\":%u}", 
               localState.currentDistance, localState.rawDistance, localState.alarmThreshold,
               localState.minDistance, localState.maxDistance, freeRam);
      
      Serial.println(txString); 
      lastTxTick = currentTick;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
