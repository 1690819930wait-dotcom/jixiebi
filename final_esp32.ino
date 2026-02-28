#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define RXD1 13        
#define TXD1 14        
#define RXD2 16        
#define TXD2 17        
#define LDR_AO_PIN 34  
#define RGB_R 25
#define RGB_G 26
#define RGB_B 27
#define BTN_COLOR 4    
#define BTN_MODE 5     

Adafruit_SSD1306 display(128, 64, &Wire, -1);
MPU6050 mpu;

bool dmpReady = false;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3], yaw_offset = 0, pitch_offset = 0, roll_offset = 0;

// 初始角度：底座120, 前臂75, 后臂100
int cur_bA = 120, cur_pA = 75, cur_rA = 100;
int colorMode = 0; 
bool isManualMode = true; 
String lastVoiceCmd = "None";
unsigned long prevMillis = 0;
unsigned long colorBtnTime = 0;
bool lastColorBtnState = HIGH;
bool lastModeBtnState = HIGH;
unsigned long resetShowTime = 0; 

const char* colorNames[] = {"RED", "GREN", "BLUE", "YELW", "PURP", "CYAN", "WHTE"};

void performSystemReset() {
    // 【核心修复】：重新校准偏移量，使当前的物理方向对应底座120度，防止跳回90
    yaw_offset = (ypr[0] * 180 / M_PI); 
    pitch_offset = ypr[1] * 180 / M_PI;
    roll_offset = ypr[2] * 180 / M_PI;
    
    cur_bA = 120; cur_pA = 75; cur_rA = 100; 
    resetShowTime = millis(); 
    
    // 立即同步给 Arduino
    Serial2.printf("[%d,%d,%d,%d]\n", cur_bA, cur_pA, cur_rA, isManualMode ? 1 : 0);
    Serial2.println("[G,OPEN]"); 
}

void setup() {
    Serial.begin(115200); 
    Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);   
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); 
    Wire.begin(21, 22, 400000);
    pinMode(BTN_COLOR, INPUT_PULLUP);
    pinMode(BTN_MODE, INPUT_PULLUP);
    ledcAttach(RGB_R, 5000, 8); ledcAttach(RGB_G, 5000, 8); ledcAttach(RGB_B, 5000, 8); 

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("OLED Fail");
    mpu.initialize();
    if (mpu.dmpInitialize() == 0) {
        mpu.CalibrateAccel(6); mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        dmpReady = true;
    }
}

void loop() {
    if (!dmpReady) return;

    if (Serial1.available()) {
        String voiceCmd = Serial1.readStringUntil('\n'); 
        voiceCmd.trim(); 
        if (voiceCmd.length() > 0 && !voiceCmd.equals("awake")) {
            lastVoiceCmd = voiceCmd;
            if (voiceCmd.equals("RESET")) performSystemReset();
            else if (voiceCmd.equals("RS")) Serial2.println("[G,OPEN]"); 
            else if (voiceCmd.equals("RZ")) Serial2.println("[G,CLOSE]");  
            else if (voiceCmd.startsWith("C") && voiceCmd.length() <= 2) { 
                int m = voiceCmd.substring(1).toInt(); if(m >= 0 && m <= 6) colorMode = m; 
            } 
            else {
                String type; int val = 0;
                if (isDigit(voiceCmd.charAt(1))) { type = voiceCmd.substring(0, 1); val = voiceCmd.substring(1).toInt(); }
                else { type = voiceCmd.substring(0, 2); val = voiceCmd.substring(2).toInt(); }

                if (type == "L")      cur_bA = constrain(cur_bA + val, 0, 180); 
                else if (type == "R") cur_bA = constrain(cur_bA - val, 0, 180); 
                else if (type == "FU") cur_pA = constrain(cur_pA - val, 30, 120); 
                else if (type == "FD") cur_pA = constrain(cur_pA + val, 30, 120); 
                else if (type == "BU") cur_rA = constrain(cur_rA + val, 45, 155); 
                else if (type == "BD") cur_rA = constrain(cur_rA - val, 45, 155);
                Serial2.printf("[%d,%d,%d,%d]\n", cur_bA, cur_pA, cur_rA, isManualMode ? 1 : 0);
            }
        }
    }

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }

    unsigned long curM = millis();
    if (curM - prevMillis >= 50) {
        prevMillis = curM;
        int ldrVal = analogRead(LDR_AO_PIN); 
        int ldrPercent = map(ldrVal, 0, 4095, 0, 100); 
        int brightness = constrain(map(ldrVal, 0, 4095, 50, 255), 50, 255);
        int rVal=0, gVal=0, bVal=0;
        switch(colorMode) {
            case 0: rVal=brightness; break; case 1: gVal=brightness; break; case 2: bVal=brightness; break;
            case 3: rVal=brightness; gVal=brightness; break; case 4: rVal=brightness; bVal=brightness; break;
            case 5: gVal=brightness; bVal=brightness; break; case 6: rVal=brightness; gVal=brightness; bVal=brightness; break;
        }
        ledcWrite(RGB_R, rVal); ledcWrite(RGB_G, gVal); ledcWrite(RGB_B, bVal);

        display.clearDisplay();
        display.setTextColor(WHITE);
        display.setTextSize(1); display.setCursor(0, 0);
        display.print(isManualMode ? "MANU" : "VOIC");
        display.setCursor(32, 0); display.print(colorNames[colorMode]);
        display.setTextSize(2); display.setCursor(0, 15); 
        display.print(ldrPercent); display.print("%");
        display.setTextSize(1); display.setCursor(0, 32); display.print("BRIGHT");
        display.drawFastVLine(56, 0, 48, WHITE); 
        display.setCursor(60, 0);  display.printf("Y:%d", (int)((ypr[0]*180/M_PI)-yaw_offset));
        display.setCursor(60, 8);  display.printf("P:%d", (int)((ypr[1]*180/M_PI)-pitch_offset));
        display.setCursor(60, 16); display.printf("R:%d", (int)((ypr[2]*180/M_PI)-roll_offset));
        display.drawFastHLine(58, 25, 70, WHITE); 
        display.setCursor(58, 28); display.printf("B:%d/0-180", cur_bA);
        display.setCursor(58, 37); display.printf("F:%d/30-120", cur_pA);
        display.setCursor(58, 46); display.printf("R:%d/45-155", cur_rA);
        display.drawFastHLine(0, 54, 128, WHITE); 
        display.setCursor(0, 57); 
        if(millis() - resetShowTime < 2000) display.print("--- RESET OK ---"); 
        else { display.print("V:"); display.print(lastVoiceCmd); }
        display.display();

        if (isManualMode) {
            float fy = (ypr[0] * 180 / M_PI) - yaw_offset;
            float fp = (ypr[1] * 180 / M_PI) - pitch_offset;
            float fr = (ypr[2] * 180 / M_PI) - roll_offset;
            while (fy > 180) fy -= 360; while (fy < -180) fy += 360;
            
            // 【核心修复】：手动映射逻辑改为以 120 为中心，而不是 90
            cur_bA = constrain(map(fy, -90, 90, 210, 30), 0, 180); 
            cur_pA = constrain(map(fp, -30, 30, 30, 120), 30, 120); 
            cur_rA = constrain(map(fr, -30, 30, 45, 155), 45, 155); 
            Serial2.printf("[%d,%d,%d,1]\n", cur_bA, cur_pA, cur_rA);
        }
    }
    
    bool cmb = digitalRead(BTN_MODE);
    if (cmb == LOW && lastModeBtnState == HIGH) { delay(50); if (digitalRead(BTN_MODE) == LOW) isManualMode = !isManualMode; }
    lastModeBtnState = cmb;
    bool ccb = digitalRead(BTN_COLOR);
    if (ccb == LOW && lastColorBtnState == HIGH) colorBtnTime = millis();
    else if (ccb == HIGH && lastColorBtnState == LOW) {
        unsigned long dur = millis() - colorBtnTime;
        if (dur > 20 && dur < 1500) colorMode = (colorMode + 1) % 7;
        else if (dur >= 1500) performSystemReset();
    }
    lastColorBtnState = ccb;
}