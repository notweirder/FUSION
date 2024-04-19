bool gbSenserConnectState = false;
int threshold = 16700;
int mood = 0;
uint8_t sad = 0;

bool hasBeenFlippedOnce = false;
bool hasBeenRotated = false;
bool hasBeenLifted = false;
bool isDark = false;
bool hasSwitchedMood = false;
bool isAlone = false;
bool isLoud = false;

long micTimer = 0;
long lightTimer = 0;

bool switch1 = false;



#include <Waveshare_10Dof-D.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include "Adafruit_Soundboard.h"
//#include "functionFearBytes.h"

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);


char *moodsChar[] = {
    "   Hello! ",
    " Oh..okay then",
    "Please let me be",
    "Stop doing this!",
    "I TOLD YOU STOP",
    "I am now fearful!"
  };

String moodsString[] = { "I am now uneasy!",
                         "I am now nervous!",
                         "I am now anxious!",
                         "I am now disturbed!",
                         "I am now fearful!" };

// Choose any two pins that can be used with SoftwareSerial to RX & TX
#define SFX_TX 9
#define SFX_RX 10

// Connect to the RST pin on the Sound Board
#define SFX_RST 8

// You can also monitor the ACT pin for when audio is playing!

// we'll be using software serial
SoftwareSerial ss = SoftwareSerial(SFX_TX, SFX_RX);

// pass the software serial to Adafruit_soundboard, the second
// argument is the debug port (not used really) and the third
// arg is the reset pin
Adafruit_Soundboard sfx = Adafruit_Soundboard(&ss, NULL, SFX_RST);
// can also try hardware serial with
// Adafruit_Soundboard sfx = Adafruit_Soundboard(&Serial1, NULL, SFX_RST);




void setup() {
  // pinMode (Analog_Input, INPUT);
  // pinMode (Digital_Input, INPUT);
  //bool bRet;
  IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
  //Used to configure the sensitivity to the 8g range (not currently using it due to how)
  //I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_ACCEL_CONFIG, REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_8g | REG_VAL_BIT_ACCEL_DLPF);

  Serial.begin(115200);
  ss.begin(9600);

  lcd.begin(16, 2);

  // imuInit(&enMotionSensorType, &enPressureType);
  // if (IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType) {
  //   Serial.println("Motion sersor is ICM-20948");
  // } else {
  //   Serial.println("Motion sersor NULL");
  // }
  // if (IMU_EN_SENSOR_TYPE_BMP280 == enPressureType) {
  //   Serial.println("Pressure sersor is BMP280");
  // } else {
  //   Serial.println("Pressure sersor NULL");
  // }
  delay(200);
  Serial.print("\nPlaying track #");
  Serial.println(sad);
  if (!sfx.playTrack((uint8_t)sad)) {
    Serial.println("Failed to play track?");
  }

  //Serial.println(mouthLList[0]);
}

void loop() {

   ldrDetection();
   micDetection();
   voiceOutput(mood);
   lcd.setCursor(0,0);
   lcd.print(moodsChar[mood]);
  delay(500);
}





void ldrDetection() {
  int ldrSensorValue = digitalRead(A0);
  //Serial.print("LDR Sensor Value: ");
 // Serial.println(ldrSensorValue);
  // LDR Sensor needs to be adjusted too be less sensitive to light
  // Arbitrary value set for the if statement
  // Most I could get out of the reading was 20 - 28 with current config

  if (ldrSensorValue == HIGH) {
    isDark = true;
    Serial.println("It's too dark!");
  } else {
    lightTimer = millis();
  }
  while (isDark == true) {
    //Serial.print("light Timer = ");
    //Serial.println(lightTimer);
    if (millis() - lightTimer > 5000) {
      Serial.println("IN DARK FOR TOO LONG");
      isDark = false;
      lightTimer = millis();
      if (mood < 5) {
        mood++;
        hasSwitchedMood = false;
      }
    } else {
      isDark = false;
      break;
    }
  }
}
void micDetection() {
  

    int micValue = digitalRead(A5);
    Serial.print("Mic Value: ");
    Serial.println(micValue);
   
   
    if (micValue == LOW )
{
  if (!switch1)
  {
    micTimer = millis();
    switch1 = true;
  }
}

if (millis() - micTimer > 5000 &&  micValue == LOW)
  {
    Serial.println("TOO LOUD");

    while (micValue == LOW)
    {
      micValue = digitalRead(A5);
      Serial.println("Waiting for quiet");
            Serial.println(micValue);
      if (micValue == HIGH)
      {
        break;
      }
    }
  }


if (millis() - micTimer > 5000 && micValue == HIGH ) {

micTimer = 0;
switch1 = false;

}
Serial.print("Mic Timer");

Serial.println(millis() - micTimer);



 }


// void lcdTextDisplay() {
 
//   // if (millis() - next > 500) {
//   //   next += 500;
// 	lcd.setCursor(0, 1);
//    lcd.print(moodsChar[mood]);
  
// // }
// }
void voiceOutput(int num) {
  uint8_t *moodsVoice[] = { 4,
                            3,
                            2,
                            1,
                            0 };
  if (!hasSwitchedMood) {
    Serial.print("\nPlaying track #");
    Serial.println(num);
    if (!sfx.playTrack((uint8_t)moodsVoice[num])) {
      Serial.println("Failed to play track?");
    }
    hasSwitchedMood = true;
  }
}

