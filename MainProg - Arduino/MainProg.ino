#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

Servo servo7;
Servo servo9;
LiquidCrystal_I2C lcd(0x27, 16, 2);

float points = 0; // Ensure points is a float for proper accumulation

void setup() {
  pinMode(8, OUTPUT);
  pinMode(11, INPUT);
  Serial.begin(9600);
  servo7.attach(6);
  servo9.attach(9);
  servo7.write(0);
  servo9.write(50);
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.print("Welcome ^^");
 }

void loop() {
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');


    if (message == "check_ready") {
      if (digitalRead(11) == HIGH) {
        Serial.println("ready");
      } else {
        Serial.println("not_ready");
      }
    } else if (message.startsWith("no_object")) {
        lcd.clear();
      lcd.print("No object detected");
      Serial.println("done");
    } else {
        lcd.clear();
       lcd.print("Processing");
      int commaIndex1 = message.indexOf(',');
      int commaIndex2 = message.indexOf(',', commaIndex1 + 1);
      String label = message.substring(0, commaIndex1);
      String sizeLabel = message.substring(commaIndex1 + 1, commaIndex2);
      float pointValue = message.substring(commaIndex2 + 1).toFloat();

      points += pointValue;
         lcd.clear();
      lcd.print("Ttl pts: ");
      lcd.print(points);
      lcd.setCursor(0, 1);
      lcd.print(label);
      lcd.print("   ");
      lcd.print(pointValue);

      if (label == "bottle") {  // Changed from "plastic_bottle"
        servo7.write(60);
      } else if(label == "can"){
        servo7.write(0);  // Set to appropriate angle if needed
      }

      delay(1000);
      servo9.write(180);
      delay(1000);
      servo9.write(50);
      delay(100);

      if (points >= 1) {
        points -= 1;
        digitalWrite(8, HIGH);
        delay(30);
        digitalWrite(8, LOW);
        delay(30);
           lcd.clear();
        lcd.print("Points: ");
        lcd.print(points);
      }

      Serial.println("done");
    }
    
  }
}
