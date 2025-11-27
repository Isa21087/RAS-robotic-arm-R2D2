#include <Arduino.h>
#include <Servo.h>

// --- 4servos ---
Servo servo1;   //Base
Servo servo2;   //Right servomotor
Servo servo3;   //Left servomotor
Servo servo4;   //Gripper

//Connection pins
const int pin1 = 7;
const int pin2 = 6;
const int pin3 = 5;
const int pin4 = 4;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);  

  //Attach servos to pins
  servo1.attach(pin1);
  servo2.attach(pin2);
  servo3.attach(pin3);
  servo4.attach(pin4);

  //Initial position
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(0);

  Serial.println("Arduino ready");
}

void loop() {

  //If a message arrived via Serial
  if (Serial.available()) {

    //Read a complete line up to '\n'
    String data = Serial.readStringUntil('\n');

    //Example: "90,45,120"
    Serial.print("Received: ");
    Serial.println(data);

    if (data.indexOf(',') != -1) 
    {
      //--- Separate the 3 values ​​---
      int firstComma  = data.indexOf(',');
      int secondComma = data.indexOf(',', firstComma + 1);

      // Extract each part as text
      String s1 = data.substring(0, firstComma);
      String s2 = data.substring(firstComma + 1, secondComma);
      String s3 = data.substring(secondComma + 1);

      // Convert to int
      int angle1 = s1.toInt();
      int angle2 = s2.toInt();
      int angle3 = s3.toInt();

      // Limit between 0 and 180 degrees
      angle1 = constrain(angle1, 0, 180);
      angle2 = constrain(angle2, 0, 180);
      angle3 = constrain(angle3, 0, 180);

      // Write to the servos
      servo1.write(angle1);
      servo2.write(angle2);
      servo3.write(angle3);

      // Mensagge
      Serial.print("Servos: ");
      Serial.print(angle1); Serial.print(", ");
      Serial.print(angle2); Serial.print(", ");
      Serial.println(angle3);
    }

    else{
      if (data == "open") {
        Serial.println("Gripper: Open");
        // Set the angle position to open the gripper
        servo4.write(50); 
      } else if (data == "close") {
        Serial.println("Gripper: Close");
        // Set the angle position to close the gripper
        servo4.write(0);
    }
  }
  delay(10);
  }
}
