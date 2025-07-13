#include <Servo.h>


const uint8_t SERVO_PIN_1 = 9;
const uint8_t SERVO_PIN_2 = 7;
const uint8_t SERVO_PIN_3 = 6;

Servo servo1, servo2, servo3;


void setSpherical() {       
  servo1.write( 90);
  servo2.write( 90);
  servo3.write( 90);
}
void setRectangle() {        
  servo1.write( 145); 
  servo2.write(30); 
  servo3.write(90); 
}
void setSquare() {          
  servo1.write(60);
  servo2.write( 120);
  servo3.write( 90);
}

void setup() {
  Serial.begin(115200);
  servo1.attach(SERVO_PIN_1);
  servo2.attach(SERVO_PIN_2);
  servo3.attach(SERVO_PIN_3);

  setSpherical();           
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'S': setSpherical(); break;
      case 'R': setRectangle(); break;
      case 'Q': setSquare();    break;
      default:                  break;
    }
  }
}
