#include <Arduino.h>

#define motor_left 5
#define dir_left 0
#define motor_right 4
#define dir_right 2

const float p = 400;

char cmd_buf[256];

void cmd_parse (char msg[256]){
  char cmd = msg[0];
  char args[256];
  for (size_t i = 0; i < 255; i++)
  {
    args[i] = msg[i+1];
  }

  switch (cmd)
  {
  case 'p':
      Serial.print("1p");
      Serial.print(args);
      Serial.print('\n');
    break;
  case 'm':
    Serial.print("1m");
    Serial.println(args);
    break;
  case 'b':
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("1b");
      break;
  case 'r':
      digitalWrite(dir_left, HIGH);
      digitalWrite(dir_right, LOW);
      analogWrite(motor_left, p);
      analogWrite(motor_right, p);
      Serial.println("r");
      break;
  case 'l':
      digitalWrite(dir_left, LOW);
      digitalWrite(dir_right, HIGH);
      analogWrite(motor_left, p);
      analogWrite(motor_right, p);
      Serial.println("l");
      break;
  case 'f':
      digitalWrite(dir_left, HIGH);
      digitalWrite(dir_right, HIGH);
      analogWrite(motor_left, p);
      analogWrite(motor_right, p);
      Serial.println("f");
      break;
  case 's':
      digitalWrite(dir_left, HIGH);
      digitalWrite(dir_right, HIGH);
      analogWrite(motor_left, 0);
      analogWrite(motor_right, 0);
      Serial.println("s");
      break;
  default:
      Serial.print("Unknown command: ");
      Serial.println(msg);
    break;
  }

  
};

void setup() {
  Serial.begin(115200); // opens serial port, sets data rate to 9600 bps
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(motor_left, OUTPUT);
  pinMode(dir_left, OUTPUT);
  pinMode(motor_right, OUTPUT);
  pinMode(dir_right, OUTPUT);
};



void loop() {
    if (Serial.available()){
        std::fill_n(cmd_buf, 256, NULL);
        Serial.readBytesUntil('\n', cmd_buf, 256);
        Serial.print("Command recived: ");
        Serial.println(cmd_buf);
        cmd_parse(cmd_buf);
        
    }
}