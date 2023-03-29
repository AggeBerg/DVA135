#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <math.h>

#define motor_left 5
#define dir_left 0
#define motor_right 4
#define dir_right 2

const char *ssid = "ABBgym_2.4";
const char *password = "mittwifiarsabra";
const char *mqtt_server = "10.22.2.8";

WiFiClient espClient;
PubSubClient client(espClient);

float integral = 0, derivative = 0;
union
{
  float f[4] = {0.005, 0.01, 0.002, 900.0};
  char b[16];
} pidConst;

char cmd_buf[256];

void setup_wifi()
{

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  if (strcmp(topic, "test") == 0)
  {
    union
    {
      uint16_t uns;
      sint16_t sig;
    } motor_voltage_l;
    union
    {
      uint16_t uns;
      sint16_t sig;
    } motor_voltage_r;

    motor_voltage_l.uns = (payload[0] | payload[1] << 8);
    motor_voltage_r.uns = (payload[2] | payload[3] << 8);
    Serial.printf("payload(L,R)%i, %i\n", motor_voltage_l.sig, motor_voltage_r.sig);
    if (motor_voltage_l.sig < 0)
    {
      analogWrite(motor_left, abs(motor_voltage_l.sig));
      digitalWrite(dir_left, false);
    }
    else
    {
      analogWrite(motor_left, motor_voltage_l.sig);
      digitalWrite(dir_left, true);
    }
    if (motor_voltage_r.sig < 0)
    {
      analogWrite(motor_right, abs(motor_voltage_r.sig));
      digitalWrite(dir_right, false);
    }
    else
    {
      analogWrite(motor_right, motor_voltage_r.sig);
      digitalWrite(dir_right, true);
    }
  }
  else if (strcmp(topic, "setPidConst") == 0)
  {
    for (uint8_t i = 0; i < length; i++)
    {
      pidConst.b[i] = payload[i];
    }
  }
  else
  {
    Serial.printf("Unknown topic\npayload(string,hex): %s, %x", payload, payload);
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("vetefan", "axel.ahman@abbindustrigymnasium.se", "axelLampa"))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("inTopic", "hello world");
      // ... and resubscribe
      client.subscribe("test");
      client.subscribe("setPidConst");

      client.publish("inTopic", "3Y");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

float pid(float err, float pc, float ic, float dc)
{
  integral = +err;
  derivative = err - derivative;
  return err * pc + integral * ic + derivative * dc;
}

void setMotors(float speed, float right, float left)
{
  digitalWrite(dir_left, left < 0);
  digitalWrite(dir_right, right < 0);
  analogWrite(motor_left, fabsf(speed * left));
  analogWrite(motor_right, fabsf(speed * right));
  Serial.printf("s:%f,r:%f,l:%f\nr:%f,l:%f\n", speed, right, left, fabsf(speed * right), fabsf(speed * left));
}

void cmd_parse(char msg[256])
{
  char cmd = msg[0];
  char args[256];
  for (size_t i = 0; i < 255; i++)
  {
    args[i] = msg[i + 1];
  }

  // String argString = args;
  // p = 10.0 * argString.toFloat();

  switch (cmd)
  {
  case 'a':
    // f[0]: speed, f[1]: right, f[2]: left
    union
    {
      float f[3];
      char b[12];
    } srl;
    for (size_t i = 0; i < 12; i++)
    {
      srl.b[i] = msg[i + 1];
    }
    setMotors(srl.f[0], srl.f[1], srl.f[2]);
    break;
  case 'e':
    float pidO, left, right;
    union
    {
      float e;
      char b[4];
    } err;
    for (size_t i = 0; i < 4; i++)
    {
      err.b[i] = msg[i + 1];
    }
    Serial.println(err.e);
    pidO = pid(err.e, pidConst.f[0], pidConst.f[1], pidConst.f[2]);

    right = (pidO >= 0 ? 1 : pidO + 1);
    left = (pidO <= 0 ? 1 : 1 - pidO);
    right = (right < -1 ? -1 : right);
    left = (left < -1 ? -1 : left);

    setMotors(pidConst.f[3], right, left);
    break;
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
  case 'f':
    digitalWrite(dir_left, LOW);
    digitalWrite(dir_right, LOW);
    analogWrite(motor_left, 700);
    analogWrite(motor_right, 700);
    Serial.println("1f");
    break;
  case 's':
    digitalWrite(dir_left, HIGH);
    digitalWrite(dir_right, HIGH);
    analogWrite(motor_left, 0);
    analogWrite(motor_right, 0);
    Serial.println("1s");
    break;
  default:
    Serial.print("Unknown command: ");
    Serial.println(msg);
    break;
  }
};

void setup()
{
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Serial.begin(115200); // opens serial port, sets data rate to 9600 bps
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(motor_left, OUTPUT);
  pinMode(dir_left, OUTPUT);
  pinMode(motor_right, OUTPUT);
  pinMode(dir_right, OUTPUT);
  pidConst.f[0] = 0.015;
  pidConst.f[0] = 0.001;
  pidConst.f[2] = 0.002;
  pidConst.f[3] = 700;
};

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  if (Serial.available())
  {
    std::fill_n(cmd_buf, 256, NULL);
    Serial.readBytesUntil('\n', cmd_buf, 256);
    Serial.print("Command recived: ");
    Serial.println(cmd_buf);
    // client.publish("message", cmd_buf);
    cmd_parse(cmd_buf);
  }
}