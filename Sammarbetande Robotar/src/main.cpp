#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define motor_left 5
#define dir_left 0
#define motor_right 4
#define dir_right 2

const char *ssid = "ABBgym_2.4";
const char *password = "mittwifiarsabra";
const char *mqtt_server = "10.22.4.161";

WiFiClient espClient;
PubSubClient client(espClient);

float p = 400;

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

void cmd_parse(char msg[256])
{
  char cmd = msg[0];
  char args[256];
  for (size_t i = 0; i < 255; i++)
  {
    args[i] = msg[i + 1];
  }

  String argString = args;
  p = 10.0 * argString.toFloat();

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
    Serial.println("1r");
    break;
  case 'l':
    digitalWrite(dir_left, LOW);
    digitalWrite(dir_right, HIGH);
    analogWrite(motor_left, p);
    analogWrite(motor_right, p);
    Serial.println("1l");
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
    client.publish("message", cmd_buf);
    cmd_parse(cmd_buf);
  }
}