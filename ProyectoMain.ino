#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "sensor.h"
#include "actuator.h"

#define WIFI_SSID "Prueba"
#define WIFI_PASS "12345678"
#define MQTT_BROKER "a25ex5q24thysh-ats.iot.us-east-1.amazonaws.com"
#define MQTT_PORT 8883
#define CLIENT_ID "ESP-32"
#define UPDATE_TOPIC "$aws/things/Esp32-sensor/shadow/update"
#define UPDATE_DELTA_TOPIC "$aws/things/Esp32-sensor/shadow/update/delta"
#define BUZZER_PIN 4
#define SENSOR_PIN 32
#define MEASUREMENT_INTERVAL 5000  // Intervalo de reporte en milisegundos

const char AMAZON_ROOT_CA1[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";


const char CLIENT_CERTIFICATE[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUSIX+wu2nuSCgoeYoU/2YogPxilwwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI0MTEwNjIyNTA0
OFoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMAJW6S40jPEWnYFMSsQ
O9EcgHLHeq02kSDdIeeqGKgw5/Cez1cDL9xYagg2nEjoNbbsBvUMMs89lg3alkyh
RHO1XgFn4cJfn6CU8lG7nnXpWEPQokeFG47Zqb2YVRYNYB7j5ZM2rBlAwM3GigFb
+TAPM1XLcLgJasZVQdvYQS05Ul//gbQ3rghzQ81AsLiL1s1KlSwmLwfOjYYX72tW
eteMeW6IWPfgvwa7gy4KsE8sbI0fcHxRKx4kbi/1L1ROB1uAwpxBVMjq+vYWZmNL
VBd21H1DopSFNiqCOiS9EDBzXf/DavX86eBspCkJ0xXDpi0AXxUz20+KgT3vFrN7
S8UCAwEAAaNgMF4wHwYDVR0jBBgwFoAUaUpkMOomc3Kd3A5MBTrHyusf4x0wHQYD
VR0OBBYEFFueXKZeHy5gqz8ZprdCRbsNl81NMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQDF29F/cgZOLUEnaciAKj8KRknM
4nHs8lGpwNkgE2f4rAC73B+4RiXVYSxdCb2KOgk6xoWmGNAvaT2n+zf0nXNp1T3H
3lTTgDnFt6fazkTz2DlbcJ+Q1F+dXVZSUWpyK3PbM2viHrckAP/TLGWJ+U0+cFvs
N1V2GOV+UMXz8Q0WcKa3KDtwNnZneiENxl2JaEGGxkv9yTrUVDegBVIUF3hOEY4j
MBti/jngI3LEqWPlx3ukk2F6vJCDSnpBJUlJ6DFuAYXWlbWiebxdAupjfhxGMa8z
UBCF1H1Y2RHFwdcywjXJPwd1YQgAgTAuK/QV3B6FcMMaI5y2FjAHu3dMPD3j
-----END CERTIFICATE-----
)KEY";



const char PRIVATE_KEY[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEAwAlbpLjSM8RadgUxKxA70RyAcsd6rTaRIN0h56oYqDDn8J7P
VwMv3FhqCDacSOg1tuwG9Qwyzz2WDdqWTKFEc7VeAWfhwl+foJTyUbuedelYQ9Ci
R4UbjtmpvZhVFg1gHuPlkzasGUDAzcaKAVv5MA8zVctwuAlqxlVB29hBLTlSX/+B
tDeuCHNDzUCwuIvWzUqVLCYvB86Nhhfva1Z614x5bohY9+C/BruDLgqwTyxsjR9w
fFErHiRuL/UvVE4HW4DCnEFUyOr69hZmY0tUF3bUfUOilIU2KoI6JL0QMHNd/8Nq
9fzp4GykKQnTFcOmLQBfFTPbT4qBPe8Ws3tLxQIDAQABAoIBAQCqHOAQyCR09SCJ
YcoPcr76W7wxcSHsJOVk7l7lgFl9TG16SsQzOUCBedTb17yUHRfQJb93/clYqh6t
wQ/ZbBZeRbCzoBlt/RVDxA3wdzpFC2NEF6JZpsElxw1FRZdCBLE8S/eHdCv55tGk
t5Zy482XVC/OdXwxmy356XaA0E3UR2LHn+gmYAM0TWEXoSy2pdkfg0pia8l963jD
fkVk9RcDhbQ12jdK7quhfIGDKVyggt3UsedjpUS8jAQLEN0fgAgGeybfcAcHPiNj
N2k3+YyPsxVLCT6//DWDOAtsRTqvftoyyyyRvaIN50FYMehL9i1+83cX3kQwlF8V
j/42+JrVAoGBAPeX921a1K2Ccz40zKr1ADgwaaP7ddvHi0Uj/1aKxXShewRaz4cG
aFVL+C694RNSbMKQE3sPdZarDgZXsgILkVe/383LaKuc7uv76WX4wUG/k1I++UXA
abLFIYr6Sn9A9edzf8E6XKMWy0dZ+JXoARDhON4B1AYkHkv0oSXvimC/AoGBAMaO
gDCB+NnwHWR9hcUKVJWQDJ6rhIrkjmkjT9dehUAvosS4ksp8ZlXk+KD6K7ROToMK
UP/maQH2jEbAElpTow4Twk39IMqMBnQeczxqgm6NNheNX96F5PWzN9StTwZefIi0
LtEzBjShgYLE63JJ55CIdu1bTEV6i9k9iy6h5TB7AoGAASjqFM4PHvz0cz8mEkrx
hFHdvNHpvGzT3zAK+ynNQP2cqOmP/U7vYC7p67Yh/sgtEEBChxoMb/c/KpJwmAfb
13BjqQAh7mPezqYwcrTWmus90m3PFx+OnzLDNHZ1sODqBfqJh6sxUeZCb4QhYPpk
PuL/NDMH/jtfFPYBZz8XA9sCgYBZU8rAB/kYsznF9+Kyns8m6kODLs7WXbTkAArv
bYqtMcEcl50Sk5zGj9h7M5Ft8LzJn16JgfND+tNpEQgwsufscV89DuAtv4k3vGtD
hvdfHRB00BmNVteG4jX/7GhdCS0lo0pcu4PTMcW+OzFhnXYcRAidCDHsIBHCjjmL
pTIi+QKBgQCrUkdwu/H6UQuQS1awzjO4pqavlj17rXXNcIzy3HAGtDNXqLGOB5CX
jB4/aNAxC/m2LWfKrfl8HPumsfmjC9lqTHpfktOj4y1wqUIEbSbiddYppW73pWaG
eyDc9DZ4UhkqpwXGcRC0qOp39XRjOc9ymiuV4cw6tP61mdXFZEv8zA==
-----END RSA PRIVATE KEY-----

)KEY";


WiFiClientSecure wiFiClient;
PubSubClient client(wiFiClient);
StaticJsonDocument<JSON_OBJECT_SIZE(64)> inputDoc;
StaticJsonDocument<JSON_OBJECT_SIZE(4)> outputDoc;
char outputBuffer[128];

Sensor heartRateSensor(SENSOR_PIN);
Actuator buzzer(BUZZER_PIN);
volatile byte builtInBuzzer = 0;
unsigned long lastMeasurementTime = 0;

// Función de interrupción para detectar pulso
void IRAM_ATTR detectPulseISR() {
    heartRateSensor.detectPulse();
}

void reportBuiltInBuzzer() {
    outputDoc.clear();
    outputDoc["state"]["reported"]["builtInBuzzer"] = builtInBuzzer;
    outputDoc["state"]["reported"]["currentBPM"] = heartRateSensor.getCurrentBPM();
    outputDoc["state"]["reported"]["validPulses"] = heartRateSensor.getValidPulseCount();
    serializeJson(outputDoc, outputBuffer);
    client.publish(UPDATE_TOPIC, outputBuffer);
}

void setBuiltInBuzzer() {
    if (builtInBuzzer) {
        heartRateSensor.initialize();
        attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), detectPulseISR, RISING);
    } else {
        detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));
        buzzer.deactivate();
    }
    reportBuiltInBuzzer();
}

void callback(char* topic, byte* payload, unsigned int length) {
    DeserializationError err = deserializeJson(inputDoc, payload);
    if (!err && String(topic) == UPDATE_DELTA_TOPIC) {
        builtInBuzzer = inputDoc["state"]["builtInBuzzer"].as<int8_t>();
        setBuiltInBuzzer();
    }
}

void setupWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
}

void setup() {
    Serial.begin(115200);
    setupWiFi();

    wiFiClient.setCACert(AMAZON_ROOT_CA1);
    wiFiClient.setCertificate(CLIENT_CERTIFICATE);
    wiFiClient.setPrivateKey(PRIVATE_KEY);

    client.setServer(MQTT_BROKER, MQTT_PORT);
    client.setCallback(callback);

    heartRateSensor.initialize();
    buzzer.initialize();
}

void reconnect() {
    while (!client.connected()) {
        if (client.connect(CLIENT_ID)) {
            client.subscribe(UPDATE_DELTA_TOPIC);
            reportBuiltInBuzzer();
        } else {
            delay(5000);
        }
    }
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    unsigned long currentTime = millis();

    // Procesar pulso detectado
    if (heartRateSensor.isPulseDetected() && builtInBuzzer) {
        buzzer.activate();
        delay(50);
        buzzer.deactivate();
        
        // Mostrar BPM y pulsos validados
        Serial.print("Frecuencia Cardíaca (BPM): ");
        Serial.println(heartRateSensor.getCurrentBPM());
        Serial.print("Número de Pulsos Validados: ");
        Serial.println(heartRateSensor.getValidPulseCount());

        heartRateSensor.clearPulseDetection();
    }

    // Reportar mediciones cada 5 segundos
    if (currentTime - lastMeasurementTime >= MEASUREMENT_INTERVAL) {
        reportBuiltInBuzzer();
        
        // Reiniciar contador de pulsos válidos
        heartRateSensor.resetValidPulseCount();
        lastMeasurementTime = currentTime;
    }
}