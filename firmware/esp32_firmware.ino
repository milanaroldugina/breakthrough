// Финальная прошивка ESP32-C3 для проекта BREAKTHROUGH.
// Роль платы намеренно ограничена транспортом:
// точка доступа (Wi-Fi AP) + DHCP + статусная веб-страница + TCP/UART мост.
// Моторы, датчики, одометрия и watchdog всегда остаются в Arduino.

#include <WebServer.h>
#include <WiFi.h>

// Настройки Wi-Fi сети, которую создаст ESP32
const char *AP_NAME = "BREAKTHROUGH_ROBOT";
const char *AP_PASSWORD = "breakthrough123";

// Настройки TCP сервера и UART
const uint16_t TCP_PORT = 8888;
const uint8_t ROBOT_RX_PIN = 4; // Пин RX ESP32 подключается к TX Arduino
const uint8_t ROBOT_TX_PIN = 5; // Пин TX ESP32 подключается к RX Arduino

HardwareSerial robotSerial(1);
WebServer server(80);
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

char tcpLine[192];
uint8_t tcpLength = 0;
char uartLine[192];
uint8_t uartLength = 0;

unsigned long commandsFromPc = 0;
unsigned long linesFromArduino = 0;

// Отправка строки обратно в Python по TCP
void sendTcp(const char *text) {
  if (tcpClient && tcpClient.connected()) {
    tcpClient.println(text);
  }
}

// Веб-страница статуса (доступна по адресу 192.168.4.1 в браузере)
void showStatus() {
  String page =
      "<!doctype html><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width'>"
      "<style>body{font-family:sans-serif;max-width:600px;margin:30px auto}"
      "b{color:#1677d2}</style><h1>BREAKTHROUGH Robot</h1>"
      "<p>Wi-Fi сеть: <b>";
  page += AP_NAME;
  page += "</b></p><p>TCP порт: <b>192.168.4.1:";
  page += String(TCP_PORT);
  page += "</b></p><p>Статус клиента (Python): <b>";
  page += (tcpClient && tcpClient.connected() ? "подключён" : "нет");
  page += "</b></p><p>Команд получено от ПК: ";
  page += String(commandsFromPc);
  page += "</p><p>Строк телеметрии от Arduino: ";
  page += String(linesFromArduino);
  page += "</p>";
  server.send(200, "text/html; charset=utf-8", page);
}

// Принятие нового TCP-подключения от Python
void acceptTcpClient() {
  if (tcpClient && tcpClient.connected()) return;
  tcpClient = tcpServer.available();
  if (tcpClient) {
    tcpClient.setNoDelay(true); // Отключаем буферизацию для мгновенной отправки
    tcpLength = 0;
  }
}

// Чтение команд от Python и пересылка их в Arduino по UART
void readTcpClient() {
  while (tcpClient && tcpClient.available()) {
    const char symbol = tcpClient.read();
    if (symbol == '\n' || symbol == '\r') {
      if (tcpLength > 0) {
        tcpLine[tcpLength] = '\0';
        robotSerial.println(tcpLine); // Отправляем команду (например, "VEL 350 0") в Arduino
        commandsFromPc++;
        tcpLength = 0;
      }
    } else if (tcpLength < sizeof(tcpLine) - 1) {
      tcpLine[tcpLength++] = symbol;
    } else {
      tcpLength = 0;
      sendTcp("ERR LINE_TOO_LONG");
    }
  }
}

// Чтение ответов/телеметрии от Arduino и пересылка их в Python по TCP
void readRobotSerial() {
  while (robotSerial.available()) {
    const char symbol = robotSerial.read();
    if (symbol == '\n' || symbol == '\r') {
      if (uartLength > 0) {
        uartLine[uartLength] = '\0';
        sendTcp(uartLine);
        linesFromArduino++;
        uartLength = 0;
      }
    } else if (uartLength < sizeof(uartLine) - 1) {
      uartLine[uartLength++] = symbol;
    } else {
      uartLength = 0;
    }
  }
}

void setup() {
  // Инициализация основного Serial (для отладки через USB)
  Serial.begin(115200);
  
  // Инициализация Serial1 для связи с Arduino (38400 бод, как в прошивке Arduino)
  robotSerial.begin(38400, SERIAL_8N1, ROBOT_RX_PIN, ROBOT_TX_PIN);

  // Настройка Wi-Fi в режиме точки доступа (AP)
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(300);
  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.setSleep(false); // Отключаем энергосбережение для стабильной связи
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.softAP(AP_NAME, AP_PASSWORD);

  // Запуск серверов
  tcpServer.begin();
  server.on("/", HTTP_GET, showStatus);
  server.begin();

  Serial.println("READY ESP32_BREAKTHROUGH");
  Serial.print("Wi-Fi: ");
  Serial.println(AP_NAME);
  Serial.print("TCP: 192.168.4.1:");
  Serial.println(TCP_PORT);
}

void loop() {
  server.handleClient();      // Обработка HTTP-запросов (страница статуса)
  acceptTcpClient();          // Проверка новых подключений от Python
  readTcpClient();            // Чтение команд от Python -> отправка в Arduino
  readRobotSerial();          // Чтение данных от Arduino -> отправка в Python
}
