// BME280_Wifi
//
// ©2021 by Charles Vercauteren 
// 30 april 2021
//
// Tested on Arduino Uno Wifi.
// BME280 connected to I2C bus of Arduino.
//
// TODO:
// Logging nakijken
// LCD setup moet naar voor, maar geeft problemen
//
// Nederlands:
//
// Applicatie:
// -----------
// Dit is een applicatie voor de wifi configuratie software die ik geschreven heb. We gebruiken de BME280
// om de temperatuur, atmosferisch druk en de vochtigheid te meten.
// Er is ook een bijpassende app voor macOS geschreven. Het is mogelijk om de tijd, het log interval en de naam
// in te stellen. We gebruiken hiertoe 1-byte commando's die we mbv. UDP versturen/ontvangen.
// Verbinden via terminal op Mac, gebruik "sudo cu -s 9600 -l /dev/tty.usbmodem14102". Gebruik "~." 
// om cu te verlaten (~ is Option-n).
// Aanpassen van dit gedeelte van de sketch (tussen "BME280 begin" en "BME280 end" voor de setup resp. loop) 
// laat ook toe andere sensoren te gebruiken.
//
// Wifi configuratie:
// ------------------
// Het SSID en wachtwoord voor het wifi-netwerk wordt gelezen uit EEPROM. Om deze waarden
// te configureren zal een drukknop, aangesloten ts D2 (zie #define PIN_CONFIG) en GND, 
// ons in het configuratiemenu brengen indien we deze ingedrukt houden bij een reset of 
// inschakelen van de voeding.
// We vragen dan een SSID, wachtwoord, DHCP/Fixed en eventueel het vaste IP-adres
// voor het draadloze netwerk en bewaren deze in de EEPROM van de processor. 
// Bij een volgende herstart of reset zullen dan deze waarden gebruikt worden.
//
// English:
//
// Application
// -----------
// This is an application for the wifi configuration software I wrote earlier. We use the BME280 to
// measure temperature, atmospheric pressure and humidity.
// I have also written an accompayning app voor macOS. Time, log interval and name can be configured. We
// use 1-byte commands that we send/receive using UDP.
// Connection via terminal on Mac, use "sudo cu -s 9600 -l /dev/tty.usbmodem14102". Ise "~." to 
// leave cu (~ is Option-n).
// By adapting this part of the software (between "BME280 begin" and "BME280 end" for setup and loop) 
// other sensors can be used.
//
// Wifi configuration:
// -------------------
// The SSID and password for the wifi-network will be read out of EEPROM. To configure these
// values a pushbutton, connected between D2 (see #define PIN_CONFIG) and GND, will
// show the configurationmenu when activated during powerup or reset.
// You will be prompted for an SSID, password, name, DHCP/Fixed and if necessary the IP-address
// for the wifi-network. Values will be saved in EEPROM and used on next powerup or reset.
//
// Credits:
// Created 13 July 2010 by dlf (Metodo2 srl)
// Modified 31 May 2012 by Tom Igoe

// Choose version, uncomment
//-----------------------
//#define DEBUG
#define LCD

// Includes
//---------
// Wifi/EEPROM includes begin
#include <WiFiNINA.h>
#include <EEPROM.h>
#include <IPAddress.h>

// BME280
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFiUdp.h>

// LCD
#ifdef LCD
#include <LiquidCrystal.h>
#endif

// Variables and defines
//----------------------
// Seriële
#define MAX_CHAR 32               // Max. number of chars for ssid, password and name
#define CR '\r'                   // You can use CR or LF to end the input on the serial
#define LF '\n'                   // line, but nog both !!! (See also include thermometer)

// EEPROM
int ssidAddress = 0 ;             // EEPROM address ssid
int passwordAddress = MAX_CHAR;   // EEPROM address password
int hostnameAddress = 2*MAX_CHAR; // EEPROM address name
int dhcpModeAddress = 3*MAX_CHAR; // EEPROM address dhcp(1)/fixed(0) choice
int fixedIpAddress = 3*MAX_CHAR+1;// EEPROM address for fixed IP-address

String ssid = "ssid";             // Wifi SSID
String password = "wachtwoord";   // Wifi password (WPA or WEP key)
String hostname = "hostname";     // Network/bord/... name
byte dhcp = 0;                    // Use DHCP by default
String fixedIp = "127.0.0.1";     // Default fixed IP

// IP
bool dhcpMode = false;
IPAddress ip;                     
int status = WL_IDLE_STATUS;      // Wifi radio status

// Diverse
#define PIN_CONFIG   2
bool configMode = false;          // Confifuration mode active.
bool runOnce = true;              // Run code in loop only once.

// UDP
WiFiUDP Udp;
unsigned int localPort = 2000;
char packetBuffer[256];              // Buffer to hold incoming packet
char replyBuffer[2048];

// Commands
#define GET_TEMPERATURE         10
#define GET_HUMIDITY            11
#define GET_PRESSURE            12
#define SET_TIME                20
#define GET_TIME                21
#define SET_LOG_INTERVAL        22
#define GET_LOG_INTERVAL        23
#define GET_LOG                 30
#define GET_HOSTNAME            40
#define SET_HOSTNAME            41
#define UNKNOWN_COMMAND         99

// Diverse
bool ledOnOff = false;    
#ifdef LCD
bool updateLCD = true;
#endif

// Time
int command = 0;
short hours = 0;
short minutes = 0;
short seconds = 0;
short milliseconds = 0;
unsigned long prevMillis = 0;
unsigned long currentMillis = 0;

// Logging
#define MAX_LOG 24              //Number of log entries (adapt replyBuffer also !!!)
                                // !!! Problemen indien 48, buffer te klein ? !!!
#define MAX_LOGSECONDS  900    // Number of seconds between logs
#ifdef DEBUG
  #define MAX_LOGSECONDS 60
#endif
short logSeconds = 0;
short maxLogSeconds = MAX_LOGSECONDS;
float currentTemperature = 0;
float currentPressure = 0;
float currentHumidity = 0;
String currentTemperatureStr = "0.00";
String currentPressureStr = "0.00";
String currentHumidityStr = "0.00";
float temperatureLog[MAX_LOG];
float pressureLog[MAX_LOG];
float humidityLog[MAX_LOG];
String timeLog[MAX_LOG];
int indexLog = 0;

// Init library objects
//---------------------
// BME
Adafruit_BME280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
Adafruit_Sensor *bmp_humidity = bmp.getHumiditySensor();

// LCD
#ifdef LCD
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
#endif

//******
// Setup
//******
void setup() {
  // Setup begins --> LED on
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Serial setup
  Serial.begin(9600);
  while (!Serial) { ;}     // wait for serial port to connect. Needed for native USB port only


  
  // Check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("No wifi module found, stopping !");
    // Don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // Pushbutton configuration active ?
  pinMode(PIN_CONFIG,INPUT_PULLUP);
  if (digitalRead(PIN_CONFIG)==LOW) { 
    configMode = true; 
    Serial.println("Config button pressed.");
  }

  // Start UDP server
  if (Udp.begin(localPort) != 1) {  
    Serial.println("UDP can't be started, stopping !");
    while(true); 
  } 



  
  // BME280 setup
  //-------------
  // Bosch Pressure/Temp sensor BME280
  // Sensor zit op I2C adres 0x76
  // Sensor ID is 0x60
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    #ifdef LCD
    lcd.clear();
    lcd.print("No BME280 sensor found !");
    #endif
    while (1) delay(10);
  }

  // Default settings from datasheet.
  bmp.setSampling(Adafruit_BME280::MODE_NORMAL,     // Operating Mode.
                  Adafruit_BME280::SAMPLING_X2,     // Temp. oversampling
                  Adafruit_BME280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BME280::SAMPLING_X16,    // Humidity
                  Adafruit_BME280::FILTER_X16,      // Filtering
                  Adafruit_BME280::STANDBY_MS_500); // Standby time

  bmp_temp->printSensorDetails();
  
  // Init currentMillis
  currentMillis = millis();

  // Init log arrays
  for (int i=0;i<MAX_LOG;i++) {
    timeLog[i] = "--:--:--";
    temperatureLog[i] = 0;
    pressureLog[i] = 0;
    humidityLog [i] = 0;
  }

       // LCD setup
  #ifdef LCD
  lcd.begin(16, 2);
  lcd.clear();
  #endif 

// Wifi setup
//-----------
  if (configMode) {
    // Configmode
    // ----------
    // Reading SSID/wachtwoord/... from serial
    
    configMode = false;
    String dhcpOrFixed = "";
    
    Serial.println("Timeout input is 60s !");
    Serial.setTimeout(60000);
    ssid = getStringFromSerial("SSID: ");
    password = getStringFromSerial("Password: ");
    hostname = getStringFromSerial("Hostname: ");
    dhcpOrFixed = getStringFromSerial("DHCP/Fixed IP-address (D/F): ");
    dhcpMode = true;
    if (dhcpOrFixed == "F" || dhcpOrFixed == "f") { 
      dhcpMode = false;
      fixedIp = getStringFromSerial("IP-address (e.g. 192.168.0.1): ");
      }
    
    Serial.println("Saving SSID/password in EEPROM.");
    Serial.print("  SSID = "); Serial.println(ssid);
    Serial.print("  Password = "); Serial.println(password);
    Serial.print("  Name = "); Serial.println(hostname);
    if (dhcpMode) { 
      Serial.println("  DHCP mode"); 
      }
    else { 
      Serial.println("  Fixed IP-address"); 
      }
    Serial.println(); Serial.println("Leaving config mode.");

    saveSsidToEeprom(ssid);
    savePasswordToEeprom(password);
    saveHostnameToEeprom(hostname);
    saveDhcpModeToEeprom(dhcpMode);
    if (!dhcpMode) { 
      saveIpAddressToEeprom(fixedIp);
    }

    Serial.println();
  }

  // Connect to wifi
  // ---------------
  // Read ssid/password/name
  ssid="";
  for (int i=0; i<MAX_CHAR; i++) { ssid+=String((char)EEPROM.read(ssidAddress+i)); }
  password="";
  for (int i=0; i<MAX_CHAR; i++) { password+=String((char)EEPROM.read(passwordAddress+i)); }
  hostname="";
  for (int i=0; i<MAX_CHAR; i++) { hostname+=String((char)EEPROM.read(hostnameAddress+i)); }
  if (EEPROM.read(dhcpModeAddress)==0) { 
    dhcpMode = false; 
    fixedIp = "";
    for (int i=0; i<MAX_CHAR; i++) { fixedIp+=String((char)EEPROM.read(fixedIpAddress+i)); }
    Serial.println("Fixed mode.");
    Serial.println(fixedIp);
    }
  else { 
    dhcpMode = true; 
    Serial.println("Dhcp mode.");
  }

  Serial.println("In EEPROM:");
  Serial.print("  SSID = "); Serial.println(ssid);
  Serial.print("  Password = "); Serial.println("********");
  Serial.print("  Name = "); Serial.println(hostname);
  Serial.print("  DHCP = ");
  if (dhcpMode) { Serial.println("Yes"); }
  else { 
    Serial.println("No");  
    Serial.print("  IP: "); Serial.println(fixedIp);
    }
      
  if (!dhcpMode) { 
    ip.fromString(fixedIp.c_str());
    WiFi.config(ip);
  }
  while (status != WL_CONNECTED) {
    #ifdef LCD
    lcd.clear();
    lcd.print("SSID: ");
    lcd.setCursor(0,1);
    lcd.print(ssid);
    #endif
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid.c_str(), password.c_str());
    // Wacht 10 seconden en probeer opnieuw
    delay(10000);
  }

  // We are connected, print info.
  Serial.println("You're connected to the network");
  printCurrentNet();
  printWifiData();

  // Init time to compilation time
  hours = String(__TIME__).substring(0,2).toInt();
  minutes = String(__TIME__).substring(3,5).toInt();

  // Set up PIT for clock
  initPIT();

// Setup ends --> LED off
  digitalWrite(LED_BUILTIN, LOW);
}

//-----
// Loop
//-----
void loop() {
  // Replystring for client
  String replyString = "";

  // Read data from client
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // Data to packetBuffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;    // Make c-string from buffer data
    }

    // Convert question from client to integer, commandos see higher.
    command = String(packetBuffer).toInt();
    #ifdef DEBUG
      Serial.print("Received: ");Serial.println(command);
    #endif
    // Create reply for client = command + " " + result
    // so client knows for which question the answer applies
    replyString = String(command) + " ";
    switch (command) {
      case GET_TEMPERATURE:
        replyString += String(currentTemperature); 
        break;
      case GET_HUMIDITY:
        replyString += String(currentHumidity); 
        break;          
      case GET_PRESSURE:
        replyString += String(currentPressure); 
        break;
      case SET_TIME:
        hours = String(packetBuffer).substring(3,5).toInt();
        minutes = String(packetBuffer).substring(6,8).toInt();
        replyString += buildTimeString();
        break;
      case GET_TIME:
        replyString += buildTimeString();
        break;
      case SET_LOG_INTERVAL:
        maxLogSeconds = String(packetBuffer).substring(3).toInt();
        replyString += String(maxLogSeconds);
        break;
      case GET_LOG_INTERVAL:
        replyString += String(maxLogSeconds);
        break;
      case GET_LOG:
        for (int i=indexLog; i<MAX_LOG; i++) {
          replyString += timeLog[i];
          replyString += "\t";
          replyString += String(temperatureLog[i]);
          replyString += "\t";
          replyString += String(pressureLog[i]);
          replyString += "\t";
          replyString += String(humidityLog[i]);
          replyString += "\n";
        }
        for (int i=0; i<indexLog; i++) {
          replyString += timeLog[i];
          replyString += "\t";
          replyString += String(temperatureLog[i]);
          replyString += "\t";
          replyString += String(pressureLog[i]);
          replyString += "\t";
          replyString += String(humidityLog[i]);
          replyString += "\n";
        }
        //Serial.println(replyString);
        break;
      case GET_HOSTNAME:
        replyString += hostname; 
        break; 
      case SET_HOSTNAME:
        hostname = String(packetBuffer).substring(3);
        replyString += hostname;
        saveHostnameToEeprom(hostname);
        break;  
      default:
        replyString = String(UNKNOWN_COMMAND) + " "; 
        break;
    }

    udpSendString(replyString);
    
  }

  // Measure and log temperature
  measure();
  logAll();
  #ifdef LCD
  if (updateLCD) {
    lcd.clear();
    lcd.print("T ");lcd.print(currentTemperature); lcd.print(" ");
    lcd.print(buildTimeString());
    lcd.setCursor(0,1);

    lcd.print("P ");lcd.print((int)currentPressure); lcd.print("  ");
    lcd.print("H ");lcd.print((int)currentHumidity); 
    updateLCD = false;
    }
    #endif
   
}

// --> EEPROM functions begin ---

void saveSsidToEeprom(String name) {
  for (int i=0; i<MAX_CHAR; i++) {
    EEPROM.write(ssidAddress+i,name.charAt(i));
  }
}

void savePasswordToEeprom(String name) {
  for (int i=0; i<MAX_CHAR; i++) {
    EEPROM.write(passwordAddress+i,name.charAt(i));
  }
}

void saveHostnameToEeprom(String name) {
  for (int i=0; i<MAX_CHAR; i++) {
    EEPROM.write(hostnameAddress+i,name.charAt(i));
  }
}

void saveDhcpModeToEeprom(bool mode) {
  if (mode) { 
    EEPROM.write(dhcpModeAddress, 1); 
    }
  else { 
    EEPROM.write(dhcpModeAddress, 0); 
  }
}

void saveIpAddressToEeprom(String name) {
  for (int i=0; i<MAX_CHAR; i++) {
    EEPROM.write(fixedIpAddress+i,name.charAt(i));
  }
}

// --> Wifi functions begin ---

void printWifiData() {
  // Print IP-adres:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // Print MAC-adres
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

/*
void recvString() {
  // Check for new data on serial.
  // If so, read it. If END_MARKER, close string with '\0'
  // to create a valid C-string. Make "dataAvailable" true.
  // Check the number of characters is smaller then  MAX_CHAR
  // and limit if necessary.
  
  static byte ndx = 0;
  char rc;

  while (Serial.available() > 0 && dataAvailable == false) {
    rc = Serial.read();
    Serial.print(rc);       //echo character

    if (rc != CR && rc != LF) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= MAX_CHAR) {
        ndx = MAX_CHAR - 1;
      }
    }
    else {
     receivedChars[ndx] = '\0'; // terminate the string
     ndx = 0;
     dataAvailable = true;
    }
  }
}
*/
// Serial functions
//------------------
String getStringFromSerial(String askfor) {
  String answer;

  Serial.print(askfor);
  answer = Serial.readStringUntil('\r');
  if (answer.length() == 0) {
    answer = "none";
  }
  Serial.println(answer);
  return answer;
}

// BME280 functions
//-----------------

void udpSendString(String toSend) {
  Serial.println(toSend);
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  toSend.toCharArray(replyBuffer,toSend.length()+1);
  Udp.write(replyBuffer);
  Udp.endPacket();
}

void measure() {
  
  sensors_event_t temp_event, pressure_event, humidity_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  bmp_humidity->getEvent(&humidity_event);

  currentTemperatureStr = temp_event.temperature;
  currentPressureStr = pressure_event.temperature;
  currentHumidityStr = humidity_event.temperature;
  currentTemperature = currentTemperatureStr.toFloat();
  currentPressure = currentPressureStr.toFloat();
  currentHumidity = currentHumidityStr.toFloat();
}

void logAll() {
  // 1 second has passed
  // logSeconds is rased in updateClock
  // Log if time is there
  if( logSeconds >= maxLogSeconds) {
    /*
      Serial.println("Updating log");
    */
    temperatureLog[indexLog] = currentTemperature;
    pressureLog[indexLog] = currentPressure;
    humidityLog[indexLog] = currentHumidity;
    timeLog[indexLog] = buildTimeString();

    logSeconds = 0;
    indexLog += 1; 
    // Rotate log if full
    if (indexLog>=MAX_LOG) { indexLog = 0; }
  }
}

String buildTimeString() {
  // Builds and returns a string format HH:MM:SS
  
  String timeString = "";
  
  if (hours<10) { timeString = "0" + String(hours); }
        else { timeString = String(hours) ;}
        timeString += ":";
        if (minutes <10) { timeString += "0" + String(minutes); }
        else { timeString += String(minutes) ; }
        timeString+= ":";
        if (seconds<10) { timeString += "0" + String(seconds); }
        else { timeString += String(seconds);
        }
  return timeString;
}

void updateClock() {
  // Update clock every second

  #ifdef DEBUG
  //Serial.print("Free memory: "); Serial.println(freeMemory());
  #endif
  
  // Create 1s delay independant of run time loop
  // Serial.println("Updating clock");
  short prevSeconds = 0;
  String str = "";

  //Wait 1s
  while ((currentMillis - prevMillis) < 1000) {
    currentMillis = millis();
  }
  prevMillis = currentMillis;

  //Update clock
  seconds += 1; 
  if (seconds >= 60) { minutes += 1; seconds=seconds - 60; }
  if (minutes >= 60) { hours += 1; minutes=minutes - 60; }
  if (hours >= 24) { hours = 0; }  
  // Update logging timer
  logSeconds += 1;
}

// PIT
//----
void initPIT() {
   // Setup x-tal klok
  // Wacht tot x-tal klok stabiel is
  while((CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm));  // Wacht tot XOSC32KS 0 is

  // Configureer klok
  // Configuration Change proctection alvorens schrijven naar CLKCTRL is nodig !!!
  CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA |= CLKCTRL_ENABLE_bm;    // Selekteer X-tal

  // Selecteer klok
  RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
  
  // Enable interrupt  PIT
  RTC.PITINTCTRL = RTC_PITEN_bm;
  // Selecteer periode (32768 cycli)
  RTC.PITCTRLA = (RTC_PITCTRLA &~ RTC_PERIOD_gm) | RTC_PERIOD1_bm  | RTC_PERIOD2_bm | RTC_PERIOD3_bm; 
  // Enable PIT
  while(RTC.STATUS);
  while (RTC.PITSTATUS);
  RTC.PITCTRLA |= RTC_PITEN_bm;

  sei();
}

ISR(RTC_PIT_vect) {  

  seconds += 1;
  logSeconds += 1;
  #ifdef LCD
  updateLCD = true;
  #endif

  if ( seconds >= 60 ) { 
    seconds = 0;
    minutes += 1;
  }
  
  if (minutes>=60) { 
    minutes=0;
    hours+=1;
    }
    
  if (hours>=24) {
    hours=0;
  }

  // Clear PIT interrupt flag
  RTC.PITINTFLAGS = 1;

}
