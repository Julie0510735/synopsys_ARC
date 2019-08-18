#include <DHT.h>
#include <DHT_U.h>

#include <ESP8266WiFi.h>

#define DHTTYPE DHT11   // DHT 11
#define I0 D7
#define I1 D6
#define I2 D5

const float zeroWindAdjustment =  0.2; 
int TMP_Therm_ADunits;  
float RV_Wind_ADunits;   
float RV_Wind_Volts;
unsigned long lastMillis;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;
float ssd;
int time_send = 0,time_interval = 0;

/*Put your SSID & Password*/
const char* ssid = "Tom Cruise";  // Enter SSID here
const char* password = "87878787";  //Enter Password here
const char* host = "maker.ifttt.com";
const char* eventName_hot   = "ssd_positive";
const char* eventName_cool   = "ssd_negative";
const char* key = "jhbIziaeimdtMDKo9K-TeqVsKF-gVg31yGKzOAZbHBu";  //Ifttt授權碼


// DHT Sensor
uint8_t DHTPin = D2; 
               
// Initialize DHT sensor.
DHT dht(DHTPin, DHTTYPE);                

int Temperature;
int Humidity;
int Wsp;
 
void setup() {
  Serial.begin(9600);

   /*Serial.print("Connecting to ");
   Serial.println(ssid);*/
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     //Serial.print(".");
   }
   /*Serial.println("");
   Serial.println("WiFi connected");  
   Serial.println("IP address: ");
   Serial.println(WiFi.localIP());*/

  pinMode(I0, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(DHTPin, INPUT);

  digitalWrite(I0, LOW);
  digitalWrite(I1, LOW);
  digitalWrite(I2, LOW);
  
  dht.begin();              

  /*Serial.println("Connecting to ");
  Serial.println(ssid);

  //connect to your local wi-fi network
  WiFi.begin(ssid, password);

  //check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected..!");
  Serial.print("Got IP: ");  Serial.println(WiFi.localIP());*/

}
void loop() {

  //Serial.print("connecting to ");
  // Serial.println(host);
   WiFiClient client;
   const int httpPort = 80;
   if (!client.connect(host, httpPort)) {
     //Serial.println("connection failed");
     return;
   }
   //Read TMP
   digitalWrite(I0, HIGH);
   digitalWrite(I1, LOW);
   digitalWrite(I2, LOW);
   TMP_Therm_ADunits = analogRead(A0);
   delay(50);
   
   //Read RV
   digitalWrite(I0, HIGH);
   digitalWrite(I1, HIGH);
   digitalWrite(I2, LOW);
   RV_Wind_ADunits = analogRead(A0);
   
   //Calculate wind speed
   RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);
   TempCtimes100 = (0.005 *((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;  

   zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39

   zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  

   WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265);   
   WindSpeed_MPH/=10;
   Wsp = WindSpeed_MPH;
   
   Temperature = dht.readTemperature(); // Gets the values of the temperature
   Humidity = dht.readHumidity(); // Gets the values of the humidity

   
   ssd = (1.818 * Temperature + 18.18) * (0.88 + 0.002 * Humidity) + (Temperature - 32)/(45 - Temperature) - 3.2 * Wsp + 18.2;
   Serial.print('|');
   Serial.print((int)ssd);
   
   delay(500);
   time_interval = (millis() - time_send)/1000;
   if(ssd >= 70 && time_interval >= 60) {
     time_send = millis();
     String url = "/trigger/"; 
     url += eventName_hot;
     url += "/with/key/";
     url += key;
     url += "?value1=";
     url += String(Temperature);
     url += "&value2=";
     url += String(Humidity);
     url += "&value3=";
     url += String(WindSpeed_MPH);
     //Serial.print("Requesting URL: ");
     //Serial.println(url);
     
     client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                  "Host: " + host + "\r\n" + 
                  "Connection: close\r\n\r\n");
     int timeout = millis() + 5000;
     while (client.available() == 0) {
       if (timeout - millis() < 0) {
         //Serial.println("Client Timeout !");
         client.stop();
         return;
       }
     }
   }
   if(ssd <= 58 && time_interval >= 60) {
     time_send = millis();
     String url = "/trigger/";  
     url += eventName_cool;
     url += "/with/key/";
     url += key;
     url += "?value1=";
     url += String(Temperature);
     url += "&value2=";
     url += String(Humidity);
     url += "&value3=";
     url += String(WindSpeed_MPH);
     //Serial.print("Requesting URL: ");
     //Serial.println(url);
     
     client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                  "Host: " + host + "\r\n" + 
                  "Connection: close\r\n\r\n");
     int timeout = millis() + 5000;
     while (client.available() == 0) {
       if (timeout - millis() < 0) {
         //Serial.println("Client Timeout !");
         client.stop();
         return;
       }
     }
   }
   //delay(30000);
}
