#include <WiFi.h>
#include <FirebaseESP32.h>

#define echoPin 25
#define trigPin 26

#define FIREBASE_HOST "https://esp32-01-5f0aa-default-rtdb.firebaseio.com/"   //replace this with your DB name
#define FIREBASE_AUTH "jxigena1L1I8Ij0Hx8L1qn8poWTTi30RauojCOJa"              //replace this with your DB secret key
#define WIFI_SSID "FreeWifi"
#define WIFI_PASSWORD "8characters"

FirebaseData firebaseData;
FirebaseJson json;

long duration;    //sensor stuff
int distance;     //sensor stuff

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT  //sensor stuff
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT    //sensor stuff

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
 
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");

  /*
  This option allows get and delete functions (PUT and DELETE HTTP requests) works for device connected behind the
  Firewall that allows only GET and POST requests.
  
  Firebase.enableClassicRequest(firebaseData, true);
  */
 
  //String path = "/data";
  
 
  Serial.println("------------------------------------");
  Serial.println("Connected...");

}

void loop() {
  // put your main code here, to run repeatedly:

  // Clears the trigPin condition (sensor stuff)
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  Serial.println(distance);
  delay(200);

  json.set("/Distance", distance);
  Firebase.updateNode(firebaseData,"/Sensor",json);

}
