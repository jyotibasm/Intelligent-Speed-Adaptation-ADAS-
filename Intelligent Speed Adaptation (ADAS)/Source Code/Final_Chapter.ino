// Jain college of Engineering and Research
// Title : Auto Speed Controll of Vehicle using Geofensing 
// Guide : Chaitnya K. J.
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
static const int RXPin = 7, TXPin = 8;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// Global Variables
int speed = 255;

// execute only once
bool flag = true;

// Reference point of JCER (Geofence Area)
float init_lat = 15.822261;
float init_lon = 74.488938;

// Live Latitude & longitute 
float lat, lon;

/* Moter Control */
// Pins - D6 D9 D10 D11
const int left_motor_forward = 6;
const int left_motor_reverse = 9;
const int right_motor_forward = 10;
const int right_motor_reverse = 11;

// Remote control car variable t to read Serial data
int t;

// Buzzer and LED
const int buzzer = A2;
const int red_led = A0;
const int green_led = A1;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);
  Serial.println("Jain college of Engineering and Research");
  Serial.println("Title : Auto Speed Controll of Vehicle using Geofensing ");
  Serial.println(F("FullExample.ino"));
  Serial.println(F("An extensive example of many interesting TinyGPSPlus features"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));


  Serial.println(F("GPS Neo 6"));
  //Remote control Car
  /* Motors */
  pinMode(left_motor_forward, OUTPUT);   //left motors forward
  pinMode(left_motor_reverse, OUTPUT);   //left motors reverse
  pinMode(right_motor_forward, OUTPUT);  //right motors forward
  pinMode(right_motor_reverse, OUTPUT);  //right motors reverse

  // Alerts
  pinMode(buzzer, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
}

//---------------------------------------------------LOOP-STARTS----------------------------------------------------------//
void loop()  // void loop starts here
{
  float distance;
  /*Remote Control Car code Starts*/
  while (Serial.available()) {
    t = Serial.read();
    Serial.println(t);
    if (t == '1') {  //move forward(all motors rotate in forward direction)
      forward();
    }

    else if (t == '2') {  //move reverse (all motors rotate in reverse direction)
      backward();
    }

    else if (t == '3') {  //turn right (left side motors rotate in forward direction, right side motors doesn't rotate)
      turn_right();
    }

    else if (t == '4') {  //turn left (right side motors rotate in forward direction, left side motors doesn't rotate)
      turn_left();
    }

    else if (t == '5') {  //STOP (all motors stop)
      stop();
    }
    delay(100);
  }
  /*Remote Control Car code Ends*/

  /*GPS Starts Here*/
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    Serial.println(lat);
    Serial.println(lon);
  }
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printDateTime(gps.date, gps.time);

  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

  /*GPS Ends Here*/
  distance = getDistance(lat, lon, init_lat, init_lon);
  Serial.print("Distance = ");
  Serial.println(distance);
  // Speed controll logic
  if (distance >= 0 && distance <= 50) {
    alert_on();
    if (flag) {
       for (float i = 255; i <= (255 * 0.60); i = i + 255 * (0.10)) {
         speed = i;
         delay(500);
       }
    }
    flag = false;
    speed = (255 * (0.60));

  } else {
    speed = 255;
    alert_off();
  }
}

//------------------------------------------------------LOOP-END-------------------------------------------------------//

// Motor control functions
void forward() {  // forward
  analogWrite(left_motor_forward, speed);
  analogWrite(left_motor_reverse, 0);
  analogWrite(right_motor_forward, speed);
  analogWrite(right_motor_reverse, 0);
}

void backward() {  // backward
  analogWrite(left_motor_forward, 0);
  analogWrite(left_motor_reverse, speed);
  analogWrite(right_motor_forward, 0);
  analogWrite(right_motor_reverse, speed);
}

void turn_right() {  // turn right
  analogWrite(left_motor_forward, 0);
  analogWrite(left_motor_reverse, 0);
  analogWrite(right_motor_forward, speed);
  analogWrite(right_motor_reverse, 0);
}

void turn_left() {  // turn left
  analogWrite(left_motor_forward, speed);
  analogWrite(left_motor_reverse, 0);
  analogWrite(right_motor_forward, 0);
  analogWrite(right_motor_reverse, 0);
}

void stop() {  // stop
  analogWrite(left_motor_forward, 0);
  analogWrite(left_motor_reverse, 0);
  analogWrite(right_motor_forward, 0);
  analogWrite(right_motor_reverse, 0);
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);

    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);  // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                           : vi >= 10  ? 2
                                       : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len) {
  char sz[32] = "*";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (!d.isValid()) {
    Serial.print(F("********** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid()) {
    Serial.print(F("******** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len) {
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
  smartDelay(0);
}

/*Haversine Formula Function*/
// Calculate distance between two points
float getDistance(float lat, float lon, float init_lat, float init_lon) {
  // Variables
  float dist_calc = 0;
  float dist_calc2 = 0;
  float diflat = 0;
  float diflon = 0;

  // Calculations
  diflat = radians(init_lat - lat);
  lat = radians(lat);
  init_lat = radians(init_lat);
  diflon = radians((init_lon) - (lon));

  dist_calc = (sin(diflat / 2.0) * sin(diflat / 2.0));
  dist_calc2 = cos(lat);
  dist_calc2 *= cos(init_lat);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc += dist_calc2;

  dist_calc = (2 * atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc)));

  dist_calc *= 6371000.0;  //Converting to meters

  return dist_calc;
}

//Alerts On
void alert_on() {
  // LED Status
  analogWrite(red_led, 255);
  analogWrite(green_led, 0);
  //Buzzer tone
  analogWrite(buzzer, 255);
  delay(500);
  analogWrite(buzzer, 0);
  delay(500);
}
void alert_off() {
  digitalWrite(buzzer, LOW);
  analogWrite(red_led, 0);
  analogWrite(green_led, 255);
}
