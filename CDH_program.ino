#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_INA260.h>
#include <Adafruit_LSM6DS33.h>
int comando;
int estado = 0;
int count = 1;
int RXPin = 2;
int TXPin = 3;
int GPSBaud = 9600;
float latitud, longitud;
int satelites, altitud, hora, minuto, segundo;
//int senVol = A1; // pin del sensor analogico de voltaje
// Create a TinyGPS++ object
TinyGPSPlus gps;
// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);
//sensor de luz
int sensor_luz_pin = A0;
float rango_de_lectura = 1024;//3.3
float vr = 3.3;
//mpl presion altura temperatura
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
//Sensor de voltaje y corriente INA
Adafruit_INA260 ina260 = Adafruit_INA260();
float voltaje,corriente;
//IMU
const float g = 9.78, pi =3.14; //g en la CDMX
float acx = 0,
      acy = 0,
      acz = 0,
      tiltx = 0,
      tilty = 0,
      tiltz = 0;
Adafruit_LSM6DS33 lsm6ds33; 
void setup() {
  analogReference(EXTERNAL);
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
  //Sensor de voltaje INA
  while (!Serial) { delay(10); }

  //Serial.println("Adafruit INA260 Test");

  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  //Serial.println("Found INA260 chip");
//IMU
 //Serial.println("Adafruit LSM6DS33 test!");

  if (!lsm6ds33.begin_I2C()) {
    // if (!lsm6ds33.begin_SPI(LSM_CS)) {
    // if (!lsm6ds33.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("LSM6DS33 Found!");

  lsm6ds33.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds33.configInt2(false, true, false); // gyro DRDY on INT2// put your setup code here, to run once:
}
void loop() {
  //revisa el comando
   if(Serial.available()>0){
    comando = Serial.read();
    if(comando == 'Y'){
      estado = 1;
      } else {
      estado = 0;
      }
   }
   if (estado == 1){
     if (! baro.begin()) 
    {
      Serial.println("Couldnt find sensor");
      return;
     }
  while (gpsSerial.available() > 0)
      if (gps.encode(gpsSerial.read()))
        gpsInfo();   
  float pascals = baro.getPressure();
  float altmTemporal = baro.getAltitude();
  float tempC = baro.getTemperature();
  float valorDeCalibracion;
  if (count == 130){
     valorDeCalibracion = altmTemporal;
    }
  //float voltaje = (float)25*analogRead(senVol)/1023;
  int valor = analogRead(sensor_luz_pin);
  //sonsor de voltaje INA
  voltaje = ina260.readBusVoltage()/1000;
  corriente = ina260.readCurrent();
  //imu
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  acx = accel.acceleration.x / g; // valor en g
  acy = accel.acceleration.y / g;
  acz = accel.acceleration.z / g;
  // put your main code here, to run repeatedly:
  tiltx = atan(acx/(sqrt(pow(acy,2)+pow(acz,2))))*(180/pi);
  tilty = atan(acy/(sqrt(pow(acx,2)+pow(acz,2))))*(180/pi);
  tiltz = atan((sqrt(pow(acx,2)+pow(acy,2)))/acz)*(180/pi);
      //Se imprimen los valores 
      Serial.print(count);  Serial.print(',');
      if(count > 130){ //solo ese enviará hasta que el valor este calibrado
        Serial.print(altmTemporal - valorDeCalibracion, 1); Serial.print(',');
       } else {Serial.print(0); Serial.print(',');}
      Serial.print(pascals, 0); Serial.print(',');
      Serial.print(tempC, 1); Serial.print(',');
      Serial.print(voltaje); Serial.print(',');//voltaje
      delay(15);  
      Serial.print(corriente); Serial.print(',');
      delay(15);
      if (hora < 10) Serial.print(F("0")); 
      Serial.print(hora); Serial.print(":");
      if (minuto < 10) Serial.print(F("0"));
      Serial.print(minuto); Serial.print(":");
      if (segundo < 10) Serial.print(F("0"));
      Serial.print(segundo); Serial.print(','); 
      delay(15);  
      Serial.print(latitud, 5); Serial.print(',');
      delay(15);
      Serial.print(longitud, 5); Serial.print(',');
      delay(15);
      Serial.print(altitud, 1); Serial.print(',');
      delay(15);
      Serial.print(satelites); Serial.print(',');
      delay(15);
      Serial.print(tiltx, 1); Serial.print(',');
      Serial.print(tilty, 1); Serial.print(',');
      Serial.print(tiltz, 1); Serial.print(',');
      delay(15);  
      Serial.println(valorALux(valor)); 
  count ++;
  delay(500);
   }
}
 void gpsInfo(){
     if (gps.location.isValid())
          { 
            latitud = gps.location.lat();      
            longitud = gps.location.lng();
          } else {
            latitud = 0;
            longitud = 0;
            }
     if (gps.altitude.isValid())
           {
            altitud = gps.altitude.meters(); 
           } else{
            altitud = 0;
            }
     if (gps.time.isValid())
          {
            hora = gps.time.hour();
            minuto = gps.time.minute();
            segundo = gps.time.second();
           } else {
            hora = 0;
            minuto = 0;
            segundo = 0;
            }             
     if (gps.satellites.isValid())
          {
           satelites = gps.satellites.value();    
            } else {
              satelites = 0; 
              }
     }
float valorALux(int valorAn)
{
  float v = valorAn*vr/rango_de_lectura; //se obtiene el voltaje de salida del sensor
  float io=(v/68000)*1000000;// se obtiene la corriente de salida del sensro por ley de ohm en uA
  //se despeja I=10log(ev) [uA]; --> ev=10^(I/10) 
   return pow(10, io/10);// ev
  }
