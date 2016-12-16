#include <TinyGPS.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include<Wire.h>
#define RXPIN 2
#define TXPIN 3
#define GPSBAUD 4800

int input1=12,input2=13,input3=11,input4=10,enable1=9,enable2=7,x=A4,y=A5;
float myres;

TinyGPS gps;
SoftwareSerial uart_gps(RXPIN, TXPIN);

int cmp_adres = 0x21;
int cmps, phy; 
float destlat =12.8207948, destlong=80.047991;
float latitude, longitude;

void setup() 
{
  Serial.begin(9600); // Initiate Serial
  pinMode(input1,OUTPUT);
  pinMode(input2,OUTPUT);
  pinMode(input3,OUTPUT);
  pinMode(input4,OUTPUT);
  pinMode(x,INPUT);
  pinMode(y,INPUT);
  
 Wire.begin();
 uart_gps.begin(GPSBAUD);
 Serial.println("    ...waiting for lock...      ");
}

void loop() 
{ // GPS Reading
  while(uart_gps.available()) 
    {
      int c = uart_gps.read();
      if(gps.encode(c)) 
         {    
          gps.f_get_position(&latitude, &longitude);
          Serial.print("\nLat/Long: "); 
          Serial.print(latitude,5); 
          Serial.print(", "); 
          Serial.println(longitude,5);
         }
         if (latitude!=0.0000&&longitude!=0.0000)
         break;
         
     }

  // Compass Reading
  Wire.beginTransmission(cmp_adres);
  Wire.write('A');
  Wire.endTransmission();
  delay(10);
  Wire.requestFrom(cmp_adres,2);
    cmps = Wire.read();
    cmps = cmps << 8;
    cmps+=Wire.read();
    cmps/=10;
  //Serial.println("\nCompass: "cmps);

  if(destlat>latitude&&destlong>longitude)
    {
      phy = atan(destlong-longitude/destlat-latitude);
    }
  else if(destlat<latitude&&destlong>longitude)
    {
      phy = atan(destlat-latitude/destlong-longitude)+90;
    }
  else if(destlat<latitude&&destlong<longitude)
    {
      phy = atan(longitude-destlong/latitude-destlat)+180;
    }
  else
    {
      phy = atan(latitude-destlat/longitude-destlong)+270;
    }

    
    
    
    
    
    if((phy-cmps)<=180)
    {
      //clockwise
      for(myres=1;myres<=(phy-cmps);myres++)
          {
            Wire.beginTransmission(0x21);
            Wire.write("A");     
            delay(100);         
            Wire.requestFrom(0x21, 2); 
            byte MSB = Wire.read();
            byte LSB = Wire.read();
            Wire.endTransmission();
            myres = ((MSB << 8) + LSB) / 10; 
            Serial.print(myres);
            Serial.println(" degrees");
            delay(100);

            digitalWrite(enable1,HIGH);
            digitalWrite(enable2,HIGH);
 
     
            digitalWrite(input1,HIGH);
            digitalWrite(input2,HIGH);
            digitalWrite(input3,LOW);
            digitalWrite(input4,HIGH);
  }
  digitalWrite(input1,HIGH);
  digitalWrite(input2,HIGH);
  digitalWrite(input3,HIGH);
  digitalWrite(input4,HIGH);
  delay(15000);
      
  }
    
    else
   {
     //anticlockwise
     for(myres=1;myres<=(360-phy);myres++)
     {
       Wire.beginTransmission(0x21);
       Wire.write("A");     
        delay(100);         
       Wire.requestFrom(0x21, 2); 
       byte MSB = Wire.read();
       byte LSB = Wire.read();
       Wire.endTransmission();
       myres = ((MSB << 8) + LSB) / 10; 
       Serial.print(myres);
       Serial.println(" degrees");
       delay(100);


  
       digitalWrite(enable1,HIGH);
       digitalWrite(enable2,HIGH);
 
 
        digitalWrite(input1,LOW);
        digitalWrite(input2,HIGH);
        digitalWrite(input3,HIGH);
        digitalWrite(input4,HIGH);
       }
     digitalWrite(input1,HIGH);
     digitalWrite(input2,HIGH);
     digitalWrite(input3,HIGH);
     digitalWrite(input4,HIGH);
     delay(15000);
      
    }
     
    
  
     
     while(abs(destlong-longitude)!=0 && abs(destlat-latitude!=0))
     {
      digitalWrite(input1,LOW);
      digitalWrite(input2,HIGH);
      digitalWrite(input3,LOW);
      digitalWrite(input4,HIGH);
      delay(3000);
     }
    
  }
