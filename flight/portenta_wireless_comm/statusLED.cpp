#include "statusLED.h"

void statusLED::init(){
  pinMode(rLED,OUTPUT);
  pinMode(gLED,OUTPUT);
  pinMode(bLED,OUTPUT);
}

void statusLED::connectingWiFi(){
  digitalWrite(rLED, HIGH); // glow blue
  digitalWrite(bLED, LOW);
  digitalWrite(gLED,HIGH);
}

void statusLED::connectingServer(){
  digitalWrite(rLED, LOW); // glow red
  digitalWrite(bLED, HIGH);
  digitalWrite(gLED,HIGH);
}

void statusLED::dataTransfer(){
  digitalWrite(rLED, HIGH); // glow green
  digitalWrite(bLED, HIGH);
  digitalWrite(gLED,LOW);
}
