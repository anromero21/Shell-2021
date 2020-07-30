#include <Arduino.h>

void setup() {
 
}

void loop() {
  chave();
}

void chave(){
  int lec = digitalRead(PUSH);

   if(PUSH == true){
    ledcWrite(A,2048);
    delay(500);
    Serial.println("Todo OK -> " + String(lec));
  }

  else{
    digitalWrite(LED,LOW);
  }

}