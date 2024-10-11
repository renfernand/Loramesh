
#include "devconfig.h"
#include "display.h"

#if 0
void display_init(){
    //O estado do GPIO16 é utilizado para controlar o display OLED
    pinMode(36, OUTPUT);
    //Reseta as configurações do display OLED
    digitalWrite(36, LOW);
    delay(50);
    //Para o OLED permanecer ligado, o GPIO16 deve permanecer HIGH
    //Deve estar em HIGH antes de chamar o display.init() e fazer as demais configurações,
    //não inverta a ordem
    digitalWrite(36, HIGH);

    //Configurações do display
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_16);
    display.clear();
}


void showdisplay(String buf){

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, buf);
  display.display();
}


void DisplayShow(int value, char *pframe) {

  char buf[10];
  char buf1[10];

#if DISPLAY_ENABLE
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_16);

  #if ROUTER == 1
    Heltec.display->drawString(0, 0, "RT");
    sprintf(buf, "TX = %d ", value);  
    Heltec.display->drawString(0, 20, buf);

  #else

    Heltec.display->drawString(0, 0, "ED");
    sprintf(buf, "RX [%d] = %s", value,pframe);  
    Heltec.display->drawString(0, 20, buf);
  #endif
  
  sprintf(buf1, "RSSI = %d", LoRa.packetRssi());  
  Heltec.display->drawString(0, 40, buf1);

  Heltec.display->display();

#else
  
  #if ROUTER == 1
    Serial.print("TX");
    Serial.println(value);
  #else
    Serial.print("RX[ ");
    Serial.print(value);
    Serial.print("] = ");
    Serial.println(pframe);
  #endif

#endif

}
#endif