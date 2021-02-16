#include "Gamer.h"

Gamer gamer;
char command[50];
int i = 0;

void setup() {
  gamer.begin();
  Serial.begin(9600);
//  Serial.begin(115200);
}

void loop() {
  gamer.clear();
  while (Serial.available() > 0 && i < 49) {
      command[i] = Serial.read();
      i++;
  }
  command[i] = '\0';

  gamer.printString(command);
  gamer.updateDisplay();
  Serial.print("get the message");
  delay(100);
  i = 0;
}
