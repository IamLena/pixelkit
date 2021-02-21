#include "Gamer.h"

Gamer gamer;
char command[50];
int i;

void setup() {
  gamer.begin();
  Serial.begin(115200);
  i = 0;
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
  if (i != 0)
    Serial.print("I've got a message.\n");
  delay(100);
  i = 0;
}
