/*
@author Joe Stauttener 
- Modify the LOL Shield examples to work with the SoOnCon 2011 badge. 
- Combine all in a program and hook up a button to switch between them.
*/

#include <CharliplexingBadge.h>
#define BTNPIN 1 // connect button to this pin (connected like http://www.arduino.cc/en/Tutorial/Button)
unsigned long lastBtnMillis = 0; //debounce var

int program = 0;

void setup() {
  pinMode(BTNPIN, INPUT);
  LedSign::Init();            //Initilizes the LoL Shield
}

void loop() {
  if(lastBtnMillis + 1000 < millis() && digitalRead(BTNPIN) == HIGH) {
    lastBtnMillis = millis();
    program++;
    program = program % 4;
    switch (program) {
      case 0:
        lifesetup();
        break;
      case 1:
        heartsetup();
        break;
      case 2:
        pongsetup();
        break;
      case 3:
        testsetup();
        break;
      default:
        lifesetup();
    }
  }
  switch (program) {
    case 0:
      lifeloop();
      break;
    case 1:
      heartloop();
      break;
    case 2:
      pongloop();
      break;
    case 3:
      testloop();
      break;
    default:
      lifeloop();
  }
}
