/*
@author: Joe Stauttener
- Let's test LEDs one-by-one. Green seems to have issues.
*/

int c;

void testsetup() {
  LedSign::Init();
  c = 2;
  x = 0;
  y = 0;
}

void testloop() {
  LedSign::Clear();
  LedSign::Set(x, y, c);
  
  x++;
  if(x == 15) {
    x = 0;
    y++;
  }
  if(y == 8) {
    y = 0;
    c++;
  }
  if(c == 4) {
    c = 1;
  }
  delay(2000);
}
