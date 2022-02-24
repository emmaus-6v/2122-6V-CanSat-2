#define buzzerPin 10

const unsigned long eventInterval = 1000;
unsigned long previousTime = 0;

class Zoomer{
  public:
  int buzzerPin = 10;
  unsigned long currentTime;
  const unsigned long eventInterval = 1000;
  unsigned long previousTime = 0;

  void update(){
    if(
    
  }
  
}

void setup() {
  Serial.begin(9600);
  pinMode(buzzerPin, OUTPUT); // Declare the LED as an output
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - previousTime >= eventInterval) {
    tone(buzzerPin, 2000, 300);
    previousTime = currentTime;
  }

}
