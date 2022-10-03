const int dirPin = 5;
const int stepPin = 2;

const int steps = 200;
uint32_t stepDelay;

void setup() {
   // Marcar los pines como salida
   pinMode(dirPin, OUTPUT);
   pinMode(stepPin, OUTPUT);
}

void loop() {
   //Activar una direccion y fijar la velocidad con stepDelay
   digitalWrite(dirPin, HIGH);
   stepDelay = 10000;
   // Giramos 200 pulsos para hacer una vuelta completa
   for (int x = 0; x < steps * 1; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepDelay);
   }
   delay(1000);

   //Cambiamos la direccion y aumentamos la velocidad
   digitalWrite(dirPin, LOW);
   stepDelay = 1000;
   // Giramos 400 pulsos para hacer dos vueltas completas
   for (int x = 0; x < steps; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepDelay);
   }
   delay(1000);
   //Cambiamos la direccion y aumentamos la velocidad
   digitalWrite(dirPin, LOW);
   stepDelay = 100000;
   // Giramos 400 pulsos para hacer dos vueltas completas
   for (int x = 0; x < steps; x++) {
      digitalWrite(stepPin, HIGH);
      delay(100);
      //delayMicroseconds(stepDelay);
      digitalWrite(stepPin, LOW);
      delay(100);
      //delayMicroseconds(stepDelay);
   }
   delay(1000);
}