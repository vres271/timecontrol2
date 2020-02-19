//  ---------------- TC Start ------------------------

// Sensor 
#define sens1PIN 3
#define sens2PIN 5




void setup() {

	pinMode(sens1PIN, OUTPUT);
	pinMode(sens2PIN, OUTPUT);

	Serial.begin(9600);
	Serial.println(" ==== TC Test ==== ");

}

void loop() {
	delay(1000);
	digitalWrite(sens1PIN, HIGH);
	delay(1000);
	digitalWrite(sens1PIN, LOW);
	delay(3000);
	digitalWrite(sens2PIN, HIGH);
	delay(1000);
	digitalWrite(sens2PIN, LOW);
}

