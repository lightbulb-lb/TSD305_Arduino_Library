#include <tsd305.h>

tsd305 m_tsd305;

void setup() {
  Serial.begin(9600);

  Serial.println(F("==== TE Connectivity ===="));
  Serial.println(F("==== TSD305 ====="));
  Serial.println(F("==== Measure ===="));

  m_tsd305.begin();
}

void loop() {
  tsd305_status status;
  float temperature;
  float object_temperature;
  bool connected;

  connected = m_tsd305.is_connected();
  if ( connected ) {
    Serial.println( connected ? "Sensor Connected" : "Sensor Disconnected" );

    status = m_tsd305.read_temperature_and_object_temperature( &temperature, &object_temperature );
    Serial.print(F("---Temperature = "));
    Serial.print(temperature, 1);
    Serial.print((char)176);
    Serial.println(F("C"));

    Serial.print(F("---Object Temperature = "));
    Serial.print(object_temperature, 1);
    Serial.print((char)176);
    Serial.println(F("C"));
    Serial.println();
  }
  else {
    Serial.println(connected ? "Serial Connected" : "Sensor Disconnected");
  }
  delay(1000);
}
