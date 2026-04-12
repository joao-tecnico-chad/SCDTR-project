#include <pico/unique_id.h>
void setup() {
  Serial.begin(115200);
}
void loop() {
  pico_unique_board_id_t id;
  pico_get_unique_board_id(&id);
  Serial.print("UID:");
  for (int i = 0; i < 8; i++) {
    if (id.id[i] < 0x10) Serial.print("0");
    Serial.print(id.id[i], HEX);
  }
  Serial.print(" key=0x");
  Serial.println(id.id[7], HEX);
  delay(1000);
}
