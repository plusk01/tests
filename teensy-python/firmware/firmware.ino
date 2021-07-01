void setup() {
    Serial.begin(9600);
}

//
// Example 1 - more robust because we wait for the '\n' sentinel
//

void loop() {
    static char buffer[32];
    static size_t pos;
    if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {  // on end of line, parse the number
            buffer[pos] = '\0';
            float value = atof(buffer);
            float a, b;
            int numValues = sscanf(buffer, "%f,%f", &a, &b);
            if (numValues == 2) {
              char buf[100];
              snprintf(buf, 100, "received a = %f, b = %f", a, b);
              Serial.println(buf);
            } else {
              // error parsing packet
            }
            pos = 0;
        } else if (pos < sizeof buffer - 1) {  // otherwise, buffer it
            buffer[pos++] = c;
        }
    }
}

//
// Example 2 - more compact, but less robust because no sentinel byte
//

//union Data {
//  float values[2];
//  char buffer[2 * sizeof(float)];
//};
//
//void loop() {
//    static Data data;
//    static size_t pos;
//    
//    if (Serial.available()) {
//        char c = Serial.read();
//        data.buffer[pos++] = c;
//        if (pos == sizeof data.buffer) {
//            Serial.print("received: a = ");
//            Serial.print(data.values[0]);
//            Serial.print(" b = ");
//            Serial.print(data.values[1]);
//            Serial.print("\tpos = ");
//            Serial.println(pos);
//            pos = 0;
//        }
//    }
//}
