#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>

class Motor {
  public:
    Motor(uint8_t port_EN_temp, uint8_t port_PH_temp, uint8_t port_SLP_temp, uint8_t port_encoder1, uint8_t port_encoder2) : encoder(port_encoder1, port_encoder2) {
      pinMode(port_EN_temp,  OUTPUT);
      pinMode(port_PH_temp,  OUTPUT);
      pinMode(port_SLP_temp, OUTPUT);
      port_EN  = port_EN_temp;
      port_PH  = port_PH_temp;
      port_SLP = port_SLP_temp;
    }
    void set_target(long target) {
      this->target = target;
    }
    void update_position() {
      this->position = this->encoder.read();
    }
    void drive() {
      int speed = 50;
      digitalWrite(port_SLP, HIGH);
      if (this->position > this->target) {
        analogWrite(port_EN, speed);
        digitalWrite(port_PH, HIGH);
      }
      else if (this->position < this->target) {
        analogWrite(port_EN, speed);
        digitalWrite(port_PH, LOW);
      }
      else {
        analogWrite(port_EN, 0);
      }
    }
  private:
    uint8_t port_EN;
    uint8_t port_PH;
    uint8_t port_SLP;
    Encoder encoder;
    long target;
    long position;
};

Motor center(5,4,12,A0,A1);
Motor left  (3,2,8,A2,A3);
Motor right (6,7,13,A4,A5);

void update_all_positions() {
  center.update_position();
  left  .update_position();
  right .update_position();
}

void setup() {}

void loop() {
  update_all_positions();
  center.drive();
  update_all_positions();
  left.  drive();
  update_all_positions();
  right. drive();
}
