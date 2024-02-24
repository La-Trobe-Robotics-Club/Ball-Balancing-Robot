// There are not enough pins with interrupts to capture all the quadrature encoders
#define ENCODER_DO_NOT_USE_INTERRUPTS
// Encoder library using arduino IDE library manager
#include <Encoder.h>

class PID {
  public:
    PID(float kP_temp, float kI_temp, float kD_temp) {
      kP = kP_temp;
      kI = kI_temp;
      kD = kD_temp;
    }
    float output(float input) {
      float proportional = input * this-> kP;
      float integral = 0; //TODO
      float derivative = 0; //TODO
      float velocity = proportional + integral + derivative;
      return velocity;
    }
  private:
    float kP;
    float kI;
    float kD;
};

class Motor {
  public:
    Motor(uint8_t port_EN_temp, uint8_t port_PH_temp, uint8_t port_SLP_temp, uint8_t port_encoder1, uint8_t port_encoder2, float kP, float kI, float kD) : encoder(port_encoder1, port_encoder2), pid(kP, kI, kD) {
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
      digitalWrite(port_SLP, HIGH);
      long speed = this->pid.output((float)(target-position));
      if (speed >= 0) {
        digitalWrite(port_PH, LOW);
      }
      else {
        digitalWrite(port_PH, HIGH);
      }
      analogWrite(port_EN, speed);
    }
  private:
    uint8_t port_EN;
    uint8_t port_PH;
    uint8_t port_SLP;
    Encoder encoder;
    long target;
    long position;
    PID pid;
};

Motor center(5,4,12,A0,A1,1.0,0.0,0.0);
Motor left  (3,2, 8,A2,A3,1.0,0.0,0.0);
Motor right (6,7,13,A4,A5,1.0,0.0,0.0);

// Has to be run every 4ms or less for reliable read (.5ms max execution time) Maximum frequency of encoder change is just under 4.96ms if motor is at max speed
void update_all_positions() {
  center.update_position();
  left  .update_position();
  right .update_position();
}

void setup() {
  Serial.setTimeout(1000);
}

void loop() {
  update_all_positions();
  // size_t = Serial.readBytesUntil("|", byte, 3); TODO
  update_all_positions();
  center.drive();
  update_all_positions();
  left.  drive();
  update_all_positions();
  right. drive();
}
