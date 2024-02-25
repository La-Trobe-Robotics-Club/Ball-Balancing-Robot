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
    void off() {
      analogWrite(port_EN, 0);
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

struct motor_targets {
  uint8_t center = 0;
  uint8_t left = 0;
  uint8_t right = 0;
};

Motor center(5,4,12,A0,A1,1.0,0.0,0.0);
Motor left  (3,2, 8,A2,A3,1.0,0.0,0.0);
Motor right (6,7,13,A4,A5,1.0,0.0,0.0);

unsigned long update_start = 0;
unsigned long update_finish = 0;
unsigned long max_time = 4000;
unsigned long time_taken = 0;
// Has to be run every 4ms or less for reliable read (.5ms max execution time) Maximum frequency of encoder change is just under 4.96ms if motor is at max speed
// returns true if error
bool update_all_positions() {
  time_taken = micros() - update_start + micros() - update_finish;
  // check it hasn't been too long since encoders were read
  if (time_taken > max_time) {
    return true;
  }
  update_start = micros();
  center.update_position();
  left  .update_position();
  right .update_position();
  update_finish = micros();
}

void turn_off_all_motors() {
  center.off();
  left  .off();
  right .off();
}

void set_motor_targets(struct motor_targets *targets) {
  center.set_target(targets->center);
  left  .set_target(targets->left);
  right .set_target(targets->right);
}

void check_encoder_time() {
  if (update_all_positions()) {
    while(true) {
      Serial.println("ERROR: Too long to read encoders");
      turn_off_all_motors();
    }
  }
}

class SerialCustom {
  public:
    void setup() {
      Serial.begin(9600);
    }
    void read(struct motor_targets *targets) {
        uint8_t serial = Serial.read();
        Serial.write(serial);
        switch(serial_count) {
          case 0:
            targets->center = serial;
            serial_count = 1;
            break;
          case 1:
            targets->left = serial;
            serial_count = 2;
            break;
          case 2:
            targets->right = serial;
            serial_count = 0;
            break;
          default:
            while(true) {
              Serial.println("ERROR: serial_count not in bounds");
              turn_off_all_motors();
            break;
            }
            
        }
    }
  private:
    uint8_t serial_count;
};

SerialCustom serial;

void setup() {
  serial.setup();
}

struct motor_targets targets;
struct motor_targets* targets_pointer = &targets;
void loop() {
  check_encoder_time();
  serial.read(targets_pointer);
  check_encoder_time();
  set_motor_targets(targets_pointer);
  check_encoder_time();
  center.drive();
  check_encoder_time();
  left.  drive();
  check_encoder_time();
  right. drive();
}
