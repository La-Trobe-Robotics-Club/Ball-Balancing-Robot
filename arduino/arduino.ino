class Encoder {
  public:
    Encoder(uint8_t port1_temp, uint8_t port2_temp) {
      pinMode(port1_temp, INPUT);
      pinMode(port2_temp, INPUT);
      current_port1 = digitalRead(port1_temp);
      current_port2 = digitalRead(port2_temp);
      last_port1 = digitalRead(port1_temp);
      last_port2 = digitalRead(port2_temp);
      port1 = port1_temp;
      port2 = port2_temp;
    }
    long read() {
      this->current_port1 = digitalRead(this->port1);
      this->current_port2 = digitalRead(this->port2);
      if (this->current_port1 != this->last_port1) {
        this->change_port1 = true;
        if (this->current_port2) {
          if (this->current_port1) {
            this->position += 1;
          } else {
            this->position -= 1;
          }
        } else {
        if (this->current_port1) {
            this->position -= 1;
          } else {
            this->position += 1;
          }
        }
      } 
      if (this->current_port2 != this->last_port2) {
        this->change_port2 = true;
        if (this->current_port1) {
          if (this->current_port2) {
            this->position -= 1;
          } else {
            this->position += 1;
          }
        } else {
        if (this->current_port2) {
            this->position += 1;
          } else {
            this->position -= 1;
          }
        }
      }
      if (this->change_port1 && this->change_port2) {
        while(true) {
          Serial.println("ERROR: Encoders didn't update in time (both states changed)");
          // turn_off_all_motors(); // TODO
          delay(500);
        }
      }
      this->last_port1 = this->current_port1;
      this->last_port2 = this->current_port2;
      this->change_port1 = false;
      this->change_port2 = false;
      return this->position;
    }
  private:
    uint8_t port1;
    uint8_t port2;
    bool current_port1;
    bool current_port2;
    bool last_port1;
    bool last_port2;
    bool change_port1;
    bool change_port2;
    long position = 0;
};

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
      long speed = this->pid.output((float)(this->target-this->position));
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
    long get_position() {
      return this->position;
    }
    long get_target() {
      return this->target;
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

Motor center(5,4,12,A2,A3,10.0,0.0,0.0);
Motor left  (6,7,13,A4,A5,10.0,0.0,0.0);
Motor right (3,2, 8,A0,A1,10.0,0.0,0.0);

unsigned long update_start = 0;
//unsigned long update_finish = 0;
unsigned long max_time = 400; // any longer and it can misread position-
unsigned long time_taken = 0;
// Has to be run 2480 times a second to make sure no encoders are missed
// returns true if error
bool update_all_positions() {
  time_taken = micros() - update_start /*+ update_finish - update_start*/;
  // check it hasn't been too long since encoders were read
  if (time_taken > max_time) {
    return true;
  }
  update_start = micros();
  center.update_position();
  left  .update_position();
  right .update_position();
  //update_finish = micros();
  return false;
}

void turn_off_all_motors() {
  center.off();
  left  .off();
  right .off();
}

uint8_t motor_scalar = 12; //23945.84/8/255 45 degrees
void set_motor_targets(struct motor_targets *targets) {
  center.set_target(int(targets->center * motor_scalar));
  left  .set_target(int(targets->left * motor_scalar));
  right .set_target(int(targets->right * motor_scalar));
}

void check_encoder_time() {
  if (update_all_positions()) {
    while(true) {
      turn_off_all_motors();
      Serial.println("");
      Serial.print("ERROR: Too long to read encoders, time: ");
      Serial.println(time_taken);
      Serial.print("Positions: ");
      Serial.print(center.get_position());
      Serial.print(" ");
      Serial.print(left.get_position());
      Serial.print(" ");
      Serial.println(right.get_position());
      delay(500);
    }
  }
}

class SerialCustom {
  public:
    void setup() {
      Serial.begin(9600);
    }
    void read(struct motor_targets *targets) {
      if (Serial.availableForWrite() == 63 && Serial.available() == 1) {
        int serial = Serial.read();
        // TODO: Check if read returns -1
        Serial.write(serial);
        switch(serial_count) {
          case 0:
            targets->center = (uint8_t)serial;
            serial_count = 1;
            break;
          case 1:
            targets->left = (uint8_t)serial;
            serial_count = 2;
            break;
          case 2:
            targets->right = (uint8_t)serial;
            serial_count = 0;
            break;
          default:
            while(true) {
              Serial.println("ERROR: serial_count not in bounds");
              turn_off_all_motors();
              delay(500);
            }
            break;
        }
      }
      if (Serial.available() > 1) {
        while(true) {
              Serial.println("ERROR: Too much serial data recieved");
              turn_off_all_motors();
              delay(500);
            }
      }
    }
  private:
    uint8_t serial_count;
};

SerialCustom serial;

void setup() {
  serial.setup();
  update_start = micros();
  //update_finish = micros();
  Serial.println("STARTED");
}

struct motor_targets targets;
struct motor_targets* targets_pointer = &targets;
int ran = 0;
void loop() {
//  check_encoder_time();
//  if (Serial.availableForWrite() == 63) {
//    Serial.print("Positions: ");
//    check_encoder_time();
//    Serial.print(center.get_position());
//    check_encoder_time();
//    Serial.print(" ");
//    check_encoder_time();
//    Serial.print(left.get_position());
//    check_encoder_time();
//    Serial.print(" ");
//    check_encoder_time();
//    Serial.print(right.get_position());
//    check_encoder_time();
//    Serial.print(" Targets: ");
//    check_encoder_time();
//    Serial.print(center.get_target());
//    check_encoder_time();
//    Serial.print(" ");
//    check_encoder_time();
//    Serial.print(left.get_target());
//    check_encoder_time();
//    Serial.print(" ");
//    check_encoder_time();
//    Serial.println(right.get_target());
//  }
  check_encoder_time();
  serial.read(targets_pointer);
//  check_encoder_time();
//  int test = 23;
//  if (Serial.availableForWrite() == 63 && ran < 2) {
//    Serial.write(test);
//    ran += 1;
//  }
//  targets_pointer->center = 0;
//  targets_pointer->left = 0;
//  targets_pointer->right = 0;
  check_encoder_time();
  set_motor_targets(targets_pointer);
  check_encoder_time();
  center.drive();
  check_encoder_time();
  left.  drive();
  check_encoder_time();
  right. drive();
}
