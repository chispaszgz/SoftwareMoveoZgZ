// Pines de la RAMPS 1.4
#define M1_STEP_PIN         54    //X_STEP_PIN         54
#define M1_DIR_PIN          55    //X_DIR_PIN          55
#define M1_ENABLE_PIN       38    //X_ENABLE_PIN       38
#define M2_STEP_PIN         60    //Y_STEP_PIN         60
#define M2_DIR_PIN          61    //Y_DIR_PIN          61
#define M2_ENABLE_PIN       56    //Y_ENABLE_PIN       56
#define M3_STEP_PIN         46     //Z_STEP_PIN         46
#define M3_DIR_PIN          48     //Z_DIR_PIN          48
#define M3_ENABLE_PIN       62     //Z_ENABLE_PIN       62
#define M4_STEP_PIN         42     //AUX2               42
#define M4_DIR_PIN          40     //AUX2               40
#define M4_ENABLE_PIN       63     //AUX2             A9 63
#define M5_STEP_PIN         26     //E0_STEP_PIN         26
#define M5_DIR_PIN          28     //E0_DIR_PIN          28
#define M5_ENABLE_PIN       24     //E0_ENABLE_PIN       24
#define M6_STEP_PIN         36     //E1_STEP_PIN         36
#define M6_DIR_PIN          34     //E1_DIR_PIN          34
#define M6_ENABLE_PIN       30     //E1_ENABLE_PIN       30
#define SERVO_PIN           11     //SERVO 1
#define POT_PIN             57     //AUX1                A3
#define FC_HOME             3      //X_MIN_PIN          3

//#define X_MAX_PIN          2
//#define Y_MIN_PIN         14
//#define Y_MAX_PIN         15
//#define Z_MIN_PIN         18z
//#define Z_MAX_PIN         19
//#define SDPOWER            -1
//#define SDSS               53
//#define LED_PIN            13
//#define FAN_PIN            9
//#define PS_ON_PIN          12
//#define KILL_PIN           -1
//#define HEATER_0_PIN       10
//#define HEATER_1_PIN       8
//#define TEMP_0_PIN          13   // ANALOG NUMBERING
//#define TEMP_1_PIN          14   // ANALOG NUMBERING

#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

Servo pinza;            // Crea el objeto pinza del tipo servo
AccelStepper stepper1 (AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper stepper2 (AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);
AccelStepper stepper3 (AccelStepper::DRIVER, M3_STEP_PIN, M3_DIR_PIN);
AccelStepper stepper4 (AccelStepper::DRIVER, M4_STEP_PIN, M4_DIR_PIN);
AccelStepper stepper5 (AccelStepper::DRIVER, M5_STEP_PIN, M5_DIR_PIN);
AccelStepper stepper6 (AccelStepper::DRIVER, M6_STEP_PIN, M6_DIR_PIN);
// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

#define velocidadLenta 8000
#define aceleracionLenta 500
#define velocidadRapida 2000
#define aceleracionRapida 500
#define pinzaAbierta 80
#define pinzaMedia 160
#define pinzaCerrada 180

int valorAnalogico;           // variable to read the value from the analog pin
int pos = 100;
int limiteM1 = 3000;
int limiteM2 = 4200;         //correcta
int limiteM3 = 10400;
int limiteM4 = 4160;         // Correcta
int limiteM5 = 1840;         // posición X1  230  X8-1840   X16  3680
int limiteM6 = 800;          //posición X1-100   X8-800   X16-1600
int eje = 0;
int vel = 0;
char seleccion = "d";

//    EJE 4   medio giro 4150
//    EJE 1   4146  ----  0  ---------
//
//  pos1 ---->   M1: 988 M2: -2967 M3: -7657 M4: 0 M5: -749 M6: 1066
//  pos2 ----->
//  pos3 ---->


long casa[6] = {0, 0, 0, 0, 0, 0};
long pos1[6] = { 1000 , -2792 , -7657 , 0 , -749 , 1066};
long pos2[6] = { -1000 , -2792 , -7657 , 0 , -749 , 1066};
long pos3[6] = { 0 , -1400 , -4300 , 0 , -749 , 1066};
long posSubir1[6] = {0, -800, -500, 0, 0, 0};
long posSubir2[6] = {0, -4050, 9700, -4160, 1530, 0};
//long posHome[6] = {0,4050,-9700,4160,-1500,0};
long posAproxTrans[6] = {0, 3500, -10500, 4160, -1700, 0};
long posTransporte[6] = {0, 4050, -9700, 4160, -1500, 0};

void setup() {
  pinza.attach(SERVO_PIN);
  Serial.begin(9600);
  pinMode(FC_HOME, INPUT_PULLUP);

  stepper1.setEnablePin(M1_ENABLE_PIN);
  stepper1.setPinsInverted(false, false, true);
  stepper1.setMaxSpeed(velocidadRapida);
  stepper1.setAcceleration(aceleracionRapida);
  stepper1.enableOutputs();

  stepper2.setEnablePin(M2_ENABLE_PIN);
  stepper2.setPinsInverted(false, false, true);
  stepper2.setMaxSpeed(velocidadLenta);
  stepper2.setAcceleration(aceleracionLenta);
  stepper2.enableOutputs();

  stepper3.setEnablePin(M3_ENABLE_PIN);
  stepper3.setPinsInverted(false, false, true);
  stepper3.setMaxSpeed(velocidadRapida);
  stepper3.setAcceleration(aceleracionRapida);
  stepper3.enableOutputs();

  stepper4.setEnablePin(M4_ENABLE_PIN);
  stepper4.setPinsInverted(false, false, true);
  stepper4.setMaxSpeed(velocidadRapida);
  stepper4.setAcceleration(aceleracionRapida);
  stepper4.enableOutputs();

  stepper5.setEnablePin(M5_ENABLE_PIN);
  stepper5.setPinsInverted(false, false, true);
  stepper5.setMaxSpeed(velocidadLenta);
  stepper5.setAcceleration(aceleracionLenta);
  stepper5.enableOutputs();

  stepper6.setEnablePin(M6_ENABLE_PIN);
  stepper6.setPinsInverted(false, false, true);
  stepper6.setMaxSpeed(velocidadRapida);
  stepper6.setAcceleration(aceleracionRapida);
  stepper6.enableOutputs();

  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);
  steppers.addStepper(stepper4);
  steppers.addStepper(stepper5);
  steppers.addStepper(stepper6);

  if (!digitalRead(FC_HOME)) {
    steppers.moveTo(posSubir1);
    steppers.runSpeedToPosition(); // Blocks until all are in position
    steppers.moveTo(posSubir2);
    steppers.runSpeedToPosition(); // Blocks until all are in position
    delay(500);
    pinza.write(pinzaAbierta);
    delay(500);
    pinza.write(pinzaCerrada);
    delay(500);
    pinza.write(pinzaMedia);
    delay(500);
  }
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);
  stepper5.setCurrentPosition(0);
  stepper6.setCurrentPosition(0);
  // delay (5000);

}

void loop() {
  if (seleccion == "d") {
    demo();
  } else {
    moverMotor (eje);
  }

  //  potenciometroPinza ();
  //  potenciometroStepper ();
  //  buscarLimites3 ();
  //  demo5();
  //  demo2();
  //  Serial.println(digitalRead(FC_HOME));

  if (Serial.available() > 0) {
    //leemos la opcion enviada
    char opcion = Serial.read();
    if (opcion == '0') {
      eje = 0;
      Serial.println("Ningun motor seleccionado");
    } else if (opcion == '1') {
      eje = 1;
      Serial.println("Motor 1 seleccionado");
    } else if (opcion == '2') {
      eje = 2;
      Serial.println("Motor 2 seleccionado");
    } else if (opcion == '3') {
      eje = 3;
      Serial.println("Motor 3 seleccionado");
    } else if (opcion == '4') {
      eje = 4;
      Serial.println("Motor 4 seleccionado");
    } else if (opcion == '5') {
      eje = 5;
      Serial.println("Motor 5 seleccionado");
    } else if (opcion == '6') {
      eje = 6;
      Serial.println("Motor 6 seleccionado");
    } else if (opcion == '7') {
      eje = 7;
      Serial.println("Pinza seleccionada");
    } else if (opcion == 'd') {
      seleccion = "d";
    } else if (opcion == 'p') {
      seleccion = "p";

      /*    }else if (opcion == 't') {
            steppers.moveTo(posAproxTrans);
            steppers.runSpeedToPosition();
            steppers.moveTo(posTransporte);
            steppers.runSpeedToPosition();
            stepper1.setCurrentPosition(0);
            stepper2.setCurrentPosition(0);
            stepper3.setCurrentPosition(0);
            stepper4.setCurrentPosition(0);
            stepper5.setCurrentPosition(0);
            stepper6.setCurrentPosition(0);
          }else if (opcion == 'h') {
            steppers.moveTo(posSubir1);
            steppers.runSpeedToPosition(); // Blocks until all are in position
            steppers.moveTo(posSubir2);
            steppers.runSpeedToPosition(); // Blocks until all are in position
            stepper1.setCurrentPosition(0);
            stepper2.setCurrentPosition(0);
            stepper3.setCurrentPosition(0);
            stepper4.setCurrentPosition(0);
            stepper5.setCurrentPosition(0);
            stepper6.setCurrentPosition(0);
      */
    } else if (opcion == 'm') {                   // Manda por puerto serie las posiciones de todos los ejes
      String posicion = "M1: ";
      posicion += (stepper1.currentPosition());
      posicion += " M2: ";
      posicion += (stepper2.currentPosition());
      posicion += " M3: ";
      posicion += (stepper3.currentPosition());
      posicion += " M4: ";
      posicion += (stepper4.currentPosition());
      posicion += " M5: ";
      posicion += (stepper5.currentPosition());
      posicion += " M6: ";
      posicion += (stepper6.currentPosition());
      posicion += " Pinza: ";
      posicion += (pinza.read());
      Serial.println(posicion);
    } else if (opcion == 'c') {
      steppers.moveTo(casa);
      steppers.runSpeedToPosition(); // Blocks until all are in position
    } else if (opcion == 'q') {
      steppers.moveTo(pos1);
      steppers.runSpeedToPosition(); // Blocks until all are in position
    } else if (opcion == 'w') {
      steppers.moveTo(pos3);
      steppers.runSpeedToPosition(); // Blocks until all are in position
    } else if (opcion == 'e') {
      steppers.moveTo(pos2);
      steppers.runSpeedToPosition(); // Blocks until all are in position
    } else if (opcion == 'a') {
      pinza.write(pinzaAbierta);
      delay(500);
    } else if (opcion == 's') {
      pinza.write(pinzaMedia);
      delay(500);
    } else if (opcion == 'z') {
      pinza.write(pinzaCerrada);
      delay(500);
    }
  }
}

void moverMotor (int m) {
  valorAnalogico = analogRead(POT_PIN);
  if (valorAnalogico < 200) {
    vel = -1000;
  } else if (valorAnalogico < 400) {
    vel = -250;
  } else if (valorAnalogico > 800) {
    vel = 1000;
  } else if (valorAnalogico > 600) {
    vel = 250;
  } else {
    vel = 0;
  }
  if (m == 1) {                                     // control de eje 1
    stepper1.setSpeed(vel);
    stepper1.runSpeed();
  } else if (m == 2) {                              // control de eje 2
    stepper2.setSpeed(vel);
    stepper2.runSpeed();
  } else if (m == 3) {                              // control de eje 3
    stepper3.setSpeed(vel);
    stepper3.runSpeed();
  } else if (m == 4) {                              // control de eje 4
    stepper4.setSpeed(vel);
    stepper4.runSpeed();
  } else if (m == 5) {                              // control de eje 5
    stepper5.setSpeed(vel);
    stepper5.runSpeed();
  } else if (m == 6) {                              // control de eje 6
    stepper6.setSpeed(vel);
    stepper6.runSpeed();
  } else if (m == 7) {                              // control de pinza
    int incremento = map(vel, -1000, 1000, -20, 20);
    int posicion = pinza.read() + incremento ;
    posicion = constrain(posicion, pinzaAbierta, pinzaCerrada);
    pinza.write(posicion);
  }
}

void potenciometroPinza () {
  valorAnalogico = analogRead(POT_PIN);                       // reads the value of the potentiometer (value between 0 and 1023)
  valorAnalogico = map(valorAnalogico, 0, 1023, 80, 180);     // scale it to use it with the servo (value between 0 and 180)
  pinza.write(valorAnalogico);                                // sets the servo position according to the scaled value
  Serial.println (valorAnalogico);
  delay(100);
}

void demo1 () {
  if (stepper1.distanceToGo() == 0)
  {
    limiteM1 = -limiteM1;
    stepper1.moveTo(limiteM1);
    Serial.println (limiteM1);
  }
  stepper1.run();
}
void demo2 () {
  if (stepper2.distanceToGo() == 0)
  {
    limiteM2 = -limiteM2;
    stepper2.moveTo(limiteM2);
    Serial.println (limiteM2);
  }
  stepper2.run();
}
void demo3 () {
  if (stepper3.distanceToGo() == 0)
  {
    limiteM3 = -limiteM3;
    stepper3.moveTo(limiteM3);
    Serial.println (limiteM3);
  }
  stepper3.run();
}
void demo4 () {
  if (stepper4.distanceToGo() == 0)
  {
    limiteM4 = -limiteM4;
    stepper4.moveTo(limiteM4);
    Serial.println (limiteM4);
  }
  stepper4.run();
}
void demo5 () {
  if (stepper5.distanceToGo() == 0)
  {
    limiteM5 = -limiteM5;
    stepper5.moveTo(limiteM5);
    Serial.println (limiteM5);
  }
  stepper5.run();
}
void demo6 () {
  if (stepper6.distanceToGo() == 0)
  {
    limiteM6 = -limiteM6;
    stepper6.moveTo(limiteM6);
    Serial.println (limiteM6);
  }
  stepper6.run();
}

void demo () {
  pinza.write(pinzaCerrada);
  delay(1000);
  pinza.write(pinzaAbierta);
  delay(1000);
  steppers.moveTo(pos1);          // Carga posición 1
  steppers.runSpeedToPosition();  // Busca posición 1
  pinza.write(pinzaMedia);        // cierra pinza
  delay(500);
  steppers.moveTo(pos3);
  steppers.runSpeedToPosition();
  steppers.moveTo(pos2);
  steppers.runSpeedToPosition();
  pinza.write(pinzaAbierta);
  delay(500);
  steppers.moveTo(casa);
  steppers.runSpeedToPosition();
  pinza.write(pinzaCerrada);
  delay(1000);
  pinza.write(pinzaAbierta);
  delay(1000);
  steppers.moveTo(pos2);          // Carga posición 1
  steppers.runSpeedToPosition();  // Busca posición 1
  pinza.write(pinzaMedia);        // cierra pinza
  delay(500);
  steppers.moveTo(pos3);
  steppers.runSpeedToPosition();
  steppers.moveTo(pos1);
  steppers.runSpeedToPosition();
  pinza.write(pinzaAbierta);
  delay(500);
  steppers.moveTo(casa);
  steppers.runSpeedToPosition();
}
