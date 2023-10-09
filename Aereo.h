#ifndef FLYCONTROLLER_AEREO_H
#define FLYCONTROLLER_AEREO_H

#include "MPU6050_light.h"
#include "Wire.h"
#include "Servo.h"
#include <math.h>

// Configurazioni dei servo
#define SERVO_DX_PIN 6
#define SERVO_SX_PIN 7
#define SERVO_CARGO_PIN 8
#define SERVO_PARACHUTE_PIN 9
#define MIN_SERVO_PULSE_WIDTH 544
#define MAX_SERVO_PULSE_WIDTH 2400
#define CARGO_OPEN_VAL 90
#define CARGO_CLOSED_VAL 0
#define PARACHUTE_OPEN_VAL 90
#define PARACHUTE_CLOSED_VAL 0
#define MAX_ANGLE_FLAP 90
#define MIN_ANGLE_FLAP (-90)
#define SERVO_MOV_UNIT 1
#define RECHARGE_TIME 5000 // Tempo per caricare i paracaduti

// Limiti strutturali
#define MIN_BATTERY_VOLTAGE 610 // Valore dell'ingresso analogico da non superare
#define BATTERY_VOLTAGE_ABORT_MISSION 740 // Tensione della batteria a cui l'aereo decide di tornare a casa
#define MAX_BATTERY_VOLTAGE 840

// Ingresso della tensione della batteria
#define BATTERY_VOLTAGE_INPUT A0

// Configurazioni del motore
#define MOTOR_COMMAND_PIN 2
#define MIN_PERC_MOTOR 0
#define MAX_PERC_MOTOR 100

// Configurazioni dei movimenti
#define LAUNCH_ACC_THRESHOLD 1000
#define LIN_ACC_ERROR 10
#define ANG_VEL_ERROR 1
#define ANG_POS_ERROR 1
#define VEL_ERROR 1
#define STD_VEL 25
//todo: trovare sperimentalmente questi valori

/*
 * Di quanto devo inclinare i flap per modificare la velocità angolare di uno
 */
#define DEG_FOR_ANG_VEL_X 1.0
#define DEG_FOR_ANG_VEL_Y 1.0
#define PERC_FOR_LIN_ACC_X 1.0

// Configurazioni MPU
#define MPU_ACC_CONFIG 0
#define MPU_GYRO_CONFIG 1
#define ORIENT 1

/*
 * NOTAZIONI UNITà DI MISURA MPU
 * - ACCELERAZIONE LINEARE: MULTIPLI DI 9.81 M/S^2 (G)
 * - VELOCITà ANGOLARE: GRADI/S
 * - POSIZIONI ANGOLARI: GRADI
 */

class Aereo:private MPU6050 {
private:
    void updateMotor();
    void updateFlaps();
    MPU6050 mpu;

    // Funzioni di confronto tra vettori
    static void vettoreDifferenza(const float vReal[3], const float vIdeal[3], float vDiff[3]);
    static float moduloVettore(const float v[3]);
    static float prodottoScalare(const float v1[3], const float v2[3]);
    static bool isSimilar(const float real[3], float angDiff[3], const float *ideal = nullptr, float error = ANG_POS_ERROR);
    bool correctAngVel(const float real[3], float angDiff[3], const float *ideal = nullptr, float error = ANG_VEL_ERROR);
    bool correctLinAcc(const float real[3], const float *ideal = nullptr, float error = LIN_ACC_ERROR);

    int angle_flap_dx, angle_flap_sx, perc_motor, dest_no;
    Servo servo_dx, servo_sx, cargo, parachute; // Oggetti Servo per il controllo dei movimenti

    // Matrice contenente le configurazioni ideali per i movimenti
    const float ideal_var[7][3][3] = {
            { // STRAIGHT
                    {0,   0,   }, // Posizione da raggiungere
                    {5,   5,   INFINITY}, // Velocità angolare da usare per raggiungere la posizione
                    {10,  0,   0} // Accelerazione lineare da usare per la velocità
            },
            { // UP
                    {0,   30,  INFINITY},
                    {0,   10,  INFINITY},
                    {0,    0,  0}
            },
            { // DOWN
                    {0,   -30, INFINITY}, // Posizione da raggiungere
                    {0,   -10, INFINITY}, // Velocità angolare da usare per raggiungere la posizione
                    {0,     0, 0} // Accelerazione da imporre
            },
            { // RIGHT 1° step
                    {89,  0,   INFINITY}, // Posizione da raggiungere
                    {45,  0,   INFINITY}, // Con quale velocità raggiungerla
                    {0, INFINITY, 0} // Accelerazione solo su Y, perché sto cadendo/salendo in modo imprevedibile ma sono in moto rettilineo
            }, // Una volta raggiunta la posizione, come devo gestire l'accelerazione
            { // RIGHT 2° step
                    {89,  90,  INFINITY},
                    {0,   45,  INFINITY},
                    {0, INFINITY, 10} // Accelerazione anche su Z, così posso impostare il raggio in base all'accelerazione centripeta impostata qui
            },
            { // LEFT 1° step
                    {-89, 0,   INFINITY},
                    {-45, 0,   INFINITY},
                    {0, INFINITY, 0} // Accelerazione relativa all'aereo
            },
            { // LEFT 2° step
                    {-89, 90,  INFINITY},
                    {0,   45,  INFINITY},
                    {0, INFINITY, 10} // Accelerazione relativa all'aereo
            }
    };

public:
    explicit Aereo(TwoWire &wire); // Costruttore della classe
    bool init(); // Inizializza i servo e l'accelerometro
    bool launchDetected();
    void moveGate(bool open);
    void emergencyStop();
    void moveS(uint8_t velReal, uint8_t velIdeal = STD_VEL);
    void moveUD(bool up);
    void moveRL(bool right);
    int8_t checkBatteryVoltage();

};

#endif // FLYCONTROLLER_AEREO_H
