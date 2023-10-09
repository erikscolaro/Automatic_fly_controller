
#include "Aereo.h"

// Costruttore della classe Aereo
Aereo::Aereo(TwoWire &wire) : MPU6050(wire), mpu(wire) {
    angle_flap_dx = 0;
    angle_flap_sx = 0;
    perc_motor = 0;
    dest_no = 0;
}

// Inizializza i componenti dell'aereo (servo e accelerometro)
bool Aereo::init() {

    // Inizializzazione dell'oggetto MPU6050 per l'accelerometro e il giroscopio
    char status = mpu.begin(MPU_GYRO_CONFIG, MPU_ACC_CONFIG); // sarebbe byte ma gg

    // Inizializzazione dai servos per i movimenti dell'aereo
    servo_dx.attach(SERVO_DX_PIN, MIN_SERVO_PULSE_WIDTH, MAX_SERVO_PULSE_WIDTH);
    servo_sx.attach(SERVO_SX_PIN, MIN_SERVO_PULSE_WIDTH, MAX_SERVO_PULSE_WIDTH);
    cargo.attach(SERVO_CARGO_PIN, MIN_SERVO_PULSE_WIDTH, MAX_SERVO_PULSE_WIDTH);
    parachute.attach(SERVO_PARACHUTE_PIN, MIN_SERVO_PULSE_WIDTH, MAX_SERVO_PULSE_WIDTH);

    // Verifica che tutti i servo siano collegati correttamente e che l'accelerometro sia stato inizializzato con successo
    if (!servo_dx.attached() || !servo_sx.attached() || !cargo.attached() || !parachute.attached() || status != 0) {
        // Se qualche componente è mancante o l'accelerometro non è stato inizializzato correttamente,
        // impostare gli angoli dei flap ai valori massimi e chiudere il cargo e il paracadute.
        angle_flap_dx = MAX_ANGLE_FLAP;
        angle_flap_sx = MAX_ANGLE_FLAP;
        updateFlaps();
        return false;
    }

    // Sequenza di comunicazione avvenuto inizio
    cargo.write(CARGO_OPEN_VAL);
    parachute.write(PARACHUTE_OPEN_VAL);
    // delay(RECHARGE_TIME); // do tempo per ricaricare il cargo
    cargo.write(CARGO_CLOSED_VAL);
    parachute.write(PARACHUTE_CLOSED_VAL);
    // delay(2000):

    // Calibrazione degli offset dell'accelerometro e del giroscopio
    mpu.calcOffsets(true, true);
    mpu.setFilterGyroCoef(0.98);

    // Posizionamento iniziale dei flap per prendere quota
    angle_flap_dx = MIN_ANGLE_FLAP;
    angle_flap_sx = MIN_ANGLE_FLAP;
    updateFlaps();
    // delay(500);

    // Test del motore
    pinMode(MOTOR_COMMAND_PIN, OUTPUT);
    perc_motor = 100; // Accelera il motore al massimo
    updateMotor();
    // delay(1000);
    perc_motor = 0; // Ferma il motore
    updateMotor();

    return true;
}

// Verifica se è stata rilevata una situazione di lancio (accelerazione superiore alla soglia)
bool Aereo::launchDetected() {
    // Aggiorna i dati di accelerazione e giroscopio dall'MPU
    mpu.update();

    // Piccolo ritardo per consentire al modulo di acquisire i dati
    delay(10);

    float acc[3]={mpu.getAccX(), mpu.getAccY(), mpu.getAccZ()};

    // Verifica se l'accelerazione supera la soglia di lancio
    if (moduloVettore(acc) > LAUNCH_ACC_THRESHOLD) {
        return true; // Lancio rilevato
    }
    return false; // Nessun lancio rilevato
}

// Calcola la differenza vIdeal-vReal e la memorizza in vDiff
void Aereo::vettoreDifferenza(const float *vReal, const float *vIdeal, float *vDiff) {
    for (uint8_t i = 0; i < 3; i++) {
        if (vIdeal[i] != INFINITY) {
            vDiff[i] = vIdeal[i] - vReal[i];
        } else {
            vDiff[i] = 0;
        }
    }
}

// Calcola il modulo di un vettore v
float Aereo::moduloVettore(const float *v) {
    return sqrt(prodottoScalare(v, v));
}

// Calcola il prodotto scalare tra due vettori v1 e v2
float Aereo::prodottoScalare(const float *v1, const float *v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

/* Check in which range is battery voltage, return:
 *  0: rawValue > MAX_BATTERY_VOLTAGE
 *  1: MAX_BATTERY_VOLTAGE > rawValue > BATTERY_VOLTAGE_ABORT_MISSION
 *  2: BATTERY_VOLTAGE_ABORT_MISSION > rawValue > MIN_BATTERY_VOLTAGE
 * -1: MIN_BATTERY_VOLTAGE > rawValue */
int8_t Aereo::checkBatteryVoltage() {
    // Leggi il valore dall'ingresso analogico A0 (valori tra 0 e 1023)
    short rawValue = 0;
    for (uint8_t i = 0; i < 3; i++) {
        rawValue += analogRead(BATTERY_VOLTAGE_INPUT);
        delay(10);
    }
    rawValue /= 3;

    if (rawValue > MAX_BATTERY_VOLTAGE) {
        return 0;
    } else if (rawValue > BATTERY_VOLTAGE_ABORT_MISSION) {
        return 1; // Tensione superiore a BATTERY_VOLTAGE_ABORT_MISSION (0)
    } else if (rawValue > MIN_BATTERY_VOLTAGE) {
        return 2; // Tensione tra MIN_BATTERY_VOLTAGE e BATTERY_VOLTAGE_ABORT_MISSION (1)
    } else {
        return -1; // Tensione inferiore a MIN_BATTERY_VOLTAGE (2)
    }
}

// Aggiorna la posizione dei flap in base agli angoli flap_dx e flap_sx
void Aereo::updateFlaps() {
    if (angle_flap_sx - 90 < MIN_ANGLE_FLAP) {
        angle_flap_sx = MIN_ANGLE_FLAP;
    } else if (angle_flap_sx - 90 > MAX_ANGLE_FLAP) {
        angle_flap_sx = MAX_ANGLE_FLAP;
    }

    if (angle_flap_dx - 90 < MIN_ANGLE_FLAP) {
        angle_flap_dx = MIN_ANGLE_FLAP;
    } else if (angle_flap_dx - 90 > MAX_ANGLE_FLAP) {
        angle_flap_dx = MAX_ANGLE_FLAP;
    }
    // Imposta gli angoli dei flap sui servomotori
    servo_dx.write(angle_flap_dx - 90);
    servo_sx.write(angle_flap_sx - 90);
}

// Aggiorna la potenza del motore in base al valore percentuale perc_motor
void Aereo::updateMotor() {
    if (perc_motor < MIN_PERC_MOTOR) perc_motor = MIN_PERC_MOTOR;
    if (perc_motor > MAX_PERC_MOTOR) perc_motor = MAX_PERC_MOTOR;
    analogWrite(MOTOR_COMMAND_PIN, perc_motor * 255 / 100);
}

// Muove il portellone (cargo) o il paracadute in base al valore di "open"
// La variabile dest_no tiene conto del movimento da eseguire
void Aereo::moveGate(bool open) {
    switch (dest_no) {
        case 0:
            cargo.write(open ? CARGO_OPEN_VAL : CARGO_CLOSED_VAL);
            dest_no++;
            break;
        case 1:
            parachute.write(open ? PARACHUTE_OPEN_VAL : PARACHUTE_CLOSED_VAL);
            break;
        default:
            parachute.write(open ? PARACHUTE_OPEN_VAL : PARACHUTE_CLOSED_VAL);
    }
}

void Aereo::moveS(uint8_t velReal, uint8_t velIdeal) {
    // Array per memorizzare le differenze angolari
    float angDiff[3];

    // Aggiorna i dati dal sensore MPU
    mpu.update();

    // Matrice 3x3 per memorizzare i dati dall'MPU
    const float data[3][3] = {
            {mpu.getAngleX(), mpu.getAngleY(), mpu.getAngleZ()},
            {mpu.getGyroX(),  mpu.getGyroY(),  mpu.getGyroZ()},
            {mpu.getAccX(),   mpu.getAccY(),   mpu.getAccZ()}
    };

    // CONTROLLO POSIZIONE ANGOLARE
    if (isSimilar(data[0], angDiff, ideal_var[0][0])) {
        // CANCELLARE LA VELOCITÀ ANGOLARE CON LO STABILIZZATORE
        if (correctAngVel(data[1], angDiff)) {
            // Aggiusta la velocità
            if (velReal > velIdeal + VEL_ERROR) {
                // Bisogna diminuire la velocità, correggi l'accelerazione lineare utilizzando ideal_var[0][2]
                correctLinAcc(data[2], ideal_var[0][2]);
            } else if (velReal < velIdeal - VEL_ERROR) {
                // Bisogna aumentare la velocità, correggi l'accelerazione lineare con un valore opposto a ideal_var[0][2]
                const float temp[3] = {-ideal_var[0][2][0],
                                       ideal_var[0][2][1],
                                       ideal_var[0][2][2]};
                correctLinAcc(data[2], temp);
            } else {
                // Azzerare le accelerazioni perché siamo nel verso giusto
                correctLinAcc(data[2]);
            }
        }
    } else {
        // Correggi la velocità angolare utilizzando ideal_var[0][1]
        correctAngVel(data[1], angDiff, ideal_var[0][1]);
        // Azzerare le accelerazioni per non perdere quota in modo incontrollato
        correctLinAcc(data[2]);
    }

    // Aggiorna i flap
    updateFlaps();

    // Aggiorna i motori
    updateMotor();
}

void Aereo::moveUD(bool up) {
    // Sezione per determinare l'indice dell' ideal_var in base alla direzione di movimento (su o giù)
    uint8_t k = 1;
    if (!up)
        k++; // Se la direzione è giù, incrementa k per accedere alle corrispondenti matrici ideal_var

    // Array per memorizzare le differenze angolari
    float angDiff[3];

    // Aggiorna i dati dal sensore MPU
    mpu.update();

    // Matrice 3x3 per memorizzare i dati dall'MPU
    const float data[3][3] = {
            {mpu.getAngleX(), mpu.getAngleY(), mpu.getAngleZ()},
            {mpu.getGyroX(),  mpu.getGyroY(),  mpu.getGyroZ()},
            {mpu.getAccX(),   mpu.getAccY(),   mpu.getAccZ()}
    };

    // Controllo se la posizione angolare è simile a ideal_var[k][0]
    if (isSimilar(data[0], angDiff, ideal_var[k][0])) {
        // La posizione angolare è simile a ideal_var[k][0], quindi correggi solo la velocità angolare
        correctAngVel(data[1], angDiff);
    } else {
        // La posizione angolare è diversa da ideal_var[k][0], quindi correggi sia la velocità angolare che la direzione
        correctAngVel(data[1], angDiff, ideal_var[k][1]);
    }

    // Correggi l'accelerazione lineare utilizzando ideal_var[k][2]
    correctLinAcc(data[2]);

    // Aggiorna i flap
    updateFlaps();

    // Aggiorna i motori
    updateMotor();
}

void Aereo::moveRL(bool right) {

    uint8_t k = 3;
    if (!right) k += 2;

    float angDiff[3];

    mpu.update();

    const float data[3][3] = {
            {mpu.getAngleX(), mpu.getAngleY(), mpu.getAngleZ()},
            {mpu.getGyroX(),  mpu.getGyroY(),  mpu.getGyroZ()},
            {mpu.getAccX(),   mpu.getAccY(),   mpu.getAccZ()}
    };

    //controllo posizione angolare
    if (isSimilar(data[0], angDiff, ideal_var[k][0])) {

        // fase 2

        correctAngVel(data[1], angDiff, ideal_var[k + 1][1]);
        correctLinAcc(data[2], ideal_var[k + 1][2]);

        //fine fase 2

    } else {
        correctAngVel(data[1], angDiff, ideal_var[k][1]);
    }

    updateFlaps();
    updateMotor();
}

// Controlla se i due vettori sono simili tra di loro in base a un errore
bool Aereo::isSimilar(const float real[3], float angDiff[3], const float ideal[3], float error) {
    vettoreDifferenza(ideal, real, angDiff);
    return moduloVettore(angDiff) <= error;
}

// Modifica i servo, ritorna false se le velocità angolari non vanno bene E SONO STATE CORRETTE
bool Aereo::correctAngVel(const float *real, float *angDiff, const float *ideal, float error) {
    //il vettore angDiff mi serve per decidere il verso del movimento angolare da imprimere
    float angVelDiff[3];
    vettoreDifferenza(real, ideal, angVelDiff);
    if (moduloVettore(angVelDiff) > error) {
        // Correggi qui la velocità angolare o effettua altre operazioni necessarie
        // non corregge la velocità angolare su z perchè non possiamo fisicamente farlo
        int flapChange = static_cast<int>(DEG_FOR_ANG_VEL_X * angVelDiff[0]);
        (angVelDiff[0] > 0) ?: flapChange *= -1;
        angle_flap_dx += flapChange;
        angle_flap_sx -= flapChange;

        flapChange = static_cast<short>(DEG_FOR_ANG_VEL_Y * angVelDiff[1]);
        (angVelDiff[1] * ORIENT > 0) ?: flapChange *= -1;
        angle_flap_dx += flapChange;
        angle_flap_sx += flapChange;

        return false;
    } else {
        return true;
    }
}

// Modifica solo la potenza del motore per avere una certa forza, ritorna false se l'accelerazione non VA BENE ED è STATA CORRETTA
bool Aereo::correctLinAcc(const float *real, const float *ideal, float error) {
    float accDiff[3];
    vettoreDifferenza(ideal, real, accDiff);
    if (moduloVettore(accDiff) > error) {
        // Correggi qui la velocità angolare o effettua altre operazioni necessarie
        // IN REALTà CORREGGE SOLO LA X LOL
        int motorChange = static_cast<int>(PERC_FOR_LIN_ACC_X * accDiff[0]);
        (accDiff[0] > 0) ?: motorChange *= -1;
        perc_motor += motorChange;
        return false;
    } else {
        return true;
    }
}

void Aereo::emergencyStop() {
    parachute.write(PARACHUTE_OPEN_VAL);
}
