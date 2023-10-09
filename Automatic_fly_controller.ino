#include "Aereo.h"
#include "Navigatore.h"

#define INTERNAL_LED 13

#define NUM_FUNCTIONS 5
#define NUM_CONFIGURATIONS 3

#define TARGET_LATITUDE 45.825806
#define TARGET_LONGITUDE 13.349972
#define TARGET_ALTITUDE 50
#define TARGET_SPEED 25 //KMH

#define RX_PIN 3
#define TX_PIN 4


//variabili globali

Aereo aereo(Wire);
SoftwareSerial mySerial(TX_PIN, RX_PIN); // Crea l'oggetto SoftwareSerial
Navigatore navigatore(mySerial , TARGET_LATITUDE,TARGET_LONGITUDE,TARGET_ALTITUDE,TARGET_SPEED);

const int configurations[NUM_CONFIGURATIONS][2][NUM_FUNCTIONS] = {
        // Configuration 1
        {
                {0, 1, 2, 3, 4},           // Priority for Configuration 1: {0, 1, 2, 3, 4}
                {500, 1000, 1500, 2000, 10000}, // Refractory for Configuration 1: {500, 1000, 1500, 2000, 10000}
        },
        // Configuration 2
        {
                {3, 2, 1, 0, 4},           // Priority for Configuration 2: {3, 2, 1, 0, 4}
                {1000, 500, 500, 250, 10000},   // Refractory for Configuration 2: {1000, 500, 500, 250, 10000}
        },
        // Configuration 3
        {
                {1, 3, 0, 2, 4},           // Priority for Configuration 3: {1, 3, 0, 2, 4}
                {2000, 2000, 2000, 2000, 2000},   // Refractory for Configuration 3: {2000, 2000, 2000, 2000, 2000}
        }
};

short currentConfig = 0;  short controlResult;
unsigned long refractoryTimers[NUM_FUNCTIONS] = {0, 0, 0, 0, 0};
/*********configurazione*******/
void setup() {
    pinMode(INTERNAL_LED, OUTPUT); // Imposta il pin del LED come OUTPUT
    digitalWrite(INTERNAL_LED, HIGH); // Accendi il LED (livello HIGH)

    Wire.begin();


    //configurazione dei sensori e delle classi
    while (!navigatore.startNavigator()){}
    while (!aereo.init()){}

    digitalWrite(INTERNAL_LED, LOW); // Accendi il LED (livello HIGH)

    //attesa prima del lancio
    while(!aereo.launchDetected()){delay(1);}



}

/********** main loop ***********/
void loop() {
    for (int i = 0; i < NUM_FUNCTIONS; i++) {
        //aggiornamento dei dati gps per il navigatore, in ogni circostanza
        navigatore.updateNavigator();
        //todo: cosa succede se non posso aggiornare il navigatore? Per ora nulla

        int index = configurations[currentConfig][0][i];

        controlResult=0;
        switch (index) {
            case 0:
                controlResult = navigatore.altitudeIsCorrect();
                break;
            case 1:
                controlResult = navigatore.speedIsCorrect();
                break;
            case 2:
                controlResult = navigatore.trajectoryIsCorrect();
                break;
            case 3:
                controlResult = navigatore.destinationReached();
                break;
            case 4:
                controlResult = aereo.checkBatteryVoltage();
                break;
        }

        if (controlResult != 0) {
            switch (index) {
                case 0:
                    aereo.moveUD(controlResult == 1); // moveUD
                    break;
                case 1:
                    aereo.moveS(static_cast<uint8_t>(navigatore.getRealSpeed())); // moveS with speed as uint6_t input
                    break;
                case 2:
                    aereo.moveRL(controlResult == 1); // moveRL
                    break;
                case 3:
                    if (controlResult == 1){
                        currentConfig=1;
                    } else if (controlResult==2) {
                        aereo.moveGate(true); // moveGate. Sgancia il pacco
                        delay(1000); //diamo tempo di sganciare il pacco
                        navigatore.setHomeCoordinates(); //
                        currentConfig = 2;
                    }
                    break;
                case 4: //checkBatteryVoltage
                    if (controlResult == 2) { //abort mission
                        navigatore.setHomeCoordinates();
                    } else if (controlResult == -1) { //emergency landing
                        aereo.emergencyStop();
                    }
                    break;
            }

            i = 0;
        } else {
            if (refractoryTimers[index] == 0) {
                refractoryTimers[index] = millis();
            } else if (millis() - refractoryTimers[index] >= configurations[currentConfig][1][index]) {
                refractoryTimers[index] = 0;
            }
        }
    }
}
