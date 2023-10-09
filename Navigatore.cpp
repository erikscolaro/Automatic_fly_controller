#include "Navigatore.h"


/**
 * Costruttore della classe Navigatore per inizializzare un oggetto navigatore con le coordinate target.
 *
 * @param t_latitude La latitudine della destinazione desiderata.
 * @param t_longitude La longitudine della destinazione desiderata.
 * @param t_altitude L'altitudine della destinazione desiderata.
 * @param t_speed La velocità della destinazione desiderata.
 */
Navigatore::Navigatore(SoftwareSerial& t_gps, float t_latitude, float t_longitude, float t_altitude, float t_speed)
        : gpsSerial(t_gps) {
    // Inizializzazione delle coordinate target e velocità della destinazione
    target_latitude = t_latitude;
    target_longitude = t_longitude;
    target_altitude = t_altitude;
    target_speed = t_speed;

    // Inizializzazione delle coordinate di partenza (home) a 0 poiché non sono ancora state registrate
    home_latitude = 0;
    home_longitude = 0;


}


/**
 * Controlla se l'altitudine attuale è corretta rispetto all'altitudine desiderata.
 *
 * @return -1 se l'altitudine attuale è maggiore dell'altitudine desiderata più un errore,
 *         1 se l'altitudine attuale è minore dell'altitudine desiderata meno un errore,
 *         0 se l'altitudine attuale è considerata corretta.
 */
short Navigatore::altitudeIsCorrect() {
    if (target_altitude > gps.f_altitude() + ALTITUDE_ERROR_M)
        return -1; // L'altitudine attuale è troppo alta, suggerisce di scendere.
    else if (target_altitude < gps.f_altitude() - ALTITUDE_ERROR_M)
        return 1; // L'altitudine attuale è troppo bassa, suggerisce di salire.
    else
        return 0; // L'altitudine attuale è considerata corretta.
}

/**
 * Controlla se la velocità attuale è corretta rispetto alla velocità desiderata.
 *
 * @return -1 se la velocità attuale è maggiore della velocità desiderata più un errore,
 *         1 se la velocità attuale è minore della velocità desiderata meno un errore,
 *         0 se la velocità attuale è considerata corretta.
 */
short Navigatore::speedIsCorrect() {
    if (target_speed > gps.f_speed_kmph() + SPEED_ERROR_KMH)
        return -1; // La velocità attuale è troppo alta, suggerisce di ridurre la velocità.
    else if (target_speed < gps.f_speed_kmph() - SPEED_ERROR_KMH)
        return 1; // La velocità attuale è troppo bassa, suggerisce di aumentare la velocità.
    else
        return 0; // La velocità attuale è considerata corretta.
}

/**
 * Controlla se la traiettoria attuale è corretta rispetto alla direzione desiderata.
 *
 * @return -1 se la direzione attuale è più grande della direzione desiderata più un errore,
 *         1 se la direzione attuale è più piccola della direzione desiderata meno un errore,
 *         0 se la direzione attuale è considerata corretta.
 */
short Navigatore::trajectoryIsCorrect() {
    float latitude, longitude, course_diff;
    gps.f_get_position(&latitude, &longitude);
    course_diff = gps.course_to(latitude, longitude, target_latitude, target_longitude) - gps.f_course();

    if (course_diff > 180) {
        course_diff -= 360;
    } else if (course_diff < -180) {
        course_diff += 360;
    }

    // Controlla la differenza con l'errore per decidere la direzione di girata
    if (course_diff > COURSE_ERROR_DEG) {
        // Girare a destra
        return 1;
    } else if (course_diff < -COURSE_ERROR_DEG) {
        // Girare a sinistra
        return -1;
    } else {
        // Non c'è bisogno di girare
        return 0;
    }
}

/**
 * Verifica se la destinazione desiderata è stata raggiunta.
 *
 * @return 0 se sono distante più di PROXIMITY_RADIUS_M dall'obbiettivo,
 * 1 se sono più vicino di PROXIMITY_RADIUS_M ma meno di DELIVERY_RADIUS_M,
 * 2 se sono più vicino di DELIVERY_RADIUS_M
 */
short Navigatore::destinationReached() {
    float latitude, longitude, distance;
    gps.f_get_position(&latitude, &longitude);

    distance=gps.distance_between(latitude, longitude, target_latitude, target_longitude);

    if (distance < PROXIMITY_RADIUS_M) {
        if (distance < DELIVERY_RADIUS_M){
            return 2; // Obiettivo raggiunto.
        }
        return 1; // La destinazione non è ancora stata raggiunta, ma siamo vicini.
    }
    return 0; // Siamo molto lontani dalla destinazione
}

/**
 * Inizia la comunicazione seriale con il modulo GPS e controlla se la comunicazione è avvenuta con successo.
 *
 * @return true se la comunicazione seriale con il GPS è stata avviata con successo, false altrimenti.
 */
bool Navigatore::startNavigator() {
    if (!gpsSerial) gpsSerial.begin(9600);  // Inizia la comunicazione seriale con il modulo GPS alla velocità di 9600 bps

    if (gpsSerial) {  // Verifica se la comunicazione seriale con il GPS è stata aperta correttamente

        while (!updateNavigator()){}

        //salvo le coordinate di casa per il comeback.
        float latitude, longitude;
        gps.f_get_position(&latitude, &longitude);
        home_latitude=latitude;
        home_longitude=longitude;

        return true;  // Restituisci true se la comunicazione è avvenuta con successo
    } else {
        return false;  // Restituisci false se la comunicazione non è stata aperta correttamente
    }
}


/**
 * Aggiorna lo stato del navigatore e verifica se è stata registrata una posizione valida.
 * La funzione tenta di registrare una posizione valida fino a MAX_INVALID_COUNT volte consecutive.
 *
 * @return true se è stata registrata una posizione valida, false altrimenti.
 */
bool Navigatore::updateNavigator() {
    int invalidCount = 0;  // Contatore per tenere traccia delle letture di posizioni non valide

    for (int i = 0; i < MAX_INVALID_POSITION_COUNT; i++) {
        // Verifica se} è stata registrata una posizione valida per due volte di fila
        if (registeredValidPosition() and registeredValidPosition()) {
            return true;  // Restituisci true se è stata registrata una posizione valida
        } else {
            delay(100); // Un minimo di delay per cercare di ristabilire una connessione con il gps
            invalidCount++;  // Incrementa il contatore delle letture di posizioni non valide
        }
    }

    // Se il numero massimo di letture di posizioni non valide consecutive è stato raggiunto, restituisci false
    return false;
}


/**
 * Verifica se è stata registrata una posizione GPS valida dal modulo GPS.
 *
 * @return true se è stata registrata una posizione valida, false altrimenti.
 */
bool Navigatore::registeredValidPosition() {
    while (gpsSerial.available() > 0) {  // Continua finché ci sono dati disponibili nella porta seriale del GPS
        char c = gpsSerial.read();  // Leggi il prossimo carattere disponibile
        if (gps.encode(c)) {  // Decodifica il carattere come parte di un messaggio GPS
            unsigned long fix_age;  // Variabile per memorizzare l'età del fix GPS
            gps.get_position(&fix_age, NULL, NULL);
            if (fix_age != TinyGPS::GPS_INVALID_AGE && fix_age <= MAX_FIX_AGE) {
                // Ottieni la posizione GPS e l'età del fix è valida e inferiore o uguale a MAX_FIX_AGE (tempo massimo considerato valido)
                return true;  // Restituisci true indicando che è stata registrata una posizione valida
            }
        }
    }

    // Se si esce dal ciclo while senza trovare una posizione valida, restituisci false
    return false;
}

void Navigatore::setHomeCoordinates() {
    target_longitude=home_longitude;
    target_latitude=home_latitude;
}




