#ifndef FLYCONTROLLER_NAVIGATORE_H
#define FLYCONTROLLER_NAVIGATORE_H

#include "TinyGPS.h"
#include <math.h>
#include <SoftwareSerial.h>

#define COURSE_ERROR_DEG 1
#define ALTITUDE_ERROR_M 10
#define SPEED_ERROR_KMH 1
#define DELIVERY_RADIUS_M 10
#define PROXIMITY_RADIUS_M 100
#define MAX_FIX_AGE 100

#define MAX_INVALID_POSITION_COUNT 5

class Navigatore:private TinyGPS {
private:
    TinyGPS gps;
    SoftwareSerial& gpsSerial;
    float target_latitude, target_longitude, target_altitude, target_speed, home_latitude, home_longitude;

    //metodi getter, solo per utilità
    inline float getRealAltitude(){
        return gps.f_altitude();
    }

    inline float getRealTrajectory() {
        return gps.f_course();
    }

    bool registeredValidPosition();

public:
    Navigatore(SoftwareSerial& t_gps, float t_latitude, float t_longitude, float t_altitude, float t_speed);

    bool startNavigator();
    bool updateNavigator();

    //metodi per confrontare
    short altitudeIsCorrect();
    short speedIsCorrect();
    short trajectoryIsCorrect();
    short destinationReached();

    void setHomeCoordinates();

    inline float getRealSpeed(){
        return gps.f_speed_kmph();
    }

    // Metodi per settare i valori target

    inline void setTargetAltitude(float altitude) {
        target_altitude = altitude;
    }

    inline void setTargetSpeed(float speed) {
        target_speed = speed;
    }
};


#endif //FLYCONTROLLER_NAVIGATORE_H
