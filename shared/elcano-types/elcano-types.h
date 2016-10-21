#pragma once

namespace elcano {

struct Junction {
    long east_mm;
    long north_mm;
    int dest[4];
    long dist_mm[4];
};

struct Waypoint {
    long latitude;
    long longitude;
    long east_mm;
    long north_mm;
    long sigma_mm;
    unsigned long time_ms;
    int east_x1k;
    int north_x1k;
    long speed_mmPs;
    int index;
    
    /*void compute_mm();
    void compute_lat_lon();
    long distance_mm(const Waypoint&);
    long distance_mm(long, long);
    void vectors(const Waypoint&);
    void fuse(const Waypoint&, int);
    void set_time(char*, char*);*/
};

struct Curve {
    Curve *prev;
    Waypoint *pres;
    bool junc;
    Curve *next;
};

}
