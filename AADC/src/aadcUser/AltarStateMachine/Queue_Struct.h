#pragma once
enum lightType {
    HEAD_ON = 0,
    LEFT_ON,
    RIGHT_ON,
    HAZARD_ON,
    REVERSE_ON,
    BRAKE_ON,
    HEAD_OFF,
    LEFT_OFF,
    RIGHT_OFF,
    HAZARD_OFF,
    REVERSE_OFF,
    BRAKE_OFF
};

enum eventList {
    EMERG_STOP = 0,
    TURN_LEFT,
    TURN_RIGHT,
    FOLLOW_LANE,
    PARKING,
    BACKING_IN,
    PARKED,
    PULL_OUT_LEFT,
    PULL_OUT_RIGHT,

};

enum priorityList {
    TOP_PRIORITY = 0,
    HIGH_PRIORITY,
    MED_PRIORITY,
    LOW_PRIORITY,
    LOWEST_PRIORITY
};


struct evtDetail {
    eventList id;
    int priority;
    double distance;

    bool operator<(const evtDetail &rhs) const {
        if (rhs.priority > priority) { return true; }
        else if (rhs.priority == priority) {
            return rhs.distance < distance;
        }
        return false;
    }
};
    
