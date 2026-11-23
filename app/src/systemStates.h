#ifndef SYSTEM_STATES
#define SYSTEM_STATES

#include <stdint.h>

typedef enum {
    STATE_IDLE = 0,     // idle
    STATE_TESTING,      // testing data reading from radeye
    STATE_MONITORING,   // scanning for radiation level
    STATE_RESCAN,       // reconfirm radiation level reading
    STATE_CLEANUP       // clean up after done with scanning

} system_state_t;

#endif