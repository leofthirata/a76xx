#pragma once

// CLBS
#include <string>
#include <vector>

const std::vector<std::string> CLBS_RET_CODES {
    "+CLBS: 0", 
    "+CLBS: 1", 
    "+CLBS: 2", 
    "+CLBS: 3", 
    "+CLBS: 4", 
    "+CLBS: 5", 
    "+CLBS: 6", 
    "+CLBS: 7", 
    "+CLBS: 8", 
    "+CLBS: 9", 
    "+CLBS: 10", 
    "+CLBS: 11", 
    "+CLBS: 12", 
    "+CLBS: 13", 
    "+CLBS: 14", 
    "+CLBS: 15", 
    "+CLBS: 16", 
    "+CLBS: 17", 
    "+CLBS: 18", 
    "+CLBS: 19", 
    "+CLBS: 20", 
    "+CLBS: 21", 
    "+CLBS: 80", 
    "+CLBS: 81", 
    "+CLBS: 82", 
    "+CLBS: 110", 
};

const std::vector<std::string> CPIN_RET_CODES {
    "READY", // ME is not pending for any password
    "SIM PIN", // ME is waiting SIM PIN to be given
    "SIM PUK", // ME is waiting SIM PUK to be given
    "PH-SIM", // PIN ME is waiting phone-to-SIM card password to be given
    "SIM PIN2", // ME is waiting SIM PIN2 to be given
    "SIM PUK2", // ME is waiting SIM PUK2 to be given
    "PH-NET PIN" // ME is waiting network personalization password to be
};

const std::vector<std::string> CREG_CGREG_RET_CODES {
    "+CREG: 0,0", // not registered, ME is not currently searching a new operator to register to.
    "+CREG: 0,1", // registered, home network.
    "+CREG: 0,2", // not registered, but ME is currently searching a new operator to register to.
    "+CREG: 0,3", // registration denied.
    "+CREG: 0,4", // unknown.
    "+CREG: 0,5", // registered, roaming.
    "+CREG: 0,6", // registered for "SMS only", home network (applicable only when E-UTRAN)
    "+CREG: 0,7", // registered for "SMS only", roaming (applicable only when <AcT> indicates E-UTRAN)
    "+CREG: 0,11", // attached for emergency bearer services only
};

const std::vector<std::string> CPSI_RET_CODES {
    "+CPSI: LTE", // ME is not pending for any password
    "+CPSI: NO SERVICE", // ME is waiting SIM PIN to be given
};
