# PPS status

time stamp              # packet timestamp
int8 state               # enum representing the state

# constants for PPS "enum"
int32 STATE_NO_PPS_DETECTED        = 0
int32 STATE_SYNCHRONIZING_TO_PPS   = 1
int32 STATE_PPS_LOCKED             = 2
int32 STATE_ERROR                  = 3
