
#ifndef MSG_H
#define MSG_H
#include <limero/codec.h>

typedef int8_t PropertyId;

typedef enum HeaderId { DST = 0, SRC, MSG_TYPE, RET_CODE, MSG_ID, QOS } HeaderId;

typedef enum InfoPropertyId : PropertyId {
    PROP_ID = 0,
    NAME,
    DESCRIPTION,
    TYPE,
    MODE,
} InfoPropertyId;

typedef enum MsgType {
    Alive = 0,  // keep alive
    Pub = 1,    // publish data if dst=None => broadcast
                // if dst=Some  => send to one as Set
    Sub = 2,    // Subscribe to data , send to src as Endpoint
    Info = 3,   // contain name, description, type, etc
} MsgType;

typedef enum ValueType {
    UINT = 0,
    INT = 1,
    STR = 2,
    BYTES = 3,
    FLOAT = 4,
} ValueType;

typedef enum ValueMode {
    READ = 0,
    WRITE = 1,
} ValueMode;

#endif