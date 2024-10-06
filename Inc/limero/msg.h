
#ifndef MSG_H
#define MSG_H
#include <limero/codec.h>

typedef int8_t PropertyId;

typedef enum MetaPropertyId : PropertyId {
    RET_CODE = -1,
    QOS = -2,
    MSG_ID = -3,
} MetaPropertyId;

typedef enum InfoPropertyId : PropertyId {
    PROP_ID = 0,
    NAME,
    DESCRIPTION,
    TYPE,
    MODE,
} InfoPropertyId;

typedef enum MsgType {
    Alive = 0,  // keep alive
    Pub = 1,   // publish data if dst=None => broadcast
    // if dst=Some  => send to one as Set
    Sub = 2,   // Subscribe to data , send to src as Endpoint
    Info = 3,  // contain name, description, type, etc
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

class MsgHeader {
public:
    Option<uint32_t> dst;
    Option<uint32_t> src;
    uint32_t msg_type;

public:
    Result<Void> encode(FrameEncoder& encoder) {
        if (dst.is_some()) {
            RET_ERR(encoder.encode_uint32(dst.unwrap()));
        }
        else {
            RET_ERR(encoder.encode_null());
        }
        if (src.is_some()) {
            RET_ERR(encoder.encode_uint32(src.unwrap()));
        }
        else {
            RET_ERR(encoder.encode_null());
        }
        RET_ERR(encoder.encode_uint32(msg_type));
        return Result<Void>::Ok(Void());
    }
};
#endif