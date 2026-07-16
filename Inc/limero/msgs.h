// Auto-generated from robot.hcl — do not edit by hand.
// Every field is `Option<T>`; missing fields are omitted from encoded CBOR.
// Messages are encoded as CBOR maps keyed by field id.
// Uses TinyCBOR for CBOR encoding/decoding.

#pragma once

#include <limero/codec.h>
#include <cbor.h>

// ── TinyCBOR helper ────────────────────────────────────────────────────────

static inline Result<Void> cbor_check(CborError err) {
    if (err == CborNoError) return Result<Void>::Ok(Void());
    return Result<Void>::Err(err, cbor_error_string(err));
}

// ── Encode macro ───────────────────────────────────────────────────────────// ── Decode macros ──────────────────────────────────────────────────────────// ── Messages ───────────────────────────────────────────────────────────────



class BrokerSubscribeRequest : public Msg {
public:
    static uint32_t msg_id() { return FNV("BrokerSubscribeRequest"); }
    static const char* msg_name() { return "BrokerSubscribeRequest"; }

    typedef enum FieldId {
        SRC = 0,
        MSG_TYPE = 1,
    } FieldId;
    Option<uint32_t> src;
    Option<uint32_t> msg_type;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        src.inspect([&](const auto&) { fieldCount++; });
        msg_type.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        src.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::SRC));
            cbor_check(cbor_encode_uint(&mapEncoder, src.unwrap()));
        });
        msg_type.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::MSG_TYPE));
            cbor_check(cbor_encode_uint(&mapEncoder, msg_type.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a BrokerSubscribeRequest from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for BrokerSubscribeRequest");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case BrokerSubscribeRequest::FieldId::SRC:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    src.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    src.some((uint32_t)val);
                }
                    break;
                case BrokerSubscribeRequest::FieldId::MSG_TYPE:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    msg_type.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    msg_type.some((uint32_t)val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class CompassEvent : public Msg {
public:
    static uint32_t msg_id() { return FNV("CompassEvent"); }
    static const char* msg_name() { return "CompassEvent"; }

    typedef enum FieldId {
        HEADING = 0,
        PITCH = 1,
        ROLL = 2,
        MAG_X = 3,
        MAG_Y = 4,
        MAG_Z = 5,
        ACCEL_X = 6,
        ACCEL_Y = 7,
        ACCEL_Z = 8,
    } FieldId;
    /// Heading in degrees
    Option<float> heading;
    /// Pitch in degrees
    Option<float> pitch;
    /// Roll in degrees
    Option<float> roll;
    /// Magnetometer X axis in uT
    Option<float> mag_x;
    /// Magnetometer Y axis in uT
    Option<float> mag_y;
    /// Magnetometer Z axis in uT
    Option<float> mag_z;
    /// Accelerometer X axis in m/s^2
    Option<float> accel_x;
    /// Accelerometer Y axis in m/s^2
    Option<float> accel_y;
    /// Accelerometer Z axis in m/s^2
    Option<float> accel_z;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        heading.inspect([&](const auto&) { fieldCount++; });
        pitch.inspect([&](const auto&) { fieldCount++; });
        roll.inspect([&](const auto&) { fieldCount++; });
        mag_x.inspect([&](const auto&) { fieldCount++; });
        mag_y.inspect([&](const auto&) { fieldCount++; });
        mag_z.inspect([&](const auto&) { fieldCount++; });
        accel_x.inspect([&](const auto&) { fieldCount++; });
        accel_y.inspect([&](const auto&) { fieldCount++; });
        accel_z.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        heading.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::HEADING));
            cbor_check(cbor_encode_float(&mapEncoder, heading.unwrap()));
        });
        pitch.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::PITCH));
            cbor_check(cbor_encode_float(&mapEncoder, pitch.unwrap()));
        });
        roll.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::ROLL));
            cbor_check(cbor_encode_float(&mapEncoder, roll.unwrap()));
        });
        mag_x.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::MAG_X));
            cbor_check(cbor_encode_float(&mapEncoder, mag_x.unwrap()));
        });
        mag_y.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::MAG_Y));
            cbor_check(cbor_encode_float(&mapEncoder, mag_y.unwrap()));
        });
        mag_z.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::MAG_Z));
            cbor_check(cbor_encode_float(&mapEncoder, mag_z.unwrap()));
        });
        accel_x.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::ACCEL_X));
            cbor_check(cbor_encode_float(&mapEncoder, accel_x.unwrap()));
        });
        accel_y.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::ACCEL_Y));
            cbor_check(cbor_encode_float(&mapEncoder, accel_y.unwrap()));
        });
        accel_z.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::ACCEL_Z));
            cbor_check(cbor_encode_float(&mapEncoder, accel_z.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a CompassEvent from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for CompassEvent");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case CompassEvent::FieldId::HEADING:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    heading.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    heading.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    heading.some((float)val);
                }
                    break;
                case CompassEvent::FieldId::PITCH:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    pitch.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    pitch.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    pitch.some((float)val);
                }
                    break;
                case CompassEvent::FieldId::ROLL:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    roll.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    roll.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    roll.some((float)val);
                }
                    break;
                case CompassEvent::FieldId::MAG_X:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    mag_x.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    mag_x.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    mag_x.some((float)val);
                }
                    break;
                case CompassEvent::FieldId::MAG_Y:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    mag_y.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    mag_y.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    mag_y.some((float)val);
                }
                    break;
                case CompassEvent::FieldId::MAG_Z:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    mag_z.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    mag_z.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    mag_z.some((float)val);
                }
                    break;
                case CompassEvent::FieldId::ACCEL_X:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    accel_x.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    accel_x.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    accel_x.some((float)val);
                }
                    break;
                case CompassEvent::FieldId::ACCEL_Y:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    accel_y.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    accel_y.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    accel_y.some((float)val);
                }
                    break;
                case CompassEvent::FieldId::ACCEL_Z:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    accel_z.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    accel_z.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    accel_z.some((float)val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class DeviceAliveEvent : public Msg {
public:
    static uint32_t msg_id() { return FNV("DeviceAliveEvent"); }
    static const char* msg_name() { return "DeviceAliveEvent"; }

    typedef enum FieldId {
        DEVICE = 0,
        ENDPOINT = 1,
        TIMESTAMP = 2,
    } FieldId;
    Option<std::string> device;
    Option<std::string> endpoint;
    /// Timestamp in milliseconds since epoch
    Option<uint64_t> timestamp;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        device.inspect([&](const auto&) { fieldCount++; });
        endpoint.inspect([&](const auto&) { fieldCount++; });
        timestamp.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        device.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::DEVICE));
            cbor_check(cbor_encode_text_string(&mapEncoder, device.unwrap().c_str(), device.unwrap().length()));
        });
        endpoint.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::ENDPOINT));
            cbor_check(cbor_encode_text_string(&mapEncoder, endpoint.unwrap().c_str(), endpoint.unwrap().length()));
        });
        timestamp.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::TIMESTAMP));
            cbor_check(cbor_encode_uint(&mapEncoder, timestamp.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a DeviceAliveEvent from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for DeviceAliveEvent");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case DeviceAliveEvent::FieldId::DEVICE:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    device.some(val);
                }
                    break;
                case DeviceAliveEvent::FieldId::ENDPOINT:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    endpoint.some(val);
                }
                    break;
                case DeviceAliveEvent::FieldId::TIMESTAMP:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    timestamp.some(val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    timestamp.some((uint64_t)val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class EndpointAnnounce : public Msg {
public:
    static uint32_t msg_id() { return FNV("EndpointAnnounce"); }
    static const char* msg_name() { return "EndpointAnnounce"; }

    typedef enum FieldId {
        ID = 0,
        NAME = 1,
        DESCRIPTION = 6,
        SERVICES = 2,
        EVENTS = 3,
        REPLIES = 4,
        SUBSCRIBES = 5,
    } FieldId;
    /// Unique identifier for the announcing endpoint
    Option<uint32_t> id;
    /// Name of the announcing endpoint
    Option<std::string> name;
    /// Description of the announcing endpoint
    Option<std::string> description;
    /// List of services provided by the endpoint
    Option<std::vector<uint32_t>> services;
    /// List of events emitted by the endpoint
    Option<std::vector<uint32_t>> events;
    /// List of replies supported by the endpoint
    Option<std::vector<uint32_t>> replies;
    /// List of subscriptions for the endpoint
    Option<std::vector<uint32_t>> subscribes;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        id.inspect([&](const auto&) { fieldCount++; });
        name.inspect([&](const auto&) { fieldCount++; });
        description.inspect([&](const auto&) { fieldCount++; });
        services.inspect([&](const auto&) { fieldCount++; });
        events.inspect([&](const auto&) { fieldCount++; });
        replies.inspect([&](const auto&) { fieldCount++; });
        subscribes.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        id.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::ID));
            cbor_check(cbor_encode_uint(&mapEncoder, id.unwrap()));
        });
        name.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::NAME));
            cbor_check(cbor_encode_text_string(&mapEncoder, name.unwrap().c_str(), name.unwrap().length()));
        });
        description.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::DESCRIPTION));
            cbor_check(cbor_encode_text_string(&mapEncoder, description.unwrap().c_str(), description.unwrap().length()));
        });
        services.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::SERVICES));
            {
            CborEncoder arrEncoder;
            cbor_check(cbor_encoder_create_array(&mapEncoder, &arrEncoder, services.unwrap().size()));
            for (const auto& item : services.unwrap()) {
                cbor_check(cbor_encode_uint(&arrEncoder, item));
            }
            cbor_check(cbor_encoder_close_container(&mapEncoder, &arrEncoder));
        }
        });
        events.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::EVENTS));
            {
            CborEncoder arrEncoder;
            cbor_check(cbor_encoder_create_array(&mapEncoder, &arrEncoder, events.unwrap().size()));
            for (const auto& item : events.unwrap()) {
                cbor_check(cbor_encode_uint(&arrEncoder, item));
            }
            cbor_check(cbor_encoder_close_container(&mapEncoder, &arrEncoder));
        }
        });
        replies.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::REPLIES));
            {
            CborEncoder arrEncoder;
            cbor_check(cbor_encoder_create_array(&mapEncoder, &arrEncoder, replies.unwrap().size()));
            for (const auto& item : replies.unwrap()) {
                cbor_check(cbor_encode_uint(&arrEncoder, item));
            }
            cbor_check(cbor_encoder_close_container(&mapEncoder, &arrEncoder));
        }
        });
        subscribes.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::SUBSCRIBES));
            {
            CborEncoder arrEncoder;
            cbor_check(cbor_encoder_create_array(&mapEncoder, &arrEncoder, subscribes.unwrap().size()));
            for (const auto& item : subscribes.unwrap()) {
                cbor_check(cbor_encode_uint(&arrEncoder, item));
            }
            cbor_check(cbor_encoder_close_container(&mapEncoder, &arrEncoder));
        }
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a EndpointAnnounce from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for EndpointAnnounce");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case EndpointAnnounce::FieldId::ID:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    id.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    id.some((uint32_t)val);
                }
                    break;
                case EndpointAnnounce::FieldId::NAME:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    name.some(val);
                }
                    break;
                case EndpointAnnounce::FieldId::DESCRIPTION:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    description.some(val);
                }
                    break;
                case EndpointAnnounce::FieldId::SERVICES:
                    if (cbor_value_is_array(&mapValue)) {
                    CborValue arrValue;
                    cbor_value_enter_container(&mapValue, &arrValue);
                    std::vector<uint32_t> val;
                    while (!cbor_value_at_end(&arrValue)) {
                        if (cbor_value_is_unsigned_integer(&arrValue)) {
                            uint64_t v;
                            cbor_value_get_uint64(&arrValue, &v);
                            val.push_back((uint32_t)v);
                        } else if (cbor_value_is_negative_integer(&arrValue)) {
                            int64_t v;
                            cbor_value_get_int64(&arrValue, &v);
                            val.push_back((uint32_t)v);
                        }
                        cbor_value_advance(&arrValue);
                    }
                    cbor_value_leave_container(&mapValue, &arrValue);
                    services.some(val);
                }
                    break;
                case EndpointAnnounce::FieldId::EVENTS:
                    if (cbor_value_is_array(&mapValue)) {
                    CborValue arrValue;
                    cbor_value_enter_container(&mapValue, &arrValue);
                    std::vector<uint32_t> val;
                    while (!cbor_value_at_end(&arrValue)) {
                        if (cbor_value_is_unsigned_integer(&arrValue)) {
                            uint64_t v;
                            cbor_value_get_uint64(&arrValue, &v);
                            val.push_back((uint32_t)v);
                        } else if (cbor_value_is_negative_integer(&arrValue)) {
                            int64_t v;
                            cbor_value_get_int64(&arrValue, &v);
                            val.push_back((uint32_t)v);
                        }
                        cbor_value_advance(&arrValue);
                    }
                    cbor_value_leave_container(&mapValue, &arrValue);
                    events.some(val);
                }
                    break;
                case EndpointAnnounce::FieldId::REPLIES:
                    if (cbor_value_is_array(&mapValue)) {
                    CborValue arrValue;
                    cbor_value_enter_container(&mapValue, &arrValue);
                    std::vector<uint32_t> val;
                    while (!cbor_value_at_end(&arrValue)) {
                        if (cbor_value_is_unsigned_integer(&arrValue)) {
                            uint64_t v;
                            cbor_value_get_uint64(&arrValue, &v);
                            val.push_back((uint32_t)v);
                        } else if (cbor_value_is_negative_integer(&arrValue)) {
                            int64_t v;
                            cbor_value_get_int64(&arrValue, &v);
                            val.push_back((uint32_t)v);
                        }
                        cbor_value_advance(&arrValue);
                    }
                    cbor_value_leave_container(&mapValue, &arrValue);
                    replies.some(val);
                }
                    break;
                case EndpointAnnounce::FieldId::SUBSCRIBES:
                    if (cbor_value_is_array(&mapValue)) {
                    CborValue arrValue;
                    cbor_value_enter_container(&mapValue, &arrValue);
                    std::vector<uint32_t> val;
                    while (!cbor_value_at_end(&arrValue)) {
                        if (cbor_value_is_unsigned_integer(&arrValue)) {
                            uint64_t v;
                            cbor_value_get_uint64(&arrValue, &v);
                            val.push_back((uint32_t)v);
                        } else if (cbor_value_is_negative_integer(&arrValue)) {
                            int64_t v;
                            cbor_value_get_int64(&arrValue, &v);
                            val.push_back((uint32_t)v);
                        }
                        cbor_value_advance(&arrValue);
                    }
                    cbor_value_leave_container(&mapValue, &arrValue);
                    subscribes.some(val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class EndpointAnnounceReply : public Msg {
public:
    static uint32_t msg_id() { return FNV("EndpointAnnounceReply"); }
    static const char* msg_name() { return "EndpointAnnounceReply"; }

    typedef enum FieldId {
        UTC = 0,
    } FieldId;
    /// Timestamp in milliseconds since epoch
    Option<uint64_t> utc;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        utc.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        utc.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::UTC));
            cbor_check(cbor_encode_uint(&mapEncoder, utc.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a EndpointAnnounceReply from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for EndpointAnnounceReply");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case EndpointAnnounceReply::FieldId::UTC:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    utc.some(val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    utc.some((uint64_t)val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class Envelope : public Msg {
public:
    static uint32_t msg_id() { return FNV("Envelope"); }
    static const char* msg_name() { return "Envelope"; }

    typedef enum FieldId {
        SRC = 0,
        DST = 1,
        MSG_TYPE = 2,
        REQUEST_ID = 3,
        INSTANCE_ID = 4,
        PAYLOAD = 5,
    } FieldId;
    /// Source endpoint name
    Option<uint32_t> src;
    /// Destination endpoint name
    Option<uint32_t> dst;
    /// Message type name
    Option<uint32_t> msg_type;
    /// Request ID for matching request/reply
    Option<uint32_t> request_id;
    /// Instance ID for matching request/reply
    Option<uint32_t> instance_id;
    /// Serialized payload of the message
    Option<std::vector<uint8_t>> payload;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        src.inspect([&](const auto&) { fieldCount++; });
        dst.inspect([&](const auto&) { fieldCount++; });
        msg_type.inspect([&](const auto&) { fieldCount++; });
        request_id.inspect([&](const auto&) { fieldCount++; });
        instance_id.inspect([&](const auto&) { fieldCount++; });
        payload.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        src.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::SRC));
            cbor_check(cbor_encode_uint(&mapEncoder, src.unwrap()));
        });
        dst.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::DST));
            cbor_check(cbor_encode_uint(&mapEncoder, dst.unwrap()));
        });
        msg_type.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::MSG_TYPE));
            cbor_check(cbor_encode_uint(&mapEncoder, msg_type.unwrap()));
        });
        request_id.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::REQUEST_ID));
            cbor_check(cbor_encode_uint(&mapEncoder, request_id.unwrap()));
        });
        instance_id.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::INSTANCE_ID));
            cbor_check(cbor_encode_uint(&mapEncoder, instance_id.unwrap()));
        });
        payload.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::PAYLOAD));
            cbor_check(cbor_encode_byte_string(&mapEncoder, payload.unwrap().data(), payload.unwrap().size()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a Envelope from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for Envelope");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case Envelope::FieldId::SRC:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    src.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    src.some((uint32_t)val);
                }
                    break;
                case Envelope::FieldId::DST:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    dst.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    dst.some((uint32_t)val);
                }
                    break;
                case Envelope::FieldId::MSG_TYPE:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    msg_type.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    msg_type.some((uint32_t)val);
                }
                    break;
                case Envelope::FieldId::REQUEST_ID:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    request_id.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    request_id.some((uint32_t)val);
                }
                    break;
                case Envelope::FieldId::INSTANCE_ID:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    instance_id.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    instance_id.some((uint32_t)val);
                }
                    break;
                case Envelope::FieldId::PAYLOAD:
                    if (cbor_value_is_byte_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::vector<uint8_t> val(len);
                    cbor_value_copy_byte_string(&mapValue, val.data(), &len, NULL);
                    payload.some(val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class GenericReply : public Msg {
public:
    static uint32_t msg_id() { return FNV("GenericReply"); }
    static const char* msg_name() { return "GenericReply"; }

    typedef enum FieldId {
        REQ_ID = 0,
        ERROR_CODE = 1,
        MESSAGE = 2,
        MSG_TYPE = 3,
    } FieldId;
    /// For request/reply matching, 0 if not a request/reply
    Option<uint32_t> req_id;
    /// Error code, 0 if no error
    Option<uint32_t> error_code;
    /// Error message or additional information
    Option<std::string> message;
    /// Message type identifier , the original request
    Option<uint32_t> msg_type;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        req_id.inspect([&](const auto&) { fieldCount++; });
        error_code.inspect([&](const auto&) { fieldCount++; });
        message.inspect([&](const auto&) { fieldCount++; });
        msg_type.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        req_id.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::REQ_ID));
            cbor_check(cbor_encode_uint(&mapEncoder, req_id.unwrap()));
        });
        error_code.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::ERROR_CODE));
            cbor_check(cbor_encode_uint(&mapEncoder, error_code.unwrap()));
        });
        message.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::MESSAGE));
            cbor_check(cbor_encode_text_string(&mapEncoder, message.unwrap().c_str(), message.unwrap().length()));
        });
        msg_type.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::MSG_TYPE));
            cbor_check(cbor_encode_uint(&mapEncoder, msg_type.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a GenericReply from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for GenericReply");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case GenericReply::FieldId::REQ_ID:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    req_id.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    req_id.some((uint32_t)val);
                }
                    break;
                case GenericReply::FieldId::ERROR_CODE:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    error_code.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    error_code.some((uint32_t)val);
                }
                    break;
                case GenericReply::FieldId::MESSAGE:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    message.some(val);
                }
                    break;
                case GenericReply::FieldId::MSG_TYPE:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    msg_type.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    msg_type.some((uint32_t)val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class HeatingEvent : public Msg {
public:
    static uint32_t msg_id() { return FNV("HeatingEvent"); }
    static const char* msg_name() { return "HeatingEvent"; }

    typedef enum FieldId {
        TEMPERATURE = 0,
        SETPOINT = 1,
        HEATING = 2,
    } FieldId;
    /// Current temperature in Celsius
    Option<float> temperature;
    /// Setpoint temperature in Celsius
    Option<float> setpoint;
    /// Heating status
    Option<bool> heating;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        temperature.inspect([&](const auto&) { fieldCount++; });
        setpoint.inspect([&](const auto&) { fieldCount++; });
        heating.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        temperature.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::TEMPERATURE));
            cbor_check(cbor_encode_float(&mapEncoder, temperature.unwrap()));
        });
        setpoint.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::SETPOINT));
            cbor_check(cbor_encode_float(&mapEncoder, setpoint.unwrap()));
        });
        heating.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::HEATING));
            cbor_check(cbor_encode_boolean(&mapEncoder, heating.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a HeatingEvent from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for HeatingEvent");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case HeatingEvent::FieldId::TEMPERATURE:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    temperature.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    temperature.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    temperature.some((float)val);
                }
                    break;
                case HeatingEvent::FieldId::SETPOINT:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    setpoint.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    setpoint.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    setpoint.some((float)val);
                }
                    break;
                case HeatingEvent::FieldId::HEATING:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    heating.some(val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class HoverboardEvent : public Msg {
public:
    static uint32_t msg_id() { return FNV("HoverboardEvent"); }
    static const char* msg_name() { return "HoverboardEvent"; }

    typedef enum FieldId {
        CTRL_MOD = 0,
        CTRL_TYP = 1,
        CUR_MOT_MAX = 2,
        RPM_MOT_MAX = 3,
        FI_WEAK_ENA = 4,
        FI_WEAK_HI = 5,
        FI_WEAK_LO = 6,
        FI_WEAK_MAX = 7,
        PHASE_ADV_MAX_DEG = 8,
        INPUT1_RAW = 9,
        INPUT1_TYP = 10,
        INPUT1_MIN = 11,
        INPUT1_MID = 12,
        INPUT1_MAX = 13,
        INPUT1_CMD = 14,
        INPUT2_RAW = 15,
        INPUT2_TYP = 16,
        INPUT2_MIN = 17,
        INPUT2_MID = 18,
        INPUT2_MAX = 19,
        INPUT2_CMD = 20,
        AUX_INPUT1_RAW = 21,
        AUX_INPUT1_TYP = 22,
        AUX_INPUT1_MIN = 23,
        AUX_INPUT1_MID = 24,
        AUX_INPUT1_MAX = 25,
        AUX_INPUT1_CMD = 26,
        AUX_INPUT2_RAW = 27,
        AUX_INPUT2_TYP = 28,
        AUX_INPUT2_MIN = 29,
        AUX_INPUT2_MID = 30,
        AUX_INPUT2_MAX = 31,
        AUX_INPUT2_CMD = 32,
        DC_CURR = 33,
        RDC_CURR = 34,
        LDC_CURR = 35,
        CMDL = 36,
        CMDR = 37,
        SPD_AVG = 38,
        SPDL = 39,
        SPDR = 40,
        FILTER_RATE = 41,
        SPD_COEF = 42,
        STR_COEF = 43,
        BATV = 44,
        TEMP = 45,
    } FieldId;
    /// 1:Voltage 2:Speed 3:Torque
    Option<int32_t> ctrl_mod;
    /// 0:Commutation 1:Sinusoidal 2:FOC
    Option<int32_t> ctrl_typ;
    /// Max phase current A
    Option<int32_t> cur_mot_max;
    /// Max motor RPM
    Option<int32_t> rpm_mot_max;
    /// Enable field weak 0:OFF 1:ON
    Option<int32_t> fi_weak_ena;
    /// Field weak high RPM
    Option<int32_t> fi_weak_hi;
    /// Field weak low RPM
    Option<int32_t> fi_weak_lo;
    /// Field weak max current A (FOC only)
    Option<int32_t> fi_weak_max;
    /// Max Phase Adv angle Deg (SIN only)
    Option<int32_t> phase_adv_max_deg;
    /// Input1 raw value
    Option<int32_t> input1_raw;
    /// Input1 type 0:Disabled, 1:Normal Pot, 2:Middle Resting Pot, 3:Auto-detect
    Option<int32_t> input1_typ;
    /// Input1 minimum value
    Option<int32_t> input1_min;
    /// Input1 middle value
    Option<int32_t> input1_mid;
    /// Input1 maximum value
    Option<int32_t> input1_max;
    /// Input1 command value
    Option<int32_t> input1_cmd;
    /// Input2 raw value
    Option<int32_t> input2_raw;
    /// Input2 type 0:Disabled, 1:Normal Pot, 2:Middle Resting Pot, 3:Auto-detect
    Option<int32_t> input2_typ;
    /// Input2 minimum value
    Option<int32_t> input2_min;
    /// Input2 middle value
    Option<int32_t> input2_mid;
    /// Input2 maximum value
    Option<int32_t> input2_max;
    /// Input2 command value
    Option<int32_t> input2_cmd;
    /// Input1 raw value
    Option<int32_t> aux_input1_raw;
    /// Input1 type 0:Disabled, 1:Normal Pot, 2:Middle Resting Pot, 3:Auto-detect
    Option<int32_t> aux_input1_typ;
    /// Input1 minimum value
    Option<int32_t> aux_input1_min;
    /// Input1 middle value
    Option<int32_t> aux_input1_mid;
    /// Input1 maximum value
    Option<int32_t> aux_input1_max;
    /// Input1 command value
    Option<int32_t> aux_input1_cmd;
    /// Input2 raw value
    Option<int32_t> aux_input2_raw;
    /// Input2 type 0:Disabled, 1:Normal Pot, 2:Middle Resting Pot, 3:Auto-detect
    Option<int32_t> aux_input2_typ;
    /// Input2 minimum value
    Option<int32_t> aux_input2_min;
    /// Input2 middle value
    Option<int32_t> aux_input2_mid;
    /// Input2 maximum value
    Option<int32_t> aux_input2_max;
    /// Input2 command value
    Option<int32_t> aux_input2_cmd;
    /// Total DC Link current A *100
    Option<int32_t> dc_curr;
    /// Right DC Link current A *100
    Option<int32_t> rdc_curr;
    /// Left DC Link current A *100
    Option<int32_t> ldc_curr;
    /// Left Motor Command RPM
    Option<int32_t> cmdl;
    /// Right Motor Command RPM
    Option<int32_t> cmdr;
    /// Motor Measured Avg RPM
    Option<int32_t> spd_avg;
    /// Left Motor Measured RPM
    Option<int32_t> spdl;
    /// Right Motor Measured RPM
    Option<int32_t> spdr;
    /// Rate *10
    Option<int32_t> filter_rate;
    /// Speed Coefficient *10
    Option<int32_t> spd_coef;
    /// Steer Coefficient *10
    Option<int32_t> str_coef;
    /// Calibrated Battery Voltage *100
    Option<int32_t> batv;
    /// Calibrated Temperature C *10
    Option<int32_t> temp;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        ctrl_mod.inspect([&](const auto&) { fieldCount++; });
        ctrl_typ.inspect([&](const auto&) { fieldCount++; });
        cur_mot_max.inspect([&](const auto&) { fieldCount++; });
        rpm_mot_max.inspect([&](const auto&) { fieldCount++; });
        fi_weak_ena.inspect([&](const auto&) { fieldCount++; });
        fi_weak_hi.inspect([&](const auto&) { fieldCount++; });
        fi_weak_lo.inspect([&](const auto&) { fieldCount++; });
        fi_weak_max.inspect([&](const auto&) { fieldCount++; });
        phase_adv_max_deg.inspect([&](const auto&) { fieldCount++; });
        input1_raw.inspect([&](const auto&) { fieldCount++; });
        input1_typ.inspect([&](const auto&) { fieldCount++; });
        input1_min.inspect([&](const auto&) { fieldCount++; });
        input1_mid.inspect([&](const auto&) { fieldCount++; });
        input1_max.inspect([&](const auto&) { fieldCount++; });
        input1_cmd.inspect([&](const auto&) { fieldCount++; });
        input2_raw.inspect([&](const auto&) { fieldCount++; });
        input2_typ.inspect([&](const auto&) { fieldCount++; });
        input2_min.inspect([&](const auto&) { fieldCount++; });
        input2_mid.inspect([&](const auto&) { fieldCount++; });
        input2_max.inspect([&](const auto&) { fieldCount++; });
        input2_cmd.inspect([&](const auto&) { fieldCount++; });
        aux_input1_raw.inspect([&](const auto&) { fieldCount++; });
        aux_input1_typ.inspect([&](const auto&) { fieldCount++; });
        aux_input1_min.inspect([&](const auto&) { fieldCount++; });
        aux_input1_mid.inspect([&](const auto&) { fieldCount++; });
        aux_input1_max.inspect([&](const auto&) { fieldCount++; });
        aux_input1_cmd.inspect([&](const auto&) { fieldCount++; });
        aux_input2_raw.inspect([&](const auto&) { fieldCount++; });
        aux_input2_typ.inspect([&](const auto&) { fieldCount++; });
        aux_input2_min.inspect([&](const auto&) { fieldCount++; });
        aux_input2_mid.inspect([&](const auto&) { fieldCount++; });
        aux_input2_max.inspect([&](const auto&) { fieldCount++; });
        aux_input2_cmd.inspect([&](const auto&) { fieldCount++; });
        dc_curr.inspect([&](const auto&) { fieldCount++; });
        rdc_curr.inspect([&](const auto&) { fieldCount++; });
        ldc_curr.inspect([&](const auto&) { fieldCount++; });
        cmdl.inspect([&](const auto&) { fieldCount++; });
        cmdr.inspect([&](const auto&) { fieldCount++; });
        spd_avg.inspect([&](const auto&) { fieldCount++; });
        spdl.inspect([&](const auto&) { fieldCount++; });
        spdr.inspect([&](const auto&) { fieldCount++; });
        filter_rate.inspect([&](const auto&) { fieldCount++; });
        spd_coef.inspect([&](const auto&) { fieldCount++; });
        str_coef.inspect([&](const auto&) { fieldCount++; });
        batv.inspect([&](const auto&) { fieldCount++; });
        temp.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        ctrl_mod.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::CTRL_MOD));
            cbor_check(cbor_encode_int(&mapEncoder, ctrl_mod.unwrap()));
        });
        ctrl_typ.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::CTRL_TYP));
            cbor_check(cbor_encode_int(&mapEncoder, ctrl_typ.unwrap()));
        });
        cur_mot_max.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::CUR_MOT_MAX));
            cbor_check(cbor_encode_int(&mapEncoder, cur_mot_max.unwrap()));
        });
        rpm_mot_max.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::RPM_MOT_MAX));
            cbor_check(cbor_encode_int(&mapEncoder, rpm_mot_max.unwrap()));
        });
        fi_weak_ena.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::FI_WEAK_ENA));
            cbor_check(cbor_encode_int(&mapEncoder, fi_weak_ena.unwrap()));
        });
        fi_weak_hi.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::FI_WEAK_HI));
            cbor_check(cbor_encode_int(&mapEncoder, fi_weak_hi.unwrap()));
        });
        fi_weak_lo.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::FI_WEAK_LO));
            cbor_check(cbor_encode_int(&mapEncoder, fi_weak_lo.unwrap()));
        });
        fi_weak_max.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::FI_WEAK_MAX));
            cbor_check(cbor_encode_int(&mapEncoder, fi_weak_max.unwrap()));
        });
        phase_adv_max_deg.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::PHASE_ADV_MAX_DEG));
            cbor_check(cbor_encode_int(&mapEncoder, phase_adv_max_deg.unwrap()));
        });
        input1_raw.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::INPUT1_RAW));
            cbor_check(cbor_encode_int(&mapEncoder, input1_raw.unwrap()));
        });
        input1_typ.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::INPUT1_TYP));
            cbor_check(cbor_encode_int(&mapEncoder, input1_typ.unwrap()));
        });
        input1_min.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::INPUT1_MIN));
            cbor_check(cbor_encode_int(&mapEncoder, input1_min.unwrap()));
        });
        input1_mid.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::INPUT1_MID));
            cbor_check(cbor_encode_int(&mapEncoder, input1_mid.unwrap()));
        });
        input1_max.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::INPUT1_MAX));
            cbor_check(cbor_encode_int(&mapEncoder, input1_max.unwrap()));
        });
        input1_cmd.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::INPUT1_CMD));
            cbor_check(cbor_encode_int(&mapEncoder, input1_cmd.unwrap()));
        });
        input2_raw.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::INPUT2_RAW));
            cbor_check(cbor_encode_int(&mapEncoder, input2_raw.unwrap()));
        });
        input2_typ.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::INPUT2_TYP));
            cbor_check(cbor_encode_int(&mapEncoder, input2_typ.unwrap()));
        });
        input2_min.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::INPUT2_MIN));
            cbor_check(cbor_encode_int(&mapEncoder, input2_min.unwrap()));
        });
        input2_mid.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::INPUT2_MID));
            cbor_check(cbor_encode_int(&mapEncoder, input2_mid.unwrap()));
        });
        input2_max.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::INPUT2_MAX));
            cbor_check(cbor_encode_int(&mapEncoder, input2_max.unwrap()));
        });
        input2_cmd.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::INPUT2_CMD));
            cbor_check(cbor_encode_int(&mapEncoder, input2_cmd.unwrap()));
        });
        aux_input1_raw.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AUX_INPUT1_RAW));
            cbor_check(cbor_encode_int(&mapEncoder, aux_input1_raw.unwrap()));
        });
        aux_input1_typ.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AUX_INPUT1_TYP));
            cbor_check(cbor_encode_int(&mapEncoder, aux_input1_typ.unwrap()));
        });
        aux_input1_min.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AUX_INPUT1_MIN));
            cbor_check(cbor_encode_int(&mapEncoder, aux_input1_min.unwrap()));
        });
        aux_input1_mid.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AUX_INPUT1_MID));
            cbor_check(cbor_encode_int(&mapEncoder, aux_input1_mid.unwrap()));
        });
        aux_input1_max.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AUX_INPUT1_MAX));
            cbor_check(cbor_encode_int(&mapEncoder, aux_input1_max.unwrap()));
        });
        aux_input1_cmd.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AUX_INPUT1_CMD));
            cbor_check(cbor_encode_int(&mapEncoder, aux_input1_cmd.unwrap()));
        });
        aux_input2_raw.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AUX_INPUT2_RAW));
            cbor_check(cbor_encode_int(&mapEncoder, aux_input2_raw.unwrap()));
        });
        aux_input2_typ.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AUX_INPUT2_TYP));
            cbor_check(cbor_encode_int(&mapEncoder, aux_input2_typ.unwrap()));
        });
        aux_input2_min.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AUX_INPUT2_MIN));
            cbor_check(cbor_encode_int(&mapEncoder, aux_input2_min.unwrap()));
        });
        aux_input2_mid.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AUX_INPUT2_MID));
            cbor_check(cbor_encode_int(&mapEncoder, aux_input2_mid.unwrap()));
        });
        aux_input2_max.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AUX_INPUT2_MAX));
            cbor_check(cbor_encode_int(&mapEncoder, aux_input2_max.unwrap()));
        });
        aux_input2_cmd.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AUX_INPUT2_CMD));
            cbor_check(cbor_encode_int(&mapEncoder, aux_input2_cmd.unwrap()));
        });
        dc_curr.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::DC_CURR));
            cbor_check(cbor_encode_int(&mapEncoder, dc_curr.unwrap()));
        });
        rdc_curr.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::RDC_CURR));
            cbor_check(cbor_encode_int(&mapEncoder, rdc_curr.unwrap()));
        });
        ldc_curr.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::LDC_CURR));
            cbor_check(cbor_encode_int(&mapEncoder, ldc_curr.unwrap()));
        });
        cmdl.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::CMDL));
            cbor_check(cbor_encode_int(&mapEncoder, cmdl.unwrap()));
        });
        cmdr.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::CMDR));
            cbor_check(cbor_encode_int(&mapEncoder, cmdr.unwrap()));
        });
        spd_avg.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::SPD_AVG));
            cbor_check(cbor_encode_int(&mapEncoder, spd_avg.unwrap()));
        });
        spdl.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::SPDL));
            cbor_check(cbor_encode_int(&mapEncoder, spdl.unwrap()));
        });
        spdr.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::SPDR));
            cbor_check(cbor_encode_int(&mapEncoder, spdr.unwrap()));
        });
        filter_rate.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::FILTER_RATE));
            cbor_check(cbor_encode_int(&mapEncoder, filter_rate.unwrap()));
        });
        spd_coef.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::SPD_COEF));
            cbor_check(cbor_encode_int(&mapEncoder, spd_coef.unwrap()));
        });
        str_coef.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::STR_COEF));
            cbor_check(cbor_encode_int(&mapEncoder, str_coef.unwrap()));
        });
        batv.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BATV));
            cbor_check(cbor_encode_int(&mapEncoder, batv.unwrap()));
        });
        temp.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::TEMP));
            cbor_check(cbor_encode_int(&mapEncoder, temp.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a HoverboardEvent from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for HoverboardEvent");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case HoverboardEvent::FieldId::CTRL_MOD:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    ctrl_mod.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    ctrl_mod.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::CTRL_TYP:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    ctrl_typ.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    ctrl_typ.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::CUR_MOT_MAX:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    cur_mot_max.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    cur_mot_max.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::RPM_MOT_MAX:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    rpm_mot_max.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    rpm_mot_max.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::FI_WEAK_ENA:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    fi_weak_ena.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    fi_weak_ena.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::FI_WEAK_HI:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    fi_weak_hi.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    fi_weak_hi.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::FI_WEAK_LO:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    fi_weak_lo.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    fi_weak_lo.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::FI_WEAK_MAX:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    fi_weak_max.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    fi_weak_max.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::PHASE_ADV_MAX_DEG:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    phase_adv_max_deg.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    phase_adv_max_deg.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::INPUT1_RAW:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    input1_raw.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    input1_raw.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::INPUT1_TYP:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    input1_typ.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    input1_typ.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::INPUT1_MIN:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    input1_min.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    input1_min.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::INPUT1_MID:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    input1_mid.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    input1_mid.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::INPUT1_MAX:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    input1_max.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    input1_max.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::INPUT1_CMD:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    input1_cmd.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    input1_cmd.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::INPUT2_RAW:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    input2_raw.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    input2_raw.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::INPUT2_TYP:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    input2_typ.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    input2_typ.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::INPUT2_MIN:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    input2_min.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    input2_min.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::INPUT2_MID:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    input2_mid.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    input2_mid.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::INPUT2_MAX:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    input2_max.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    input2_max.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::INPUT2_CMD:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    input2_cmd.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    input2_cmd.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::AUX_INPUT1_RAW:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    aux_input1_raw.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    aux_input1_raw.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::AUX_INPUT1_TYP:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    aux_input1_typ.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    aux_input1_typ.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::AUX_INPUT1_MIN:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    aux_input1_min.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    aux_input1_min.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::AUX_INPUT1_MID:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    aux_input1_mid.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    aux_input1_mid.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::AUX_INPUT1_MAX:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    aux_input1_max.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    aux_input1_max.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::AUX_INPUT1_CMD:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    aux_input1_cmd.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    aux_input1_cmd.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::AUX_INPUT2_RAW:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    aux_input2_raw.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    aux_input2_raw.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::AUX_INPUT2_TYP:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    aux_input2_typ.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    aux_input2_typ.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::AUX_INPUT2_MIN:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    aux_input2_min.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    aux_input2_min.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::AUX_INPUT2_MID:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    aux_input2_mid.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    aux_input2_mid.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::AUX_INPUT2_MAX:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    aux_input2_max.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    aux_input2_max.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::AUX_INPUT2_CMD:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    aux_input2_cmd.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    aux_input2_cmd.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::DC_CURR:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    dc_curr.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    dc_curr.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::RDC_CURR:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    rdc_curr.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    rdc_curr.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::LDC_CURR:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    ldc_curr.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    ldc_curr.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::CMDL:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    cmdl.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    cmdl.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::CMDR:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    cmdr.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    cmdr.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::SPD_AVG:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    spd_avg.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    spd_avg.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::SPDL:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    spdl.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    spdl.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::SPDR:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    spdr.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    spdr.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::FILTER_RATE:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    filter_rate.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    filter_rate.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::SPD_COEF:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    spd_coef.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    spd_coef.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::STR_COEF:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    str_coef.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    str_coef.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::BATV:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    batv.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    batv.some((int32_t)val);
                }
                    break;
                case HoverboardEvent::FieldId::TEMP:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    temp.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    temp.some((int32_t)val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class HoverboardRequest : public Msg {
public:
    static uint32_t msg_id() { return FNV("HoverboardRequest"); }
    static const char* msg_name() { return "HoverboardRequest"; }

    typedef enum FieldId {
        REQ_ID = 0,
        SPEED = 1,
        STEER = 2,
    } FieldId;
    /// For request/reply matching, 0 if not a request/reply
    Option<uint32_t> req_id;
    /// Speed command for the hoverboard
    Option<int32_t> speed;
    /// Steering command for the hoverboard
    Option<int32_t> steer;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        req_id.inspect([&](const auto&) { fieldCount++; });
        speed.inspect([&](const auto&) { fieldCount++; });
        steer.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        req_id.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::REQ_ID));
            cbor_check(cbor_encode_uint(&mapEncoder, req_id.unwrap()));
        });
        speed.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::SPEED));
            cbor_check(cbor_encode_int(&mapEncoder, speed.unwrap()));
        });
        steer.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::STEER));
            cbor_check(cbor_encode_int(&mapEncoder, steer.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a HoverboardRequest from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for HoverboardRequest");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case HoverboardRequest::FieldId::REQ_ID:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    req_id.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    req_id.some((uint32_t)val);
                }
                    break;
                case HoverboardRequest::FieldId::SPEED:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    speed.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    speed.some((int32_t)val);
                }
                    break;
                case HoverboardRequest::FieldId::STEER:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    steer.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    steer.some((int32_t)val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class ImuEvent : public Msg {
public:
    static uint32_t msg_id() { return FNV("ImuEvent"); }
    static const char* msg_name() { return "ImuEvent"; }

    typedef enum FieldId {
        GYRO_X = 0,
        GYRO_Y = 1,
        GYRO_Z = 2,
        ACCEL_X = 3,
        ACCEL_Y = 4,
        ACCEL_Z = 5,
    } FieldId;
    /// Gyroscope X axis in deg/s
    Option<float> gyro_x;
    /// Gyroscope Y axis in deg/s
    Option<float> gyro_y;
    /// Gyroscope Z axis in deg/s
    Option<float> gyro_z;
    /// Accelerometer X axis in m/s^2
    Option<float> accel_x;
    /// Accelerometer Y axis in m/s^2
    Option<float> accel_y;
    /// Accelerometer Z axis in m/s^2
    Option<float> accel_z;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        gyro_x.inspect([&](const auto&) { fieldCount++; });
        gyro_y.inspect([&](const auto&) { fieldCount++; });
        gyro_z.inspect([&](const auto&) { fieldCount++; });
        accel_x.inspect([&](const auto&) { fieldCount++; });
        accel_y.inspect([&](const auto&) { fieldCount++; });
        accel_z.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        gyro_x.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::GYRO_X));
            cbor_check(cbor_encode_float(&mapEncoder, gyro_x.unwrap()));
        });
        gyro_y.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::GYRO_Y));
            cbor_check(cbor_encode_float(&mapEncoder, gyro_y.unwrap()));
        });
        gyro_z.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::GYRO_Z));
            cbor_check(cbor_encode_float(&mapEncoder, gyro_z.unwrap()));
        });
        accel_x.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::ACCEL_X));
            cbor_check(cbor_encode_float(&mapEncoder, accel_x.unwrap()));
        });
        accel_y.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::ACCEL_Y));
            cbor_check(cbor_encode_float(&mapEncoder, accel_y.unwrap()));
        });
        accel_z.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::ACCEL_Z));
            cbor_check(cbor_encode_float(&mapEncoder, accel_z.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a ImuEvent from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for ImuEvent");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case ImuEvent::FieldId::GYRO_X:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    gyro_x.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    gyro_x.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    gyro_x.some((float)val);
                }
                    break;
                case ImuEvent::FieldId::GYRO_Y:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    gyro_y.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    gyro_y.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    gyro_y.some((float)val);
                }
                    break;
                case ImuEvent::FieldId::GYRO_Z:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    gyro_z.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    gyro_z.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    gyro_z.some((float)val);
                }
                    break;
                case ImuEvent::FieldId::ACCEL_X:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    accel_x.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    accel_x.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    accel_x.some((float)val);
                }
                    break;
                case ImuEvent::FieldId::ACCEL_Y:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    accel_y.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    accel_y.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    accel_y.some((float)val);
                }
                    break;
                case ImuEvent::FieldId::ACCEL_Z:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    accel_z.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    accel_z.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    accel_z.some((float)val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class Max31855Event : public Msg {
public:
    static uint32_t msg_id() { return FNV("Max31855Event"); }
    static const char* msg_name() { return "Max31855Event"; }

    typedef enum FieldId {
        THERMOCOUPLE_TEMP = 0,
        INTERNAL_TEMP = 1,
        FAULT = 2,
        FAULT_SHORT_VCC = 3,
        FAULT_SHORT_GND = 4,
        FAULT_OPEN_TC = 5,
    } FieldId;
    /// Thermocouple temperature in Celsius
    Option<float> thermocouple_temp;
    /// Internal temperature in Celsius
    Option<float> internal_temp;
    /// Fault detected
    Option<bool> fault;
    /// Short to VCC detected
    Option<bool> fault_short_vcc;
    /// Short to GND detected
    Option<bool> fault_short_gnd;
    /// Open thermocouple detected
    Option<bool> fault_open_tc;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        thermocouple_temp.inspect([&](const auto&) { fieldCount++; });
        internal_temp.inspect([&](const auto&) { fieldCount++; });
        fault.inspect([&](const auto&) { fieldCount++; });
        fault_short_vcc.inspect([&](const auto&) { fieldCount++; });
        fault_short_gnd.inspect([&](const auto&) { fieldCount++; });
        fault_open_tc.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        thermocouple_temp.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::THERMOCOUPLE_TEMP));
            cbor_check(cbor_encode_float(&mapEncoder, thermocouple_temp.unwrap()));
        });
        internal_temp.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::INTERNAL_TEMP));
            cbor_check(cbor_encode_float(&mapEncoder, internal_temp.unwrap()));
        });
        fault.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::FAULT));
            cbor_check(cbor_encode_boolean(&mapEncoder, fault.unwrap()));
        });
        fault_short_vcc.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::FAULT_SHORT_VCC));
            cbor_check(cbor_encode_boolean(&mapEncoder, fault_short_vcc.unwrap()));
        });
        fault_short_gnd.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::FAULT_SHORT_GND));
            cbor_check(cbor_encode_boolean(&mapEncoder, fault_short_gnd.unwrap()));
        });
        fault_open_tc.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::FAULT_OPEN_TC));
            cbor_check(cbor_encode_boolean(&mapEncoder, fault_open_tc.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a Max31855Event from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for Max31855Event");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case Max31855Event::FieldId::THERMOCOUPLE_TEMP:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    thermocouple_temp.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    thermocouple_temp.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    thermocouple_temp.some((float)val);
                }
                    break;
                case Max31855Event::FieldId::INTERNAL_TEMP:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    internal_temp.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    internal_temp.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    internal_temp.some((float)val);
                }
                    break;
                case Max31855Event::FieldId::FAULT:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    fault.some(val);
                }
                    break;
                case Max31855Event::FieldId::FAULT_SHORT_VCC:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    fault_short_vcc.some(val);
                }
                    break;
                case Max31855Event::FieldId::FAULT_SHORT_GND:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    fault_short_gnd.some(val);
                }
                    break;
                case Max31855Event::FieldId::FAULT_OPEN_TC:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    fault_open_tc.some(val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class PingReply : public Msg {
public:
    static uint32_t msg_id() { return FNV("PingReply"); }
    static const char* msg_name() { return "PingReply"; }

    typedef enum FieldId {
        REQ_ID = 0,
        TIMESTAMP = 1,
    } FieldId;
    Option<uint32_t> req_id;
    /// Timestamp in milliseconds since epoch
    Option<uint64_t> timestamp;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        req_id.inspect([&](const auto&) { fieldCount++; });
        timestamp.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        req_id.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::REQ_ID));
            cbor_check(cbor_encode_uint(&mapEncoder, req_id.unwrap()));
        });
        timestamp.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::TIMESTAMP));
            cbor_check(cbor_encode_uint(&mapEncoder, timestamp.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a PingReply from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for PingReply");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case PingReply::FieldId::REQ_ID:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    req_id.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    req_id.some((uint32_t)val);
                }
                    break;
                case PingReply::FieldId::TIMESTAMP:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    timestamp.some(val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    timestamp.some((uint64_t)val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class PingRequest : public Msg {
public:
    static uint32_t msg_id() { return FNV("PingRequest"); }
    static const char* msg_name() { return "PingRequest"; }

    typedef enum FieldId {
        REQ_ID = 0,
        TIMESTAMP = 1,
    } FieldId;
    Option<uint32_t> req_id;
    /// Timestamp in milliseconds since epoch
    Option<uint64_t> timestamp;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        req_id.inspect([&](const auto&) { fieldCount++; });
        timestamp.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        req_id.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::REQ_ID));
            cbor_check(cbor_encode_uint(&mapEncoder, req_id.unwrap()));
        });
        timestamp.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::TIMESTAMP));
            cbor_check(cbor_encode_uint(&mapEncoder, timestamp.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a PingRequest from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for PingRequest");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case PingRequest::FieldId::REQ_ID:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    req_id.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    req_id.some((uint32_t)val);
                }
                    break;
                case PingRequest::FieldId::TIMESTAMP:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    timestamp.some(val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    timestamp.some((uint64_t)val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class Ps4Event : public Msg {
public:
    static uint32_t msg_id() { return FNV("Ps4Event"); }
    static const char* msg_name() { return "Ps4Event"; }

    typedef enum FieldId {
        BUTTON_LEFT = 0,
        BUTTON_RIGHT = 1,
        BUTTON_UP = 2,
        BUTTON_DOWN = 3,
        BUTTON_SQUARE = 4,
        BUTTON_CROSS = 5,
        BUTTON_CIRCLE = 6,
        BUTTON_TRIANGLE = 7,
        BUTTON_LEFT_SHOULDER = 8,
        BUTTON_RIGHT_SHOULDER = 9,
        BUTTON_LEFT_TRIGGER = 10,
        BUTTON_RIGHT_TRIGGER = 11,
        BUTTON_LEFT_JOYSTICK = 12,
        BUTTON_RIGHT_JOYSTICK = 13,
        BUTTON_SHARE = 14,
        BUTTON_OPTIONS = 15,
        BUTTON_TOUCHPAD = 16,
        BUTTON_PS = 17,
        AXIS_LX = 18,
        AXIS_LY = 19,
        AXIS_RX = 20,
        AXIS_RY = 21,
        GYRO_X = 22,
        GYRO_Y = 23,
        GYRO_Z = 24,
        ACCEL_X = 25,
        ACCEL_Y = 26,
        ACCEL_Z = 27,
        CONNECTED = 28,
        BATTERY_LEVEL = 29,
        BLUETOOTH = 30,
        DEBUG = 31,
        TEMP = 32,
    } FieldId;
    Option<bool> button_left;
    Option<bool> button_right;
    Option<bool> button_up;
    Option<bool> button_down;
    Option<bool> button_square;
    Option<bool> button_cross;
    Option<bool> button_circle;
    Option<bool> button_triangle;
    Option<bool> button_left_shoulder;
    Option<bool> button_right_shoulder;
    Option<bool> button_left_trigger;
    Option<bool> button_right_trigger;
    Option<bool> button_left_joystick;
    Option<bool> button_right_joystick;
    Option<bool> button_share;
    Option<bool> button_options;
    Option<bool> button_touchpad;
    Option<bool> button_ps;
    Option<int32_t> axis_lx;
    Option<int32_t> axis_ly;
    Option<int32_t> axis_rx;
    Option<int32_t> axis_ry;
    Option<int32_t> gyro_x;
    Option<int32_t> gyro_y;
    Option<int32_t> gyro_z;
    Option<int32_t> accel_x;
    Option<int32_t> accel_y;
    Option<int32_t> accel_z;
    Option<bool> connected;
    Option<int32_t> battery_level;
    Option<bool> bluetooth;
    Option<std::string> debug;
    Option<int32_t> temp;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        button_left.inspect([&](const auto&) { fieldCount++; });
        button_right.inspect([&](const auto&) { fieldCount++; });
        button_up.inspect([&](const auto&) { fieldCount++; });
        button_down.inspect([&](const auto&) { fieldCount++; });
        button_square.inspect([&](const auto&) { fieldCount++; });
        button_cross.inspect([&](const auto&) { fieldCount++; });
        button_circle.inspect([&](const auto&) { fieldCount++; });
        button_triangle.inspect([&](const auto&) { fieldCount++; });
        button_left_shoulder.inspect([&](const auto&) { fieldCount++; });
        button_right_shoulder.inspect([&](const auto&) { fieldCount++; });
        button_left_trigger.inspect([&](const auto&) { fieldCount++; });
        button_right_trigger.inspect([&](const auto&) { fieldCount++; });
        button_left_joystick.inspect([&](const auto&) { fieldCount++; });
        button_right_joystick.inspect([&](const auto&) { fieldCount++; });
        button_share.inspect([&](const auto&) { fieldCount++; });
        button_options.inspect([&](const auto&) { fieldCount++; });
        button_touchpad.inspect([&](const auto&) { fieldCount++; });
        button_ps.inspect([&](const auto&) { fieldCount++; });
        axis_lx.inspect([&](const auto&) { fieldCount++; });
        axis_ly.inspect([&](const auto&) { fieldCount++; });
        axis_rx.inspect([&](const auto&) { fieldCount++; });
        axis_ry.inspect([&](const auto&) { fieldCount++; });
        gyro_x.inspect([&](const auto&) { fieldCount++; });
        gyro_y.inspect([&](const auto&) { fieldCount++; });
        gyro_z.inspect([&](const auto&) { fieldCount++; });
        accel_x.inspect([&](const auto&) { fieldCount++; });
        accel_y.inspect([&](const auto&) { fieldCount++; });
        accel_z.inspect([&](const auto&) { fieldCount++; });
        connected.inspect([&](const auto&) { fieldCount++; });
        battery_level.inspect([&](const auto&) { fieldCount++; });
        bluetooth.inspect([&](const auto&) { fieldCount++; });
        debug.inspect([&](const auto&) { fieldCount++; });
        temp.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        button_left.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_LEFT));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_left.unwrap()));
        });
        button_right.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_RIGHT));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_right.unwrap()));
        });
        button_up.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_UP));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_up.unwrap()));
        });
        button_down.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_DOWN));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_down.unwrap()));
        });
        button_square.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_SQUARE));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_square.unwrap()));
        });
        button_cross.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_CROSS));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_cross.unwrap()));
        });
        button_circle.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_CIRCLE));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_circle.unwrap()));
        });
        button_triangle.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_TRIANGLE));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_triangle.unwrap()));
        });
        button_left_shoulder.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_LEFT_SHOULDER));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_left_shoulder.unwrap()));
        });
        button_right_shoulder.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_RIGHT_SHOULDER));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_right_shoulder.unwrap()));
        });
        button_left_trigger.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_LEFT_TRIGGER));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_left_trigger.unwrap()));
        });
        button_right_trigger.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_RIGHT_TRIGGER));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_right_trigger.unwrap()));
        });
        button_left_joystick.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_LEFT_JOYSTICK));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_left_joystick.unwrap()));
        });
        button_right_joystick.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_RIGHT_JOYSTICK));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_right_joystick.unwrap()));
        });
        button_share.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_SHARE));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_share.unwrap()));
        });
        button_options.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_OPTIONS));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_options.unwrap()));
        });
        button_touchpad.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_TOUCHPAD));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_touchpad.unwrap()));
        });
        button_ps.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUTTON_PS));
            cbor_check(cbor_encode_boolean(&mapEncoder, button_ps.unwrap()));
        });
        axis_lx.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AXIS_LX));
            cbor_check(cbor_encode_int(&mapEncoder, axis_lx.unwrap()));
        });
        axis_ly.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AXIS_LY));
            cbor_check(cbor_encode_int(&mapEncoder, axis_ly.unwrap()));
        });
        axis_rx.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AXIS_RX));
            cbor_check(cbor_encode_int(&mapEncoder, axis_rx.unwrap()));
        });
        axis_ry.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::AXIS_RY));
            cbor_check(cbor_encode_int(&mapEncoder, axis_ry.unwrap()));
        });
        gyro_x.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::GYRO_X));
            cbor_check(cbor_encode_int(&mapEncoder, gyro_x.unwrap()));
        });
        gyro_y.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::GYRO_Y));
            cbor_check(cbor_encode_int(&mapEncoder, gyro_y.unwrap()));
        });
        gyro_z.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::GYRO_Z));
            cbor_check(cbor_encode_int(&mapEncoder, gyro_z.unwrap()));
        });
        accel_x.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::ACCEL_X));
            cbor_check(cbor_encode_int(&mapEncoder, accel_x.unwrap()));
        });
        accel_y.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::ACCEL_Y));
            cbor_check(cbor_encode_int(&mapEncoder, accel_y.unwrap()));
        });
        accel_z.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::ACCEL_Z));
            cbor_check(cbor_encode_int(&mapEncoder, accel_z.unwrap()));
        });
        connected.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::CONNECTED));
            cbor_check(cbor_encode_boolean(&mapEncoder, connected.unwrap()));
        });
        battery_level.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BATTERY_LEVEL));
            cbor_check(cbor_encode_int(&mapEncoder, battery_level.unwrap()));
        });
        bluetooth.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BLUETOOTH));
            cbor_check(cbor_encode_boolean(&mapEncoder, bluetooth.unwrap()));
        });
        debug.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::DEBUG));
            cbor_check(cbor_encode_text_string(&mapEncoder, debug.unwrap().c_str(), debug.unwrap().length()));
        });
        temp.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::TEMP));
            cbor_check(cbor_encode_int(&mapEncoder, temp.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a Ps4Event from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for Ps4Event");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case Ps4Event::FieldId::BUTTON_LEFT:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_left.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_RIGHT:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_right.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_UP:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_up.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_DOWN:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_down.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_SQUARE:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_square.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_CROSS:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_cross.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_CIRCLE:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_circle.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_TRIANGLE:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_triangle.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_LEFT_SHOULDER:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_left_shoulder.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_RIGHT_SHOULDER:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_right_shoulder.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_LEFT_TRIGGER:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_left_trigger.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_RIGHT_TRIGGER:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_right_trigger.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_LEFT_JOYSTICK:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_left_joystick.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_RIGHT_JOYSTICK:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_right_joystick.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_SHARE:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_share.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_OPTIONS:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_options.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_TOUCHPAD:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_touchpad.some(val);
                }
                    break;
                case Ps4Event::FieldId::BUTTON_PS:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    button_ps.some(val);
                }
                    break;
                case Ps4Event::FieldId::AXIS_LX:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    axis_lx.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    axis_lx.some((int32_t)val);
                }
                    break;
                case Ps4Event::FieldId::AXIS_LY:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    axis_ly.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    axis_ly.some((int32_t)val);
                }
                    break;
                case Ps4Event::FieldId::AXIS_RX:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    axis_rx.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    axis_rx.some((int32_t)val);
                }
                    break;
                case Ps4Event::FieldId::AXIS_RY:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    axis_ry.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    axis_ry.some((int32_t)val);
                }
                    break;
                case Ps4Event::FieldId::GYRO_X:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    gyro_x.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    gyro_x.some((int32_t)val);
                }
                    break;
                case Ps4Event::FieldId::GYRO_Y:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    gyro_y.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    gyro_y.some((int32_t)val);
                }
                    break;
                case Ps4Event::FieldId::GYRO_Z:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    gyro_z.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    gyro_z.some((int32_t)val);
                }
                    break;
                case Ps4Event::FieldId::ACCEL_X:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    accel_x.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    accel_x.some((int32_t)val);
                }
                    break;
                case Ps4Event::FieldId::ACCEL_Y:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    accel_y.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    accel_y.some((int32_t)val);
                }
                    break;
                case Ps4Event::FieldId::ACCEL_Z:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    accel_z.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    accel_z.some((int32_t)val);
                }
                    break;
                case Ps4Event::FieldId::CONNECTED:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    connected.some(val);
                }
                    break;
                case Ps4Event::FieldId::BATTERY_LEVEL:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    battery_level.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    battery_level.some((int32_t)val);
                }
                    break;
                case Ps4Event::FieldId::BLUETOOTH:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    bluetooth.some(val);
                }
                    break;
                case Ps4Event::FieldId::DEBUG:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    debug.some(val);
                }
                    break;
                case Ps4Event::FieldId::TEMP:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    temp.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    temp.some((int32_t)val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class Ps4Request : public Msg {
public:
    static uint32_t msg_id() { return FNV("Ps4Request"); }
    static const char* msg_name() { return "Ps4Request"; }

    typedef enum FieldId {
        REQ_ID = 0,
        RUMBLE_SMALL = 1,
        RUMBLE_LARGE = 2,
        LED_RED = 3,
        LED_GREEN = 4,
        LED_BLUE = 5,
        LED_FLASH_ON = 6,
        LED_FLASH_OFF = 7,
    } FieldId;
    /// For request/reply matching, 0 if not a request/reply
    Option<uint32_t> req_id;
    Option<int32_t> rumble_small;
    Option<int32_t> rumble_large;
    Option<int32_t> led_red;
    Option<int32_t> led_green;
    Option<int32_t> led_blue;
    Option<int32_t> led_flash_on;
    Option<int32_t> led_flash_off;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        req_id.inspect([&](const auto&) { fieldCount++; });
        rumble_small.inspect([&](const auto&) { fieldCount++; });
        rumble_large.inspect([&](const auto&) { fieldCount++; });
        led_red.inspect([&](const auto&) { fieldCount++; });
        led_green.inspect([&](const auto&) { fieldCount++; });
        led_blue.inspect([&](const auto&) { fieldCount++; });
        led_flash_on.inspect([&](const auto&) { fieldCount++; });
        led_flash_off.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        req_id.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::REQ_ID));
            cbor_check(cbor_encode_uint(&mapEncoder, req_id.unwrap()));
        });
        rumble_small.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::RUMBLE_SMALL));
            cbor_check(cbor_encode_int(&mapEncoder, rumble_small.unwrap()));
        });
        rumble_large.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::RUMBLE_LARGE));
            cbor_check(cbor_encode_int(&mapEncoder, rumble_large.unwrap()));
        });
        led_red.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::LED_RED));
            cbor_check(cbor_encode_int(&mapEncoder, led_red.unwrap()));
        });
        led_green.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::LED_GREEN));
            cbor_check(cbor_encode_int(&mapEncoder, led_green.unwrap()));
        });
        led_blue.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::LED_BLUE));
            cbor_check(cbor_encode_int(&mapEncoder, led_blue.unwrap()));
        });
        led_flash_on.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::LED_FLASH_ON));
            cbor_check(cbor_encode_int(&mapEncoder, led_flash_on.unwrap()));
        });
        led_flash_off.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::LED_FLASH_OFF));
            cbor_check(cbor_encode_int(&mapEncoder, led_flash_off.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a Ps4Request from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for Ps4Request");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case Ps4Request::FieldId::REQ_ID:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    req_id.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    req_id.some((uint32_t)val);
                }
                    break;
                case Ps4Request::FieldId::RUMBLE_SMALL:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    rumble_small.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    rumble_small.some((int32_t)val);
                }
                    break;
                case Ps4Request::FieldId::RUMBLE_LARGE:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    rumble_large.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    rumble_large.some((int32_t)val);
                }
                    break;
                case Ps4Request::FieldId::LED_RED:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    led_red.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    led_red.some((int32_t)val);
                }
                    break;
                case Ps4Request::FieldId::LED_GREEN:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    led_green.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    led_green.some((int32_t)val);
                }
                    break;
                case Ps4Request::FieldId::LED_BLUE:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    led_blue.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    led_blue.some((int32_t)val);
                }
                    break;
                case Ps4Request::FieldId::LED_FLASH_ON:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    led_flash_on.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    led_flash_on.some((int32_t)val);
                }
                    break;
                case Ps4Request::FieldId::LED_FLASH_OFF:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    led_flash_off.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    led_flash_off.some((int32_t)val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class SysEvent : public Msg {
public:
    static uint32_t msg_id() { return FNV("SysEvent"); }
    static const char* msg_name() { return "SysEvent"; }

    typedef enum FieldId {
        UTC = 0,
        UPTIME = 1,
        FREE_HEAP = 2,
        FLASH_SIZE = 3,
        CPU_BOARD_TYPE = 4,
        BUILD_DATE_TIME = 5,
    } FieldId;
    Option<uint64_t> utc;
    Option<uint64_t> uptime;
    Option<uint64_t> free_heap;
    Option<uint64_t> flash_size;
    Option<std::string> cpu_board_type;
    Option<std::string> build_date_time;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        utc.inspect([&](const auto&) { fieldCount++; });
        uptime.inspect([&](const auto&) { fieldCount++; });
        free_heap.inspect([&](const auto&) { fieldCount++; });
        flash_size.inspect([&](const auto&) { fieldCount++; });
        cpu_board_type.inspect([&](const auto&) { fieldCount++; });
        build_date_time.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        utc.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::UTC));
            cbor_check(cbor_encode_uint(&mapEncoder, utc.unwrap()));
        });
        uptime.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::UPTIME));
            cbor_check(cbor_encode_uint(&mapEncoder, uptime.unwrap()));
        });
        free_heap.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::FREE_HEAP));
            cbor_check(cbor_encode_uint(&mapEncoder, free_heap.unwrap()));
        });
        flash_size.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::FLASH_SIZE));
            cbor_check(cbor_encode_uint(&mapEncoder, flash_size.unwrap()));
        });
        cpu_board_type.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::CPU_BOARD_TYPE));
            cbor_check(cbor_encode_text_string(&mapEncoder, cpu_board_type.unwrap().c_str(), cpu_board_type.unwrap().length()));
        });
        build_date_time.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BUILD_DATE_TIME));
            cbor_check(cbor_encode_text_string(&mapEncoder, build_date_time.unwrap().c_str(), build_date_time.unwrap().length()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a SysEvent from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for SysEvent");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case SysEvent::FieldId::UTC:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    utc.some(val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    utc.some((uint64_t)val);
                }
                    break;
                case SysEvent::FieldId::UPTIME:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    uptime.some(val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    uptime.some((uint64_t)val);
                }
                    break;
                case SysEvent::FieldId::FREE_HEAP:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    free_heap.some(val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    free_heap.some((uint64_t)val);
                }
                    break;
                case SysEvent::FieldId::FLASH_SIZE:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    flash_size.some(val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    flash_size.some((uint64_t)val);
                }
                    break;
                case SysEvent::FieldId::CPU_BOARD_TYPE:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    cpu_board_type.some(val);
                }
                    break;
                case SysEvent::FieldId::BUILD_DATE_TIME:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    build_date_time.some(val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class SysReply : public Msg {
public:
    static uint32_t msg_id() { return FNV("SysReply"); }
    static const char* msg_name() { return "SysReply"; }

    typedef enum FieldId {
        REQ_ID = 0,
        RC = 1,
        MESSAGE = 2,
    } FieldId;
    /// For request/reply matching, 0 if not a request/reply
    Option<uint32_t> req_id;
    Option<int32_t> rc;
    Option<std::string> message;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        req_id.inspect([&](const auto&) { fieldCount++; });
        rc.inspect([&](const auto&) { fieldCount++; });
        message.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        req_id.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::REQ_ID));
            cbor_check(cbor_encode_uint(&mapEncoder, req_id.unwrap()));
        });
        rc.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::RC));
            cbor_check(cbor_encode_int(&mapEncoder, rc.unwrap()));
        });
        message.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::MESSAGE));
            cbor_check(cbor_encode_text_string(&mapEncoder, message.unwrap().c_str(), message.unwrap().length()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a SysReply from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for SysReply");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case SysReply::FieldId::REQ_ID:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    req_id.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    req_id.some((uint32_t)val);
                }
                    break;
                case SysReply::FieldId::RC:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    rc.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    rc.some((int32_t)val);
                }
                    break;
                case SysReply::FieldId::MESSAGE:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    message.some(val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class SysRequest : public Msg {
public:
    static uint32_t msg_id() { return FNV("SysRequest"); }
    static const char* msg_name() { return "SysRequest"; }

    typedef enum FieldId {
        REQ_ID = 0,
        SET_TIME = 1,
        REBOOT = 2,
        CONSOLE = 3,
    } FieldId;
    /// For request/reply matching, 0 if not a request/reply
    Option<uint32_t> req_id;
    Option<uint64_t> set_time;
    Option<bool> reboot;
    Option<std::string> console;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        req_id.inspect([&](const auto&) { fieldCount++; });
        set_time.inspect([&](const auto&) { fieldCount++; });
        reboot.inspect([&](const auto&) { fieldCount++; });
        console.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        req_id.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::REQ_ID));
            cbor_check(cbor_encode_uint(&mapEncoder, req_id.unwrap()));
        });
        set_time.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::SET_TIME));
            cbor_check(cbor_encode_uint(&mapEncoder, set_time.unwrap()));
        });
        reboot.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::REBOOT));
            cbor_check(cbor_encode_boolean(&mapEncoder, reboot.unwrap()));
        });
        console.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::CONSOLE));
            cbor_check(cbor_encode_text_string(&mapEncoder, console.unwrap().c_str(), console.unwrap().length()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a SysRequest from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for SysRequest");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case SysRequest::FieldId::REQ_ID:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    req_id.some((uint32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    req_id.some((uint32_t)val);
                }
                    break;
                case SysRequest::FieldId::SET_TIME:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    set_time.some(val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    set_time.some((uint64_t)val);
                }
                    break;
                case SysRequest::FieldId::REBOOT:
                    if (cbor_value_is_boolean(&mapValue)) {
                    bool val;
                    cbor_value_get_boolean(&mapValue, &val);
                    reboot.some(val);
                }
                    break;
                case SysRequest::FieldId::CONSOLE:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    console.some(val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class UsEvent : public Msg {
public:
    static uint32_t msg_id() { return FNV("UsEvent"); }
    static const char* msg_name() { return "UsEvent"; }

    typedef enum FieldId {
        DISTANCE = 0,
        TEMPERATURE = 1,
        STATUS = 2,
    } FieldId;
    /// Distance in meters
    Option<float> distance;
    /// Temperature in Celsius
    Option<float> temperature;
    /// Status code, 0 if no error
    Option<int32_t> status;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        distance.inspect([&](const auto&) { fieldCount++; });
        temperature.inspect([&](const auto&) { fieldCount++; });
        status.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        distance.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::DISTANCE));
            cbor_check(cbor_encode_float(&mapEncoder, distance.unwrap()));
        });
        temperature.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::TEMPERATURE));
            cbor_check(cbor_encode_float(&mapEncoder, temperature.unwrap()));
        });
        status.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::STATUS));
            cbor_check(cbor_encode_int(&mapEncoder, status.unwrap()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a UsEvent from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for UsEvent");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case UsEvent::FieldId::DISTANCE:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    distance.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    distance.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    distance.some((float)val);
                }
                    break;
                case UsEvent::FieldId::TEMPERATURE:
                    if (cbor_value_is_float(&mapValue) || cbor_value_is_double(&mapValue)) {
                    float val;
                    cbor_value_get_float(&mapValue, &val);
                    temperature.some(val);
                } else if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    temperature.some((float)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    temperature.some((float)val);
                }
                    break;
                case UsEvent::FieldId::STATUS:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    status.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    status.some((int32_t)val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};



class WifiEvent : public Msg {
public:
    static uint32_t msg_id() { return FNV("WifiEvent"); }
    static const char* msg_name() { return "WifiEvent"; }

    typedef enum FieldId {
        IP = 0,
        GATEWAY = 1,
        NETMASK = 2,
        SSID = 3,
        BSSID = 4,
        CHANNEL = 5,
        RSSI = 6,
        MAC = 7,
    } FieldId;
    Option<std::string> ip;
    Option<std::string> gateway;
    Option<std::string> netmask;
    Option<std::string> ssid;
    Option<std::string> bssid;
    Option<int32_t> channel;
    Option<int32_t> rssi;
    Option<std::string> mac;

    /// Serialize this message into a CBOR map keyed by field id.
    /// Writes into the parent encoder as a single map item.
    Result<Void> encode(CborEncoder& encoder)  {
        // Count how many optional fields are set.
        uint32_t fieldCount = 0;
        ip.inspect([&](const auto&) { fieldCount++; });
        gateway.inspect([&](const auto&) { fieldCount++; });
        netmask.inspect([&](const auto&) { fieldCount++; });
        ssid.inspect([&](const auto&) { fieldCount++; });
        bssid.inspect([&](const auto&) { fieldCount++; });
        channel.inspect([&](const auto&) { fieldCount++; });
        rssi.inspect([&](const auto&) { fieldCount++; });
        mac.inspect([&](const auto&) { fieldCount++; });

        CborEncoder mapEncoder;
        RET_ERR(cbor_check(cbor_encoder_create_map(&encoder, &mapEncoder, fieldCount)));
        ip.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::IP));
            cbor_check(cbor_encode_text_string(&mapEncoder, ip.unwrap().c_str(), ip.unwrap().length()));
        });
        gateway.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::GATEWAY));
            cbor_check(cbor_encode_text_string(&mapEncoder, gateway.unwrap().c_str(), gateway.unwrap().length()));
        });
        netmask.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::NETMASK));
            cbor_check(cbor_encode_text_string(&mapEncoder, netmask.unwrap().c_str(), netmask.unwrap().length()));
        });
        ssid.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::SSID));
            cbor_check(cbor_encode_text_string(&mapEncoder, ssid.unwrap().c_str(), ssid.unwrap().length()));
        });
        bssid.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::BSSID));
            cbor_check(cbor_encode_text_string(&mapEncoder, bssid.unwrap().c_str(), bssid.unwrap().length()));
        });
        channel.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::CHANNEL));
            cbor_check(cbor_encode_int(&mapEncoder, channel.unwrap()));
        });
        rssi.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::RSSI));
            cbor_check(cbor_encode_int(&mapEncoder, rssi.unwrap()));
        });
        mac.inspect([&](const auto& value) {
            (void)value;
            cbor_check(cbor_encode_uint(&mapEncoder, FieldId::MAC));
            cbor_check(cbor_encode_text_string(&mapEncoder, mac.unwrap().c_str(), mac.unwrap().length()));
        });

        return cbor_check(cbor_encoder_close_container(&encoder, &mapEncoder));
    }

    /// Deserialize a WifiEvent from a CBOR map value.
    /// `it` must point to the map; after decoding it is advanced past the map.
     Result<Void> decode(CborValue& it) {
        if (!cbor_value_is_map(&it)) {
            return Result<Void>::Err(-1, "Expected CBOR map for WifiEvent");
        }

        CborValue mapValue;
        cbor_value_enter_container(&it, &mapValue);

        while (!cbor_value_at_end(&mapValue)) {
            // Read the map key (must be an unsigned integer — field id).
            if (!cbor_value_is_unsigned_integer(&mapValue)) {
                // Skip unknown key type and its value.
                cbor_value_advance(&mapValue);  // skip key
                if (!cbor_value_at_end(&mapValue)) {
                    cbor_value_advance(&mapValue);  // skip value
                }
                continue;
            }

            uint64_t keyVal;
            cbor_value_get_uint64(&mapValue, &keyVal);
            cbor_value_advance(&mapValue);  // advance to value

            switch ((uint32_t)keyVal) {
                case WifiEvent::FieldId::IP:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    ip.some(val);
                }
                    break;
                case WifiEvent::FieldId::GATEWAY:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    gateway.some(val);
                }
                    break;
                case WifiEvent::FieldId::NETMASK:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    netmask.some(val);
                }
                    break;
                case WifiEvent::FieldId::SSID:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    ssid.some(val);
                }
                    break;
                case WifiEvent::FieldId::BSSID:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    bssid.some(val);
                }
                    break;
                case WifiEvent::FieldId::CHANNEL:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    channel.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    channel.some((int32_t)val);
                }
                    break;
                case WifiEvent::FieldId::RSSI:
                    if (cbor_value_is_unsigned_integer(&mapValue)) {
                    uint64_t val;
                    cbor_value_get_uint64(&mapValue, &val);
                    rssi.some((int32_t)val);
                } else if (cbor_value_is_negative_integer(&mapValue)) {
                    int64_t val;
                    cbor_value_get_int64(&mapValue, &val);
                    rssi.some((int32_t)val);
                }
                    break;
                case WifiEvent::FieldId::MAC:
                    if (cbor_value_is_text_string(&mapValue)) {
                    size_t len;
                    cbor_value_get_string_length(&mapValue, &len);
                    std::string val(len, '\0');
                    cbor_value_copy_text_string(&mapValue, &val[0], &len, NULL);
                    val.resize(len);
                    mac.some(val);
                }
                    break;
                default:
                    // Unknown field id — skip value.
                    break;
            }

            cbor_value_advance(&mapValue);  // advance past value to next key (or end)
        }

        cbor_value_leave_container(&it, &mapValue);
        return Result<Void>::Ok(Void());
    }
};


