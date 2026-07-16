#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "eeprom.h"
#include "BLDC_controller.h"
#include "util.h"
#include "comms.h"

#include <limero/Log.h>
#include <limero/codec.h>
#include <limero/msgs.h>

extern "C"
{
    extern ExtY rtY_Left; /* External outputs */
    extern ExtU rtU_Left; /* External inputs */
    extern P rtP_Left;
    extern ExtY rtY_Right; /* External outputs */
    extern ExtU rtU_Right; /* External inputs */
    extern P rtP_Right;
    extern InputStruct input1[]; // input structure
    extern InputStruct input2[]; // input structure
    extern uint16_t VirtAddVarTab[NB_OF_VAR];
    extern int16_t speedAvg;    // average measured speed
    extern int16_t speedAvgAbs; // average measured speed in absolute
    extern uint8_t ctrlModReqRaw;
    extern int16_t batVoltageCalib;
    extern int16_t board_temp_deg_c;
    extern int16_t left_dc_curr;
    extern int16_t right_dc_curr;
    extern int16_t dc_curr;
    extern int16_t cmdL;
    extern int16_t cmdR;
}

void fill_hb_event(HoverboardEvent &hb_event)
{

    hb_event.ctrl_mod = ctrlModReqRaw;
    hb_event.ctrl_typ = rtP_Left.z_ctrlTypSel;
    hb_event.cur_mot_max = rtP_Left.i_max;
    hb_event.rpm_mot_max = rtP_Left.n_max;
    hb_event.fi_weak_ena = rtP_Left.b_fieldWeakEna;
    hb_event.fi_weak_hi = rtP_Left.r_fieldWeakHi;
    hb_event.fi_weak_lo = rtP_Left.r_fieldWeakLo;
    hb_event.fi_weak_max = rtP_Left.id_fieldWeakMax;
    hb_event.phase_adv_max_deg = rtP_Left.a_phaAdvMax;
    hb_event.input1_raw = input1[0].raw;
    hb_event.input1_typ = input1[0].typ;
    hb_event.input1_min = input1[0].min;
    hb_event.input1_mid = input1[0].mid;
    hb_event.input1_max = input1[0].max;
    hb_event.input1_cmd = input1[0].cmd;
    hb_event.input2_raw = input2[0].raw;
    hb_event.input2_typ = input2[0].typ;
    hb_event.input2_min = input2[0].min;
    hb_event.input2_mid = input2[0].mid;
    hb_event.input2_max = input2[0].max;
    hb_event.input2_cmd = input2[0].cmd;
    hb_event.aux_input1_raw = input1[1].raw;
    hb_event.aux_input1_typ = input1[1].typ;
    hb_event.aux_input1_min = input1[1].min;
    hb_event.aux_input1_mid = input1[1].mid;
    hb_event.aux_input1_max = input1[1].max;
    hb_event.aux_input1_cmd = input1[1].cmd;
    hb_event.aux_input2_raw = input2[1].raw;
    hb_event.aux_input2_typ = input2[1].typ;
    hb_event.aux_input2_min = input2[1].min;
    hb_event.aux_input2_mid = input2[1].mid;
    hb_event.aux_input2_max = input2[1].max;
    hb_event.aux_input2_cmd = input2[1].cmd;
    hb_event.dc_curr = dc_curr;
    hb_event.ldc_curr = left_dc_curr;
    hb_event.rdc_curr = right_dc_curr;
    hb_event.cmdl = cmdL;
    hb_event.cmdr = cmdR;
    hb_event.spd_avg = speedAvg;
    hb_event.spdl = rtY_Left.n_mot;
    hb_event.spdr = rtY_Right.n_mot;
    hb_event.filter_rate = 0;
    hb_event.spd_coef = SPEED_COEFFICIENT;
    hb_event.str_coef = STEER_COEFFICIENT;
    hb_event.batv = batVoltageCalib;
    hb_event.temp = board_temp_deg_c;
}

void fill_endpoint_announce(EndpointAnnounce &ep_announce)
{

    ep_announce.id = FNV("hoverboard");
    ep_announce.name = "hoverboard";
    ep_announce.description = "Hoverboard FOC Controller";
    ep_announce.services = std::vector<uint32_t>{FNV("HoverboardRequest")};
    ep_announce.events = std::vector<uint32_t>{FNV("HoverboardEvent")};
    ep_announce.replies = std::vector<uint32_t>{FNV("HoverboardReply")};
}

uint8_t envelope_buffer[512];

size_t encode_envelope(Envelope &envelope, uint8_t *buffer, size_t buffer_size)
{

    CborEncoder encoder;
    cbor_encoder_init(&encoder, buffer, buffer_size, 0);
    if (envelope.encode(encoder).is_err())
    {
        return 0;
    }
    size_t payload_size = cbor_encoder_get_buffer_size(&encoder, buffer);
    return payload_size;
}

size_t encode_hb_event(HoverboardEvent &hb_event, uint8_t *buffer, size_t buffer_size)
{
    CborEncoder encoder;
    cbor_encoder_init(&encoder, buffer, buffer_size, 0);
    if (hb_event.encode(encoder).is_err())
    {
        return 0;
    }
    return cbor_encoder_get_buffer_size(&encoder, buffer);
}

size_t encode_endpoint_announce(EndpointAnnounce &ep_announce, uint8_t *buffer, size_t buffer_size)
{
    CborEncoder encoder;
    cbor_encoder_init(&encoder, buffer, buffer_size, 0);
    if (ep_announce.encode(encoder).is_err())
    {
        return 0;
    }
    return cbor_encoder_get_buffer_size(&encoder, buffer);
}

Log logger(256);

bool send_announce()
{
    static uint32_t send_count = 0;
    return (send_count++ % 10) == 0;
}

HoverboardEvent hb_event;
EndpointAnnounce ep_announce;
uint8_t payload_buffer[512];
Bytes buffer;
extern "C" uint32_t get_txd(uint8_t **buffer)
{

    Envelope envelope;
    envelope.src = FNV("hoverboard");
    envelope.msg_type = HoverboardEvent::msg_id();

    if (send_announce())
    {
        fill_endpoint_announce(ep_announce);
        size_t payload_size = encode_endpoint_announce(ep_announce, payload_buffer, sizeof(payload_buffer));
        envelope.payload = Bytes(payload_buffer, payload_buffer + payload_size);
    }
    else
    {
        fill_hb_event(hb_event);
        size_t payload_size = encode_hb_event(hb_event, payload_buffer, sizeof(payload_buffer));
        envelope.payload = Bytes(payload_buffer, payload_buffer + payload_size);
    }

    size_t envelope_size = encode_envelope(envelope, envelope_buffer, sizeof(envelope_buffer));
    // re-use the envelope_buffer to encode the frame
    FrameEncoder frame_encoder(envelope_buffer, sizeof(envelope_buffer), envelope_size);

    if (frame_encoder.add_crc().is_err())
    {
        return 0;
    }
    if (frame_encoder.add_cobs().is_err())
    {
        return 0;
    }
    *buffer = frame_encoder.data();

    return frame_encoder.size();
}

void handle_rxd_frame(uint8_t *buffer, size_t size)
{
    CborParser parser;
    CborValue value;
    cbor_parser_init(buffer, size, 0, &parser, &value);

    Envelope envelope;
    if (envelope.decode(value).is_err())
    {
        return;
    }

    if (envelope.msg_type.is_some() && envelope.payload.is_some())
    {
        uint32_t msg_type = envelope.msg_type.unwrap();
        Bytes payload = envelope.payload.unwrap();
        if (msg_type == HoverboardRequest::msg_id())
        {
            HoverboardRequest request;
            CborParser payload_parser;
            CborValue payload_value;
            cbor_parser_init(payload.data(), payload.size(), 0, &payload_parser, &payload_value);
            if (request.decode(payload_value).is_ok())
            {
                // Handle the request
                logger.printf("Received HoverboardRequest\n");
            }
        }
        else
        {
            logger.printf("Received unknown message type: %u\n", msg_type);
        }
    }
}

void handle_rxd_byte(uint8_t byte)
{
    static FrameDecoder frame_decoder(512);
    if (byte == 0x00)
    {
        // End of frame, process the accumulated bytes
        if (frame_decoder.decode_cobs().is_ok() && frame_decoder.check_crc().is_ok())
        {
            handle_rxd_frame(frame_decoder.data(), frame_decoder.size());
        }
        frame_decoder.rewind();
    }
    else
    {
        if (frame_decoder.add_byte(byte).is_err())
        {
            frame_decoder.rewind();
        }
    }
}

extern "C" void handle_rxd(uint8_t *buffer, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        handle_rxd_byte(buffer[i]);
    }
}
