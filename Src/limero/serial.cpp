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
#include <limero/hb_vars.h>
#include <limero/msgs.h>

extern "C" {
    extern ExtY rtY_Left;                   /* External outputs */
    extern ExtU rtU_Left;                   /* External inputs */
    extern P    rtP_Left;

    extern ExtY rtY_Right;                  /* External outputs */
    extern ExtU rtU_Right;                      /* External inputs */
    extern P    rtP_Right;


    extern InputStruct input1[];            // input structure
    extern InputStruct input2[];            // input structure

    extern uint16_t VirtAddVarTab[NB_OF_VAR];
    extern int16_t speedAvg;                      // average measured speed
    extern int16_t speedAvgAbs;                   // average measured speed in absolute
    extern uint8_t ctrlModReqRaw;
    extern int16_t batVoltageCalib;
    extern int16_t board_temp_deg_c;
    extern int16_t left_dc_curr;
    extern int16_t right_dc_curr;
    extern int16_t dc_curr;
    extern int16_t cmdL;
    extern int16_t cmdR;
}


struct PropDescriptor {
    HbVar id;
    const char* name;
    const char* description;
    uint8_t ValueType;
    uint8_t ValueMode;
} props[] = {
    {HbVar::CTRL_MOD, "CTRL_MOD", "Ctrl mode 1:Voltage 2:Speed 3:Torque", ValueType::UINT, ValueMode::READ},
    { HbVar::CTRL_TYP, "CTRL_TYP", "Ctrl type 0:Commutation 1:Sinusoidal 2:FOC", ValueType::UINT, ValueMode::READ },
    { HbVar::CUR_MOT_MAX,"I_MOT_MAX","Max phase current A", ValueType::UINT, ValueMode::WRITE },
    { HbVar::RPM_MOT_MAX,"RPM_MOT_MAX","Max motor RPM", ValueType::UINT, ValueMode::WRITE },
    { HbVar::FI_WEAK_ENA,"FI_WEAK_ENA","Enable field weak 0:OFF 1:ON", ValueType::UINT, ValueMode::READ },
    { HbVar::FI_WEAK_HI,"FI_WEAK_HI","Field weak high RPM", ValueType::UINT, ValueMode::WRITE },
    { HbVar::FI_WEAK_LO,"FI_WEAK_LO","Field weak low RPM", ValueType::UINT, ValueMode::WRITE },
    { HbVar::FI_WEAK_MAX,"FI_WEAK_MAX","Field weak max current A(FOC only)", ValueType::UINT, ValueMode::WRITE },
    { HbVar::PHASE_ADV_MAX_DEG,"PHA_ADV_MAX","Max Phase Adv angle Deg(SIN only)", ValueType::UINT, ValueMode::WRITE },
    { HbVar::IN1_RAW,"IN1_RAW","Input1 raw value", ValueType::UINT, ValueMode::READ },
    { HbVar::IN1_TYP,"IN1_TYP","Input1 type 0:Disabled, 1:Normal Pot, 2:Middle Resting Pot, 3:Auto-detect", ValueType::UINT, ValueMode::WRITE },
    { HbVar::IN1_MIN,"IN1_MIN","Input1 minimum value", ValueType::UINT, ValueMode::WRITE },
    { HbVar::IN1_MID,"IN1_MID","Input1 middle value", ValueType::UINT, ValueMode::WRITE },
    { HbVar::IN1_MAX,"IN1_MAX","Input1 maximum value", ValueType::UINT, ValueMode::WRITE },
    { HbVar::IN1_CMD,"IN1_CMD","Input1 command value", ValueType::UINT, ValueMode::READ },
    { HbVar::IN2_RAW,"IN2_RAW","Input2 raw value", ValueType::UINT, ValueMode::READ },
    { HbVar::IN2_TYP,"IN2_TYP","Input2 type 0:Disabled, 1:Normal Pot, 2:Middle Resting Pot, 3:Auto-detect", ValueType::UINT, ValueMode::WRITE },
    { HbVar::IN2_MIN,"IN2_MIN","Input2 minimum value", ValueType::UINT, ValueMode::WRITE },
    { HbVar::IN2_MID,"IN2_MID","Input2 middle value", ValueType::UINT, ValueMode::WRITE },
    { HbVar::IN2_MAX,"IN2_MAX","Input2 maximum value", ValueType::UINT, ValueMode::WRITE },
    { HbVar::IN2_CMD,"IN2_CMD","Input2 command value", ValueType::UINT, ValueMode::READ },
    { HbVar::DC_CURR,"DC_CURR","Total DC Link current A *100", ValueType::UINT, ValueMode::READ },
    { HbVar::LDC_CURR,"LDC_CURR","Left DC Link current A *100", ValueType::UINT, ValueMode::READ },
    { HbVar::RDC_CURR,"RDC_CURR","Right DC Link current A *100", ValueType::UINT, ValueMode::READ },
    { HbVar::CMDL,"CMDL","Left Motor Command RPM", ValueType::UINT, ValueMode::READ },
    { HbVar::CMDR,"CMDR","Right Motor Command RPM", ValueType::UINT, ValueMode::READ },
    { HbVar::SPD_AVG,"SPD_AVG","Motor Measured Avg RPM", ValueType::UINT, ValueMode::READ },
    { HbVar::SPDL,"SPDL","Left Motor Measured RPM", ValueType::UINT, ValueMode::READ },
    { HbVar::SPDR,"SPDR","Right Motor Measured RPM", ValueType::UINT, ValueMode::READ },
    { HbVar::FILTER_RATE,"RATE","Rate *10", ValueType::UINT, ValueMode::READ },
    { HbVar::SPD_COEF,"SPD_COEF","Speed Coefficient *10", ValueType::UINT, ValueMode::READ },
    { HbVar::STR_COEF,"STR_COEF","Steer Coefficient *10", ValueType::UINT, ValueMode::READ },
    { HbVar::BATV,"BATV","Calibrated Battery Voltage *100", ValueType::UINT, ValueMode::READ },
    { HbVar::TEMP,"TEMP","Calibrated Temperature °C *10", ValueType::UINT, ValueMode::READ },
};



void fill_hb_event(HoverboardEvent& hb_event) {
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

void fill_endpoint_announce(EndpointAnnounce& ep_announce){
    
    ep_announce.id = FNV("hoverboard");
    ep_announce.name = "hoverboard";
    ep_announce.description = "Hoverboard FOC Controller";
    ep_announce.services = std::vector<uint32_t>{FNV("HoverboardRequest")};
    ep_announce.events = std::vector<uint32_t>{FNV("HoverboardEvent")};
    ep_announce.replies = std::vector<uint32_t>{FNV("HoverboardReply")};
}

uint8_t hb_event_buffer[512];
uint8_t ep_announce_buffer[256];
uint8_t envelope_buffer[512];
uint8_t frame_buffer[1024];

size_t encode_envelope(Envelope& envelope, uint8_t* buffer,size_t buffer_size) {
    CborEncoder encoder;
    cbor_encoder_init(&encoder, buffer, buffer_size, 0);
    if (envelope.encode(encoder).is_err()) {
        return 0;
    }
    size_t payload_size = cbor_encoder_get_buffer_size(&encoder, buffer);
    return payload_size;
}

size_t encode_hb_event(HoverboardEvent& hb_event, uint8_t* buffer,size_t buffer_size) {
    CborEncoder encoder;
    cbor_encoder_init(&encoder, buffer, buffer_size, 0);
    if (hb_event.encode(encoder).is_err()) {
        return;
    }
    size_t payload_size = cbor_encoder_get_buffer_size(&encoder, buffer);
}

size_t encode_endpoint_announce(EndpointAnnounce& ep_announce, uint8_t* buffer,size_t buffer_size) {
    CborEncoder encoder;
    cbor_encoder_init(&encoder, buffer, buffer_size, 0);
    if (ep_announce.encode(encoder).is_err()) {
        return;
    }
    size_t payload_size = cbor_encoder_get_buffer_size(&encoder, buffer);
}




Log logger(256);

static bool send_info = false;

bool toggle() {
    send_info = !send_info;
    return send_info;
}



HoverboardEvent hb_event;
Bytes buffer;
extern "C" uint32_t get_txd(uint8_t** buffer) {

    Envelope envelope;
    envelope.src = FNV("hoverboard");
    envelope.msg_type = HoverboardEvent::msg_id();

    HoverboardEvent hb_event;
    fill_hb_event(hb_event);
    size_t payload_size = encode_hb_event(hb_event, hb_event_buffer, sizeof(hb_event_buffer));    
    FrameEncoder frame_encoder(frame_buffer, sizeof(frame_buffer));

    envelope.payload = Bytes(hb_event_buffer, hb_event_buffer + payload_size);
    size_t envelope_size = encode_envelope(envelope, envelope_buffer, sizeof(envelope_buffer));

    FrameEncoder frame_encoder(frame_buffer, sizeof(frame_buffer));

    if (frame_encoder.add_crc().is_err()) {
        return 0;
    }
    if (frame_encoder.add_cobs().is_err()) {
        return 0;
    }
    *buffer = frame_encoder.data();

    return frame_encoder.size();
}





/*

Values are being translated to external<>internal format automatically.

Type	Name	Description	Can be Set	Can be saved to EEPROM
Parameter	CTRL_MOD	Ctrl mode 1:Voltage 2:Speed 3:Torque	Yes	No
Parameter	CTRL_TYP	Ctrl type 0:Commutation 1:Sinusoidal 2:FOC	Yes	No
Parameter	I_MOT_MAX	Max phase current A	Yes	Yes
Parameter	N_MOT_MAX	Max motor RPM	Yes	Yes
Parameter	FI_WEAK_ENA	Enable field weak 0:OFF 1:ON	Yes	No
Parameter	FI_WEAK_HI	Field weak high RPM	Yes	No
Parameter	FI_WEAK_LO	Field weak low RPM	Yes	No
Parameter	FI_WEAK_MAX	Field weak max current A(FOC only)	Yes	No
Parameter	PHA_ADV_MAX	Max Phase Adv angle Deg(SIN only)	Yes	No
Variable	IN1_RAW	Input1 raw value	No	No
Parameter	IN1_TYP	Input1 type 0:Disabled, 1:Normal Pot, 2:Middle Resting Pot, 3:Auto-detect	Yes	Yes
Parameter	IN1_MIN	Input1 minimum value	Yes	Yes
Parameter	IN1_MID	Input1 middle value	Yes	Yes
Parameter	IN1_MAX	Input1 maximum value	Yes	Yes
Variable	IN1_CMD	Input1 command value	No	No
Variable	IN2_RAW	Input2 raw value	No	No
Parameter	IN2_TYP	Input2 type 0:Disabled, 1:Normal Pot, 2:Middle Resting Pot, 3:Auto-detect	Yes	Yes
Parameter	IN2_MIN	Input2 minimum value	Yes	Yes
Parameter	IN2_MID	Input2 middle value	Yes	Yes
Parameter	IN2_MAX	Input2 maximum value	Yes	Yes
Variable	IN2_CMD	Input2 command value	No	No
Variable	DC_CURR	Total DC Link current A *100	No	No
Variable	LDC_CURR	Left DC Link current A *100	No	No
Variable	RDC_CURR	Right DC Link current A *100	No	No
Variable	CMDL	Left Motor Command RPM	No	No
Variable	CMDR	Right Motor Command RPM	No	No
Variable	SPD_AVG	Motor Measured Avg RPM	No	No
Variable	SPDL	Left Motor Measured RPM	No	No
Variable	SPDR	Right Motor Measured RPM	No	No
Variable	RATE	Rate *10	No	No
Variable	SPD_COEF	Speed Coefficient *10	No	No
Variable	STR_COEF	Steer Coefficient *10	No	No
Variable	BATV	Calibrated Battery Voltage *100	No	No
Variable	TEMP	Calibrated Temperature °C *10	No	No
🧰 Troubleshooting:
Make sure the baud rate is 115200
Make sure you are using the sensor cable(left=DEBUG_SERIAL_USART2 or right=DEBUG_SERIAL_USART3) selected in config.h
TX can be defective on your mainboard, you can switch to other sensor cable if not used already in config.h
On some boards the wire colors might differ, try switching the green wire
Make sure the RX on your FTDI is working. You can connect the RX and TX on the FTDI and check if you received the commands you send in the Web Tool tool or any serial monitor
 Add a custom footer
Pages 28
Buying a used hoverboard
Firmware compatibility
Hoverboard Wheels
How to Unlock MCU Flash
Variants
ADC
USART
NUNCHUK
PPM
PWM
IBUS
HOVERCAR
TRANSPOTTER
SKATEBOARD
HOVERBOARD
BBCAR
Dual Inputs
Compiling and flashing the firmware
Input Calibration
Parameters
Setup current and speed limits
Battery
Diagnostics
Troubleshooting
Debug Serial
Sideboards
Clone this wiki locally
*/