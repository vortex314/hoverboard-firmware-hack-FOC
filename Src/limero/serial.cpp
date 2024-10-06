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

void encode_vars(FrameEncoder frame_encoder) {
    frame_encoder.begin_map();
    frame_encoder.add_map(HbVar::CTRL_MOD, ctrlModReqRaw);
    frame_encoder.add_map(HbVar::CTRL_TYP, rtP_Left.z_ctrlTypSel);
    frame_encoder.add_map(HbVar::CUR_MOT_MAX, rtP_Left.i_max);
    frame_encoder.add_map(HbVar::RPM_MOT_MAX, rtP_Left.n_max);
    frame_encoder.add_map(HbVar::FI_WEAK_ENA, rtP_Left.b_fieldWeakEna);
    frame_encoder.add_map(HbVar::FI_WEAK_HI, rtP_Left.r_fieldWeakHi);
    frame_encoder.add_map(HbVar::FI_WEAK_LO, rtP_Left.r_fieldWeakLo);
    frame_encoder.add_map(HbVar::FI_WEAK_MAX, rtP_Left.id_fieldWeakMax);
    frame_encoder.add_map(HbVar::PHASE_ADV_MAX_DEG, rtP_Left.a_phaAdvMax);
    frame_encoder.add_map(HbVar::IN1_RAW, input1[0].raw);
    frame_encoder.add_map(HbVar::IN1_TYP, input1[0].typ);
    frame_encoder.add_map(HbVar::IN1_MIN, input1[0].min);
    frame_encoder.add_map(HbVar::IN1_MID, input1[0].mid);
    frame_encoder.add_map(HbVar::IN1_MAX, input1[0].max);
    frame_encoder.add_map(HbVar::IN1_CMD, input1[0].cmd);
    frame_encoder.add_map(HbVar::IN2_RAW, input2[0].raw);
    frame_encoder.add_map(HbVar::IN2_TYP, input2[0].typ);
    frame_encoder.add_map(HbVar::IN2_MIN, input2[0].min);
    frame_encoder.add_map(HbVar::IN2_MID, input2[0].mid);
    frame_encoder.add_map(HbVar::IN2_MAX, input2[0].max);
    frame_encoder.add_map(HbVar::IN2_CMD, input2[0].cmd);
    frame_encoder.add_map(HbVar::AUX_IN1_RAW, input1[1].raw);
    frame_encoder.add_map(HbVar::AUX_IN1_TYP, input1[1].typ);
    frame_encoder.add_map(HbVar::AUX_IN1_MIN, input1[1].min);
    frame_encoder.add_map(HbVar::AUX_IN1_MID, input1[1].mid);
    frame_encoder.add_map(HbVar::AUX_IN1_MAX, input1[1].max);
    frame_encoder.add_map(HbVar::AUX_IN1_CMD, input1[1].cmd);
    frame_encoder.add_map(HbVar::AUX_IN2_RAW, input2[1].raw);
    frame_encoder.add_map(HbVar::AUX_IN2_TYP, input2[1].typ);
    frame_encoder.add_map(HbVar::AUX_IN2_MIN, input2[1].min);
    frame_encoder.add_map(HbVar::AUX_IN2_MID, input2[1].mid);
    frame_encoder.add_map(HbVar::AUX_IN2_MAX, input2[1].max);
    frame_encoder.add_map(HbVar::AUX_IN2_CMD, input2[1].cmd);
    frame_encoder.add_map(HbVar::DC_CURR, dc_curr);
    frame_encoder.add_map(HbVar::RDC_CURR, right_dc_curr);
    frame_encoder.add_map(HbVar::LDC_CURR, left_dc_curr);
    frame_encoder.add_map(HbVar::CMDL, cmdL);
    frame_encoder.add_map(HbVar::CMDR, cmdR);
    frame_encoder.add_map(HbVar::SPD_AVG, speedAvg);
    frame_encoder.add_map(HbVar::SPDL, rtY_Left.n_mot);
    frame_encoder.add_map(HbVar::SPDR, rtY_Right.n_mot);
    frame_encoder.add_map(HbVar::FILTER_RATE, 0);
    frame_encoder.add_map(HbVar::SPD_COEF, SPEED_COEFFICIENT);
    frame_encoder.add_map(HbVar::STR_COEF, STEER_COEFFICIENT);
    frame_encoder.add_map(HbVar::BATV, batVoltageCalib);
    frame_encoder.add_map(HbVar::TEMP, board_temp_deg_c);
    frame_encoder.end_map();
}


FrameEncoder frame_encoder(256);
const uint32_t FNV_ID = FNV("hb");


extern "C" uint32_t get_txd(uint8_t** buffer) {
    frame_encoder.clear();
    frame_encoder.begin_array();
    frame_encoder.encode_null();
    frame_encoder.encode_uint32(FNV_ID);
    frame_encoder.encode_uint32(MsgType::Pub);
    encode_vars(frame_encoder);
    frame_encoder.add_crc();
    frame_encoder.add_cobs();
    frame_encoder.end_array();
    uint8_t* data = frame_encoder.buffer();
    *buffer = data;
    return frame_encoder.length();
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