
#ifndef DAVID_H
#define DAVID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifndef EINVAL
#    define EINVAL 22
#endif

/* Frame ids. */
#define DAVID_E_STOP_FRAME_ID (0x00u)
#define DAVID_PITCH_CTRL_FRAME_ID (0x100u)
#define DAVID_PITCH_POSITION_TELEM_FRAME_ID (0x110u)
#define DAVID_PITCH_DRIVER_TELEM_FRAME_ID (0x111u)
#define DAVID_LOCO_CTRL_FRAME_ID (0x200u)
#define DAVID_EXCAV_CTRL_FRAME_ID (0x201u)
#define DAVID_STEPPER_CTRL_FRAME_ID (0x300u)
#define DAVID_STEPPER_TELEM_FRAME_ID (0x311u)

/* Frame lengths in bytes. */
#define DAVID_E_STOP_LENGTH (1u)
#define DAVID_PITCH_CTRL_LENGTH (4u)
#define DAVID_PITCH_POSITION_TELEM_LENGTH (3u)
#define DAVID_PITCH_DRIVER_TELEM_LENGTH (4u)
#define DAVID_LOCO_CTRL_LENGTH (4u)
#define DAVID_EXCAV_CTRL_LENGTH (3u)
#define DAVID_STEPPER_CTRL_LENGTH (2u)
#define DAVID_STEPPER_TELEM_LENGTH (3u)

/* Extended or standard frame types. */
#define DAVID_E_STOP_IS_EXTENDED (0)
#define DAVID_PITCH_CTRL_IS_EXTENDED (0)
#define DAVID_PITCH_POSITION_TELEM_IS_EXTENDED (0)
#define DAVID_PITCH_DRIVER_TELEM_IS_EXTENDED (0)
#define DAVID_LOCO_CTRL_IS_EXTENDED (0)
#define DAVID_EXCAV_CTRL_IS_EXTENDED (0)
#define DAVID_STEPPER_CTRL_IS_EXTENDED (0)
#define DAVID_STEPPER_TELEM_IS_EXTENDED (0)

/* Frame cycle times in milliseconds. */


/* Signal choices. */
#define DAVID_STEPPER_CTRL_HOME_FALSE_CHOICE (0u)
#define DAVID_STEPPER_CTRL_HOME_TRUE_CHOICE (1u)

#define DAVID_STEPPER_TELEM_AT_MIN_STOP_FALSE_CHOICE (0u)
#define DAVID_STEPPER_TELEM_AT_MIN_STOP_TRUE_CHOICE (1u)

#define DAVID_STEPPER_TELEM_AT_MAX_STOP_FALSE_CHOICE (0u)
#define DAVID_STEPPER_TELEM_AT_MAX_STOP_TRUE_CHOICE (1u)

/**
 * Signals in message EStop.
 *
 * All signal values are as on the CAN bus.
 */
struct david_e_stop_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t stop;
};

/**
 * Signals in message PitchCtrl.
 *
 * All signal values are as on the CAN bus.
 */
struct david_pitch_ctrl_t {
    /**
     * Range: 0..515.3684210526315789473684211 (0.0..153 mm)
     * Scale: 0.296875
     * Offset: 0.0
     */
    uint16_t set_point;

    /**
     * Range: 0..515.3684210526315789473684211 (-76.5..76.5 mm)
     * Scale: 0.296875
     * Offset: -76.5
     */
    uint16_t left_offset;

    /**
     * Range: 0..515.3684210526315789473684211 (-76.5..76.5 mm)
     * Scale: 0.296875
     * Offset: -76.5
     */
    uint16_t right_offset;
};

/**
 * Signals in message PitchPositionTelem.
 *
 * All signal values are as on the CAN bus.
 */
struct david_pitch_position_telem_t {
    /**
     * Range: 0..515.3684210526315789473684211 (0.0..153 mm)
     * Scale: 0.296875
     * Offset: 0.0
     */
    uint16_t left_position;

    /**
     * Range: 0..515.3684210526315789473684211 (0.0..153 mm)
     * Scale: 0.296875
     * Offset: 0.0
     */
    uint16_t right_position;
};

/**
 * Signals in message PitchDriverTelem.
 *
 * All signal values are as on the CAN bus.
 */
struct david_pitch_driver_telem_t {
    /**
     * Range: 0..186.0119047619047619047619048 (0.0..20.0 Amp)
     * Scale: 0.10752
     * Offset: 0.0
     */
    uint8_t left_current;

    /**
     * Range: 0..186.0119047619047619047619048 (0.0..20.0 Amp)
     * Scale: 0.10752
     * Offset: 0.0
     */
    uint8_t right_current;

    /**
     * Range: 0..256 (-40..125 Cel)
     * Scale: 0.64453125
     * Offset: -40
     */
    uint8_t left_temperature;

    /**
     * Range: 0..256 (-40..125 Cel)
     * Scale: 0.64453125
     * Offset: -40
     */
    uint8_t right_temperature;
};

/**
 * Signals in message LocoCtrl.
 *
 * All signal values are as on the CAN bus.
 */
struct david_loco_ctrl_t {
    /**
     * Range: 0..65536 (-100..100 -)
     * Scale: 0.0030517578125
     * Offset: -100
     */
    uint16_t left_vel;

    /**
     * Range: 0..65536 (-100..100 -)
     * Scale: 0.0030517578125
     * Offset: -100
     */
    uint16_t right_vel;
};

/**
 * Signals in message ExcavCtrl.
 *
 * 
                Negative power is reverse.
                -100 is 100% power in reverse
                100 is 100% power in reverse
            
 *
 * All signal values are as on the CAN bus.
 */
struct david_excav_ctrl_t {
    /**
     * Range: 0..65536 (-100..100 -)
     * Scale: 0.0030517578125
     * Offset: -100
     */
    uint16_t vel;
};

/**
 * Signals in message StepperCtrl.
 *
 * 
                Home may not actually be something we want at this level.
                Steppers should home/reset some internal measure of position
                whenever they hit a limit switch.
                We should home on startup
            
 *
 * All signal values are as on the CAN bus.
 */
struct david_stepper_ctrl_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t home;

    /**
     * Range: 0..515.3684210526315789473684211 (0.0..153 mm)
     * Scale: 0.296875
     * Offset: 0.0
     */
    uint16_t set_point;
};

/**
 * Signals in message StepperTelem.
 *
 * 
                Min and Max stop refer to if we are currently at a limit switch
                TODO: Get correct slope for steppers
            
 *
 * All signal values are as on the CAN bus.
 */
struct david_stepper_telem_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t at_min_stop;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t at_max_stop;

    /**
     * Range: 0..515.3684210526315789473684211 (0.0..153 mm)
     * Scale: 0.296875
     * Offset: 0.0
     */
    uint16_t left_position;

    /**
     * Range: 0..515.3684210526315789473684211 (0.0..153 mm)
     * Scale: 0.296875
     * Offset: 0.0
     */
    uint16_t right_position;
};

/**
 * Pack message EStop.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_e_stop_pack(
    uint8_t *dst_p,
    const struct david_e_stop_t *src_p,
    size_t size);

/**
 * Unpack message EStop.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_e_stop_unpack(
    struct david_e_stop_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_e_stop_stop_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_e_stop_stop_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_e_stop_stop_is_in_range(uint8_t value);

/**
 * Pack message PitchCtrl.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_pitch_ctrl_pack(
    uint8_t *dst_p,
    const struct david_pitch_ctrl_t *src_p,
    size_t size);

/**
 * Unpack message PitchCtrl.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_pitch_ctrl_unpack(
    struct david_pitch_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_pitch_ctrl_set_point_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_ctrl_set_point_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_ctrl_set_point_is_in_range(uint16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_pitch_ctrl_left_offset_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_ctrl_left_offset_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_ctrl_left_offset_is_in_range(uint16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_pitch_ctrl_right_offset_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_ctrl_right_offset_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_ctrl_right_offset_is_in_range(uint16_t value);

/**
 * Pack message PitchPositionTelem.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_pitch_position_telem_pack(
    uint8_t *dst_p,
    const struct david_pitch_position_telem_t *src_p,
    size_t size);

/**
 * Unpack message PitchPositionTelem.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_pitch_position_telem_unpack(
    struct david_pitch_position_telem_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_pitch_position_telem_left_position_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_position_telem_left_position_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_position_telem_left_position_is_in_range(uint16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_pitch_position_telem_right_position_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_position_telem_right_position_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_position_telem_right_position_is_in_range(uint16_t value);

/**
 * Pack message PitchDriverTelem.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_pitch_driver_telem_pack(
    uint8_t *dst_p,
    const struct david_pitch_driver_telem_t *src_p,
    size_t size);

/**
 * Unpack message PitchDriverTelem.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_pitch_driver_telem_unpack(
    struct david_pitch_driver_telem_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_pitch_driver_telem_left_current_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_driver_telem_left_current_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_driver_telem_left_current_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_pitch_driver_telem_right_current_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_driver_telem_right_current_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_driver_telem_right_current_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_pitch_driver_telem_left_temperature_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_driver_telem_left_temperature_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_driver_telem_left_temperature_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_pitch_driver_telem_right_temperature_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_driver_telem_right_temperature_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_driver_telem_right_temperature_is_in_range(uint8_t value);

/**
 * Pack message LocoCtrl.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_loco_ctrl_pack(
    uint8_t *dst_p,
    const struct david_loco_ctrl_t *src_p,
    size_t size);

/**
 * Unpack message LocoCtrl.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_loco_ctrl_unpack(
    struct david_loco_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_loco_ctrl_left_vel_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_loco_ctrl_left_vel_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_loco_ctrl_left_vel_is_in_range(uint16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_loco_ctrl_right_vel_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_loco_ctrl_right_vel_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_loco_ctrl_right_vel_is_in_range(uint16_t value);

/**
 * Pack message ExcavCtrl.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_excav_ctrl_pack(
    uint8_t *dst_p,
    const struct david_excav_ctrl_t *src_p,
    size_t size);

/**
 * Unpack message ExcavCtrl.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_excav_ctrl_unpack(
    struct david_excav_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_excav_ctrl_vel_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_excav_ctrl_vel_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_excav_ctrl_vel_is_in_range(uint16_t value);

/**
 * Pack message StepperCtrl.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_stepper_ctrl_pack(
    uint8_t *dst_p,
    const struct david_stepper_ctrl_t *src_p,
    size_t size);

/**
 * Unpack message StepperCtrl.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_stepper_ctrl_unpack(
    struct david_stepper_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_stepper_ctrl_home_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_ctrl_home_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_ctrl_home_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_stepper_ctrl_set_point_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_ctrl_set_point_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_ctrl_set_point_is_in_range(uint16_t value);

/**
 * Pack message StepperTelem.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_stepper_telem_pack(
    uint8_t *dst_p,
    const struct david_stepper_telem_t *src_p,
    size_t size);

/**
 * Unpack message StepperTelem.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_stepper_telem_unpack(
    struct david_stepper_telem_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_stepper_telem_at_min_stop_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_telem_at_min_stop_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_telem_at_min_stop_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_stepper_telem_at_max_stop_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_telem_at_max_stop_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_telem_at_max_stop_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_stepper_telem_left_position_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_telem_left_position_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_telem_left_position_is_in_range(uint16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_stepper_telem_right_position_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_telem_right_position_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_telem_right_position_is_in_range(uint16_t value);


#ifdef __cplusplus
}
#endif

#endif
