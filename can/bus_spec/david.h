yes
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
#define DAVID_E_START_FRAME_ID (0x01u)
#define DAVID_PITCH_CTRL_HOME_FRAME_ID (0x10u)
#define DAVID_SET_TRIG_DELAY_FRAME_ID (0x20u)
#define DAVID_PITCH_CTRL_LEFT_FRAME_ID (0x101u)
#define DAVID_PITCH_CTRL_RIGHT_FRAME_ID (0x110u)
#define DAVID_PITCH_CTRL_BOTH_FRAME_ID (0x111u)
#define DAVID_PITCH_TELEM_LEFT_FRAME_ID (0x200u)
#define DAVID_PITCH_TELEM_RIGHT_FRAME_ID (0x201u)
#define DAVID_LOCO_CTRL_LEFT_FRAME_ID (0x300u)
#define DAVID_LOCO_CTRL_RIGHT_FRAME_ID (0x301u)
#define DAVID_LOCO_CTRL_BOTH_FRAME_ID (0x311u)
#define DAVID_STEPPER_CTRL_LEFT_FRAME_ID (0x400u)
#define DAVID_STEPPER_CTRL_RIGHT_FRAME_ID (0x401u)
#define DAVID_STEPPER_CTRL_BOTH_FRAME_ID (0x411u)
#define DAVID_EXCAV_CTRL_FRAME_ID (0x500u)

/* Frame lengths in bytes. */
#define DAVID_E_STOP_LENGTH (0u)
#define DAVID_E_START_LENGTH (0u)
#define DAVID_PITCH_CTRL_HOME_LENGTH (0u)
#define DAVID_SET_TRIG_DELAY_LENGTH (8u)
#define DAVID_PITCH_CTRL_LEFT_LENGTH (1u)
#define DAVID_PITCH_CTRL_RIGHT_LENGTH (1u)
#define DAVID_PITCH_CTRL_BOTH_LENGTH (8u)
#define DAVID_PITCH_TELEM_LEFT_LENGTH (8u)
#define DAVID_PITCH_TELEM_RIGHT_LENGTH (8u)
#define DAVID_LOCO_CTRL_LEFT_LENGTH (4u)
#define DAVID_LOCO_CTRL_RIGHT_LENGTH (4u)
#define DAVID_LOCO_CTRL_BOTH_LENGTH (4u)
#define DAVID_STEPPER_CTRL_LEFT_LENGTH (8u)
#define DAVID_STEPPER_CTRL_RIGHT_LENGTH (8u)
#define DAVID_STEPPER_CTRL_BOTH_LENGTH (8u)
#define DAVID_EXCAV_CTRL_LENGTH (4u)

/* Extended or standard frame types. */
#define DAVID_E_STOP_IS_EXTENDED (0)
#define DAVID_E_START_IS_EXTENDED (0)
#define DAVID_PITCH_CTRL_HOME_IS_EXTENDED (0)
#define DAVID_SET_TRIG_DELAY_IS_EXTENDED (0)
#define DAVID_PITCH_CTRL_LEFT_IS_EXTENDED (0)
#define DAVID_PITCH_CTRL_RIGHT_IS_EXTENDED (0)
#define DAVID_PITCH_CTRL_BOTH_IS_EXTENDED (0)
#define DAVID_PITCH_TELEM_LEFT_IS_EXTENDED (0)
#define DAVID_PITCH_TELEM_RIGHT_IS_EXTENDED (0)
#define DAVID_LOCO_CTRL_LEFT_IS_EXTENDED (0)
#define DAVID_LOCO_CTRL_RIGHT_IS_EXTENDED (0)
#define DAVID_LOCO_CTRL_BOTH_IS_EXTENDED (0)
#define DAVID_STEPPER_CTRL_LEFT_IS_EXTENDED (0)
#define DAVID_STEPPER_CTRL_RIGHT_IS_EXTENDED (0)
#define DAVID_STEPPER_CTRL_BOTH_IS_EXTENDED (0)
#define DAVID_EXCAV_CTRL_IS_EXTENDED (0)

/* Frame cycle times in milliseconds. */


/* Signal choices. */


/**
 * Signals in message EStop.
 *
 * All signal values are as on the CAN bus.
 */
struct david_e_stop_t {
    /**
     * Dummy signal in empty message.
     */
    uint8_t dummy;
};

/**
 * Signals in message EStart.
 *
 * All signal values are as on the CAN bus.
 */
struct david_e_start_t {
    /**
     * Dummy signal in empty message.
     */
    uint8_t dummy;
};

/**
 * Signals in message PitchCtrlHome.
 *
 * All signal values are as on the CAN bus.
 */
struct david_pitch_ctrl_home_t {
    /**
     * Dummy signal in empty message.
     */
    uint8_t dummy;
};

/**
 * Signals in message SetTrigDelay.
 *
 * All signal values are as on the CAN bus.
 */
struct david_set_trig_delay_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint64_t microseconds;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t max_count;
};

/**
 * Signals in message PitchCtrlLeft.
 *
 * All signal values are as on the CAN bus.
 */
struct david_pitch_ctrl_left_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t command;
};

/**
 * Signals in message PitchCtrlRight.
 *
 * All signal values are as on the CAN bus.
 */
struct david_pitch_ctrl_right_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t command;
};

/**
 * Signals in message PitchCtrlBoth.
 *
 * All signal values are as on the CAN bus.
 */
struct david_pitch_ctrl_both_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint64_t count;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t tolerance;
};

/**
 * Signals in message PitchTelemLeft.
 *
 * All signal values are as on the CAN bus.
 */
struct david_pitch_telem_left_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint64_t count;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t direction;
};

/**
 * Signals in message PitchTelemRight.
 *
 * All signal values are as on the CAN bus.
 */
struct david_pitch_telem_right_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint64_t count;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t direction;
};

/**
 * Signals in message LocoCtrlLeft.
 *
 * All signal values are as on the CAN bus.
 */
struct david_loco_ctrl_left_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t velocity;
};

/**
 * Signals in message LocoCtrlRight.
 *
 * All signal values are as on the CAN bus.
 */
struct david_loco_ctrl_right_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t velocity;
};

/**
 * Signals in message LocoCtrlBoth.
 *
 * All signal values are as on the CAN bus.
 */
struct david_loco_ctrl_both_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t velocity;
};

/**
 * Signals in message StepperCtrlLeft.
 *
 * All signal values are as on the CAN bus.
 */
struct david_stepper_ctrl_left_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t rpm;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t direction;
};

/**
 * Signals in message StepperCtrlRight.
 *
 * All signal values are as on the CAN bus.
 */
struct david_stepper_ctrl_right_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t rpm;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t direction;
};

/**
 * Signals in message StepperCtrlBoth.
 *
 * All signal values are as on the CAN bus.
 */
struct david_stepper_ctrl_both_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t rpm;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t direction;
};

/**
 * Signals in message ExcavCtrl.
 *
 * All signal values are as on the CAN bus.
 */
struct david_excav_ctrl_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t rpm;
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
 * Pack message EStart.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_e_start_pack(
    uint8_t *dst_p,
    const struct david_e_start_t *src_p,
    size_t size);

/**
 * Unpack message EStart.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_e_start_unpack(
    struct david_e_start_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Pack message PitchCtrlHome.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_pitch_ctrl_home_pack(
    uint8_t *dst_p,
    const struct david_pitch_ctrl_home_t *src_p,
    size_t size);

/**
 * Unpack message PitchCtrlHome.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_pitch_ctrl_home_unpack(
    struct david_pitch_ctrl_home_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Pack message SetTrigDelay.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_set_trig_delay_pack(
    uint8_t *dst_p,
    const struct david_set_trig_delay_t *src_p,
    size_t size);

/**
 * Unpack message SetTrigDelay.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_set_trig_delay_unpack(
    struct david_set_trig_delay_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint64_t david_set_trig_delay_microseconds_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_set_trig_delay_microseconds_decode(uint64_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_set_trig_delay_microseconds_is_in_range(uint64_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_set_trig_delay_max_count_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_set_trig_delay_max_count_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_set_trig_delay_max_count_is_in_range(uint16_t value);

/**
 * Pack message PitchCtrlLeft.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_pitch_ctrl_left_pack(
    uint8_t *dst_p,
    const struct david_pitch_ctrl_left_t *src_p,
    size_t size);

/**
 * Unpack message PitchCtrlLeft.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_pitch_ctrl_left_unpack(
    struct david_pitch_ctrl_left_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_pitch_ctrl_left_command_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_ctrl_left_command_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_ctrl_left_command_is_in_range(uint8_t value);

/**
 * Pack message PitchCtrlRight.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_pitch_ctrl_right_pack(
    uint8_t *dst_p,
    const struct david_pitch_ctrl_right_t *src_p,
    size_t size);

/**
 * Unpack message PitchCtrlRight.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_pitch_ctrl_right_unpack(
    struct david_pitch_ctrl_right_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_pitch_ctrl_right_command_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_ctrl_right_command_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_ctrl_right_command_is_in_range(uint8_t value);

/**
 * Pack message PitchCtrlBoth.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_pitch_ctrl_both_pack(
    uint8_t *dst_p,
    const struct david_pitch_ctrl_both_t *src_p,
    size_t size);

/**
 * Unpack message PitchCtrlBoth.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_pitch_ctrl_both_unpack(
    struct david_pitch_ctrl_both_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint64_t david_pitch_ctrl_both_count_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_ctrl_both_count_decode(uint64_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_ctrl_both_count_is_in_range(uint64_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_pitch_ctrl_both_tolerance_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_ctrl_both_tolerance_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_ctrl_both_tolerance_is_in_range(uint16_t value);

/**
 * Pack message PitchTelemLeft.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_pitch_telem_left_pack(
    uint8_t *dst_p,
    const struct david_pitch_telem_left_t *src_p,
    size_t size);

/**
 * Unpack message PitchTelemLeft.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_pitch_telem_left_unpack(
    struct david_pitch_telem_left_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint64_t david_pitch_telem_left_count_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_telem_left_count_decode(uint64_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_telem_left_count_is_in_range(uint64_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_pitch_telem_left_direction_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_telem_left_direction_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_telem_left_direction_is_in_range(uint16_t value);

/**
 * Pack message PitchTelemRight.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_pitch_telem_right_pack(
    uint8_t *dst_p,
    const struct david_pitch_telem_right_t *src_p,
    size_t size);

/**
 * Unpack message PitchTelemRight.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_pitch_telem_right_unpack(
    struct david_pitch_telem_right_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint64_t david_pitch_telem_right_count_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_telem_right_count_decode(uint64_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_telem_right_count_is_in_range(uint64_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_pitch_telem_right_direction_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_telem_right_direction_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_telem_right_direction_is_in_range(uint16_t value);

/**
 * Pack message LocoCtrlLeft.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_loco_ctrl_left_pack(
    uint8_t *dst_p,
    const struct david_loco_ctrl_left_t *src_p,
    size_t size);

/**
 * Unpack message LocoCtrlLeft.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_loco_ctrl_left_unpack(
    struct david_loco_ctrl_left_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint32_t david_loco_ctrl_left_velocity_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_loco_ctrl_left_velocity_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_loco_ctrl_left_velocity_is_in_range(uint32_t value);

/**
 * Pack message LocoCtrlRight.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_loco_ctrl_right_pack(
    uint8_t *dst_p,
    const struct david_loco_ctrl_right_t *src_p,
    size_t size);

/**
 * Unpack message LocoCtrlRight.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_loco_ctrl_right_unpack(
    struct david_loco_ctrl_right_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint32_t david_loco_ctrl_right_velocity_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_loco_ctrl_right_velocity_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_loco_ctrl_right_velocity_is_in_range(uint32_t value);

/**
 * Pack message LocoCtrlBoth.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_loco_ctrl_both_pack(
    uint8_t *dst_p,
    const struct david_loco_ctrl_both_t *src_p,
    size_t size);

/**
 * Unpack message LocoCtrlBoth.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_loco_ctrl_both_unpack(
    struct david_loco_ctrl_both_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint32_t david_loco_ctrl_both_velocity_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_loco_ctrl_both_velocity_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_loco_ctrl_both_velocity_is_in_range(uint32_t value);

/**
 * Pack message StepperCtrlLeft.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_stepper_ctrl_left_pack(
    uint8_t *dst_p,
    const struct david_stepper_ctrl_left_t *src_p,
    size_t size);

/**
 * Unpack message StepperCtrlLeft.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_stepper_ctrl_left_unpack(
    struct david_stepper_ctrl_left_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint32_t david_stepper_ctrl_left_rpm_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_ctrl_left_rpm_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_ctrl_left_rpm_is_in_range(uint32_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint32_t david_stepper_ctrl_left_direction_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_ctrl_left_direction_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_ctrl_left_direction_is_in_range(uint32_t value);

/**
 * Pack message StepperCtrlRight.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_stepper_ctrl_right_pack(
    uint8_t *dst_p,
    const struct david_stepper_ctrl_right_t *src_p,
    size_t size);

/**
 * Unpack message StepperCtrlRight.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_stepper_ctrl_right_unpack(
    struct david_stepper_ctrl_right_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint32_t david_stepper_ctrl_right_rpm_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_ctrl_right_rpm_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_ctrl_right_rpm_is_in_range(uint32_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint32_t david_stepper_ctrl_right_direction_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_ctrl_right_direction_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_ctrl_right_direction_is_in_range(uint32_t value);

/**
 * Pack message StepperCtrlBoth.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_stepper_ctrl_both_pack(
    uint8_t *dst_p,
    const struct david_stepper_ctrl_both_t *src_p,
    size_t size);

/**
 * Unpack message StepperCtrlBoth.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_stepper_ctrl_both_unpack(
    struct david_stepper_ctrl_both_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint32_t david_stepper_ctrl_both_rpm_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_ctrl_both_rpm_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_ctrl_both_rpm_is_in_range(uint32_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint32_t david_stepper_ctrl_both_direction_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_ctrl_both_direction_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_ctrl_both_direction_is_in_range(uint32_t value);

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
uint32_t david_excav_ctrl_rpm_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_excav_ctrl_rpm_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_excav_ctrl_rpm_is_in_range(uint32_t value);


#ifdef __cplusplus
}
#endif

#endif
