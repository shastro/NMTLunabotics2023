
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
#define DAVID_PITCH_ADJUST_FRAME_ID (0x101u)
#define DAVID_PITCH_TELEM_FRAME_ID (0x111u)
#define DAVID_LOCO_CTRL_FRAME_ID (0x200u)
#define DAVID_EXCAV_CTRL_FRAME_ID (0x201u)
#define DAVID_STEPPER_CTRL_FRAME_ID (0x300u)
#define DAVID_STEPPER_CTRL_ADJUST_FRAME_ID (0x301u)

/* Frame lengths in bytes. */
#define DAVID_E_STOP_LENGTH (1u)
#define DAVID_PITCH_CTRL_LENGTH (2u)
#define DAVID_PITCH_ADJUST_LENGTH (6u)
#define DAVID_PITCH_TELEM_LENGTH (8u)
#define DAVID_LOCO_CTRL_LENGTH (8u)
#define DAVID_EXCAV_CTRL_LENGTH (1u)
#define DAVID_STEPPER_CTRL_LENGTH (8u)
#define DAVID_STEPPER_CTRL_ADJUST_LENGTH (6u)

/* Extended or standard frame types. */
#define DAVID_E_STOP_IS_EXTENDED (0)
#define DAVID_PITCH_CTRL_IS_EXTENDED (0)
#define DAVID_PITCH_ADJUST_IS_EXTENDED (0)
#define DAVID_PITCH_TELEM_IS_EXTENDED (0)
#define DAVID_LOCO_CTRL_IS_EXTENDED (0)
#define DAVID_EXCAV_CTRL_IS_EXTENDED (0)
#define DAVID_STEPPER_CTRL_IS_EXTENDED (0)
#define DAVID_STEPPER_CTRL_ADJUST_IS_EXTENDED (0)

/* Frame cycle times in milliseconds. */


/* Signal choices. */


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
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t home;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t direction;
};

/**
 * Signals in message PitchAdjust.
 *
 * All signal values are as on the CAN bus.
 */
struct david_pitch_adjust_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t left_dir;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t left_amount;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t right_dir;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t right_amount;
};

/**
 * Signals in message PitchTelem.
 *
 * All signal values are as on the CAN bus.
 */
struct david_pitch_telem_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t left_count;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t left_direction;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t right_count;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t right_direction;
};

/**
 * Signals in message LocoCtrl.
 *
 * All signal values are as on the CAN bus.
 */
struct david_loco_ctrl_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t left_vel;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint32_t right_vel;
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
    uint8_t dir;
};

/**
 * Signals in message StepperCtrl.
 *
 * All signal values are as on the CAN bus.
 */
struct david_stepper_ctrl_t {
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
 * Signals in message StepperCtrlAdjust.
 *
 * All signal values are as on the CAN bus.
 */
struct david_stepper_ctrl_adjust_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t left_dir;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t left_amount;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t right_dir;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t right_amount;
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
uint8_t david_pitch_ctrl_home_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_ctrl_home_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_ctrl_home_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_pitch_ctrl_direction_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_ctrl_direction_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_ctrl_direction_is_in_range(uint8_t value);

/**
 * Pack message PitchAdjust.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_pitch_adjust_pack(
    uint8_t *dst_p,
    const struct david_pitch_adjust_t *src_p,
    size_t size);

/**
 * Unpack message PitchAdjust.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_pitch_adjust_unpack(
    struct david_pitch_adjust_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_pitch_adjust_left_dir_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_adjust_left_dir_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_adjust_left_dir_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_pitch_adjust_left_amount_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_adjust_left_amount_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_adjust_left_amount_is_in_range(uint16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_pitch_adjust_right_dir_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_adjust_right_dir_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_adjust_right_dir_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_pitch_adjust_right_amount_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_adjust_right_amount_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_adjust_right_amount_is_in_range(uint16_t value);

/**
 * Pack message PitchTelem.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_pitch_telem_pack(
    uint8_t *dst_p,
    const struct david_pitch_telem_t *src_p,
    size_t size);

/**
 * Unpack message PitchTelem.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_pitch_telem_unpack(
    struct david_pitch_telem_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_pitch_telem_left_count_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_telem_left_count_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_telem_left_count_is_in_range(uint16_t value);

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
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_pitch_telem_right_count_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_pitch_telem_right_count_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_pitch_telem_right_count_is_in_range(uint16_t value);

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
uint32_t david_loco_ctrl_left_vel_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_loco_ctrl_left_vel_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_loco_ctrl_left_vel_is_in_range(uint32_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint32_t david_loco_ctrl_right_vel_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_loco_ctrl_right_vel_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_loco_ctrl_right_vel_is_in_range(uint32_t value);

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
uint8_t david_excav_ctrl_dir_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_excav_ctrl_dir_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_excav_ctrl_dir_is_in_range(uint8_t value);

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
uint32_t david_stepper_ctrl_rpm_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_ctrl_rpm_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_ctrl_rpm_is_in_range(uint32_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint32_t david_stepper_ctrl_direction_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_ctrl_direction_decode(uint32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_ctrl_direction_is_in_range(uint32_t value);

/**
 * Pack message StepperCtrlAdjust.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int david_stepper_ctrl_adjust_pack(
    uint8_t *dst_p,
    const struct david_stepper_ctrl_adjust_t *src_p,
    size_t size);

/**
 * Unpack message StepperCtrlAdjust.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int david_stepper_ctrl_adjust_unpack(
    struct david_stepper_ctrl_adjust_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_stepper_ctrl_adjust_left_dir_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_ctrl_adjust_left_dir_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_ctrl_adjust_left_dir_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_stepper_ctrl_adjust_left_amount_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_ctrl_adjust_left_amount_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_ctrl_adjust_left_amount_is_in_range(uint16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t david_stepper_ctrl_adjust_right_dir_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_ctrl_adjust_right_dir_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_ctrl_adjust_right_dir_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t david_stepper_ctrl_adjust_right_amount_encode(double value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
double david_stepper_ctrl_adjust_right_amount_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool david_stepper_ctrl_adjust_right_amount_is_in_range(uint16_t value);


#ifdef __cplusplus
}
#endif

#endif
