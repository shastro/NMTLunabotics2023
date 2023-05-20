
#include <string.h>

#include "david.h"

static inline uint8_t pack_left_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_right_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint16_t unpack_left_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) << shift);
}

static inline uint8_t unpack_right_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) >> shift);
}

static inline uint16_t unpack_right_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) >> shift);
}

int david_e_stop_pack(
    uint8_t *dst_p,
    const struct david_e_stop_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 1);

    dst_p[0] |= pack_left_shift_u8(src_p->stop, 0u, 0xffu);

    return (1);
}

int david_e_stop_unpack(
    struct david_e_stop_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    dst_p->stop = unpack_right_shift_u8(src_p[0], 0u, 0xffu);

    return (0);
}

uint8_t david_e_stop_stop_encode(double value)
{
    return (uint8_t)(value);
}

double david_e_stop_stop_decode(uint8_t value)
{
    return ((double)value);
}

bool david_e_stop_stop_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int david_pitch_ctrl_pack(
    uint8_t *dst_p,
    const struct david_pitch_ctrl_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 1);

    dst_p[0] |= pack_left_shift_u8(src_p->left, 0u, 0x03u);
    dst_p[0] |= pack_left_shift_u8(src_p->right, 2u, 0x0cu);

    return (1);
}

int david_pitch_ctrl_unpack(
    struct david_pitch_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    dst_p->left = unpack_right_shift_u8(src_p[0], 0u, 0x03u);
    dst_p->right = unpack_right_shift_u8(src_p[0], 2u, 0x0cu);

    return (0);
}

uint8_t david_pitch_ctrl_left_encode(double value)
{
    return (uint8_t)(value);
}

double david_pitch_ctrl_left_decode(uint8_t value)
{
    return ((double)value);
}

bool david_pitch_ctrl_left_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t david_pitch_ctrl_right_encode(double value)
{
    return (uint8_t)(value);
}

double david_pitch_ctrl_right_decode(uint8_t value)
{
    return ((double)value);
}

bool david_pitch_ctrl_right_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

int david_pitch_position_telem_pack(
    uint8_t *dst_p,
    const struct david_pitch_position_telem_t *src_p,
    size_t size)
{
    if (size < 3u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 3);

    dst_p[0] |= pack_left_shift_u16(src_p->left_position, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(src_p->left_position, 8u, 0x01u);
    dst_p[1] |= pack_left_shift_u16(src_p->right_position, 1u, 0xfeu);
    dst_p[2] |= pack_right_shift_u16(src_p->right_position, 7u, 0x03u);
    dst_p[2] |= pack_left_shift_u8(src_p->left_direction, 3u, 0x18u);
    dst_p[2] |= pack_left_shift_u8(src_p->right_direction, 5u, 0x60u);

    return (3);
}

int david_pitch_position_telem_unpack(
    struct david_pitch_position_telem_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 3u) {
        return (-EINVAL);
    }

    dst_p->left_position = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    dst_p->left_position |= unpack_left_shift_u16(src_p[1], 8u, 0x01u);
    dst_p->right_position = unpack_right_shift_u16(src_p[1], 1u, 0xfeu);
    dst_p->right_position |= unpack_left_shift_u16(src_p[2], 7u, 0x03u);
    dst_p->left_direction = unpack_right_shift_u8(src_p[2], 3u, 0x18u);
    dst_p->right_direction = unpack_right_shift_u8(src_p[2], 5u, 0x60u);

    return (0);
}

uint16_t david_pitch_position_telem_left_position_encode(double value)
{
    return (uint16_t)((value - -180.0) / 0.703125);
}

double david_pitch_position_telem_left_position_decode(uint16_t value)
{
    return (((double)value * 0.703125) + -180.0);
}

bool david_pitch_position_telem_left_position_is_in_range(uint16_t value)
{
    return (value <= 512u);
}

uint16_t david_pitch_position_telem_right_position_encode(double value)
{
    return (uint16_t)((value - -180.0) / 0.703125);
}

double david_pitch_position_telem_right_position_decode(uint16_t value)
{
    return (((double)value * 0.703125) + -180.0);
}

bool david_pitch_position_telem_right_position_is_in_range(uint16_t value)
{
    return (value <= 512u);
}

uint8_t david_pitch_position_telem_left_direction_encode(double value)
{
    return (uint8_t)(value);
}

double david_pitch_position_telem_left_direction_decode(uint8_t value)
{
    return ((double)value);
}

bool david_pitch_position_telem_left_direction_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t david_pitch_position_telem_right_direction_encode(double value)
{
    return (uint8_t)(value);
}

double david_pitch_position_telem_right_direction_decode(uint8_t value)
{
    return ((double)value);
}

bool david_pitch_position_telem_right_direction_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

int david_pitch_driver_telem_pack(
    uint8_t *dst_p,
    const struct david_pitch_driver_telem_t *src_p,
    size_t size)
{
    if (size < 5u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 5);

    dst_p[0] |= pack_left_shift_u8(src_p->left_current, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u8(src_p->right_current, 0u, 0xffu);
    dst_p[2] |= pack_left_shift_u8(src_p->left_temperature, 0u, 0xffu);
    dst_p[3] |= pack_left_shift_u8(src_p->right_temperature, 0u, 0xffu);
    dst_p[4] |= pack_left_shift_u8(src_p->left_direction, 0u, 0x03u);
    dst_p[4] |= pack_left_shift_u8(src_p->right_direction, 2u, 0x0cu);

    return (5);
}

int david_pitch_driver_telem_unpack(
    struct david_pitch_driver_telem_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 5u) {
        return (-EINVAL);
    }

    dst_p->left_current = unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->right_current = unpack_right_shift_u8(src_p[1], 0u, 0xffu);
    dst_p->left_temperature = unpack_right_shift_u8(src_p[2], 0u, 0xffu);
    dst_p->right_temperature = unpack_right_shift_u8(src_p[3], 0u, 0xffu);
    dst_p->left_direction = unpack_right_shift_u8(src_p[4], 0u, 0x03u);
    dst_p->right_direction = unpack_right_shift_u8(src_p[4], 2u, 0x0cu);

    return (0);
}

uint8_t david_pitch_driver_telem_left_current_encode(double value)
{
    return (uint8_t)(value / 0.10752);
}

double david_pitch_driver_telem_left_current_decode(uint8_t value)
{
    return ((double)value * 0.10752);
}

bool david_pitch_driver_telem_left_current_is_in_range(uint8_t value)
{
    return (value <= 186u);
}

uint8_t david_pitch_driver_telem_right_current_encode(double value)
{
    return (uint8_t)(value / 0.10752);
}

double david_pitch_driver_telem_right_current_decode(uint8_t value)
{
    return ((double)value * 0.10752);
}

bool david_pitch_driver_telem_right_current_is_in_range(uint8_t value)
{
    return (value <= 186u);
}

uint8_t david_pitch_driver_telem_left_temperature_encode(double value)
{
    return (uint8_t)((value - -40.0) / 0.64453125);
}

double david_pitch_driver_telem_left_temperature_decode(uint8_t value)
{
    return (((double)value * 0.64453125) + -40.0);
}

bool david_pitch_driver_telem_left_temperature_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t david_pitch_driver_telem_right_temperature_encode(double value)
{
    return (uint8_t)((value - -40.0) / 0.64453125);
}

double david_pitch_driver_telem_right_temperature_decode(uint8_t value)
{
    return (((double)value * 0.64453125) + -40.0);
}

bool david_pitch_driver_telem_right_temperature_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t david_pitch_driver_telem_left_direction_encode(double value)
{
    return (uint8_t)(value);
}

double david_pitch_driver_telem_left_direction_decode(uint8_t value)
{
    return ((double)value);
}

bool david_pitch_driver_telem_left_direction_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t david_pitch_driver_telem_right_direction_encode(double value)
{
    return (uint8_t)(value);
}

double david_pitch_driver_telem_right_direction_decode(uint8_t value)
{
    return ((double)value);
}

bool david_pitch_driver_telem_right_direction_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

int david_loco_ctrl_pack(
    uint8_t *dst_p,
    const struct david_loco_ctrl_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 4);

    dst_p[0] |= pack_left_shift_u16(src_p->left_vel, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(src_p->left_vel, 8u, 0xffu);
    dst_p[2] |= pack_left_shift_u16(src_p->right_vel, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(src_p->right_vel, 8u, 0xffu);

    return (4);
}

int david_loco_ctrl_unpack(
    struct david_loco_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    dst_p->left_vel = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    dst_p->left_vel |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->right_vel = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    dst_p->right_vel |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);

    return (0);
}

uint16_t david_loco_ctrl_left_vel_encode(double value)
{
    return (uint16_t)((value - -100.0) / 0.0030517578125);
}

double david_loco_ctrl_left_vel_decode(uint16_t value)
{
    return (((double)value * 0.0030517578125) + -100.0);
}

bool david_loco_ctrl_left_vel_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

uint16_t david_loco_ctrl_right_vel_encode(double value)
{
    return (uint16_t)((value - -100.0) / 0.0030517578125);
}

double david_loco_ctrl_right_vel_decode(uint16_t value)
{
    return (((double)value * 0.0030517578125) + -100.0);
}

bool david_loco_ctrl_right_vel_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

int david_excav_ctrl_pack(
    uint8_t *dst_p,
    const struct david_excav_ctrl_t *src_p,
    size_t size)
{
    if (size < 2u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 2);

    dst_p[0] |= pack_left_shift_u16(src_p->vel, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(src_p->vel, 8u, 0xffu);

    return (2);
}

int david_excav_ctrl_unpack(
    struct david_excav_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 2u) {
        return (-EINVAL);
    }

    dst_p->vel = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    dst_p->vel |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);

    return (0);
}

uint16_t david_excav_ctrl_vel_encode(double value)
{
    return (uint16_t)((value - -100.0) / 0.0030517578125);
}

double david_excav_ctrl_vel_decode(uint16_t value)
{
    return (((double)value * 0.0030517578125) + -100.0);
}

bool david_excav_ctrl_vel_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

int david_stepper_ctrl_pack(
    uint8_t *dst_p,
    const struct david_stepper_ctrl_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 1);

    dst_p[0] |= pack_left_shift_u8(src_p->left_dir, 0u, 0x03u);
    dst_p[0] |= pack_left_shift_u8(src_p->right_dir, 2u, 0x3cu);

    return (1);
}

int david_stepper_ctrl_unpack(
    struct david_stepper_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    dst_p->left_dir = unpack_right_shift_u8(src_p[0], 0u, 0x03u);
    dst_p->right_dir = unpack_right_shift_u8(src_p[0], 2u, 0x3cu);

    return (0);
}

uint8_t david_stepper_ctrl_left_dir_encode(double value)
{
    return (uint8_t)(value);
}

double david_stepper_ctrl_left_dir_decode(uint8_t value)
{
    return ((double)value);
}

bool david_stepper_ctrl_left_dir_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t david_stepper_ctrl_right_dir_encode(double value)
{
    return (uint8_t)(value);
}

double david_stepper_ctrl_right_dir_decode(uint8_t value)
{
    return ((double)value);
}

bool david_stepper_ctrl_right_dir_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

int david_stepper_telem_pack(
    uint8_t *dst_p,
    const struct david_stepper_telem_t *src_p,
    size_t size)
{
    if (size < 3u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 3);

    dst_p[0] |= pack_left_shift_u8(src_p->at_min_stop, 0u, 0x01u);
    dst_p[0] |= pack_left_shift_u16(src_p->left_position, 2u, 0xfcu);
    dst_p[1] |= pack_right_shift_u16(src_p->left_position, 6u, 0x1fu);
    dst_p[1] |= pack_left_shift_u16(src_p->right_position, 5u, 0xe0u);
    dst_p[2] |= pack_right_shift_u16(src_p->right_position, 3u, 0xffu);

    return (3);
}

int david_stepper_telem_unpack(
    struct david_stepper_telem_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 3u) {
        return (-EINVAL);
    }

    dst_p->at_min_stop = unpack_right_shift_u8(src_p[0], 0u, 0x01u);
    dst_p->left_position = unpack_right_shift_u16(src_p[0], 2u, 0xfcu);
    dst_p->left_position |= unpack_left_shift_u16(src_p[1], 6u, 0x1fu);
    dst_p->right_position = unpack_right_shift_u16(src_p[1], 5u, 0xe0u);
    dst_p->right_position |= unpack_left_shift_u16(src_p[2], 3u, 0xffu);

    return (0);
}

uint8_t david_stepper_telem_at_min_stop_encode(double value)
{
    return (uint8_t)(value);
}

double david_stepper_telem_at_min_stop_decode(uint8_t value)
{
    return ((double)value);
}

bool david_stepper_telem_at_min_stop_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint16_t david_stepper_telem_left_position_encode(double value)
{
    return (uint16_t)(value / 0.25390625);
}

double david_stepper_telem_left_position_decode(uint16_t value)
{
    return ((double)value * 0.25390625);
}

bool david_stepper_telem_left_position_is_in_range(uint16_t value)
{
    return (value <= 2048u);
}

uint16_t david_stepper_telem_right_position_encode(double value)
{
    return (uint16_t)(value / 0.25390625);
}

double david_stepper_telem_right_position_decode(uint16_t value)
{
    return ((double)value * 0.25390625);
}

bool david_stepper_telem_right_position_is_in_range(uint16_t value)
{
    return (value <= 2048u);
}

int david_mast_ctrl_pack(
    uint8_t *dst_p,
    const struct david_mast_ctrl_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 1);

    dst_p[0] |= pack_left_shift_u8(src_p->direction, 0u, 0x03u);

    return (1);
}

int david_mast_ctrl_unpack(
    struct david_mast_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    dst_p->direction = unpack_right_shift_u8(src_p[0], 0u, 0x03u);

    return (0);
}

uint8_t david_mast_ctrl_direction_encode(double value)
{
    return (uint8_t)(value);
}

double david_mast_ctrl_direction_decode(uint8_t value)
{
    return ((double)value);
}

bool david_mast_ctrl_direction_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

int david_mast_telem_pack(
    uint8_t *dst_p,
    const struct david_mast_telem_t *src_p,
    size_t size)
{
    if (size < 2u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 2);

    dst_p[0] |= pack_left_shift_u16(src_p->angle, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(src_p->angle, 8u, 0xffu);

    return (2);
}

int david_mast_telem_unpack(
    struct david_mast_telem_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 2u) {
        return (-EINVAL);
    }

    dst_p->angle = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    dst_p->angle |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);

    return (0);
}

uint16_t david_mast_telem_angle_encode(double value)
{
    return (uint16_t)((value - -360.0) / 0.010986328125);
}

double david_mast_telem_angle_decode(uint16_t value)
{
    return (((double)value * 0.010986328125) + -360.0);
}

bool david_mast_telem_angle_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}
