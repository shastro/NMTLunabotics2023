
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

static inline uint8_t pack_left_shift_u32(
    uint32_t value,
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

static inline uint8_t pack_right_shift_u32(
    uint32_t value,
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

static inline uint32_t unpack_left_shift_u32(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint32_t)((uint32_t)(value & mask) << shift);
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

static inline uint32_t unpack_right_shift_u32(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint32_t)((uint32_t)(value & mask) >> shift);
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
    if (size < 2u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 2);

    dst_p[0] |= pack_left_shift_u8(src_p->home, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u8(src_p->direction, 0u, 0xffu);

    return (2);
}

int david_pitch_ctrl_unpack(
    struct david_pitch_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 2u) {
        return (-EINVAL);
    }

    dst_p->home = unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->direction = unpack_right_shift_u8(src_p[1], 0u, 0xffu);

    return (0);
}

uint8_t david_pitch_ctrl_home_encode(double value)
{
    return (uint8_t)(value);
}

double david_pitch_ctrl_home_decode(uint8_t value)
{
    return ((double)value);
}

bool david_pitch_ctrl_home_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t david_pitch_ctrl_direction_encode(double value)
{
    return (uint8_t)(value);
}

double david_pitch_ctrl_direction_decode(uint8_t value)
{
    return ((double)value);
}

bool david_pitch_ctrl_direction_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int david_pitch_adjust_pack(
    uint8_t *dst_p,
    const struct david_pitch_adjust_t *src_p,
    size_t size)
{
    if (size < 3u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 3);

    dst_p[0] |= pack_left_shift_u8(src_p->motor, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u8(src_p->dir, 0u, 0xffu);
    dst_p[2] |= pack_left_shift_u8(src_p->amount, 0u, 0xffu);

    return (3);
}

int david_pitch_adjust_unpack(
    struct david_pitch_adjust_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 3u) {
        return (-EINVAL);
    }

    dst_p->motor = unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->dir = unpack_right_shift_u8(src_p[1], 0u, 0xffu);
    dst_p->amount = unpack_right_shift_u8(src_p[2], 0u, 0xffu);

    return (0);
}

uint8_t david_pitch_adjust_motor_encode(double value)
{
    return (uint8_t)(value);
}

double david_pitch_adjust_motor_decode(uint8_t value)
{
    return ((double)value);
}

bool david_pitch_adjust_motor_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t david_pitch_adjust_dir_encode(double value)
{
    return (uint8_t)(value);
}

double david_pitch_adjust_dir_decode(uint8_t value)
{
    return ((double)value);
}

bool david_pitch_adjust_dir_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t david_pitch_adjust_amount_encode(double value)
{
    return (uint8_t)(value);
}

double david_pitch_adjust_amount_decode(uint8_t value)
{
    return ((double)value);
}

bool david_pitch_adjust_amount_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int david_pitch_telem_pack(
    uint8_t *dst_p,
    const struct david_pitch_telem_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u16(src_p->left_count, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(src_p->left_count, 8u, 0xffu);
    dst_p[2] |= pack_left_shift_u16(src_p->left_direction, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(src_p->left_direction, 8u, 0xffu);
    dst_p[4] |= pack_left_shift_u16(src_p->right_count, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u16(src_p->right_count, 8u, 0xffu);
    dst_p[6] |= pack_left_shift_u16(src_p->right_direction, 0u, 0xffu);
    dst_p[7] |= pack_right_shift_u16(src_p->right_direction, 8u, 0xffu);

    return (8);
}

int david_pitch_telem_unpack(
    struct david_pitch_telem_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->left_count = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    dst_p->left_count |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->left_direction = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    dst_p->left_direction |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);
    dst_p->right_count = unpack_right_shift_u16(src_p[4], 0u, 0xffu);
    dst_p->right_count |= unpack_left_shift_u16(src_p[5], 8u, 0xffu);
    dst_p->right_direction = unpack_right_shift_u16(src_p[6], 0u, 0xffu);
    dst_p->right_direction |= unpack_left_shift_u16(src_p[7], 8u, 0xffu);

    return (0);
}

uint16_t david_pitch_telem_left_count_encode(double value)
{
    return (uint16_t)(value);
}

double david_pitch_telem_left_count_decode(uint16_t value)
{
    return ((double)value);
}

bool david_pitch_telem_left_count_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

uint16_t david_pitch_telem_left_direction_encode(double value)
{
    return (uint16_t)(value);
}

double david_pitch_telem_left_direction_decode(uint16_t value)
{
    return ((double)value);
}

bool david_pitch_telem_left_direction_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

uint16_t david_pitch_telem_right_count_encode(double value)
{
    return (uint16_t)(value);
}

double david_pitch_telem_right_count_decode(uint16_t value)
{
    return ((double)value);
}

bool david_pitch_telem_right_count_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

uint16_t david_pitch_telem_right_direction_encode(double value)
{
    return (uint16_t)(value);
}

double david_pitch_telem_right_direction_decode(uint16_t value)
{
    return ((double)value);
}

bool david_pitch_telem_right_direction_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

int david_loco_ctrl_pack(
    uint8_t *dst_p,
    const struct david_loco_ctrl_t *src_p,
    size_t size)
{
    uint16_t left_vel;
    uint16_t right_vel;

    if (size < 4u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 4);

    left_vel = (uint16_t)src_p->left_vel;
    dst_p[0] |= pack_left_shift_u16(left_vel, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(left_vel, 8u, 0xffu);
    right_vel = (uint16_t)src_p->right_vel;
    dst_p[2] |= pack_left_shift_u16(right_vel, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(right_vel, 8u, 0xffu);

    return (4);
}

int david_loco_ctrl_unpack(
    struct david_loco_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t left_vel;
    uint16_t right_vel;

    if (size < 4u) {
        return (-EINVAL);
    }

    left_vel = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    left_vel |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->left_vel = (int16_t)left_vel;
    right_vel = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    right_vel |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);
    dst_p->right_vel = (int16_t)right_vel;

    return (0);
}

int16_t david_loco_ctrl_left_vel_encode(double value)
{
    return (int16_t)(value);
}

double david_loco_ctrl_left_vel_decode(int16_t value)
{
    return ((double)value);
}

bool david_loco_ctrl_left_vel_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int16_t david_loco_ctrl_right_vel_encode(double value)
{
    return (int16_t)(value);
}

double david_loco_ctrl_right_vel_decode(int16_t value)
{
    return ((double)value);
}

bool david_loco_ctrl_right_vel_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int david_excav_ctrl_pack(
    uint8_t *dst_p,
    const struct david_excav_ctrl_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 1);

    dst_p[0] |= pack_left_shift_u8(src_p->direction, 0u, 0xffu);

    return (1);
}

int david_excav_ctrl_unpack(
    struct david_excav_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    dst_p->direction = unpack_right_shift_u8(src_p[0], 0u, 0xffu);

    return (0);
}

uint8_t david_excav_ctrl_direction_encode(double value)
{
    return (uint8_t)(value);
}

double david_excav_ctrl_direction_decode(uint8_t value)
{
    return ((double)value);
}

bool david_excav_ctrl_direction_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int david_stepper_ctrl_pack(
    uint8_t *dst_p,
    const struct david_stepper_ctrl_t *src_p,
    size_t size)
{
    if (size < 6u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 6);

    dst_p[0] |= pack_left_shift_u8(src_p->home, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u8(src_p->direction, 0u, 0xffu);
    dst_p[2] |= pack_left_shift_u32(src_p->rpm, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u32(src_p->rpm, 8u, 0xffu);
    dst_p[4] |= pack_right_shift_u32(src_p->rpm, 16u, 0xffu);
    dst_p[5] |= pack_right_shift_u32(src_p->rpm, 24u, 0xffu);

    return (6);
}

int david_stepper_ctrl_unpack(
    struct david_stepper_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 6u) {
        return (-EINVAL);
    }

    dst_p->home = unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->direction = unpack_right_shift_u8(src_p[1], 0u, 0xffu);
    dst_p->rpm = unpack_right_shift_u32(src_p[2], 0u, 0xffu);
    dst_p->rpm |= unpack_left_shift_u32(src_p[3], 8u, 0xffu);
    dst_p->rpm |= unpack_left_shift_u32(src_p[4], 16u, 0xffu);
    dst_p->rpm |= unpack_left_shift_u32(src_p[5], 24u, 0xffu);

    return (0);
}

uint8_t david_stepper_ctrl_home_encode(double value)
{
    return (uint8_t)(value);
}

double david_stepper_ctrl_home_decode(uint8_t value)
{
    return ((double)value);
}

bool david_stepper_ctrl_home_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t david_stepper_ctrl_direction_encode(double value)
{
    return (uint8_t)(value);
}

double david_stepper_ctrl_direction_decode(uint8_t value)
{
    return ((double)value);
}

bool david_stepper_ctrl_direction_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint32_t david_stepper_ctrl_rpm_encode(double value)
{
    return (uint32_t)(value);
}

double david_stepper_ctrl_rpm_decode(uint32_t value)
{
    return ((double)value);
}

bool david_stepper_ctrl_rpm_is_in_range(uint32_t value)
{
    (void)value;

    return (true);
}

int david_stepper_ctrl_adjust_pack(
    uint8_t *dst_p,
    const struct david_stepper_ctrl_adjust_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 4);

    dst_p[0] |= pack_left_shift_u8(src_p->motor, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u8(src_p->dir, 0u, 0xffu);
    dst_p[2] |= pack_left_shift_u16(src_p->amount, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(src_p->amount, 8u, 0xffu);

    return (4);
}

int david_stepper_ctrl_adjust_unpack(
    struct david_stepper_ctrl_adjust_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    dst_p->motor = unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->dir = unpack_right_shift_u8(src_p[1], 0u, 0xffu);
    dst_p->amount = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    dst_p->amount |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);

    return (0);
}

uint8_t david_stepper_ctrl_adjust_motor_encode(double value)
{
    return (uint8_t)(value);
}

double david_stepper_ctrl_adjust_motor_decode(uint8_t value)
{
    return ((double)value);
}

bool david_stepper_ctrl_adjust_motor_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t david_stepper_ctrl_adjust_dir_encode(double value)
{
    return (uint8_t)(value);
}

double david_stepper_ctrl_adjust_dir_decode(uint8_t value)
{
    return ((double)value);
}

bool david_stepper_ctrl_adjust_dir_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint16_t david_stepper_ctrl_adjust_amount_encode(double value)
{
    return (uint16_t)(value);
}

double david_stepper_ctrl_adjust_amount_decode(uint16_t value)
{
    return ((double)value);
}

bool david_stepper_ctrl_adjust_amount_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}
