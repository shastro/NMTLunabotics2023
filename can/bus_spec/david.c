
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

static inline uint8_t pack_left_shift_u64(
    uint64_t value,
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

static inline uint8_t pack_right_shift_u64(
    uint64_t value,
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

static inline uint64_t unpack_left_shift_u64(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint64_t)((uint64_t)(value & mask) << shift);
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

static inline uint64_t unpack_right_shift_u64(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint64_t)((uint64_t)(value & mask) >> shift);
}

int david_e_stop_pack(
    uint8_t *dst_p,
    const struct david_e_stop_t *src_p,
    size_t size)
{
    (void)dst_p;
    (void)src_p;
    (void)size;

    return (0);
}

int david_e_stop_unpack(
    struct david_e_stop_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    (void)dst_p;
    (void)src_p;
    (void)size;

    return (0);
}

int david_e_start_pack(
    uint8_t *dst_p,
    const struct david_e_start_t *src_p,
    size_t size)
{
    (void)dst_p;
    (void)src_p;
    (void)size;

    return (0);
}

int david_e_start_unpack(
    struct david_e_start_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    (void)dst_p;
    (void)src_p;
    (void)size;

    return (0);
}

int david_pitch_ctrl_home_pack(
    uint8_t *dst_p,
    const struct david_pitch_ctrl_home_t *src_p,
    size_t size)
{
    (void)dst_p;
    (void)src_p;
    (void)size;

    return (0);
}

int david_pitch_ctrl_home_unpack(
    struct david_pitch_ctrl_home_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    (void)dst_p;
    (void)src_p;
    (void)size;

    return (0);
}

int david_set_trig_delay_pack(
    uint8_t *dst_p,
    const struct david_set_trig_delay_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u64(src_p->microseconds, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u64(src_p->microseconds, 8u, 0xffu);
    dst_p[2] |= pack_right_shift_u64(src_p->microseconds, 16u, 0xffu);
    dst_p[3] |= pack_right_shift_u64(src_p->microseconds, 24u, 0xffu);
    dst_p[4] |= pack_right_shift_u64(src_p->microseconds, 32u, 0xffu);
    dst_p[5] |= pack_right_shift_u64(src_p->microseconds, 40u, 0xffu);
    dst_p[6] |= pack_left_shift_u16(src_p->max_count, 0u, 0xffu);
    dst_p[7] |= pack_right_shift_u16(src_p->max_count, 8u, 0xffu);

    return (8);
}

int david_set_trig_delay_unpack(
    struct david_set_trig_delay_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->microseconds = unpack_right_shift_u64(src_p[0], 0u, 0xffu);
    dst_p->microseconds |= unpack_left_shift_u64(src_p[1], 8u, 0xffu);
    dst_p->microseconds |= unpack_left_shift_u64(src_p[2], 16u, 0xffu);
    dst_p->microseconds |= unpack_left_shift_u64(src_p[3], 24u, 0xffu);
    dst_p->microseconds |= unpack_left_shift_u64(src_p[4], 32u, 0xffu);
    dst_p->microseconds |= unpack_left_shift_u64(src_p[5], 40u, 0xffu);
    dst_p->max_count = unpack_right_shift_u16(src_p[6], 0u, 0xffu);
    dst_p->max_count |= unpack_left_shift_u16(src_p[7], 8u, 0xffu);

    return (0);
}

uint64_t david_set_trig_delay_microseconds_encode(double value)
{
    return (uint64_t)(value);
}

double david_set_trig_delay_microseconds_decode(uint64_t value)
{
    return ((double)value);
}

bool david_set_trig_delay_microseconds_is_in_range(uint64_t value)
{
    return (value <= 281474976710655ull);
}

uint16_t david_set_trig_delay_max_count_encode(double value)
{
    return (uint16_t)(value);
}

double david_set_trig_delay_max_count_decode(uint16_t value)
{
    return ((double)value);
}

bool david_set_trig_delay_max_count_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

int david_pitch_ctrl_left_pack(
    uint8_t *dst_p,
    const struct david_pitch_ctrl_left_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 1);

    dst_p[0] |= pack_left_shift_u8(src_p->command, 0u, 0xffu);

    return (1);
}

int david_pitch_ctrl_left_unpack(
    struct david_pitch_ctrl_left_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    dst_p->command = unpack_right_shift_u8(src_p[0], 0u, 0xffu);

    return (0);
}

uint8_t david_pitch_ctrl_left_command_encode(double value)
{
    return (uint8_t)(value);
}

double david_pitch_ctrl_left_command_decode(uint8_t value)
{
    return ((double)value);
}

bool david_pitch_ctrl_left_command_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int david_pitch_ctrl_right_pack(
    uint8_t *dst_p,
    const struct david_pitch_ctrl_right_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 1);

    dst_p[0] |= pack_left_shift_u8(src_p->command, 0u, 0xffu);

    return (1);
}

int david_pitch_ctrl_right_unpack(
    struct david_pitch_ctrl_right_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    dst_p->command = unpack_right_shift_u8(src_p[0], 0u, 0xffu);

    return (0);
}

uint8_t david_pitch_ctrl_right_command_encode(double value)
{
    return (uint8_t)(value);
}

double david_pitch_ctrl_right_command_decode(uint8_t value)
{
    return ((double)value);
}

bool david_pitch_ctrl_right_command_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int david_pitch_ctrl_both_pack(
    uint8_t *dst_p,
    const struct david_pitch_ctrl_both_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 1);

    dst_p[0] |= pack_left_shift_u8(src_p->command, 0u, 0xffu);

    return (1);
}

int david_pitch_ctrl_both_unpack(
    struct david_pitch_ctrl_both_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 1u) {
        return (-EINVAL);
    }

    dst_p->command = unpack_right_shift_u8(src_p[0], 0u, 0xffu);

    return (0);
}

uint8_t david_pitch_ctrl_both_command_encode(double value)
{
    return (uint8_t)(value);
}

double david_pitch_ctrl_both_command_decode(uint8_t value)
{
    return ((double)value);
}

bool david_pitch_ctrl_both_command_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int david_pitch_telem_left_pack(
    uint8_t *dst_p,
    const struct david_pitch_telem_left_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u64(src_p->count, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u64(src_p->count, 8u, 0xffu);
    dst_p[2] |= pack_right_shift_u64(src_p->count, 16u, 0xffu);
    dst_p[3] |= pack_right_shift_u64(src_p->count, 24u, 0xffu);
    dst_p[4] |= pack_right_shift_u64(src_p->count, 32u, 0xffu);
    dst_p[5] |= pack_right_shift_u64(src_p->count, 40u, 0xffu);
    dst_p[6] |= pack_left_shift_u16(src_p->direction, 0u, 0xffu);
    dst_p[7] |= pack_right_shift_u16(src_p->direction, 8u, 0xffu);

    return (8);
}

int david_pitch_telem_left_unpack(
    struct david_pitch_telem_left_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->count = unpack_right_shift_u64(src_p[0], 0u, 0xffu);
    dst_p->count |= unpack_left_shift_u64(src_p[1], 8u, 0xffu);
    dst_p->count |= unpack_left_shift_u64(src_p[2], 16u, 0xffu);
    dst_p->count |= unpack_left_shift_u64(src_p[3], 24u, 0xffu);
    dst_p->count |= unpack_left_shift_u64(src_p[4], 32u, 0xffu);
    dst_p->count |= unpack_left_shift_u64(src_p[5], 40u, 0xffu);
    dst_p->direction = unpack_right_shift_u16(src_p[6], 0u, 0xffu);
    dst_p->direction |= unpack_left_shift_u16(src_p[7], 8u, 0xffu);

    return (0);
}

uint64_t david_pitch_telem_left_count_encode(double value)
{
    return (uint64_t)(value);
}

double david_pitch_telem_left_count_decode(uint64_t value)
{
    return ((double)value);
}

bool david_pitch_telem_left_count_is_in_range(uint64_t value)
{
    return (value <= 281474976710655ull);
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

int david_pitch_telem_right_pack(
    uint8_t *dst_p,
    const struct david_pitch_telem_right_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u64(src_p->count, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u64(src_p->count, 8u, 0xffu);
    dst_p[2] |= pack_right_shift_u64(src_p->count, 16u, 0xffu);
    dst_p[3] |= pack_right_shift_u64(src_p->count, 24u, 0xffu);
    dst_p[4] |= pack_right_shift_u64(src_p->count, 32u, 0xffu);
    dst_p[5] |= pack_right_shift_u64(src_p->count, 40u, 0xffu);
    dst_p[6] |= pack_left_shift_u16(src_p->direction, 0u, 0xffu);
    dst_p[7] |= pack_right_shift_u16(src_p->direction, 8u, 0xffu);

    return (8);
}

int david_pitch_telem_right_unpack(
    struct david_pitch_telem_right_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->count = unpack_right_shift_u64(src_p[0], 0u, 0xffu);
    dst_p->count |= unpack_left_shift_u64(src_p[1], 8u, 0xffu);
    dst_p->count |= unpack_left_shift_u64(src_p[2], 16u, 0xffu);
    dst_p->count |= unpack_left_shift_u64(src_p[3], 24u, 0xffu);
    dst_p->count |= unpack_left_shift_u64(src_p[4], 32u, 0xffu);
    dst_p->count |= unpack_left_shift_u64(src_p[5], 40u, 0xffu);
    dst_p->direction = unpack_right_shift_u16(src_p[6], 0u, 0xffu);
    dst_p->direction |= unpack_left_shift_u16(src_p[7], 8u, 0xffu);

    return (0);
}

uint64_t david_pitch_telem_right_count_encode(double value)
{
    return (uint64_t)(value);
}

double david_pitch_telem_right_count_decode(uint64_t value)
{
    return ((double)value);
}

bool david_pitch_telem_right_count_is_in_range(uint64_t value)
{
    return (value <= 281474976710655ull);
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

int david_loco_ctrl_left_pack(
    uint8_t *dst_p,
    const struct david_loco_ctrl_left_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 4);

    dst_p[0] |= pack_left_shift_u32(src_p->velocity, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u32(src_p->velocity, 8u, 0xffu);
    dst_p[2] |= pack_right_shift_u32(src_p->velocity, 16u, 0xffu);
    dst_p[3] |= pack_right_shift_u32(src_p->velocity, 24u, 0xffu);

    return (4);
}

int david_loco_ctrl_left_unpack(
    struct david_loco_ctrl_left_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    dst_p->velocity = unpack_right_shift_u32(src_p[0], 0u, 0xffu);
    dst_p->velocity |= unpack_left_shift_u32(src_p[1], 8u, 0xffu);
    dst_p->velocity |= unpack_left_shift_u32(src_p[2], 16u, 0xffu);
    dst_p->velocity |= unpack_left_shift_u32(src_p[3], 24u, 0xffu);

    return (0);
}

uint32_t david_loco_ctrl_left_velocity_encode(double value)
{
    return (uint32_t)(value / 0.001);
}

double david_loco_ctrl_left_velocity_decode(uint32_t value)
{
    return ((double)value * 0.001);
}

bool david_loco_ctrl_left_velocity_is_in_range(uint32_t value)
{
    (void)value;

    return (true);
}

int david_loco_ctrl_right_pack(
    uint8_t *dst_p,
    const struct david_loco_ctrl_right_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 4);

    dst_p[0] |= pack_left_shift_u32(src_p->velocity, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u32(src_p->velocity, 8u, 0xffu);
    dst_p[2] |= pack_right_shift_u32(src_p->velocity, 16u, 0xffu);
    dst_p[3] |= pack_right_shift_u32(src_p->velocity, 24u, 0xffu);

    return (4);
}

int david_loco_ctrl_right_unpack(
    struct david_loco_ctrl_right_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    dst_p->velocity = unpack_right_shift_u32(src_p[0], 0u, 0xffu);
    dst_p->velocity |= unpack_left_shift_u32(src_p[1], 8u, 0xffu);
    dst_p->velocity |= unpack_left_shift_u32(src_p[2], 16u, 0xffu);
    dst_p->velocity |= unpack_left_shift_u32(src_p[3], 24u, 0xffu);

    return (0);
}

uint32_t david_loco_ctrl_right_velocity_encode(double value)
{
    return (uint32_t)(value / 0.001);
}

double david_loco_ctrl_right_velocity_decode(uint32_t value)
{
    return ((double)value * 0.001);
}

bool david_loco_ctrl_right_velocity_is_in_range(uint32_t value)
{
    (void)value;

    return (true);
}

int david_loco_ctrl_both_pack(
    uint8_t *dst_p,
    const struct david_loco_ctrl_both_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 4);

    dst_p[0] |= pack_left_shift_u32(src_p->velocity, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u32(src_p->velocity, 8u, 0xffu);
    dst_p[2] |= pack_right_shift_u32(src_p->velocity, 16u, 0xffu);
    dst_p[3] |= pack_right_shift_u32(src_p->velocity, 24u, 0xffu);

    return (4);
}

int david_loco_ctrl_both_unpack(
    struct david_loco_ctrl_both_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    dst_p->velocity = unpack_right_shift_u32(src_p[0], 0u, 0xffu);
    dst_p->velocity |= unpack_left_shift_u32(src_p[1], 8u, 0xffu);
    dst_p->velocity |= unpack_left_shift_u32(src_p[2], 16u, 0xffu);
    dst_p->velocity |= unpack_left_shift_u32(src_p[3], 24u, 0xffu);

    return (0);
}

uint32_t david_loco_ctrl_both_velocity_encode(double value)
{
    return (uint32_t)(value / 0.001);
}

double david_loco_ctrl_both_velocity_decode(uint32_t value)
{
    return ((double)value * 0.001);
}

bool david_loco_ctrl_both_velocity_is_in_range(uint32_t value)
{
    (void)value;

    return (true);
}

int david_stepper_ctrl_left_pack(
    uint8_t *dst_p,
    const struct david_stepper_ctrl_left_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u32(src_p->rpm, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u32(src_p->rpm, 8u, 0xffu);
    dst_p[2] |= pack_right_shift_u32(src_p->rpm, 16u, 0xffu);
    dst_p[3] |= pack_right_shift_u32(src_p->rpm, 24u, 0xffu);
    dst_p[4] |= pack_left_shift_u32(src_p->direction, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u32(src_p->direction, 8u, 0xffu);
    dst_p[6] |= pack_right_shift_u32(src_p->direction, 16u, 0xffu);
    dst_p[7] |= pack_right_shift_u32(src_p->direction, 24u, 0xffu);

    return (8);
}

int david_stepper_ctrl_left_unpack(
    struct david_stepper_ctrl_left_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->rpm = unpack_right_shift_u32(src_p[0], 0u, 0xffu);
    dst_p->rpm |= unpack_left_shift_u32(src_p[1], 8u, 0xffu);
    dst_p->rpm |= unpack_left_shift_u32(src_p[2], 16u, 0xffu);
    dst_p->rpm |= unpack_left_shift_u32(src_p[3], 24u, 0xffu);
    dst_p->direction = unpack_right_shift_u32(src_p[4], 0u, 0xffu);
    dst_p->direction |= unpack_left_shift_u32(src_p[5], 8u, 0xffu);
    dst_p->direction |= unpack_left_shift_u32(src_p[6], 16u, 0xffu);
    dst_p->direction |= unpack_left_shift_u32(src_p[7], 24u, 0xffu);

    return (0);
}

uint32_t david_stepper_ctrl_left_rpm_encode(double value)
{
    return (uint32_t)(value);
}

double david_stepper_ctrl_left_rpm_decode(uint32_t value)
{
    return ((double)value);
}

bool david_stepper_ctrl_left_rpm_is_in_range(uint32_t value)
{
    (void)value;

    return (true);
}

uint32_t david_stepper_ctrl_left_direction_encode(double value)
{
    return (uint32_t)(value);
}

double david_stepper_ctrl_left_direction_decode(uint32_t value)
{
    return ((double)value);
}

bool david_stepper_ctrl_left_direction_is_in_range(uint32_t value)
{
    (void)value;

    return (true);
}

int david_stepper_ctrl_right_pack(
    uint8_t *dst_p,
    const struct david_stepper_ctrl_right_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u32(src_p->rpm, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u32(src_p->rpm, 8u, 0xffu);
    dst_p[2] |= pack_right_shift_u32(src_p->rpm, 16u, 0xffu);
    dst_p[3] |= pack_right_shift_u32(src_p->rpm, 24u, 0xffu);
    dst_p[4] |= pack_left_shift_u32(src_p->direction, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u32(src_p->direction, 8u, 0xffu);
    dst_p[6] |= pack_right_shift_u32(src_p->direction, 16u, 0xffu);
    dst_p[7] |= pack_right_shift_u32(src_p->direction, 24u, 0xffu);

    return (8);
}

int david_stepper_ctrl_right_unpack(
    struct david_stepper_ctrl_right_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->rpm = unpack_right_shift_u32(src_p[0], 0u, 0xffu);
    dst_p->rpm |= unpack_left_shift_u32(src_p[1], 8u, 0xffu);
    dst_p->rpm |= unpack_left_shift_u32(src_p[2], 16u, 0xffu);
    dst_p->rpm |= unpack_left_shift_u32(src_p[3], 24u, 0xffu);
    dst_p->direction = unpack_right_shift_u32(src_p[4], 0u, 0xffu);
    dst_p->direction |= unpack_left_shift_u32(src_p[5], 8u, 0xffu);
    dst_p->direction |= unpack_left_shift_u32(src_p[6], 16u, 0xffu);
    dst_p->direction |= unpack_left_shift_u32(src_p[7], 24u, 0xffu);

    return (0);
}

uint32_t david_stepper_ctrl_right_rpm_encode(double value)
{
    return (uint32_t)(value);
}

double david_stepper_ctrl_right_rpm_decode(uint32_t value)
{
    return ((double)value);
}

bool david_stepper_ctrl_right_rpm_is_in_range(uint32_t value)
{
    (void)value;

    return (true);
}

uint32_t david_stepper_ctrl_right_direction_encode(double value)
{
    return (uint32_t)(value);
}

double david_stepper_ctrl_right_direction_decode(uint32_t value)
{
    return ((double)value);
}

bool david_stepper_ctrl_right_direction_is_in_range(uint32_t value)
{
    (void)value;

    return (true);
}

int david_stepper_ctrl_both_pack(
    uint8_t *dst_p,
    const struct david_stepper_ctrl_both_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u32(src_p->rpm, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u32(src_p->rpm, 8u, 0xffu);
    dst_p[2] |= pack_right_shift_u32(src_p->rpm, 16u, 0xffu);
    dst_p[3] |= pack_right_shift_u32(src_p->rpm, 24u, 0xffu);
    dst_p[4] |= pack_left_shift_u32(src_p->direction, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u32(src_p->direction, 8u, 0xffu);
    dst_p[6] |= pack_right_shift_u32(src_p->direction, 16u, 0xffu);
    dst_p[7] |= pack_right_shift_u32(src_p->direction, 24u, 0xffu);

    return (8);
}

int david_stepper_ctrl_both_unpack(
    struct david_stepper_ctrl_both_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->rpm = unpack_right_shift_u32(src_p[0], 0u, 0xffu);
    dst_p->rpm |= unpack_left_shift_u32(src_p[1], 8u, 0xffu);
    dst_p->rpm |= unpack_left_shift_u32(src_p[2], 16u, 0xffu);
    dst_p->rpm |= unpack_left_shift_u32(src_p[3], 24u, 0xffu);
    dst_p->direction = unpack_right_shift_u32(src_p[4], 0u, 0xffu);
    dst_p->direction |= unpack_left_shift_u32(src_p[5], 8u, 0xffu);
    dst_p->direction |= unpack_left_shift_u32(src_p[6], 16u, 0xffu);
    dst_p->direction |= unpack_left_shift_u32(src_p[7], 24u, 0xffu);

    return (0);
}

uint32_t david_stepper_ctrl_both_rpm_encode(double value)
{
    return (uint32_t)(value);
}

double david_stepper_ctrl_both_rpm_decode(uint32_t value)
{
    return ((double)value);
}

bool david_stepper_ctrl_both_rpm_is_in_range(uint32_t value)
{
    (void)value;

    return (true);
}

uint32_t david_stepper_ctrl_both_direction_encode(double value)
{
    return (uint32_t)(value);
}

double david_stepper_ctrl_both_direction_decode(uint32_t value)
{
    return ((double)value);
}

bool david_stepper_ctrl_both_direction_is_in_range(uint32_t value)
{
    (void)value;

    return (true);
}

int david_excav_ctrl_pack(
    uint8_t *dst_p,
    const struct david_excav_ctrl_t *src_p,
    size_t size)
{
    uint32_t rpm;

    if (size < 4u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 4);

    rpm = (uint32_t)src_p->rpm;
    dst_p[0] |= pack_left_shift_u32(rpm, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u32(rpm, 8u, 0xffu);
    dst_p[2] |= pack_right_shift_u32(rpm, 16u, 0xffu);
    dst_p[3] |= pack_right_shift_u32(rpm, 24u, 0xffu);

    return (4);
}

int david_excav_ctrl_unpack(
    struct david_excav_ctrl_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint32_t rpm;

    if (size < 4u) {
        return (-EINVAL);
    }

    rpm = unpack_right_shift_u32(src_p[0], 0u, 0xffu);
    rpm |= unpack_left_shift_u32(src_p[1], 8u, 0xffu);
    rpm |= unpack_left_shift_u32(src_p[2], 16u, 0xffu);
    rpm |= unpack_left_shift_u32(src_p[3], 24u, 0xffu);
    dst_p->rpm = (int32_t)rpm;

    return (0);
}

int32_t david_excav_ctrl_rpm_encode(double value)
{
    return (int32_t)(value / 0.001);
}

double david_excav_ctrl_rpm_decode(int32_t value)
{
    return ((double)value * 0.001);
}

bool david_excav_ctrl_rpm_is_in_range(int32_t value)
{
    (void)value;

    return (true);
}
