#ifndef _M18_H_
#define _M18_H_
#include <vector>
#include <iostream>
#include "Console.hpp"
extern "C" {
#include "canopen.h"
}

/**
 * 2.4mm for 131072 count
 */
#define POS_UNIT    (131072.0f / 2.4f)
#define MM_2_UNIT(mm)      ((mm) * POS_UNIT)
#define UNIT_2_MM(unit)    ((unit) / POS_UNIT)

// CST target and 0x6077 feedback use signed raw drive units; no force calibration.

typedef enum {
    MRUN_CSP = 0x08,
    MRUN_CST = 0x0A,
} MRunMode;

void m18_enable_driver(uint8_t slave_id, uint8_t passage, uint32_t node_id);
void m18_disable_driver(uint8_t slave_id, uint8_t passage, uint32_t node_id);

int m18_enter_pre_op(uint8_t slave_id, uint8_t passage, uint32_t node_id);
int m18_start_node(uint8_t slave_id, uint8_t passage, uint32_t node_id);
int m18_stop_node(uint8_t slave_id, uint8_t passage, uint32_t node_id);
int m18_reset_node(uint8_t slave_id, uint8_t passage, uint32_t node_id);
int m18_reset_comm(uint8_t slave_id, uint8_t passage, uint32_t node_id);

void m18_build_csp_message(struct can_frame* frame, uint32_t node_id, int32_t pos_count);
void m18_build_cst_message(struct can_frame* frame, uint32_t node_id, int16_t tor);

void m18_set_position(uint8_t slave_id, uint8_t passage, uint32_t node_id, int32_t pos_count);
void m18_set_position_pp(uint8_t slave_id, uint8_t passage, uint32_t node_id, int32_t pos_count);
void m18_set_PP_plan_param(uint8_t slave_id, uint8_t passage, uint32_t node_id,
            uint32_t speed, uint32_t acce, uint32_t dece, uint32_t speed_max, uint32_t acce_dece_max);
void m18_set_mode(uint8_t slave_id, uint8_t passage, uint32_t node_id, uint8_t mode);
int m18_read_sdo(uint8_t slave_id, uint8_t passage, uint32_t node_id, uint16_t index,
                 uint8_t subindex, uint32_t* value, uint8_t* value_size);
int m18_write_sdo(uint8_t slave_id, uint8_t passage, uint32_t node_id, uint16_t index,
                  uint8_t subindex, uint32_t value, SDO_CMD command);
void m18_set_elec_gear_ration(uint8_t slave_id, uint8_t passage, uint32_t node_id, uint32_t Numerator, uint32_t denominator);
void m18_set_pdo_map(uint8_t slave_id, uint8_t passage, uint32_t node_id, uint8_t pdo_num, uint32_t* obj_list, uint8_t obj_count);

#endif  // _M18_H_
