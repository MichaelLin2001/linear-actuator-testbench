#ifndef _COMM_MESSAGE_H_
#define _COMM_MESSAGE_H_

typedef enum {
    APP_MODE_STOP,
    APP_MODE_USER = 0x02,
    APP_MODE_TOR = 0x0A,
    APP_MODE_POS = 0x08,
    APP_MODE_POS_TOR,
} AppMode;

typedef enum {
    RM_PP = 0x01,
    RM_FP,
    RM_PV,
    RM_PT,
    RM_R2,
    RM_HM,
    RM_R3,
    RM_CSP,
    RM_CSV,
    RM_CST,
    RM_MAX
} MotorRunMode;
extern const char* mode_list[];

typedef enum { MT_M18 = 1 } MotorType;

typedef struct {
    uint8_t run_mode;   /** see @MotorRunMode */
    uint8_t motor_type; /** see @MotorType */
    uint8_t reserved[2];

    union {
        struct {
            float target_speed;  /** for motor_type = MT_M18, the unit is mm/s */
            float target_pos;    /** for motor_type = MT_M18, the unit is mm */
            float target_torque; /** MT_M18: signed raw 0x6071; reserved bytes carry the CST command sequence. */
        };

        struct {
            float feedback_speed;  /** for motor_type = MT_M18, the unit is mm/s */
            float feedback_pos;    /** for motor_type = MT_M18, the unit is mm */
            float feedback_torque; /** MT_M18: signed raw 0x6077, not N or A; legacy field name preserves ABI. */
        };
    };

} MHandle;

typedef struct MotorParam {
    uint8_t motor_type;
    uint8_t run_mode;
    uint8_t reserved[2];
    float tor_const; /** unit: N*m/A */
    float lead_mm;   /** only valid for motor_type = MT_M18 */
    float gear_ratio;
    float eff_total;

    float pos_limit[2]; /**  pos_limit[2] = {pos_min, pos_max} */
    uint16_t cur_limit; /** unit: A */
    uint16_t tor_limit; /** unit: N路m */

    void (*decode_frame)(MHandle* handle, const MotorParam* param, const struct can_frame msg);
} MotorParam;
#endif  // _COMM_MESSAGE_H_
