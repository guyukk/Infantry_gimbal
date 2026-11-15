/* Current H723 calibration; see calibration-manual.md for supported entries and ABI. */
#include "calibrate_task.h"
#include "string.h"
#include "cmsis_os.h"
#include "adc.h"

#include "bsp_flash.h"
#include "bsp_usart.h"
#include "bsp_rc.h"
#include "bsp_buzzer.h"

#include "remote_control.h"
#include "INS_task.h"
#include "gimbal_task.h"
#include "can_bsp.h"
#include "robot_param.h"
#include "yaw_pitch_direct.h"
#include <math.h>
#include <stddef.h>
#include "health.h"


//include head,gimbal,gyro,accel,mag. gyro,accel and mag have the same data struct. total 5(CALI_LIST_LENGHT) devices, need data lenght + 5 * 4 bytes(name[3]+cali)
#define FLASH_WRITE_BUF_LENGHT 4096U /* preserve the whole legacy sector, including reserved bytes */




/**
  * @brief          use remote control to begin a calibrate,such as gyro, gimbal, chassis
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
static void RC_cmd_to_calibrate(void);

/**
  * @brief          read cali data from flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          从flash读取校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void);

/**
  * @brief          write the data to flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          往flash写入校准数据
  * @param[in]      none
  * @retval         none
  */
static int8_t cali_data_write(void);


/**
  * @brief          "head" sensor cali function
  * @param[in][out] cali:the point to head data. when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          "head"设备校准
  * @param[in][out] cali:指针指向head数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd);   //header device cali function

/**
  * @brief          gyro cali function
  * @param[in][out] cali:the point to gyro data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          陀螺仪设备校准
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */

/**
  * @brief          gimbal cali function
  * @param[in][out] cali:the point to gimbal data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          云台设备校准
  * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd); //gimbal device cali function



#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif


static const RC_ctrl_t *calibrate_RC;   //remote control point
static head_cali_t     head_cali;       //head cali data
static gimbal_cali_t   gimbal_cali;     //gimbal cali data
static uint8_t calibration_flash_ready;
static uint8_t pitch_dm_flash_ready;
//static imu_cali_t      accel_cali;      //accel cali data
//static imu_cali_t      gyro_cali;       //gyro cali data
//static imu_cali_t      mag_cali;        //mag cali data


static uint32_t flash_sector_words[FLASH_WRITE_BUF_LENGHT/4U];
#define flash_write_buf ((uint8_t*)flash_sector_words)
volatile cali_storage_status_t cali_storage_status;

cali_sensor_t cali_sensor[CALI_LIST_LENGHT]; 

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM"};

//cali data address
static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] = {
        (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
       };


static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
    {
        sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4
    };

void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, cali_gimbal_hook};

static uint32_t calibrate_systemTick;

// Separate 4 KB W25Q sector. The legacy HD/GM record at sector 0 is unchanged.
#define PITCH_DM_CALI_ADDR 0x1000UL
#define PITCH_DM_CALI_MAGIC 0x50444D31UL
#define PITCH_DM_CALI_VERSION 1UL
#define PITCH_DM_CALI_RETRY_MS 1000U
typedef struct
{
    uint32_t magic;
    uint32_t version;
    float min_pos;
    float max_pos;
    uint32_t check;
} pitch_dm_cali_record_t;

typedef char pitch_record_size_check[(sizeof(pitch_dm_cali_record_t)==20) ? 1:-1];
typedef char pitch_record_fields_check[(offsetof(pitch_dm_cali_record_t,min_pos)==8 && offsetof(pitch_dm_cali_record_t,max_pos)==12 && offsetof(pitch_dm_cali_record_t,check)==16) ? 1:-1];
static uint8_t legacy_record_valid(uint8_t id)
{
    if(id==CALI_HEAD) return isfinite(head_cali.latitude) && head_cali.latitude>=-90.0f && head_cali.latitude<=90.0f;
    /* The unused legacy Pitch fields do not control this DM motor. */
    return gimbal_cali.yaw_offset<8192U && isfinite(gimbal_cali.yaw_min_angle) &&
        isfinite(gimbal_cali.yaw_max_angle) && gimbal_cali.yaw_min_angle>=-PI &&
        gimbal_cali.yaw_max_angle<=PI && gimbal_cali.yaw_min_angle<gimbal_cali.yaw_max_angle;
}
static uint8_t pitch_cali_feedback(dm_measure_t *out)
{
    dm_measure_t *source=get_dm_motor_measure_point(PITCH_DM_MOTOR_INDEX);
    uint8_t ready;
    if(source==NULL) return 0U;
    taskENTER_CRITICAL();
    *out=*source;
    ready=health_fresh(HEALTH_PITCH,HAL_GetTick());
    taskEXIT_CRITICAL();
    return ready && isfinite(out->fdb.pos) && isfinite(out->fdb.vel) &&
        (out->fdb.state==1U || (PITCH_DM_CAN_ID==0x53U && out->fdb.id==3U && out->fdb.state==5U));
}
volatile pitch_dm_cali_status_t pitch_dm_cali_status;

static uint32_t pitch_dm_cali_check(const pitch_dm_cali_record_t *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint8_t bit;
    for (i = 0; i < sizeof(*record) - sizeof(record->check); i++)
    {
        crc ^= bytes[i];
        for (bit = 0; bit < 8U; bit++)
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
    }
    return ~crc;
}

static uint8_t pitch_dm_cali_valid_range(float min_pos, float max_pos)
{
    float span = max_pos - min_pos;
    return isfinite(min_pos) && isfinite(max_pos) &&
           min_pos > -PITCH_MIT_P_MAX && max_pos < PITCH_MIT_P_MAX &&
           span >= 0.1f && span <= 1.5f;
}

static uint8_t pitch_dm_cali_read_record(pitch_dm_cali_record_t *record)
{
    if (!pitch_dm_flash_ready)
    {
        pitch_dm_flash_ready = (flash_init() == 0);
    }
    if (!pitch_dm_flash_ready) return 0U;
    if (flash_read(PITCH_DM_CALI_ADDR, (uint32_t *)record, sizeof(*record) / 4U) != 0)
    {
        pitch_dm_flash_ready = 0U;
        return 0U;
    }
    return record->magic == PITCH_DM_CALI_MAGIC &&
           record->version == PITCH_DM_CALI_VERSION &&
           record->check == pitch_dm_cali_check(record) &&
           pitch_dm_cali_valid_range(record->min_pos, record->max_pos);
}

void pitch_dm_cali_load(float *min_pos, float *max_pos)
{
    pitch_dm_cali_record_t record;
    if (min_pos == NULL || max_pos == NULL || !pitch_dm_cali_read_record(&record)) return;
    *min_pos = record.min_pos;
    *max_pos = record.max_pos;
    pitch_dm_cali_status.min_pos = record.min_pos;
    pitch_dm_cali_status.max_pos = record.max_pos;
    pitch_dm_cali_status.stored_valid = 1U;
}

static void pitch_dm_cali_recover(void)
{
    static uint32_t last_retry_tick;
    pitch_dm_cali_record_t record;
    dm_measure_t sample;
    dm_measure_t *motor=&sample;
    uint32_t now = HAL_GetTick();

    if (!pitch_dm_cali_status.init_done || (pitch_dm_cali_status.stored_valid && !pitch_dm_cali_status.saved_pending) ||
        pitch_dm_cali_status.request != 0U || pitch_dm_cali_status.captured != 0U ||
        (uint32_t)(now - last_retry_tick) < PITCH_DM_CALI_RETRY_MS) return;
    last_retry_tick = now;
    if (!pitch_dm_cali_read_record(&record)) return;

    if (!pitch_cali_feedback(motor) ||
        motor->fdb.pos < record.min_pos - PITCH_DM_LOWER_RECOVERY_RAD ||
        motor->fdb.pos > record.max_pos) return;

    // Publish limits and the current target together; the gimbal task must not
    // see stored_valid before its position limits have been updated.
    taskENTER_CRITICAL();
    pitch_dm_apply_calibration(&gimbal_control, record.min_pos, record.max_pos,
                               motor->fdb.pos);
    pitch_dm_cali_status.min_pos = record.min_pos;
    pitch_dm_cali_status.max_pos = record.max_pos;
    pitch_dm_cali_status.stored_valid = 1U;
    if (pitch_dm_cali_status.saved_pending) pitch_dm_cali_status.result=2;
    pitch_dm_cali_status.saved_pending = 0U;
    taskEXIT_CRITICAL();
}

static void pitch_dm_cali_poll(void)
{
    uint8_t request = pitch_dm_cali_status.request;
    dm_measure_t sample;
    dm_measure_t *motor=&sample;
    float min_pos, max_pos;
    pitch_dm_cali_record_t record;
    pitch_dm_cali_record_t readback;
    if (request == 0U) return;
    pitch_dm_cali_status.request = 0U;
    if (request == 4U)
    {
        pitch_dm_cali_status.captured = 0U;
        pitch_dm_cali_status.result = 0;
        return;
    }
    if (!pitch_cali_feedback(motor))
    {
        pitch_dm_cali_status.result = -1; // stale or unusable feedback
        return;
    }
    if (request == 1U || request == 2U)
    {
        if (!isfinite(motor->fdb.pos) || !isfinite(motor->fdb.vel) ||
            fabsf(motor->fdb.vel) > 0.2f)
        {
            pitch_dm_cali_status.result = -2; // moving or invalid position
            return;
        }
        if (request == 1U)
        {
            pitch_dm_cali_status.first_pos = motor->fdb.pos;
            pitch_dm_cali_status.captured |= 1U;
        }
        else
        {
            pitch_dm_cali_status.second_pos = motor->fdb.pos;
            pitch_dm_cali_status.captured |= 2U;
        }
        pitch_dm_cali_status.result = 1;
        return;
    }
    if (request != 3U || pitch_dm_cali_status.captured != 3U)
    {
        pitch_dm_cali_status.result = -3; // two samples required
        return;
    }
    min_pos = fminf(pitch_dm_cali_status.first_pos, pitch_dm_cali_status.second_pos);
    max_pos = fmaxf(pitch_dm_cali_status.first_pos, pitch_dm_cali_status.second_pos);
    if (!isfinite(motor->fdb.pos) || !isfinite(motor->fdb.vel) || fabsf(motor->fdb.vel)>0.2f ||
        !pitch_dm_cali_valid_range(min_pos, max_pos) ||
        motor->fdb.pos < min_pos || motor->fdb.pos > max_pos)
    {
        pitch_dm_cali_status.result = -4; // unsafe range or current position outside it
        return;
    }
    if (!pitch_dm_flash_ready) pitch_dm_flash_ready = (flash_init() == 0);
    if (!pitch_dm_flash_ready) { pitch_dm_cali_status.result = -5; return; }
    record.magic = PITCH_DM_CALI_MAGIC;
    record.version = PITCH_DM_CALI_VERSION;
    record.min_pos = min_pos;
    record.max_pos = max_pos;
    record.check = pitch_dm_cali_check(&record);
    if (flash_write_single_address(PITCH_DM_CALI_ADDR, (uint32_t *)&record,
                                   sizeof(record) / 4U) != 0)
    {
        pitch_dm_cali_status.result = -5; // flash write failed
        return;
    }
    if (flash_read(PITCH_DM_CALI_ADDR, (uint32_t *)&readback, sizeof(readback) / 4U) != 0 ||
        memcmp(&readback, &record, sizeof(record)) != 0)
    {
        pitch_dm_cali_status.result = -5;
        return;
    }
    /* Flash may block long enough for feedback to expire or the mechanism to move. */
    if (!pitch_cali_feedback(motor) || fabsf(motor->fdb.vel)>0.2f ||
        motor->fdb.pos<min_pos || motor->fdb.pos>max_pos)
    {
        pitch_dm_cali_status.saved_pending=1U;
        pitch_dm_cali_status.captured=0U;
        pitch_dm_cali_status.result=-6; /* persisted, not yet applied; old runtime limits stay */
        return;
    }
    taskENTER_CRITICAL();
    pitch_dm_apply_calibration(&gimbal_control, min_pos, max_pos, motor->fdb.pos);
    pitch_dm_cali_status.min_pos = min_pos;
    pitch_dm_cali_status.max_pos = max_pos;
    pitch_dm_cali_status.stored_valid = 1U;
    pitch_dm_cali_status.saved_pending = 0U;
    taskEXIT_CRITICAL();
    pitch_dm_cali_status.captured = 0U;
    pitch_dm_cali_status.result = 2;
}


/**
  * @brief          calibrate task, created by main function
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          校准任务，由main函数创建
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void calibrate_task(void const *pvParameters)
{
    static uint8_t i = 0;
    
    calibrate_RC = get_remote_ctrl_point_cali();

    while (1)
    {

        RC_cmd_to_calibrate();
        pitch_dm_cali_poll();
        pitch_dm_cali_recover();

        for (i = 0; i < CALI_LIST_LENGHT; i++)
        {
            if (cali_sensor[i].cali_cmd)
            {
                if (cali_sensor[i].cali_hook != NULL)
                {

                    if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON))
                    {
                        //done
                        cali_sensor[i].name[0] = cali_name[i][0];
                        cali_sensor[i].name[1] = cali_name[i][1];
                        cali_sensor[i].name[2] = cali_name[i][2];
                        //set 0x55
                        cali_sensor[i].cali_done = CALIED_FLAG;

                        cali_sensor[i].cali_cmd = 0;
                        //write
                        if (cali_data_write()!=0)
                            cali_sensor[i].cali_done=0U; /* RAM flag must not claim persisted success. */
                    }
                }
            }
        }
        osDelay(CALIBRATE_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          get imu control temperature, unit ℃
  * @param[in]      none
  * @retval         imu control temperature
  */
/**
  * @brief          获取imu控制温度, 单位℃
  * @param[in]      none
  * @retval         imu控制温度
  */
int8_t get_control_temperature(void)
{

    return head_cali.temperature;
}

/**
  * @brief          get latitude, default 22.0f
  * @param[out]     latitude: the point to fp32 
  * @retval         none
  */
/**
  * @brief          获取纬度,默认22.0f
  * @param[out]     latitude:fp32指针 
  * @retval         none
  */
void get_flash_latitude(float *latitude)
{

    if (latitude == NULL)
    {

        return;
    }
    if (cali_sensor[CALI_HEAD].cali_done == CALIED_FLAG)
    {
        *latitude = head_cali.latitude;
    }
    else
    {
        *latitude = 22.0f;
    }
}

/**
  * @brief          use remote control to begin a calibrate,such as gyro, gimbal, chassis
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
static void RC_cmd_to_calibrate(void)
{
    static const uint8_t BEGIN_FLAG   = 1;
    static const uint8_t GIMBAL_FLAG  = 2;



    static uint8_t  i;
    static uint32_t rc_cmd_systemTick = 0;
    static uint16_t buzzer_time       = 0;
    static uint16_t rc_cmd_time       = 0;
    static uint8_t  rc_action_flag    = 0;

    //if something is calibrating, return
    //如果已经在校准，就返回
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_sensor[i].cali_cmd)
        {
            buzzer_time = 0;
            rc_cmd_time = 0;
            rc_action_flag = 0;

            return;
        }
    }

    if (rc_action_flag == 0 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        rc_cmd_systemTick = xTaskGetTickCount();
        rc_action_flag = BEGIN_FLAG;
        rc_cmd_time = 0;
    }
    else if (rc_action_flag == GIMBAL_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        //gimbal cali, 
        rc_action_flag = 0;
        rc_cmd_time = 0;
        // The legacy GM routine drives a DJI Pitch by raw current and reads its
        // encoder. It must not run with the DM MIT Pitch installed.
        cali_buzzer_off();
    }

    if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_mid(calibrate_RC->rc.s[0]) && switch_is_mid(calibrate_RC->rc.s[1]) && rc_action_flag == 0)
    {
        //two rockers set to  \../, hold for 2 seconds,
        //两个摇杆打成 \../,保持2s
        rc_cmd_time++;
    }
    else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_mid(calibrate_RC->rc.s[0]) && switch_is_mid(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
    {
        //two rockers set '\/', hold for 2 seconds
        //两个摇杆打成'\/',保持2s
        rc_cmd_time++;
        rc_action_flag = GIMBAL_FLAG;
    }
    else
    {
        rc_cmd_time = 0;
    }

    calibrate_systemTick = xTaskGetTickCount();

    if (calibrate_systemTick - rc_cmd_systemTick > CALIBRATE_END_TIME)
    {
        //over 20 seconds, end
        //超过20s,停止
        rc_action_flag = 0;
        return;
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME && rc_cmd_systemTick != 0 && rc_action_flag != 0)
    {
        rc_cali_buzzer_middle_on();
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > 0 && rc_cmd_systemTick != 0 && rc_action_flag != 0)
    {
        rc_cali_buzzer_start_on();
    }

    if (rc_action_flag != 0)
    {
        buzzer_time++;
    }
    
    if (buzzer_time > RCCALI_BUZZER_CYCLE_TIME && rc_action_flag != 0)
    {
        buzzer_time = 0;
    }
    if (buzzer_time > RC_CALI_BUZZER_PAUSE_TIME && rc_action_flag != 0)
    {
        cali_buzzer_off();
    }
}

/**
  * @brief          use remote control to begin a calibrate,such as gyro, gimbal, chassis
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
void cali_param_init(void)
{
    uint8_t i = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        cali_sensor[i].flash_len = cali_sensor_size[i];
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
    }

    calibration_flash_ready = (flash_init() == 0);
    pitch_dm_flash_ready = calibration_flash_ready;
    if (calibration_flash_ready)
    {
        cali_data_read();
    }

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_sensor[i].cali_done == CALIED_FLAG)
        {
            if (cali_sensor[i].cali_hook != NULL)
            {
                //if has been calibrated, set to init 
                cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
            }
        }
    }
}

/**
  * @brief          read cali data from flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          从flash读取校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void)
{
    uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
    uint8_t i = 0;
    uint16_t offset = 0;
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {

        //read the data in flash, 
        if (cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len) != 0)
        {
            calibration_flash_ready = 0U;
            cali_storage_status.read_failures++;
            cali_storage_status.last_result=-1;
            for (uint8_t j = 0U; j < CALI_LIST_LENGHT; j++) cali_sensor[j].cali_done = 0U;
            return;
        }
        
        offset += cali_sensor[i].flash_len * 4;

        //read the name and cali flag,
        if (cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT) != 0)
        {
            calibration_flash_ready = 0U;
            cali_storage_status.read_failures++;
            cali_storage_status.last_result=-1;
            for (uint8_t j = 0U; j < CALI_LIST_LENGHT; j++) cali_sensor[j].cali_done = 0U;
            return;
        }
        
        cali_sensor[i].name[0] = flash_read_buf[0];
        cali_sensor[i].name[1] = flash_read_buf[1];
        cali_sensor[i].name[2] = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];
        if(cali_sensor[i].cali_done==CALIED_FLAG &&
           (memcmp(flash_read_buf,cali_name[i],3)!=0 || !legacy_record_valid(i)))
        {
            cali_sensor[i].cali_done=0U;
            if(i==CALI_HEAD) cali_storage_status.invalid_head++;
            else cali_storage_status.invalid_yaw++;
        }
        
        offset += CALI_SENSOR_HEAD_LEGHT * 4;

        if (i != CALI_GIMBAL && cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL)
        {
            cali_sensor[i].cali_cmd = 1;
        }
    }
}


/**
  * @brief          write the data to flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          往flash写入校准数据
  * @param[in]      none
  * @retval         none
  */
static int8_t cali_data_write(void)
{
uint8_t i; uint16_t offset=0; uint32_t n;
    uint32_t verify[64];
    if(!calibration_flash_ready) {cali_storage_status.last_result=-1;return -1;}
    /* Sector 0 is the owner boundary; preserve all bytes beyond the 36B HD/GM prefix. */
    if(cali_flash_read(FLASH_USER_ADDR,flash_sector_words,FLASH_WRITE_BUF_LENGHT/4U)!=0)
    {cali_storage_status.read_failures++;cali_storage_status.last_result=-1;return -1;}
    for(i=0;i<CALI_LIST_LENGHT;i++)
    {
        memcpy(flash_write_buf+offset,cali_sensor[i].flash_buf,cali_sensor[i].flash_len*4U);
        offset+=cali_sensor[i].flash_len*4U;
        memcpy(flash_write_buf+offset,cali_sensor[i].name,4U);
        offset+=4U;
    }
    if(cali_flash_write(FLASH_USER_ADDR,flash_sector_words,FLASH_WRITE_BUF_LENGHT/4U)!=0)
    {cali_storage_status.write_failures++;cali_storage_status.last_result=-2;return -2;}
    for(n=0;n<FLASH_WRITE_BUF_LENGHT;n+=sizeof(verify))
    {
        if(cali_flash_read(FLASH_USER_ADDR+n,verify,sizeof(verify)/4U)!=0 ||
           memcmp(verify,flash_write_buf+n,sizeof(verify))!=0)
        {cali_storage_status.verify_failures++;cali_storage_status.last_result=-3;return -3;}
    }
    cali_storage_status.last_result=1;
    return 0;
}


/**
  * @brief          "head" sensor cali function
  * @param[in][out] cali:the point to head data. when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          "head"设备校准
  * @param[in][out] cali:指针指向head数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd)
{
    head_cali_t *local_cali_t = (head_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
//        memcpy(&head_cali, local_cali_t, sizeof(head_cali_t));

        return 1;
    }
    // self id
    local_cali_t->self_id = SELF_ID;
    //imu control temperature
    local_cali_t->temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    //head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    if (local_cali_t->temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
    {
        local_cali_t->temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
    }
    
    local_cali_t->firmware_version = FIRMWARE_VERSION;
    //shenzhen latitude 
    local_cali_t->latitude = 22.0f;

    return 1;
}

/**
  * @brief          gyro cali function
  * @param[in][out] cali:the point to gyro data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          陀螺仪设备校准
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */


/**
  * @brief          gimbal cali function
  * @param[in][out] cali:the point to gimbal data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          云台设备校准
  * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd)
{

    gimbal_cali_t *local_cali_t = (gimbal_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        if(!legacy_record_valid(CALI_GIMBAL)) return 0;
        set_cali_yaw_hook(local_cali_t->yaw_offset,
                          local_cali_t->yaw_max_angle, local_cali_t->yaw_min_angle);
        
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        // The old automatic GM routine cannot calibrate a DM MIT Pitch.
        // It must not drive the mechanism or persist its DJI encoder readings.
        return 0;
    }
    
    return 0;
}
