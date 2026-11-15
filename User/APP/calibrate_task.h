/* H723 calibration: W25Q sector 0 HD/GM legacy records; DM Pitch v1 at 0x1000.
 * Only historical Yaw load and Watch-driven Pitch two-point calibration are active.
 * No legacy stick-driven automatic end-stop or gyro calibration is available. */
#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H

#include "struct_typedef.h"
#include <stddef.h>

//when imu is calibrating ,buzzer set frequency and strength. 当imu在校准,蜂鸣器的设置频率和强度
#define imu_start_buzzer()          buzzer_on(95, 10000)    
//when gimbal is calibrating ,buzzer set frequency and strength.当云台在校准,蜂鸣器的设置频率和强度
#define gimbal_start_buzzer()       buzzer_on(31, 19999)    
#define cali_buzzer_off()           buzzer_off()            //buzzer off，关闭蜂鸣器


//get stm32 chip temperature, to calc imu control temperature.获取stm32片内温度，计算imu的控制温度
#define cali_get_mcu_temperature()  get_temprate()      



#define cali_flash_read(address, buf, len)  flash_read((address), (buf), (len))                     //flash read function, flash 读取函数
#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len))     //flash write function,flash 写入函数
#define cali_flash_erase(address, page_num) flash_erase_address((address), (page_num))              //flash erase function,flash擦除函数


#define get_remote_ctrl_point_cali()        get_remote_control_point()  //get the remote control point，获取遥控器指针
#define gyro_cali_disable_control()         RC_unable()                 //when imu is calibrating, disable the remote control.当imu在校准时候,失能遥控器
#define gyro_cali_enable_control()          RC_restart(SBUS_RX_BUF_NUM*2)

// calc the zero drift function of gyro, 计算陀螺仪零漂
#define gyro_cali_fun(cali_scale, cali_offset, time_count)  INS_cali_gyro((cali_scale), (cali_offset), (time_count))
//set the zero drift to the INS task, 设置在INS task内的陀螺仪零漂
#define gyro_set_cali(cali_scale, cali_offset)              INS_set_cali_gyro((cali_scale), (cali_offset))



#define FLASH_USER_ADDR         0x000000UL //保存的页地址，在w25q64的开头

#define GYRO_CONST_MAX_TEMP     45.0f               //max control temperature of gyro,最大陀螺仪控制温度

#define CALI_FUNC_CMD_ON        1                   //need calibrate,设置校准
#define CALI_FUNC_CMD_INIT      0                   //has been calibrated, set value to init.已经校准过，设置校准值

#define CALIBRATE_CONTROL_TIME  1                   //osDelay time,  means 1ms.1ms 系统延时

#define CALI_SENSOR_HEAD_LEGHT  1

#define SELF_ID                 0                   //ID 
#define FIRMWARE_VERSION        12345               //handware version.
#define CALIED_FLAG             0x55                // means it has been calibrated
//you have 20 seconds to calibrate by remote control. 有20s可以用遥控器进行校准
#define CALIBRATE_END_TIME          20000
//when 10 second, buzzer frequency change to high frequency of gimbal calibration.当10s的时候,蜂鸣器切成高频声音
#define RC_CALI_BUZZER_MIDDLE_TIME  10000
//in the beginning, buzzer frequency change to low frequency of imu calibration.当开始校准的时候,蜂鸣器切成低频声音
#define RC_CALI_BUZZER_START_TIME   0


#define rc_cali_buzzer_middle_on()  gimbal_start_buzzer()
#define rc_cali_buzzer_start_on()   imu_start_buzzer()
#define RC_CMD_LONG_TIME            2000    

#define RCCALI_BUZZER_CYCLE_TIME    400        
#define RC_CALI_BUZZER_PAUSE_TIME   200       
#define RC_CALI_VALUE_HOLE          600     //remote control threshold, the max value of remote control channel is 660. 


#define GYRO_CALIBRATE_TIME         20000   //gyro calibrate time,陀螺仪校准时间

//cali device name
typedef enum
{
    CALI_HEAD = 0,
    CALI_GIMBAL = 1,
    //add more...
    CALI_LIST_LENGHT,
} cali_id_e;


#pragma pack(push, 1)
typedef  struct
{
    uint8_t name[3];                                    //device name
    uint8_t cali_done;                                  //0x55 means has been calibrated
    uint8_t flash_len : 7;                              //buf lenght
    uint8_t cali_cmd : 1;                               //1 means to run cali hook function,
    uint32_t *flash_buf;                                //link to device calibration data
    bool_t (*cali_hook)(uint32_t *point, bool_t cmd);   //cali function
} cali_sensor_t;
#pragma pack(pop)

//header device
#pragma pack(push, 1)
typedef  struct
{
    uint8_t self_id;            // the "SELF_ID"
    uint16_t firmware_version;  // set to the "FIRMWARE_VERSION"
    //'temperature' and 'latitude' should not be in the head_cali, because don't want to create a new sensor
    //'temperature' and 'latitude'不应该在head_cali,因为不想创建一个新的设备就放这了
    int8_t temperature;         // imu control temperature
    fp32 latitude;              // latitude
} head_cali_t;
#pragma pack(pop)
//gimbal device
typedef struct
{
    uint16_t yaw_offset;
    uint16_t pitch_offset;
    fp32 yaw_max_angle;
    fp32 yaw_min_angle;
    fp32 pitch_max_angle;
    fp32 pitch_min_angle;
} gimbal_cali_t;
//gyro, accel, mag device
typedef struct
{
    fp32 offset[3]; //x,y,z
    fp32 scale[3];  //x,y,z
} imu_cali_t;


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
extern void cali_param_init(void);

/* Owner ABI is explicit and independent of USB's __packed macro. */
typedef char cali_head_size_check[(sizeof(head_cali_t)==8) ? 1:-1];
typedef char cali_head_version_check[(offsetof(head_cali_t,firmware_version)==1) ? 1:-1];
typedef char cali_head_temp_check[(offsetof(head_cali_t,temperature)==3) ? 1:-1];
typedef char cali_head_latitude_check[(offsetof(head_cali_t,latitude)==4) ? 1:-1];
typedef char cali_gm_size_check[(sizeof(gimbal_cali_t)==20) ? 1:-1];
typedef char cali_gm_yaw_check[(offsetof(gimbal_cali_t,yaw_max_angle)==4 && offsetof(gimbal_cali_t,yaw_min_angle)==8) ? 1:-1];
typedef char cali_sensor_size_check[(sizeof(cali_sensor_t)==13 && offsetof(cali_sensor_t,flash_buf)==5) ? 1:-1];
typedef struct {
    uint32_t read_failures, write_failures, verify_failures, invalid_head, invalid_yaw;
    int8_t last_result; /* 0 idle; 1 verified write; -1 unavailable/read; -2 write; -3 readback */
} cali_storage_status_t;
extern volatile cali_storage_status_t cali_storage_status;

// DM Pitch safe-position calibration. Set request in Keil Watch:
// 1=capture first position, 2=capture second, 3=validate/save, 4=cancel.
// Capture positions already inside the mechanical travel; the motor is not driven by this routine.
typedef struct
{
    volatile uint8_t request;
    volatile uint8_t captured; // bit 0: first, bit 1: second
    volatile uint8_t stored_valid; // 1 if a valid DM range was loaded or saved
    volatile uint8_t saved_pending; // saved v1 exists, waiting for fresh in-range feedback to apply
    volatile uint8_t init_done; // gimbal_init finished; runtime reload may apply the range
    volatile int8_t result;     // 0 idle, 1 captured, 2 saved/applied; -6 saved pending; other negatives: rejected
    float first_pos;
    float second_pos;
    float min_pos;
    float max_pos;
} pitch_dm_cali_status_t;
extern volatile pitch_dm_cali_status_t pitch_dm_cali_status;
extern void pitch_dm_cali_load(float *min_pos, float *max_pos);
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
extern int8_t get_control_temperature(void);

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
extern void get_flash_latitude(float *latitude);

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
extern void calibrate_task(void const *pvParameters);


#endif
