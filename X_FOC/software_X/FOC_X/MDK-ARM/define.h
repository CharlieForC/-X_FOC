#ifndef __define_H
#define __define_H
//******运行状态指示灯******//

#define Run_ON() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14 ,GPIO_PIN_RESET)
#define Run_OFF() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14 ,GPIO_PIN_SET)

#define Fault_ON() 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13 ,GPIO_PIN_RESET)
#define Fault_OFF() 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13 ,GPIO_PIN_SET)


#define PI 3.14159265358979f
#define _2PI 6.28318530717958f

unsigned char angle_mon_flag,angle_start_mon;///初始角度记录
#define rotor_phy_angle (angle - angle_start_mon)     // 转子物理角度
#define rotor_logic_angle rotor_phy_angle *10          // 转子多圈角度
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

unsigned char motor_mode=0;

float motor_speed_set=0,motor_tor_set=0,motor_pos_set;
float motor_spe_tor_set=10,motor_tor_spe_set=0.7;
float motor_pos_tor_set=90,motor_tor_pos_set=0.8;
float motor_tor_spe_pos_set=0.3,motor_spe_tor_pos_set=10,motor_pos_spe_tor_set=90;


#define deg2rad(a) (PI * (a) / 180)
#define rad2deg(a) (180 * (a) / PI)
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define position_cycle 6 * 3.14159265358979 
#define MAX_CURRENT 10 ///最大电流设置

#endif
