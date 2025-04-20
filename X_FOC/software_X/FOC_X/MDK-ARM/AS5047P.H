#ifndef AS5047P_H
#define AS5047P_H

#define NOP 0x0000
#define ERRFL 0x0001
#define PROG 0x0003
#define DIAAGC 0x3FFC
#define MAG 0x3FFD
#define ANGLEUNC 0x3FFE
#define ANGLECOM 0x3FFF

#define ZPOSM 0x0016
#define ZPOSL 0x0017
#define SETTINGS1 0x0018
#define SETTINGS2 0x0019

#define AS5047_ANGLE_REG 0x3FFF  // ½Ç¶È¼Ä´æÆ÷µØÖ·

#define AS5407P_CS_L  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
#define AS5407P_CS_H  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);









#endif
