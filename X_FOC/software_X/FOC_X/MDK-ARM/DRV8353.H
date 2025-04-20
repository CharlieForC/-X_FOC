#ifndef drv8353_H
#define drv8353_H


///*****DRV8353≈‰÷√*****//
#define DRV8353_NSS_LOW()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)///DRV8353 CS
#define DRV8353_NSS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define DRV8353_PWM_cotr_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET)//DRV8353 PWM_øÿ÷∆
#define DRV8353_PWM_cotr_ON() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET)
#define DRV8353_ENABLE_ON() HAL_GPIO_WritePin (GPIOC,GPIO_PIN_14,GPIO_PIN_SET)
#define DRV8353_ENABLE_OFF() HAL_GPIO_WritePin (GPIOC,GPIO_PIN_14,GPIO_PIN_RESET)

typedef struct {
	 unsigned int TIMP0;
	 unsigned int TIMP1;
	 unsigned int TIMP2;
	 unsigned int TIMP3;
	 unsigned int TIMP4;
	 unsigned int TIMP5;
	 unsigned int TIMP6;
} DRV_8353_info;

void Set_DRV8353(void);

#endif
