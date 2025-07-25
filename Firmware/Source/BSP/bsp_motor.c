#include "bsp_motor.h"
#include "bsp.h"
#include "app_motion.h"


uint8_t motor_enable = 0;


static int16_t Motor_Ignore_Dead_Zone(int16_t pulse)
{
    if (Motion_Get_Car_Type() == CAR_SUNRISE)
    {
        if (pulse > 0) return pulse + MOTOR_SUNRISE_IGNORE_PULSE;
        if (pulse < 0) return pulse - MOTOR_SUNRISE_IGNORE_PULSE;
    }
    else
    {
        if (pulse > 0) return pulse + MOTOR_IGNORE_PULSE;
        if (pulse < 0) return pulse - MOTOR_IGNORE_PULSE;
    }
    return 0;
}

// ���PWM�ڳ�ʼ��, arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ��
void Motor_PWM_Init(uint16_t arr, uint16_t psc)
{
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);
    //���½�Timer����Ϊȱʡֵ
    TIM_DeInit(TIM8);
    TIM_DeInit(TIM1);
    //Ԥ��Ƶϵ��Ϊ0����������Ԥ��Ƶ����ʱTIMER��Ƶ��Ϊ72MHzre.TIM_Prescaler =0;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    //���ü��������С��ÿ��xxx�����Ͳ���һ�������¼�
    TIM_TimeBaseStructure.TIM_Period = arr - 1;
    //����ʱ�ӷָ�
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    //���ü�����ģʽΪ���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    //������Ӧ�õ�TIM8��
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    //����ȱʡֵ
    TIM_OCStructInit(&TIM_OCInitStructure);
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //������PWMģʽ���ǱȽ�ģʽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ�ܣ�ʹ��PWM������˿�
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //���ü����Ǹ߻��ǵ�
    // TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;	  //���ü����Ǹ߻��ǵ�
    //����ռ�ձȣ�ռ�ձ�=(CCRx/ARR)*100%��(TIM_Pulse/TIM_Period)*100%
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);                      //TIM8��CHx���
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);                      //TIM8��CHx���
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);                      //TIM8��CHx���
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);                      //TIM8��CHx���
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);                      //TIM1��CHx���
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);                      //TIM1��CHx���
    

    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //������PWMģʽ���ǱȽ�ģʽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; //�Ƚ����ʧ��
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //����Ƚ����ʹ�ܣ�ʹ��PWM������˿�
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //���ü����Ǹ߻��ǵ�
    // TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;	  //���ü����Ǹ߻��ǵ�
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);                      //TIM1��CHxN���
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);                      //TIM1��CHxN���

    //����PWM���Ϊʹ��
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

// ���е��ֹͣ
void Motor_Stop(uint8_t brake)
{
    if (brake != 0) brake = 1;
    TIM_SetCompare1(TIM8, brake * MOTOR_MAX_PULSE);
    TIM_SetCompare2(TIM8, brake * MOTOR_MAX_PULSE);
    TIM_SetCompare3(TIM8, brake * MOTOR_MAX_PULSE);
    TIM_SetCompare4(TIM8, brake * MOTOR_MAX_PULSE);

    TIM_SetCompare1(TIM1, brake * MOTOR_MAX_PULSE);
    TIM_SetCompare2(TIM1, brake * MOTOR_MAX_PULSE);
    TIM_SetCompare3(TIM1, brake * MOTOR_MAX_PULSE);
    TIM_SetCompare4(TIM1, brake * MOTOR_MAX_PULSE);
}

// ���õ���ٶȣ�speed:��3600, 0Ϊֹͣ
void Motor_Set_Pwm(uint8_t id, int16_t speed)
{
    int16_t pulse = Motor_Ignore_Dead_Zone(speed);
    // ��������
    if (pulse >= MOTOR_MAX_PULSE)
        pulse = MOTOR_MAX_PULSE;
    if (pulse <= -MOTOR_MAX_PULSE)
        pulse = -MOTOR_MAX_PULSE;

    switch (id)
    {
    case MOTOR_ID_M1:
    {
        pulse = -pulse;
        if (pulse >= 0)
        {
            PWM_M1_A = pulse;
            PWM_M1_B = 0;
        }
        else
        {
            PWM_M1_A = 0;
            PWM_M1_B = -pulse;
        }
        break;
    }
    case MOTOR_ID_M2:
    {
        pulse = -pulse;
        if (pulse >= 0)
        {
            PWM_M2_A = pulse;
            PWM_M2_B = 0;
        }
        else
        {
            PWM_M2_A = 0;
            PWM_M2_B = -pulse;
        }
        break;
    }

    case MOTOR_ID_M3:
    {
        if (pulse >= 0)
        {
            PWM_M3_A = pulse;
            PWM_M3_B = 0;
        }
        else
        {
            PWM_M3_A = 0;
            PWM_M3_B = -pulse;
        }
        break;
    }
    case MOTOR_ID_M4:
    {
        if (pulse >= 0)
        {
            PWM_M4_A = pulse;
            PWM_M4_B = 0;
        }
        else
        {
            PWM_M4_A = 0;
            PWM_M4_B = -pulse;
        }
        break;
    }

    default:
        break;
    }
}

// ��ʼ���������
void MOTOR_GPIO_Init(void)
{
    /*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
    GPIO_InitTypeDef GPIO_InitStructure;
    /* ��ʼ�����Žṹ�� */
    gpio_t pwm[] = {
        {M1A_PORT, M1A_PIN, M1A_CLK},
        {M2A_PORT, M2A_PIN, M2A_CLK},
        {M3A_PORT, M3A_PIN, M3A_CLK},
        {M4A_PORT, M4A_PIN, M4A_CLK},

        {M1B_PORT, M1B_PIN, M1B_CLK},
        {M2B_PORT, M2B_PIN, M2B_CLK},
        {M3B_PORT, M3B_PIN, M3B_CLK},
        {M4B_PORT, M4B_PIN, M4B_CLK},
    };
    
    // ��ʼ��PWM����
    for (int i = 0; i < MAX_MOTOR*2; i++)
    {
        /* PWM������� */
        RCC_APB2PeriphClockCmd(pwm[i].clock, ENABLE);
        GPIO_InitStructure.GPIO_Pin = pwm[i].pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
        GPIO_Init(pwm[i].port, &GPIO_InitStructure);
    }

    motor_enable = MOTOR_ENABLE_A | MOTOR_ENABLE_B | MOTOR_ENABLE_C | MOTOR_ENABLE_D;
}

// �жϵ���Ƿ��ʼ�����Ƿ���1���񷵻�0
uint8_t Motor_Get_Enable_State(uint8_t enable_m)
{
    return motor_enable & enable_m;
}
