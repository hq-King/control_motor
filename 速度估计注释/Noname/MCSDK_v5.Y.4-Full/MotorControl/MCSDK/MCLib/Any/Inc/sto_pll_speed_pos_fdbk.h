/**
  ******************************************************************************
  * @file    sto_pll_speed_pos_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          State Observer + PLL Speed & Position Feedback component of the Motor
  *          Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup SpeednPosFdbk_STO
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STO_PLL_SPEEDNPOSFDBK_H
#define STO_PLL_SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"
#include "sto_speed_pos_fdbk.h"
#include "pid_regulator.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @addtogroup SpeednPosFdbk_STO
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief This structure is used to handle an instance of the STO_SpeednPosFdbk component
  *
  */

typedef struct
{
  SpeednPosFdbk_Handle_t _Super;// 继承自SpeednPosFdbk_Handle_t的成员变量

  int16_t  hC1;                 /*!< Variable containing state observer constant
                                     C1 to speed-up computations */ //< 状态观测器常数C1，用于加快计算速度 
  int16_t  hC2;                /*!< Variable containing state observer constant
																/!< 状态观测器常数C2，可以计算为F1 * K1 / 状态观测器执行速率[Hz]，其中K1是两个观测器增益之一 /
                                     C2, it can be computed as F1 * K1/ State
                                     observer execution rate [Hz] being K1 one
                                     of the two observer gains   */
  int16_t  hC3; //!< 状态观测器常数C3                /*!< Variable containing state observer constant
                                     
  int16_t  hC4;                  /*!< State Observer constant C4, it can
	//< 状态观测器常数C4，可以计算为K2 * 最大可测量电流（A）/
	//（最大应用速度[#SPEED_UNIT] * 电机BEMF常数[Vllrms/krpm] * sqrt(2) * F2 * 状态观测器执行速率[Hz]），其中K2是两个观测器增益之一 /
                                      be computed as K2 * max measurable
                                      current (A) / (Max application speed
                                      [#SPEED_UNIT] * Motor B-emf constant
                                      [Vllrms/krpm] * sqrt(2) * F2 * State
                                      observer execution rate [Hz]) being
                                       K2 one of the two observer gains  */
  int16_t  hC5;                 /*!< Variable containing state observer constant
	/!< 状态观测器常数C5 /
                                     C5 */
  int16_t  hC6;                 /*!< State observer constant C6, computed with a
                                   /!< 状态观测器常数C6，通过特定过程从其他常数计算得出 /  specific procedure starting from the other
                                    constants */
  int16_t  hF1;                 /*!< Variable containing state observer scaling
                                    /!< 状态观测器缩放因子F1 / factor F1 */
  int16_t  hF2;                 /*!< Variable containing state observer scaling factor F2/!< 状态观测器缩放因子F2 /*/
  int16_t  hF3;                  /*!< State observer scaling factor F3/!< 状态观测器缩放因子F3 / */
  uint16_t F3POW2;              /*!< State observer scaling factor F3 /!< 状态观测器缩放因子F3的2的幂次方表示。例如，如果增益除数为512，则该值必须为9，因为2^9 = 512 /expressed as power of 2.
                                     E.g. if gain divisor is 512 the value
                                     must be 9 because 2^9 = 512 */
  PID_Handle_t PIRegulator;     /*!< PI regulator component handle, used for
                                     PLL implementation  /!< PI调节器组件句柄，用于PLL实现 /*/
  int32_t Ialfa_est;           /*!< Estimated Ialfa current in int32 format /!< 以int32格式估计的Ialfa电流 /*/
  int32_t Ibeta_est;           /*!< Estimated Ibeta current in int32 format /!< 以int32格式估计的Ibeta电流 /*/
  int32_t wBemf_alfa_est;       /*!< Estimated B-emf alfa in int32_t format /!< 以int32_t格式估计的BEMF（反电动势） alfa /*/
  int32_t wBemf_beta_est;       /*!< Estimated B-emf beta in int32_t format /!< 以int32_t格式估计的BEMF beta /*/
  int16_t hBemf_alfa_est;       /*!< Estimated B-emf alfa in int16_t format /!< 以int16_t格式估计的BEMF alfa /*/
  int16_t hBemf_beta_est;       /*!< Estimated B-emf beta in int16_t format /!< 以int16_t格式估计的BEMF beta /*/
  int16_t Speed_Buffer[64];    /*!< Estimated DPP speed FIFO, it contains latest
                                     SpeedBufferSizeDpp speed measurements/!< 估计的DPP速度FIFO，包含最新的SpeedBufferSizeDpp速度测量值 /*/
  uint8_t Speed_Buffer_Index;  /*!< Position of latest speed estimation in
                                     estimated speed FIFO /!< 估计速度FIFO中最新速度估计的位置 /*/
  bool IsSpeedReliable;        /*!< Latest private speed reliability information,
                                     updated by SPD_CalcAvrgMecSpeedUnit, it is
                                     true if the speed measurement variance is
																		 /!< 最新的私有速度可靠性信息，由SPD_CalcAvrgMecSpeedUnit更新，如果速度测量方差小于等于hVariancePercentage对应的阈值，则为true /
                                     lower then threshold corresponding to
                                     hVariancePercentage */
  uint8_t ConsistencyCounter;  /*!< Counter of passed tests for start-up
                                     validation /!< 启动验证中通过的连续测试次数计数器 /*/
  uint8_t ReliabilityCounter; /*!< Counter for reliability check /!< 可靠性检查计数器 /*/
  bool IsAlgorithmConverged;   /*!/!< 包含观测器收敛信息的布尔变量 /< Boolean variable containing observer
                                     convergence information */
  bool IsBemfConsistent;       /*!< Sensor reliability information, updated by
                                     SPD_CalcAvrgMecSpeedUnit, it is true if the
                                     observed back-emfs are consistent with
                                     expectation/!< 传感器可靠性信息，由SPD_CalcAvrgMecSpeedUnit更新，如果观测到的反电动势与期望值一致，则为true */

  int32_t Obs_Bemf_Level;      /*!< Observed back-emf Level< 观测到的反电动势电平*/
  int32_t Est_Bemf_Level;      /*!< Estimated back-emf Level估计的反电动势电平*/
  bool EnableDualCheck;        /*!< Consistency check enabler/!< 一致性检查使能器 /*/
  int32_t DppBufferSum;        /*!< summation of speed buffer elements [dpp]/!< 速度缓冲区元素之和[dpp] /*/
  int16_t SpeedBufferOldestEl; /*!< Oldest element of the speed buffer/!< 速度缓冲区中最旧的元素 */

  uint8_t SpeedBufferSizeUnit;       /*!< Depth of FIFO used to average
                                           estimated speed exported by
                                           SPD_GetAvrgMecSpeedUnit. It
                                           must be an integer number between 1
                                           and 64 < 用于平均SPD_GetAvrgMecSpeedUnit导出的估计速度的FIFO的深度。它必须是1到64之间的整数 */
  uint8_t SpeedBufferSizeDpp;       /*!< Depth of FIFO used for both averaging
                                           estimated speed exported by
                                           SPD_GetElSpeedDpp and state
                                           observer equations. It must be an
                                           integer number between 1 and
                                           bSpeedBufferSizeUnit < 用于平均SPD_GetElSpeedDpp和状态观测器方程估计速度的FIFO的深度。它必须是1到bSpeedBufferSizeUnit之间的整数*/
  uint16_t VariancePercentage;        /*!< Parameter expressing the maximum
                                           allowed variance of speed estimation
                                           < 表示速度估计的最大允许方差的参数 */
  uint8_t SpeedValidationBand_H;   /*!< It expresses how much estimated speed
                                           can exceed forced stator electrical
                                           frequency during start-up without
                                           being considered wrong. The
                                           measurement unit is 1/16 of forced
                                           speed < 在启动期间，估计速度可以超过强制定子电气频率多少而不被视为错误。测量单位为强制速度的1/16*/
  uint8_t SpeedValidationBand_L;   /*!< It expresses how much estimated speed
                                           can be below forced stator electrical
                                           frequency during start-up without
                                           being considered wrong. The
                                           measurement unit is 1/16 of forced
                                           speed < 在启动期间，估计速度可以低于强制定子电气频率多少而不被视为错误。测量单位为强制速度的1/16 */
  uint16_t MinStartUpValidSpeed;     /*!< Absolute value of minimum mechanical
                                            speed  required to validate the start-up.
                                            Expressed in the unit defined by #SPEED_UNIT. < 验证启动所需的最小机械速度的绝对值。以#SPEED_UNIT定义的单位表示 */
  uint8_t StartUpConsistThreshold;   /*!< Number of consecutive tests on speed
                                           consistency to be passed before
                                           validating the start-up /!< 在验证启动之前通过的速度一致性测试次数 */
  uint8_t Reliability_hysteresys;    /*!< Number of reliability failed
                                           consecutive tests before a speed
                                           check fault is returned to _Super.bSpeedErrorNumber可靠性失败连续测试次数，在速度检查故障返回到_Super.bSpeedErrorNumber之前
                                           */
  uint8_t BemfConsistencyCheck;      /*!< Degree of consistency of the observed
                                           back-emfs, it must be an integer
                                           number ranging from 1 (very high
                                           consistency) down to 64 (very poor
																					 观测到的反电动势的一致性程度，必须是从1（非常高的一致性）到64（非常差的一致性）的整数
                                           consistency) */
  uint8_t BemfConsistencyGain;       /*!< Gain to be applied when checking
                                           back-emfs consistency; default value
                                           is 64 (neutral), max value 105
                                           (x1.64 amplification), min value 1
																					 检查反电动势一致性时应用的增益；默认值为64（中性），最大值为105（x1.64放大），最小值为1（/64衰减
                                           (/64 attenuation) */
  uint16_t MaxAppPositiveMecSpeedUnit; /*!< Maximum positive value of rotor speed. Expressed in
                                             the unit defined by #SPEED_UNIT. It can be
                                             x1.1 greater than max application speed*/
  uint16_t F1LOG;                    /*!< F1 gain divisor expressed as power of 2.
                                            E.g. if gain divisor is 512 the value
																						uint16_t F1LOG; /!< F1增益除数的2的幂次方表示。例如，如果增益除数为512，则该值必须为9，因为2^9 = 512 /
                                            must be 9 because 2^9 = 512 */
  uint16_t F2LOG;                    /*!< F2 gain divisor expressed as power of 2.
                                            E.g. if gain divisor is 512 the value
                                            must be 9 because 2^9 = 512 */
  uint16_t SpeedBufferSizeDppLOG;    /*!< bSpeedBufferSizedpp expressed as power of 2.
                                            E.g. if gain divisor is 512 the value
                                            must be 9 because 2^9 = 512 */
  bool ForceConvergency;       /*!< Variable to force observer convergence.*/
  bool ForceConvergency2;      /*!< Variable to force observer convergence.*/

  int8_t hForcedDirection;

} STO_PLL_Handle_t;


/* Exported functions ------------------------------------------------------- */

/* It initializes the state observer object *//* 初始化状态观测器对象 */
void STO_PLL_Init(STO_PLL_Handle_t *pHandle);
__weak int16_t Medin_Average_filter(int16_t Size,int16_t Speed_Buffer[]);//中值平均值滤波

/* It only returns, necessary to implement fictitious IRQ_Handler *//* 只返回，实现虚构的IRQ_Handler所必需 */
void STO_PLL_Return(STO_PLL_Handle_t *pHandle, uint8_t flag);

/* It clears state observer object by re-initializing private variables*//* 通过重新初始化私有变量来清除状态观测器对象 */
void STO_PLL_Clear(STO_PLL_Handle_t *pHandle);

/* It executes Luenberger state observer and calls PLL to compute a new speed
*  estimation and update the estimated electrical angle.
*//* 执行Luenberger状态观测器并调用PLL计算新的速度估计和更新估计的电角度 */
int16_t STO_PLL_CalcElAngle(STO_PLL_Handle_t *pHandle, Observer_Inputs_t *pInputs);

/* Computes the rotor average mechanical speed in the unit defined by #SPEED_UNIT and returns it in pMecSpeedUnit. */
/* 计算以#SPEED_UNIT定义的机械转子平均速度，并将其以pMecSpeedUnit形式返回 */
bool STO_PLL_CalcAvrgMecSpeedUnit(STO_PLL_Handle_t *pHandle, int16_t *pMecSpeedUnit);

/* It resets integral term of PLL during on-the-fly startup *//* 在动态启动期间重置PLL的积分项 */
void STO_OTF_ResetPLL(STO_Handle_t *pHandle);

/* It resets integral term of PLL*//* 重置PLL的积分项 */
void STO_ResetPLL(STO_PLL_Handle_t *pHandle);

/* It checks whether the state observer algorithm converged.*//* 检查状态观测器算法是否收敛 */
bool STO_PLL_IsObserverConverged(STO_PLL_Handle_t *pHandle, int16_t *phForcedMecSpeedUnit);

/* It computes the estimated average electrical speed ElSpeedDpp expressed in dpp *//* 计算以dpp表示的估计平均电速度ElSpeedDpp */
void STO_PLL_CalcAvrgElSpeedDpp(STO_PLL_Handle_t *pHandle);

/* It exports estimated Bemf alpha-beta in alphabeta_t format *//* 导出以alphabeta_t格式表示的估计Bemf alpha-beta */
alphabeta_t STO_PLL_GetEstimatedBemf(STO_PLL_Handle_t *pHandle);

/* It exports the stator current alpha-beta as estimated by state  observer *//* 导出由状态观测器估计的定子电流alpha-beta */
alphabeta_t STO_PLL_GetEstimatedCurrent(STO_PLL_Handle_t *pHandle);

/* It set new values for observer gains*//* 设置观测器增益的新值 */
void STO_PLL_SetObserverGains(STO_PLL_Handle_t *pHandle, int16_t hhC1, int16_t hhC2);

/* It exports current observer gains through parameters pC2 and pC4 *//* 通过参数pC2和pC4导出当前观测器增益 */
void STO_PLL_GetObserverGains(STO_PLL_Handle_t *pHandle, int16_t *phC2, int16_t *phC4);

/* It exports current PLL gains through parameters pPgain and pIgain *//* 通过参数pPgain和pIgain导出当前PLL增益 */
void STO_GetPLLGains(STO_PLL_Handle_t *pHandle, int16_t *pPgain, int16_t *pIgain);

/* It set new values for PLL gains *//* 设置PLL增益的新值 */
void STO_SetPLLGains(STO_PLL_Handle_t *pHandle, int16_t hPgain, int16_t hIgain);

void STO_PLL_SetMecAngle(STO_PLL_Handle_t *pHandle, int16_t hMecAngle);

void STO_SetPLL(STO_PLL_Handle_t *pHandle, int16_t hElSpeedDpp, int16_t hElAngle);

/* It exports estimated Bemf squared level*//* 导出估计的Bemf平方电平 */
int32_t STO_PLL_GetEstimatedBemfLevel(STO_PLL_Handle_t *pHandle);

/* It exports observed Bemf squared level*//* 导出观测到的Bemf平方电平 */
int32_t STO_PLL_GetObservedBemfLevel(STO_PLL_Handle_t *pHandle);

/* It enables / disables the Bemf consistency check *//* 导出Bemf一致性 */
void STO_PLL_BemfConsistencyCheckSwitch(STO_PLL_Handle_t *pHandle, bool bSel);

/* It exports Bemf consistency *//* 返回最后方差检查的结果 */
bool STO_PLL_IsBemfConsistent(STO_PLL_Handle_t *pHandle);

/* It returns the result of the last variance check*//* 返回最后方差检查的结果 */
bool STO_PLL_IsVarianceTight(const STO_Handle_t *pHandle);

/* It set internal ForceConvergency1 to true*//* 将内部ForceConvergency1设置为true */
void STO_PLL_ForceConvergency1(STO_Handle_t *pHandle);

/* It set internal ForceConvergency2 to true*//* 将内部ForceConvergency2设置为true */
void STO_PLL_ForceConvergency2(STO_Handle_t *pHandle);

/* It set absolute value of minimum mechanical speed required to validate the start-up. *//* 设置启动时要求的最小机械转速的绝对值 */
void STO_SetMinStartUpValidSpeedUnit(STO_PLL_Handle_t *pHandle, uint16_t hMinStartUpValidSpeed);

/* @brief  forces the rotation direction *//* 设置启动时要求的最小机械转速的绝对值 */
__weak void STO_SetDirection(STO_PLL_Handle_t *pHandle, int8_t direction);


#define CORDIC              ((CORDIC_TypeDef *) CORDIC_BASE)


static inline int16_t fanzhengqie(int32_t wBemf_alfa_est, int32_t wBemf_beta_est);//反正切

//-------------------------------------------------滑膜
typedef struct{
	SpeednPosFdbk_Handle_t _Super;
	int16_t theta;
	int16_t speed[64];
	int16_t ia_est;
	int16_t ib_est;
	int16_t bma_est;
	int16_t bmb_est;
	 PID_Handle_t PIRegulator; 
}SMO_Handle;

int16_t update_E(SMO_Handle* Phandle, Observer_Inputs_t *pInputs);

inline static int16_t SMO_ExecutePLL(SMO_Handle *pHandle, int16_t hBemf_alfa_est, int16_t hBemf_beta_est);
/**
  * @}
  */

/**
  * @}
  */

/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*STO_PLL_SPEEDNPOSFDBK_H*/

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
