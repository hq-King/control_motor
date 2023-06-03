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
  SpeednPosFdbk_Handle_t _Super;// �̳���SpeednPosFdbk_Handle_t�ĳ�Ա����

  int16_t  hC1;                 /*!< Variable containing state observer constant
                                     C1 to speed-up computations */ //< ״̬�۲�������C1�����ڼӿ�����ٶ� 
  int16_t  hC2;                /*!< Variable containing state observer constant
																/!< ״̬�۲�������C2�����Լ���ΪF1 * K1 / ״̬�۲���ִ������[Hz]������K1�������۲�������֮һ /
                                     C2, it can be computed as F1 * K1/ State
                                     observer execution rate [Hz] being K1 one
                                     of the two observer gains   */
  int16_t  hC3; //!< ״̬�۲�������C3                /*!< Variable containing state observer constant
                                     
  int16_t  hC4;                  /*!< State Observer constant C4, it can
	//< ״̬�۲�������C4�����Լ���ΪK2 * ���ɲ���������A��/
	//�����Ӧ���ٶ�[#SPEED_UNIT] * ���BEMF����[Vllrms/krpm] * sqrt(2) * F2 * ״̬�۲���ִ������[Hz]��������K2�������۲�������֮һ /
                                      be computed as K2 * max measurable
                                      current (A) / (Max application speed
                                      [#SPEED_UNIT] * Motor B-emf constant
                                      [Vllrms/krpm] * sqrt(2) * F2 * State
                                      observer execution rate [Hz]) being
                                       K2 one of the two observer gains  */
  int16_t  hC5;                 /*!< Variable containing state observer constant
	/!< ״̬�۲�������C5 /
                                     C5 */
  int16_t  hC6;                 /*!< State observer constant C6, computed with a
                                   /!< ״̬�۲�������C6��ͨ���ض����̴�������������ó� /  specific procedure starting from the other
                                    constants */
  int16_t  hF1;                 /*!< Variable containing state observer scaling
                                    /!< ״̬�۲�����������F1 / factor F1 */
  int16_t  hF2;                 /*!< Variable containing state observer scaling factor F2/!< ״̬�۲�����������F2 /*/
  int16_t  hF3;                  /*!< State observer scaling factor F3/!< ״̬�۲�����������F3 / */
  uint16_t F3POW2;              /*!< State observer scaling factor F3 /!< ״̬�۲�����������F3��2���ݴη���ʾ�����磬����������Ϊ512�����ֵ����Ϊ9����Ϊ2^9 = 512 /expressed as power of 2.
                                     E.g. if gain divisor is 512 the value
                                     must be 9 because 2^9 = 512 */
  PID_Handle_t PIRegulator;     /*!< PI regulator component handle, used for
                                     PLL implementation  /!< PI������������������PLLʵ�� /*/
  int32_t Ialfa_est;           /*!< Estimated Ialfa current in int32 format /!< ��int32��ʽ���Ƶ�Ialfa���� /*/
  int32_t Ibeta_est;           /*!< Estimated Ibeta current in int32 format /!< ��int32��ʽ���Ƶ�Ibeta���� /*/
  int32_t wBemf_alfa_est;       /*!< Estimated B-emf alfa in int32_t format /!< ��int32_t��ʽ���Ƶ�BEMF�����綯�ƣ� alfa /*/
  int32_t wBemf_beta_est;       /*!< Estimated B-emf beta in int32_t format /!< ��int32_t��ʽ���Ƶ�BEMF beta /*/
  int16_t hBemf_alfa_est;       /*!< Estimated B-emf alfa in int16_t format /!< ��int16_t��ʽ���Ƶ�BEMF alfa /*/
  int16_t hBemf_beta_est;       /*!< Estimated B-emf beta in int16_t format /!< ��int16_t��ʽ���Ƶ�BEMF beta /*/
  int16_t Speed_Buffer[64];    /*!< Estimated DPP speed FIFO, it contains latest
                                     SpeedBufferSizeDpp speed measurements/!< ���Ƶ�DPP�ٶ�FIFO���������µ�SpeedBufferSizeDpp�ٶȲ���ֵ /*/
  uint8_t Speed_Buffer_Index;  /*!< Position of latest speed estimation in
                                     estimated speed FIFO /!< �����ٶ�FIFO�������ٶȹ��Ƶ�λ�� /*/
  bool IsSpeedReliable;        /*!< Latest private speed reliability information,
                                     updated by SPD_CalcAvrgMecSpeedUnit, it is
                                     true if the speed measurement variance is
																		 /!< ���µ�˽���ٶȿɿ�����Ϣ����SPD_CalcAvrgMecSpeedUnit���£�����ٶȲ�������С�ڵ���hVariancePercentage��Ӧ����ֵ����Ϊtrue /
                                     lower then threshold corresponding to
                                     hVariancePercentage */
  uint8_t ConsistencyCounter;  /*!< Counter of passed tests for start-up
                                     validation /!< ������֤��ͨ�����������Դ��������� /*/
  uint8_t ReliabilityCounter; /*!< Counter for reliability check /!< �ɿ��Լ������� /*/
  bool IsAlgorithmConverged;   /*!/!< �����۲���������Ϣ�Ĳ������� /< Boolean variable containing observer
                                     convergence information */
  bool IsBemfConsistent;       /*!< Sensor reliability information, updated by
                                     SPD_CalcAvrgMecSpeedUnit, it is true if the
                                     observed back-emfs are consistent with
                                     expectation/!< �������ɿ�����Ϣ����SPD_CalcAvrgMecSpeedUnit���£�����۲⵽�ķ��綯��������ֵһ�£���Ϊtrue */

  int32_t Obs_Bemf_Level;      /*!< Observed back-emf Level< �۲⵽�ķ��綯�Ƶ�ƽ*/
  int32_t Est_Bemf_Level;      /*!< Estimated back-emf Level���Ƶķ��綯�Ƶ�ƽ*/
  bool EnableDualCheck;        /*!< Consistency check enabler/!< һ���Լ��ʹ���� /*/
  int32_t DppBufferSum;        /*!< summation of speed buffer elements [dpp]/!< �ٶȻ�����Ԫ��֮��[dpp] /*/
  int16_t SpeedBufferOldestEl; /*!< Oldest element of the speed buffer/!< �ٶȻ���������ɵ�Ԫ�� */

  uint8_t SpeedBufferSizeUnit;       /*!< Depth of FIFO used to average
                                           estimated speed exported by
                                           SPD_GetAvrgMecSpeedUnit. It
                                           must be an integer number between 1
                                           and 64 < ����ƽ��SPD_GetAvrgMecSpeedUnit�����Ĺ����ٶȵ�FIFO����ȡ���������1��64֮������� */
  uint8_t SpeedBufferSizeDpp;       /*!< Depth of FIFO used for both averaging
                                           estimated speed exported by
                                           SPD_GetElSpeedDpp and state
                                           observer equations. It must be an
                                           integer number between 1 and
                                           bSpeedBufferSizeUnit < ����ƽ��SPD_GetElSpeedDpp��״̬�۲������̹����ٶȵ�FIFO����ȡ���������1��bSpeedBufferSizeUnit֮�������*/
  uint16_t VariancePercentage;        /*!< Parameter expressing the maximum
                                           allowed variance of speed estimation
                                           < ��ʾ�ٶȹ��Ƶ����������Ĳ��� */
  uint8_t SpeedValidationBand_H;   /*!< It expresses how much estimated speed
                                           can exceed forced stator electrical
                                           frequency during start-up without
                                           being considered wrong. The
                                           measurement unit is 1/16 of forced
                                           speed < �������ڼ䣬�����ٶȿ��Գ���ǿ�ƶ��ӵ���Ƶ�ʶ��ٶ�������Ϊ���󡣲�����λΪǿ���ٶȵ�1/16*/
  uint8_t SpeedValidationBand_L;   /*!< It expresses how much estimated speed
                                           can be below forced stator electrical
                                           frequency during start-up without
                                           being considered wrong. The
                                           measurement unit is 1/16 of forced
                                           speed < �������ڼ䣬�����ٶȿ��Ե���ǿ�ƶ��ӵ���Ƶ�ʶ��ٶ�������Ϊ���󡣲�����λΪǿ���ٶȵ�1/16 */
  uint16_t MinStartUpValidSpeed;     /*!< Absolute value of minimum mechanical
                                            speed  required to validate the start-up.
                                            Expressed in the unit defined by #SPEED_UNIT. < ��֤�����������С��е�ٶȵľ���ֵ����#SPEED_UNIT����ĵ�λ��ʾ */
  uint8_t StartUpConsistThreshold;   /*!< Number of consecutive tests on speed
                                           consistency to be passed before
                                           validating the start-up /!< ����֤����֮ǰͨ�����ٶ�һ���Բ��Դ��� */
  uint8_t Reliability_hysteresys;    /*!< Number of reliability failed
                                           consecutive tests before a speed
                                           check fault is returned to _Super.bSpeedErrorNumber�ɿ���ʧ���������Դ��������ٶȼ����Ϸ��ص�_Super.bSpeedErrorNumber֮ǰ
                                           */
  uint8_t BemfConsistencyCheck;      /*!< Degree of consistency of the observed
                                           back-emfs, it must be an integer
                                           number ranging from 1 (very high
                                           consistency) down to 64 (very poor
																					 �۲⵽�ķ��綯�Ƶ�һ���Գ̶ȣ������Ǵ�1���ǳ��ߵ�һ���ԣ���64���ǳ����һ���ԣ�������
                                           consistency) */
  uint8_t BemfConsistencyGain;       /*!< Gain to be applied when checking
                                           back-emfs consistency; default value
                                           is 64 (neutral), max value 105
                                           (x1.64 amplification), min value 1
																					 ��鷴�綯��һ����ʱӦ�õ����棻Ĭ��ֵΪ64�����ԣ������ֵΪ105��x1.64�Ŵ󣩣���СֵΪ1��/64˥��
                                           (/64 attenuation) */
  uint16_t MaxAppPositiveMecSpeedUnit; /*!< Maximum positive value of rotor speed. Expressed in
                                             the unit defined by #SPEED_UNIT. It can be
                                             x1.1 greater than max application speed*/
  uint16_t F1LOG;                    /*!< F1 gain divisor expressed as power of 2.
                                            E.g. if gain divisor is 512 the value
																						uint16_t F1LOG; /!< F1���������2���ݴη���ʾ�����磬����������Ϊ512�����ֵ����Ϊ9����Ϊ2^9 = 512 /
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

/* It initializes the state observer object *//* ��ʼ��״̬�۲������� */
void STO_PLL_Init(STO_PLL_Handle_t *pHandle);
__weak int16_t Medin_Average_filter(int16_t Size,int16_t Speed_Buffer[]);//��ֵƽ��ֵ�˲�

/* It only returns, necessary to implement fictitious IRQ_Handler *//* ֻ���أ�ʵ���鹹��IRQ_Handler������ */
void STO_PLL_Return(STO_PLL_Handle_t *pHandle, uint8_t flag);

/* It clears state observer object by re-initializing private variables*//* ͨ�����³�ʼ��˽�б��������״̬�۲������� */
void STO_PLL_Clear(STO_PLL_Handle_t *pHandle);

/* It executes Luenberger state observer and calls PLL to compute a new speed
*  estimation and update the estimated electrical angle.
*//* ִ��Luenberger״̬�۲���������PLL�����µ��ٶȹ��ƺ͸��¹��Ƶĵ�Ƕ� */
int16_t STO_PLL_CalcElAngle(STO_PLL_Handle_t *pHandle, Observer_Inputs_t *pInputs);

/* Computes the rotor average mechanical speed in the unit defined by #SPEED_UNIT and returns it in pMecSpeedUnit. */
/* ������#SPEED_UNIT����Ļ�еת��ƽ���ٶȣ���������pMecSpeedUnit��ʽ���� */
bool STO_PLL_CalcAvrgMecSpeedUnit(STO_PLL_Handle_t *pHandle, int16_t *pMecSpeedUnit);

/* It resets integral term of PLL during on-the-fly startup *//* �ڶ�̬�����ڼ�����PLL�Ļ����� */
void STO_OTF_ResetPLL(STO_Handle_t *pHandle);

/* It resets integral term of PLL*//* ����PLL�Ļ����� */
void STO_ResetPLL(STO_PLL_Handle_t *pHandle);

/* It checks whether the state observer algorithm converged.*//* ���״̬�۲����㷨�Ƿ����� */
bool STO_PLL_IsObserverConverged(STO_PLL_Handle_t *pHandle, int16_t *phForcedMecSpeedUnit);

/* It computes the estimated average electrical speed ElSpeedDpp expressed in dpp *//* ������dpp��ʾ�Ĺ���ƽ�����ٶ�ElSpeedDpp */
void STO_PLL_CalcAvrgElSpeedDpp(STO_PLL_Handle_t *pHandle);

/* It exports estimated Bemf alpha-beta in alphabeta_t format *//* ������alphabeta_t��ʽ��ʾ�Ĺ���Bemf alpha-beta */
alphabeta_t STO_PLL_GetEstimatedBemf(STO_PLL_Handle_t *pHandle);

/* It exports the stator current alpha-beta as estimated by state  observer *//* ������״̬�۲������ƵĶ��ӵ���alpha-beta */
alphabeta_t STO_PLL_GetEstimatedCurrent(STO_PLL_Handle_t *pHandle);

/* It set new values for observer gains*//* ���ù۲����������ֵ */
void STO_PLL_SetObserverGains(STO_PLL_Handle_t *pHandle, int16_t hhC1, int16_t hhC2);

/* It exports current observer gains through parameters pC2 and pC4 *//* ͨ������pC2��pC4������ǰ�۲������� */
void STO_PLL_GetObserverGains(STO_PLL_Handle_t *pHandle, int16_t *phC2, int16_t *phC4);

/* It exports current PLL gains through parameters pPgain and pIgain *//* ͨ������pPgain��pIgain������ǰPLL���� */
void STO_GetPLLGains(STO_PLL_Handle_t *pHandle, int16_t *pPgain, int16_t *pIgain);

/* It set new values for PLL gains *//* ����PLL�������ֵ */
void STO_SetPLLGains(STO_PLL_Handle_t *pHandle, int16_t hPgain, int16_t hIgain);

void STO_PLL_SetMecAngle(STO_PLL_Handle_t *pHandle, int16_t hMecAngle);

void STO_SetPLL(STO_PLL_Handle_t *pHandle, int16_t hElSpeedDpp, int16_t hElAngle);

/* It exports estimated Bemf squared level*//* �������Ƶ�Bemfƽ����ƽ */
int32_t STO_PLL_GetEstimatedBemfLevel(STO_PLL_Handle_t *pHandle);

/* It exports observed Bemf squared level*//* �����۲⵽��Bemfƽ����ƽ */
int32_t STO_PLL_GetObservedBemfLevel(STO_PLL_Handle_t *pHandle);

/* It enables / disables the Bemf consistency check *//* ����Bemfһ���� */
void STO_PLL_BemfConsistencyCheckSwitch(STO_PLL_Handle_t *pHandle, bool bSel);

/* It exports Bemf consistency *//* ������󷽲���Ľ�� */
bool STO_PLL_IsBemfConsistent(STO_PLL_Handle_t *pHandle);

/* It returns the result of the last variance check*//* ������󷽲���Ľ�� */
bool STO_PLL_IsVarianceTight(const STO_Handle_t *pHandle);

/* It set internal ForceConvergency1 to true*//* ���ڲ�ForceConvergency1����Ϊtrue */
void STO_PLL_ForceConvergency1(STO_Handle_t *pHandle);

/* It set internal ForceConvergency2 to true*//* ���ڲ�ForceConvergency2����Ϊtrue */
void STO_PLL_ForceConvergency2(STO_Handle_t *pHandle);

/* It set absolute value of minimum mechanical speed required to validate the start-up. *//* ��������ʱҪ�����С��еת�ٵľ���ֵ */
void STO_SetMinStartUpValidSpeedUnit(STO_PLL_Handle_t *pHandle, uint16_t hMinStartUpValidSpeed);

/* @brief  forces the rotation direction *//* ��������ʱҪ�����С��еת�ٵľ���ֵ */
__weak void STO_SetDirection(STO_PLL_Handle_t *pHandle, int8_t direction);


#define CORDIC              ((CORDIC_TypeDef *) CORDIC_BASE)


static inline int16_t fanzhengqie(int32_t wBemf_alfa_est, int32_t wBemf_beta_est);//������

//-------------------------------------------------��Ĥ
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
