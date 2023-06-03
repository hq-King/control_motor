/**
  ******************************************************************************
  * @file    speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the  features
  *          of the Speed & Position Feedback component of the Motor Control SDK.
  *
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
  */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup SpeednPosFdbk Speed & Position Feedback
  *
  * @brief Speed & Position Feedback components of the Motor Control SDK
  *
  * These components provide the speed and the angular position of the rotor of a motor (both
  * electrical and mechanical).
  *
  * Several implementations of the Speed and Position Feedback feature are provided by the Motor
  * to account for the specificities of the motor used on the application:
  *
  * - @ref hall_speed_pos_fdbk "Hall Speed & Position Feedback" for motors with Hall effect sensors
  * - @ref Encoder  "Encoder Speed & Position Feedback" for motors with a quadrature encoder
  * - two general purpose sensorless implementations are provided:
  *   @ref SpeednPosFdbk_STO "State Observer with PLL" and//PLL观测器
  *   @ref STO_CORDIC_SpeednPosFdbk "State Observer with CORDIC"//滑膜观测器
  * - "High Frequency Injection" for anisotropic I-PMSM motors (Not included in this release).
  *
  * @{
  */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  It returns the last computed rotor electrical angle, expressed in
  *         s16degrees. 1 s16degree = 360ï¿½/65536
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor electrical angle (s16degrees)
  */
__weak int16_t SPD_GetElAngle(const SpeednPosFdbk_Handle_t *pHandle)//得到电角度
{
#ifdef NULL_PTR_SPD_POS_FBK
  return ((MC_NULL == pHandle) ? 0 : pHandle->hElAngle);
#else
  return (pHandle->hElAngle);//返回电角度
#endif
}

/**
  * @brief  It returns the last computed rotor mechanical angle, expressed in
  *         s16degrees. Mechanical angle frame is based on parameter bElToMecRatio
  *         and, if occasionally provided - through function SPD_SetMecAngle -
  *         of a measured mechanical angle, on information computed thereof.
  * @note   both Hall sensor and Sensor-less do not implement either
  *         mechanical angle computation or acceleration computation.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor mechanical angle (s16degrees)
  */
__weak int32_t SPD_GetMecAngle(const SpeednPosFdbk_Handle_t *pHandle)
{
#ifdef NULL_PTR_SPD_POS_FBK
  return ((MC_NULL == pHandle) ? 0 : pHandle->wMecAngle);
#else
  return (pHandle->wMecAngle);//得到机械角度
#endif
}

/**
  * @brief  Returns the last computed average mechanical speed, expressed in
  *         the unit defined by #SPEED_UNIT.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  */
__weak int16_t SPD_GetAvrgMecSpeedUnit(const SpeednPosFdbk_Handle_t *pHandle)
{
#ifdef NULL_PTR_SPD_POS_FBK
  return ((MC_NULL == pHandle) ? 0 : pHandle->hAvrMecSpeedUnit);
#else
  return (pHandle->hAvrMecSpeedUnit);//平均机械角度
#endif
}

/**
  * @brief  It returns the last computed electrical speed, expressed in Dpp.
  *         1 Dpp = 1 s16Degree/control Period. The control period is the period
  *         on which the rotor electrical angle is computed (through function
  *         SPD_CalcElectricalAngle).它返回最后计算的电速度，以Dpp表示。
						1 Dpp = 1 s16Degree/控制周期。控制周期是指 计算转子电角度的周期（通过函数 SPD_CalcElectricalAngle）。
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor electrical speed (Dpp)
  */
__weak int16_t SPD_GetElSpeedDpp(const SpeednPosFdbk_Handle_t *pHandle)
{
#ifdef NULL_PTR_SPD_POS_FBK
  return ((MC_NULL == pHandle) ? 0 : pHandle->hElSpeedDpp);
#else
  return (pHandle->hElSpeedDpp);
#endif
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  It returns the last instantaneous computed electrical speed, expressed in Dpp.
  *         1 Dpp = 1 s16Degree/control Period. The control period is the period
  *         on which the rotor electrical angle is computed (through function
  *         SPD_CalcElectricalAngle).它返回最后的瞬时计算的电速度，以Dpp表示。
1 Dpp = 1 s16Degree/控制周期。控制周期是指计算转子电角度的周期（通过函数 SPD_CalcElectricalAngle）。
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t rotor instantaneous electrical speed (Dpp)
  */
__weak int16_t SPD_GetInstElSpeedDpp(const SpeednPosFdbk_Handle_t *pHandle)
{
#ifdef NULL_PTR_SPD_POS_FBK
  return ((MC_NULL == pHandle) ? 0 : pHandle->InstantaneousElSpeedDpp);
#else
  return (pHandle->InstantaneousElSpeedDpp);
#endif
}

/**
  * @brief  It returns the result of the last reliability check performed.
  *         Reliability is measured with reference to parameters
  *         hMaxReliableElSpeedUnit, hMinReliableElSpeedUnit,
  *         bMaximumSpeedErrorsNumber and/or specific parameters of the derived
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
 它返回最后一次进行的可靠性检查的结果。
  * 可靠性是参照参数来衡量的
  * hMaxReliableElSpeedUnit, hMinReliableElSpeedUnit、
  * bMaximumSpeedErrorsNumber和/或派生的具体参数。
  * true = 传感器信息是可靠的
  * false = 传感器信息不可靠
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval bool sensor reliability state
该函数用于返回上次可靠性检查的结果。可靠性是根据参数hMaxReliableElSpeedUnit、hMinReliableElSpeedUnit、bMaximumSpeedErrorsNumber和/或派生的特定参数来衡量的。

返回值：

bool类型的值，表示传感器的可靠性状态。
true：传感器信息可靠。
false：传感器信息不可靠。
参数说明：

pHandle：当前SpeednPosFdbk组件实例的句柄。
注意：当bSpeedErrorNumber等于bMaximumSpeedErrorsNumber时，表示传感器信息不可靠。
  */
__weak bool SPD_Check(const SpeednPosFdbk_Handle_t *pHandle)
{
  bool SpeedSensorReliability = true;//先认为传感信息是可靠的
#ifdef NULL_PTR_SPD_POS_FBK
  if ((MC_NULL == pHandle) || (pHandle->bSpeedErrorNumber == pHandle->bMaximumSpeedErrorsNumber))
#else
  if (pHandle->bSpeedErrorNumber == pHandle->bMaximumSpeedErrorsNumber)//如果传感器的错误信息数量==最大可控数量
#endif
  {
    SpeedSensorReliability = false;//传感器信息不可靠，改为false
  }
  return (SpeedSensorReliability);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and returns - through
  *         parameter pMecSpeedUnit - the rotor average mechanical speed,
  *         expressed in the unit defined by #SPEED_UNIT. It computes and returns
  *         the reliability state of the sensor; reliability is measured with
  *         reference to parameters hMaxReliableElSpeedUnit, hMinReliableElSpeedUnit,
  *         bMaximumSpeedErrorsNumber and/or specific parameters of the derived
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
该函数需要以与速度控制执行的周期相同的周期性调用。
它计算并通过参数pMecSpeedUnit返回转子的平均机械速度，
以#SPEED_UNIT定义的单位表示。它还计算并返回传感器的可靠性状态；
可靠性是根据参数hMaxReliableElSpeedUnit、hMinReliableElSpeedUnit、bMaximumSpeedErrorsNumber和/或派生的特定参数来衡量的。

返回值：

bool类型的值，表示传感器的可靠性状态。
true：传感器信息可靠。
false：传感器信息不可靠。
参数说明：

pHandle：当前SpeednPosFdbk组件实例的句柄。
pMecSpeedUnit：指向int16_t的指针，用于返回转子的平均机械速度（以#SPEED_UNIT定义的单位表示）。
注意：如果转子的平均机械速度超过hMaxReliableMecSpeedUnit或低于hMinReliableMecSpeedUnit，
或者机械加速度超过hMaxReliableMecAccelUnitP，则会发生速度错误。如果连续发生bMaximumSpeedErrorsNumber次速度错误，表示传感器信息不可靠。
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @param  pMecSpeedUnit pointer to int16_t, used to return the rotor average
  *         mechanical speed (expressed in the unit defined by #SPEED_UNIT)
  * @retval none
  */
__weak bool SPD_IsMecSpeedReliable(SpeednPosFdbk_Handle_t *pHandle, const int16_t *pMecSpeedUnit)
{
  bool SpeedSensorReliability = true;//传感器信息先认为是可靠的
#ifdef NULL_PTR_SPD_POS_FBK
  if ((MC_NULL == pHandle) || (MC_NULL == pMecSpeedUnit))
  {
    SpeedSensorReliability = false;
  }
  else
  {
#endif
	uint16_t hAbsMecSpeedUnit;//定义机械速度绝对值
	uint16_t hAbsMecAccelUnitP;//机械加速度绝对值
    int16_t hAux;
    uint8_t bSpeedErrorNumber;//速度出错数量
    uint8_t bMaximumSpeedErrorsNumber = pHandle->bMaximumSpeedErrorsNumber;//最大犯错数量
	bool SpeedError = false;//定义为不可靠

    bSpeedErrorNumber = pHandle->bSpeedErrorNumber;

	/* Compute absoulte value of mechanical speed *///计算机械速度的绝对值
    if (*pMecSpeedUnit < 0)//小于0
    {
      hAux = -(*pMecSpeedUnit);//化为正值，hAux为临时变量
      hAbsMecSpeedUnit = (uint16_t)hAux;//把速度化为unit16_t类型
    }
    else//速度大于0
    {
      hAbsMecSpeedUnit = (uint16_t)(*pMecSpeedUnit);//直接把变量化为对应的类型
    }

    if (hAbsMecSpeedUnit > pHandle->hMaxReliableMecSpeedUnit)//当得到的速度大于了预定义的阈值，将其设置为错误状态
    {
      SpeedError = true;
    }

    if (hAbsMecSpeedUnit < pHandle->hMinReliableMecSpeedUnit)//小于预定义的阈值则没有问题
    {
      SpeedError = true;
    }

    /* Compute absoulte value of mechanical acceleration *///计算机械加速度的绝对值
    if (pHandle->hMecAccelUnitP < 0)
    {
      hAux = -(pHandle->hMecAccelUnitP);
      hAbsMecAccelUnitP = (uint16_t)hAux;
    }
    else
    {
      hAbsMecAccelUnitP = (uint16_t)pHandle->hMecAccelUnitP;
    }

    if (hAbsMecAccelUnitP > pHandle->hMaxReliableMecAccelUnitP)
    {
      SpeedError = true;
    }

    if (true == SpeedError)//发生错误
    {
      if (bSpeedErrorNumber < bMaximumSpeedErrorsNumber)
      {
        bSpeedErrorNumber++;//错误数量增加
      }
    }
    else
    {
      if (bSpeedErrorNumber < bMaximumSpeedErrorsNumber)
      {
        bSpeedErrorNumber = 0u;//将速度错误计数器初始化为 0
      }
    }

    if (bSpeedErrorNumber == bMaximumSpeedErrorsNumber)//错误次数达到上限，返回传感器信号不可靠
    {
      SpeedSensorReliability = false;
    }

    pHandle->bSpeedErrorNumber = bSpeedErrorNumber;
#ifdef NULL_PTR_SPD_POS_FBK
  }
#endif
  return (SpeedSensorReliability);//返回得到的结果
}

/**
  * @brief  This method returns the average mechanical rotor speed expressed in
  *         "S16Speed". It means that:\n
  *         - it is zero for zero speed,\n
  *         - it become INT16_MAX when the average mechanical speed is equal to
  *           hMaxReliableMecSpeedUnit,\n
  *         - it becomes -INT16_MAX when the average mechanical speed is equal to
  *         -hMaxReliableMecSpeedUnit.
当平均机械转速为零时，返回值为零。
当平均机械转速等于 hMaxReliableMecSpeedUnit 时，返回值变为 INT16_MAX。
当平均机械转速等于 -hMaxReliableMecSpeedUnit 时，返回值变为 -INT16_MAX。
用于将平均机械转速以一种统一的标准格式返回，便于在不同的应用中进行比较和处理。
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t The average mechanical rotor speed expressed in "S16Speed".
  */
__weak int16_t SPD_GetS16Speed(const SpeednPosFdbk_Handle_t *pHandle)//
{
  int16_t tempValue;
#ifdef NULL_PTR_SPD_POS_FBK
  if (MC_NULL == pHandle)
  {
    tempValue = 0;
  }
  else
  {
#endif
	int32_t wAux = (int32_t)pHandle->hAvrMecSpeedUnit;//强制转换为int32类型
    wAux *= INT16_MAX;
    wAux /= (int16_t)pHandle->hMaxReliableMecSpeedUnit;
    tempValue = (int16_t)wAux;
	/*这段代码的作用是将平均机械转速 pHandle->hAvrMecSpeedUnit 转换为一个标准化的值 tempValue。

首先，将 pHandle->hAvrMecSpeedUnit 强制转换为 int32_t 类型，并将结果赋给变量 wAux。

接下来，将 wAux 乘以 INT16_MAX，然后除以 pHandle->hMaxReliableMecSpeedUnit 强制转换为 int16_t 类型的值。这个操作将 wAux 缩放到一个与最大可靠机械转速相对应的范围内。

最后，将缩放后的结果强制转换为 int16_t 类型，并将其赋值给 tempValue。

这段代码的目的是将平均机械转速的范围映射到一个标准化的范围内，以便于在后续的计算或比较中使用。

32767为INT16_MAX的值，为一个宏定义



*/
#ifdef NULL_PTR_SPD_POS_FBK
  }
#endif
  return (tempValue);
}

/**
  * @brief  This method returns the coefficient used to transform electrical to
  *         mechanical quantities and viceversa. It usually coincides with motor
  *         pole pairs number.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval uint8_t The motor pole pairs number.
  */
__weak uint8_t SPD_GetElToMecRatio(const SpeednPosFdbk_Handle_t *pHandle)
{
#ifdef NULL_PTR_SPD_POS_FBK
  return ((MC_NULL == pHandle) ? 0U : pHandle->bElToMecRatio);
#else
  return (pHandle->bElToMecRatio);
#endif
}

/**
  * @brief  This method sets the coefficient used to transform electrical to
  *         mechanical quantities and viceversa. It usually coincides with motor
  *         pole pairs number.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @param  bPP The motor pole pairs number to be set.
  */
__weak void SPD_SetElToMecRatio(SpeednPosFdbk_Handle_t *pHandle, uint8_t bPP)
{
#ifdef NULL_PTR_SPD_POS_FBK
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->bElToMecRatio = bPP;
#ifdef NULL_PTR_SPD_POS_FBK
  }
#endif
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
