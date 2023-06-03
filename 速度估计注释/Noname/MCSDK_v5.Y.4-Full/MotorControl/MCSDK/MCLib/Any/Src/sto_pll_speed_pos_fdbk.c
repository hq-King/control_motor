/**
  ******************************************************************************
  * @file    sto_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the State Observer + PLL Speed & Position Feedback component of the
  *          Motor Control SDK.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sto_pll_speed_pos_fdbk.h"
#include "mc_math.h"
#include "math.h"




/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @defgroup SpeednPosFdbk_STO State Observer Speed & Position Feedback
  * @brief State Observer with PLL Speed & Position Feedback implementation
  *
  * This component uses a State Observer coupled with a software PLL to provide an estimation of
  * the speed and the position of the rotor of the motor.
  *
  * @todo Document the State Observer + PLL Speed & Position Feedback "module".
  * @{
  */

/* Private defines -----------------------------------------------------------*/
#define C6_COMP_CONST1  ((int32_t)1043038)
#define C6_COMP_CONST2  ((int32_t)10430)

/* Private function prototypes -----------------------------------------------*/
static void STO_Store_Rotor_Speed(STO_PLL_Handle_t *pHandle, int16_t hRotor_Speed);
static int16_t STO_ExecutePLL(STO_PLL_Handle_t *pHandle, int16_t hBemf_alfa_est, int16_t hBemf_beta_est);
static void STO_InitSpeedBuffer(STO_PLL_Handle_t *pHandle);


/**
  * @brief  It initializes the state observer component
  * @param  pHandle: handler of the current instance of the STO component
  * @retval none
  */
__weak void STO_PLL_Init(STO_PLL_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int16_t htempk;
    int32_t wAux;

    pHandle->ConsistencyCounter = pHandle->StartUpConsistThreshold;//// 设置一致性计数器为起始一致性阈值
    pHandle->EnableDualCheck = true;//启用双倍计数

    wAux = ((int32_t)1);// wAux 初始化为 1
    pHandle->F3POW2 = 0U;// F3POW2 初始化为 0

    htempk = (int16_t)(C6_COMP_CONST1 / pHandle->hF2); // 计算 htempk

    while (htempk != 0)// 循环直到 htempk 为 0
    {
      htempk /= ((int16_t)2);
      wAux *= ((int32_t)2);
      pHandle->F3POW2++;
    }

    pHandle->hF3 = (int16_t)wAux;
    wAux = ((int32_t)(pHandle->hF2)) * pHandle->hF3;
    pHandle->hC6 = (int16_t)(wAux / C6_COMP_CONST2);// 计算 hC6

    STO_PLL_Clear(pHandle);//清楚句柄里面的变量

    PID_HandleInit(&pHandle->PIRegulator);//PI控制器初始化

    /* Acceleration measurement set to zero */
    pHandle->_Super.hMecAccelUnitP = 0;//加速度初始化0
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
  return;
}

/**
  * @brief  It only returns, necessary to implement fictitious IRQ_Handler
  * @param  pHandle: handler of the current instance of the STO component
  * @param  uint8_t Fictitious interrupt flag
  * @retval none
  */
//cstat !RED-func-no-effect
__weak void STO_PLL_Return(STO_PLL_Handle_t *pHandle, uint8_t flag)//
	//这段函数可能是为了在某些特定情况下提供一个占位函数，
//以满足代码结构的要求或兼容性需求。它可能被其他部分的代码调用，但在这个给定的代码段中并没有给出具体的调用情况或更多的细节。
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == (void *)pHandle) || ((uint8_t)0 == flag))
  {
    /* Nothing to do */
  }
  else
  {
    /* Nothing to do */
  }
#endif
  return;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  This method executes Luenberger state observer equations and calls
  *         PLL with the purpose of computing a new speed estimation and调用龙伯格观测器来分程并调用PLL来计算新的速度估计值和电角度
  *         updating the estimated electrical angle.
  * @param  pHandle: handler of the current instance of the STO component
  * @param  pInputVars_str pointer to the observer inputs structure
  * @retval int16_t rotor electrical angle (s16Degrees)
  */
__weak int16_t STO_PLL_CalcElAngle(STO_PLL_Handle_t *pHandle, Observer_Inputs_t *pInputs)
{
  int16_t retValue;

  if ((MC_NULL == pHandle) || (MC_NULL == pInputs))
  {
    retValue = 0;
  }
  else
  {
    int32_t wAux;
    int32_t wDirection;
    int32_t wIalfa_est_Next;//定义的alfa电流估计值
    int32_t wIbeta_est_Next;//定义的beta电流估计值
    int32_t wBemf_alfa_est_Next;
    int32_t wBemf_beta_est_Next;
    int16_t hAux;
    int16_t hAux_Alfa;
    int16_t hAux_Beta;
    int16_t hIalfa_err;
    int16_t hIbeta_err;
    int16_t hRotor_Speed;//速度
    int16_t hValfa;
    int16_t hVbeta;
		//int16_t hPrev_Rotor_Angle = pHandle->_Super.hElAngle;
		//int16_t hOrRotor_Speed,hRotor_Angle;

    if (pHandle->wBemf_alfa_est > (((int32_t)pHandle->hF2) * INT16_MAX))//限幅
    {
      pHandle->wBemf_alfa_est = INT16_MAX * ((int32_t)pHandle->hF2);
    }
    else if (pHandle->wBemf_alfa_est <= (-INT16_MAX * ((int32_t)pHandle->hF2)))
    {
      pHandle->wBemf_alfa_est = -INT16_MAX * ((int32_t)pHandle->hF2);
    }
    else
    {
      /* Nothing to do */
    }
		
		//计算估计的反电动势的alfa分量
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hAux_Alfa = (int16_t)(pHandle->wBemf_alfa_est >> pHandle->F2LOG);
#else
      hAux_Alfa = (int16_t)(pHandle->wBemf_alfa_est / pHandle->hF2);
#endif
// 对估计的反电动势的beta分量进行范围限制
    if (pHandle->wBemf_beta_est > (INT16_MAX * ((int32_t)pHandle->hF2)))
    {
      pHandle->wBemf_beta_est = INT16_MAX * ((int32_t)pHandle->hF2);
    }
    else if (pHandle->wBemf_beta_est <= (-INT16_MAX * ((int32_t)pHandle->hF2)))
    {
      pHandle->wBemf_beta_est = (-INT16_MAX * ((int32_t)pHandle->hF2));
    }
    else
    {
      /* Nothing to do */
    }
		// 计算估计的反电动势的beta分量
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    hAux_Beta = (int16_t)(pHandle->wBemf_beta_est >> pHandle->F2LOG);
#else
    hAux_Beta = (int16_t)(pHandle->wBemf_beta_est / pHandle->hF2);
#endif
//对估计的alfa电流分量进行限幅
    if (pHandle->Ialfa_est > (INT16_MAX * ((int32_t)pHandle->hF1)))
    {
      pHandle->Ialfa_est = INT16_MAX * ((int32_t)pHandle->hF1);
    }
    else if (pHandle->Ialfa_est <= (-INT16_MAX * ((int32_t)pHandle->hF1)))
    {
      pHandle->Ialfa_est = -INT16_MAX * ((int32_t)pHandle->hF1);
    }
    else
    {
      /* Nothing to do */
    }
//对估计的beta电流分量进行限幅
    if (pHandle->Ibeta_est > (INT16_MAX * ((int32_t)pHandle->hF1)))
    {
      pHandle->Ibeta_est = INT16_MAX * ((int32_t)pHandle->hF1);
    }
    else if (pHandle->Ibeta_est <= (-INT16_MAX * ((int32_t)pHandle->hF1)))
    {
      pHandle->Ibeta_est = -INT16_MAX * ((int32_t)pHandle->hF1);
    }
    else
    {
      /* Nothing to do */
    }

	
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    hIalfa_err = (int16_t)(pHandle->Ialfa_est >> pHandle->F1LOG);
#else
    hIalfa_err = (int16_t)(pHandle->Ialfa_est / pHandle->hF1);
#endif
	// 计算a轴电流误差
    hIalfa_err = hIalfa_err - pInputs->Ialfa_beta.alpha;
//beta轴电流误差
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    hIbeta_err = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);
#else
    hIbeta_err = (int16_t)(pHandle->Ibeta_est / pHandle->hF1);
#endif

    hIbeta_err = hIbeta_err - pInputs->Ialfa_beta.beta;

    wAux = ((int32_t)pInputs->Vbus) * pInputs->Valfa_beta.alpha;
		//a相电压
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    hValfa = (int16_t)(wAux >> 16); //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    hValfa = (int16_t)(wAux / 65536);
#endif

    wAux = ((int32_t)pInputs->Vbus) * pInputs->Valfa_beta.beta;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    hVbeta = ( int16_t ) ( wAux >> 16 ); //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    hVbeta = (int16_t)(wAux / 65536);
#endif
//a轴观测器
    /*alfa axes observer*/
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    hAux = (int16_t)(pHandle->Ialfa_est >> pHandle->F1LOG);
		// 将Ialfa_est右移pHandle->F1LOG位，并转换为int16_t类型
#else
    hAux = (int16_t)(pHandle->Ialfa_est / pHandle->hF1);
#endif

    wAux = ((int32_t)pHandle->hC1) * hAux;// 将Ialfa_est除以pHandle->hF1，并转换为int16_t类型
    wIalfa_est_Next = pHandle->Ialfa_est - wAux;
// 计算wIalfa_est_Next
    wAux = ((int32_t)pHandle->hC2) * hIalfa_err;
    wIalfa_est_Next += wAux;

    wAux = ((int32_t)pHandle->hC5) * hValfa;
    wIalfa_est_Next += wAux;

    wAux = ((int32_t)pHandle->hC3) * hAux_Alfa;
    wIalfa_est_Next -= wAux;

    wAux = ((int32_t)pHandle->hC4) * hIalfa_err;
    wBemf_alfa_est_Next = pHandle->wBemf_alfa_est + wAux;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    wAux = (int32_t)hAux_Beta >> pHandle->F3POW2; //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    wAux = ((int32_t)hAux_Beta) / pHandle->hF3;
#endif

    wAux = wAux * pHandle->hC6;
    wAux = pHandle->_Super.hElSpeedDpp * wAux;
    wBemf_alfa_est_Next += wAux;

    /*beta axes observer*/
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    hAux = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);
#else
    hAux = (int16_t)(pHandle->Ibeta_est / pHandle->hF1);
#endif

    wAux = ((int32_t)pHandle->hC1) * hAux;
    wIbeta_est_Next = pHandle->Ibeta_est - wAux;

    wAux = ((int32_t)pHandle->hC2) * hIbeta_err;
    wIbeta_est_Next += wAux;

    wAux = ((int32_t)pHandle->hC5) * hVbeta;
    wIbeta_est_Next += wAux;

    wAux = ((int32_t)pHandle->hC3) * hAux_Beta;
    wIbeta_est_Next -= wAux;

    wAux = ((int32_t)pHandle->hC4) * hIbeta_err;
    wBemf_beta_est_Next = pHandle->wBemf_beta_est + wAux;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    wAux = (int32_t)hAux_Alfa >> pHandle->F3POW2; //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    wAux = ((int32_t)hAux_Alfa) / pHandle->hF3;
#endif

    wAux = wAux * pHandle->hC6;
    wAux = pHandle->_Super.hElSpeedDpp * wAux;
    wBemf_beta_est_Next -= wAux;

    /*Calls the PLL blockset*/
    pHandle->hBemf_alfa_est = hAux_Alfa;//反电动势观测值a相
    pHandle->hBemf_beta_est = hAux_Beta;

    if (0 == pHandle->hForcedDirection)
    {
      /* we are in auxiliary mode, then rely on the speed detected */
      if(pHandle->_Super.hElSpeedDpp >= 0)
      {
        wDirection = 1;
      }
      else
      {
        wDirection = -1;
      }
    }
    else
    {
      /* we are in main sensor mode, use a forced direction */
      wDirection = pHandle->hForcedDirection;
    }

    hAux_Alfa = (int16_t)(hAux_Alfa * wDirection);//a相反电动势
    hAux_Beta = (int16_t)(hAux_Beta * wDirection);//b相反电动势
		
		//hRotor_Angle = MCM_PhaseComputation(hAux_Alfa, -hAux_Beta);//反正切求角度

    //hOrRotor_Speed = (int16_t)(hRotor_Angle - hPrev_Rotor_Angle);//求速度
   
   // hRotor_Speed = hOrRotor_Speed;
		
    hRotor_Speed = STO_ExecutePLL(pHandle, hAux_Alfa, -hAux_Beta);//速度
    pHandle->_Super.InstantaneousElSpeedDpp = hRotor_Speed;

    STO_Store_Rotor_Speed(pHandle, hRotor_Speed);

    pHandle->_Super.hElAngle += hRotor_Speed;

    /*storing previous values of currents and bemfs*/
		/* 存储前一次电流和反电动势的值 */
    pHandle->Ialfa_est = wIalfa_est_Next;
    pHandle->wBemf_alfa_est = wBemf_alfa_est_Next;

    pHandle->Ibeta_est = wIbeta_est_Next;
    pHandle->wBemf_beta_est = wBemf_beta_est_Next;
    retValue = pHandle->_Super.hElAngle;//电角度
  }
  return (retValue);
}

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and returns - through
  *         parameter hMecSpeedUnit - the rotor average mechanical speed,
  *         expressed in Unit. Average is computed considering a FIFO depth
  *         equal to bSpeedBufferSizeUnit. Moreover it also computes and returns
  *         the reliability state of the sensor.
  * @param  pHandle: handler of the current instance of the STO component
  * @param  pMecSpeedUnit pointer to int16_t, used to return the rotor average
  *         mechanical speed (expressed in the unit defined by #SPEED_UNIT)
  * @retval bool speed sensor reliability, measured with reference to parameters
  *         bReliability_hysteresys, hVariancePercentage and bSpeedBufferSize
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
  */


//-----------------------------------------中值平均值滤波----------------------

__weak int16_t Medin_Average_filter(int16_t Size,int16_t Speed_Buffer[])//
{
	int16_t sum = 0;
	int16_t temp;
	for( int j = 0; j < Size - 1; j++ )
    {
        for( int i = 0; i < Size- j - 1; i++ )
        {
            if( Speed_Buffer[i] >Speed_Buffer[i + 1] )
            {
                temp = Speed_Buffer[i];
                Speed_Buffer[i] = Speed_Buffer[i + 1];
                Speed_Buffer[i + 1] = temp;
            }
        }
    }
	for(int i=1;i<Size-1;i++)
		{
			sum +=Speed_Buffer[i];
		}
	sum = sum/ ((int16_t)(Size-2));
	return (int32_t)sum;
}

//---------------------------


__weak bool STO_PLL_CalcAvrgMecSpeedUnit(STO_PLL_Handle_t *pHandle, int16_t *pMecSpeedUnit)//计算速度单元
{
  bool bAux;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == pHandle) || (MC_NULL == pMecSpeedUnit))
  {
    bAux = false;
  }
  else
  {
#endif
		uint16_t AverageFilter_Size = 16;
    int32_t wAvrSpeed_dpp = (int32_t)0;//定义平均速度单元
    int32_t wError;//定义误差
    int32_t wAux;
    int32_t wAvrSquareSpeed;//定义平均速度平方
    int32_t wAvrQuadraticError = 0;//误差
    int32_t wObsBemf, wEstBemf;//观测的和估计的反电动势
    int32_t wObsBemfSq = 0;
    int32_t wEstBemfSq = 0;
		int16_t Speed_Buffer[AverageFilter_Size];
    int32_t wEstBemfSqLo;
    bool bIs_Speed_Reliable = false;//是否可靠
    bool bIs_Bemf_Consistent = false;
    uint8_t i, bSpeedBufferSizeUnit = pHandle->SpeedBufferSizeUnit;//速度数组单元大小
			
		if(bSpeedBufferSizeUnit<AverageFilter_Size)//判断均值滤波的大小和当前大小哪个大
		{
			AverageFilter_Size = bSpeedBufferSizeUnit;
			
		}
		
		//这可以注释掉
   // for (i = 0U; i < bSpeedBufferSizeUnit; i++)//读速度数组里面的单元进行求和
   // {
   //   wAvrSpeed_dpp += (int32_t)(pHandle->Speed_Buffer[i]);
   // }
  for (i = AverageFilter_Size-1; i >= 0; i--)//读速度数组里面的单元进行求和
    {
     Speed_Buffer[i] = (int16_t)(pHandle->Speed_Buffer[i]);
		}
    if (0U == bSpeedBufferSizeUnit)//速度数组没有东西则不进行处理
    {
      /* Nothing to do */
    }
    else
			wAvrSpeed_dpp=Medin_Average_filter(AverageFilter_Size,Speed_Buffer);
	
		

		
    for (i = 0U; i < bSpeedBufferSizeUnit; i++)
    {
      wError = ((int32_t)pHandle->Speed_Buffer[i]) - wAvrSpeed_dpp;//每个速度单元减去平均值
      wError = (wError * wError);//求平方
      wAvrQuadraticError += wError;//求和
    }

    /* It computes the measurement variance */// 计算测量方差
    wAvrQuadraticError = wAvrQuadraticError / ((int16_t)bSpeedBufferSizeUnit);//求平均

    /* The maximum variance acceptable is here calculated as a function of average speed */
		    // 根据平均速度计算可接受的最大方差

    wAvrSquareSpeed = wAvrSpeed_dpp * wAvrSpeed_dpp;//速度求平方
    int64_t lAvrSquareSpeed = (int64_t)(wAvrSquareSpeed) * pHandle->VariancePercentage;      
    wAvrSquareSpeed = lAvrSquareSpeed / (int16_t)128;
    //判断传感器是否可靠
    if (wAvrQuadraticError < wAvrSquareSpeed)
    {
      bIs_Speed_Reliable = true;
    }
// 计算机械速度单元
    /* Computation of Mechanical speed Unit */
    wAux = wAvrSpeed_dpp * ((int32_t)pHandle->_Super.hMeasurementFrequency);//速度单元乘以频率
    wAux = wAux * ((int32_t)pHandle->_Super.SpeedUnit);
    wAux = wAux / ((int32_t)pHandle->_Super.DPPConvFactor);
    wAux = wAux / ((int16_t)pHandle->_Super.bElToMecRatio);

    *pMecSpeedUnit = (int16_t)wAux;
    pHandle->_Super.hAvrMecSpeedUnit = (int16_t)wAux;//更新结构体里面的机械速度值
    pHandle->IsSpeedReliable = bIs_Speed_Reliable;

    /*Bemf Consistency Check algorithm*/ // Bemf一致性检查算法
    if (true == pHandle->EnableDualCheck) /*do algorithm if it's enabled*/
    {
      /* wAux abs value   */
      //cstat !MISRAC2012-Rule-14.3_b !RED-func-no-effect !RED-cmp-never !RED-cond-never
      wAux = ((wAux < 0) ? (-wAux) : (wAux));
      if (wAux < (int32_t)(pHandle->MaxAppPositiveMecSpeedUnit))
      {
        /* Computation of Observed back-emf */
        wObsBemf = (int32_t)pHandle->hBemf_alfa_est;
        wObsBemfSq = wObsBemf * wObsBemf;
        wObsBemf = (int32_t)pHandle->hBemf_beta_est;
        wObsBemfSq += wObsBemf * wObsBemf;

        /* Computation of Estimated back-emf */
        wEstBemf = (wAux * 32767) / ((int16_t)pHandle->_Super.hMaxReliableMecSpeedUnit);
        wEstBemfSq = (wEstBemf * ((int32_t)pHandle->BemfConsistencyGain)) / 64;
        wEstBemfSq *= wEstBemf;

        /* Computation of threshold *///阈值判断
        wEstBemfSqLo = wEstBemfSq - ((wEstBemfSq / 64) * ((int32_t)pHandle->BemfConsistencyCheck));

        /* Check */
        if (wObsBemfSq > wEstBemfSqLo)
        {
          bIs_Bemf_Consistent = true;
        }
      }

      pHandle->IsBemfConsistent = bIs_Bemf_Consistent;
      pHandle->Obs_Bemf_Level = wObsBemfSq;
      pHandle->Est_Bemf_Level = wEstBemfSq;
    }
    else
    {
      bIs_Bemf_Consistent = true;
    }

    /* Decision making *///判断是否进行速度可靠性检验
    if (false == pHandle->IsAlgorithmConverged)
    {
      bAux = SPD_IsMecSpeedReliable (&pHandle->_Super, pMecSpeedUnit);
    }
    else
    {
      if ((false == pHandle->IsSpeedReliable) || (false == bIs_Bemf_Consistent))
      {
        pHandle->ReliabilityCounter++;
        if (pHandle->ReliabilityCounter >= pHandle->Reliability_hysteresys)
        {
          pHandle->ReliabilityCounter = 0U;
          pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
          bAux = false;
        }
        else
        {
          bAux = SPD_IsMecSpeedReliable (&pHandle->_Super, pMecSpeedUnit);
        }
      }
      else
      {
        pHandle->ReliabilityCounter = 0U;
        bAux = SPD_IsMecSpeedReliable (&pHandle->_Super, pMecSpeedUnit);
      }
    }
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
  return (bAux);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and update component
  *         variable hElSpeedDpp that is estimated average electrical speed
  *         expressed in dpp used for instance in observer equations.
  *         Average is computed considering a FIFO depth equal to
  *         bSpeedBufferSizedpp.
用于在无传感器电机控制算法中计算平均电速（以"dpp"，即每周期的脉冲数为单位）。以下是该函数执行的关键步骤的说明：
  * @param  pHandle: handler of the current instance of the STO component
  * @retval none
  */
__weak void STO_PLL_CalcAvrgElSpeedDpp(STO_PLL_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int16_t hIndexNew = (int16_t)pHandle->Speed_Buffer_Index;
    int16_t hIndexOld;
    int16_t hIndexOldTemp;
    int32_t wSum = pHandle->DppBufferSum;
    int32_t wAvrSpeed_dpp;
    int16_t hSpeedBufferSizedpp = (int16_t)pHandle->SpeedBufferSizeDpp;
    int16_t hSpeedBufferSizeUnit = (int16_t)pHandle->SpeedBufferSizeUnit;
    int16_t hBufferSizeDiff;

    hBufferSizeDiff = hSpeedBufferSizeUnit - hSpeedBufferSizedpp;

    if (0 == hBufferSizeDiff)
    {
      wSum = wSum + pHandle->Speed_Buffer[hIndexNew] - pHandle->SpeedBufferOldestEl;
    }
    else
    {
      hIndexOldTemp = hIndexNew + hBufferSizeDiff;

      if (hIndexOldTemp >= hSpeedBufferSizeUnit)
      {
        hIndexOld = hIndexOldTemp - hSpeedBufferSizeUnit;
      }
      else
      {
        hIndexOld = hIndexOldTemp;
      }

      wSum = wSum + pHandle->Speed_Buffer[hIndexNew] - pHandle->Speed_Buffer[hIndexOld];
    }

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
  wAvrSpeed_dpp = wSum >> pHandle->SpeedBufferSizeDppLOG;
#else
  if ((int16_t )0 == hSpeedBufferSizedpp)
  {
    /* Nothing to do */
    wAvrSpeed_dpp = wSum;
  }
  else
  {
    wAvrSpeed_dpp = wSum / hSpeedBufferSizedpp;
  }
#endif
    pHandle->_Super.hElSpeedDpp = (int16_t)wAvrSpeed_dpp;
    pHandle->DppBufferSum = wSum;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  It clears state observer component by re-initializing private variables
  * @param  pHandle related object of class CSTO_SPD
  * @retval none
  */
__weak void STO_PLL_Clear(STO_PLL_Handle_t *pHandle)//清除PLL锁相环
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->Ialfa_est = (int32_t)0;
    pHandle->Ibeta_est = (int32_t)0;
    pHandle->wBemf_alfa_est = (int32_t)0;
    pHandle->wBemf_beta_est = (int32_t)0;
    pHandle->_Super.hElAngle = (int16_t)0;
    pHandle->_Super.hElSpeedDpp = (int16_t)0;
    pHandle->ConsistencyCounter = 0u;
    pHandle->ReliabilityCounter = 0u;
    pHandle->IsAlgorithmConverged = false;
    pHandle->IsBemfConsistent = false;
    pHandle->Obs_Bemf_Level = (int32_t)0;
    pHandle->Est_Bemf_Level = (int32_t)0;
    pHandle->DppBufferSum = (int32_t)0;
    pHandle->ForceConvergency = false;
    pHandle->ForceConvergency2 = false;

    STO_InitSpeedBuffer(pHandle);
    PID_SetIntegralTerm(& pHandle->PIRegulator, (int32_t)0);
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  It stores in estimated speed FIFO latest calculated value of motor
  *         speed
  * @param  pHandle: handler of the current instance of the STO component
  * @retval none
  */
inline static void STO_Store_Rotor_Speed(STO_PLL_Handle_t *pHandle, int16_t hRotor_Speed)//存储最新的电机速度
{
  uint8_t bBuffer_index = pHandle->Speed_Buffer_Index;

  bBuffer_index++;
  if (bBuffer_index == pHandle->SpeedBufferSizeUnit)
  {
    bBuffer_index = 0U;
  }

  pHandle->SpeedBufferOldestEl = pHandle->Speed_Buffer[bBuffer_index];
  pHandle->Speed_Buffer[bBuffer_index] = hRotor_Speed;
   pHandle->Speed_Buffer_Index = bBuffer_index;
}

/**
  * @brief  It executes PLL algorithm for rotor position extraction from B-emf
  *         alpha and beta
  * @param  pHandle: handler of the current instance of the STO component
  *         hBemf_alfa_est estimated Bemf alpha on the stator reference frame
这是STO_ExecutePLL函数的实现，用于执行基于B-emf（BEMF alpha和beta）的PLL算法，用于提取转子位置。以下是该函数执行的关键步骤的说明：

函数开始时，声明了一些变量，包括wAlfa_Sin_tmp、wBeta_Cos_tmp、Local_Components、hAux1、hAux2和hOutput。
函数使用电角度（pHandle->_Super.hElAngle）调用MCM_Trig_Functions函数，获取本地三角函数组件（Local_Components）。
函数将BEMF alpha和beta与本地三角函数组件进行乘法运算，得到乘积结果wAlfa_Sin_tmp和wBeta_Cos_tmp。
根据编译选项（FULL_MISRA_C_COMPLIANCY_STO_PLL），函数对乘积结果进行右移或除以32768（即2的15次方）来得到修正后的值hAux1和hAux2。
函数使用速度PI控制器（PI_Controller函数）对hAux1 - hAux2进行控制，并将输出结果存储在hOutput中。
函数返回hOutput作为函数的结果。
总体而言，该函数执行基于B-emf的PLL算法，计算转子位置，并使用速度PI控制器进行调节，最终返回调节后的输出结果。
  *         hBemf_beta_est estimated Bemf beta on the stator reference frame
  * @retval none
  */
inline static int16_t STO_ExecutePLL(STO_PLL_Handle_t *pHandle, int16_t hBemf_alfa_est, int16_t hBemf_beta_est)
{
  int32_t wAlfa_Sin_tmp;
  int32_t wBeta_Cos_tmp;
  Trig_Components Local_Components;
  int16_t hAux1;
  int16_t hAux2;
  int16_t hOutput;
	int32_t e;
  Local_Components = MCM_Trig_Functions(pHandle->_Super.hElAngle);

  /* Alfa & Beta BEMF multiplied by Cos & Sin*/
  wAlfa_Sin_tmp = ((int32_t )hBemf_alfa_est) * ((int32_t )Local_Components.hSin);
  wBeta_Cos_tmp = ((int32_t )hBemf_beta_est) * ((int32_t )Local_Components.hCos);

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  hAux1 = (int16_t)(wBeta_Cos_tmp >> 15); //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
  hAux1 = (int16_t)(wBeta_Cos_tmp / 32768);
#endif

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  hAux2 = (int16_t)(wAlfa_Sin_tmp >> 15); //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
  hAux2 = (int16_t)(wAlfa_Sin_tmp / 32768);
#endif
	e = (int32_t)sqrt(hAux1*hAux1 +hAux2 *hAux2  );//单位化
	
  /* Speed PI regulator */
 // hOutput = PI_Controller(& pHandle->PIRegulator, (int32_t)(hAux1 ) - hAux2);
  hOutput = PI_Controller(& pHandle->PIRegulator, ((int32_t)(hAux1 ) - hAux2)/e);
  return (hOutput);
}

/**
  * @brief  It clears the estimated speed buffer
  * @param  pHandle: handler of the current instance of the STO component
  * @retval none
  */
static void STO_InitSpeedBuffer(STO_PLL_Handle_t * pHandle)//速度初始化
{
  uint8_t b_i;
  uint8_t bSpeedBufferSize = pHandle->SpeedBufferSizeUnit;

  /*init speed buffer*/
  for (b_i = 0U; b_i < bSpeedBufferSize; b_i++)
  {
    pHandle->Speed_Buffer[b_i] = (int16_t)0;
  }
  pHandle->Speed_Buffer_Index = 0U;
  pHandle->SpeedBufferOldestEl = (int16_t)0;

  return;
}

/**
  * @brief  It internally performs a set of checks necessary to state whether
  *         the state observer algorithm converged. To be periodically called
  *         during motor open-loop ramp-up (e.g. at the same frequency of
  *         SPD_CalcElAngle), it returns true if the estimated angle and speed


函数开始时，声明了一些变量，包括bAux、hEstimatedSpeedUnit、hUpperThreshold、hLowerThreshold、wAux和wtemp。
如果pHandle->ForceConvergency2为true，则将pHandle->_Super.hAvrMecSpeedUnit赋值给*phForcedMecSpeedUnit。
如果pHandle->ForceConvergency为true，则将bAux设置为true，将pHandle->IsAlgorithmConverged设置为true，将pHandle->_Super.bSpeedErrorNumber设置为0。
否则，获取估计的速度值hEstimatedSpeedUnit。
计算wtemp，即hEstimatedSpeedUnit和*phForcedMecSpeedUnit的乘积。
如果wtemp大于0，则执行以下步骤：
如果hEstimatedSpeedUnit为负数，则取其绝对值。
如果*phForcedMecSpeedUnit为负数，则取其绝对值。
根据编译选项（FULL_MISRA_C_COMPLIANCY_STO_PLL），计算上限阈值hUpperThreshold和下限阈值hLowerThreshold。
如果估计速度的方差足够小（pHandle->IsSpeedReliable为true）：
如果估计速度大于pHandle->MinStartUpValidSpeed（最小启动有效速度）：
如果估计速度在下限阈值和上限阈值之间：
pHandle->ConsistencyCounter递增。
如果pHandle->ConsistencyCounter大于等于pHandle->StartUpConsistThreshold（启动一致性阈值）：
算法收敛，将bAux设置为true，将pHandle->IsAlgorithmConverged设置为true，将pHandle->_Super.bSpeedErrorNumber设置为0。
否则，将pHandle->ConsistencyCounter重置为0。
否则，将pHandle->ConsistencyCounter重置为0。
否则，将pHandle->ConsistencyCounter重置为0。
根据编译选项（NULL_PTR_STO_PLL_SPD_POS_FDB），检查pHandle和phForcedMecSpeedUnit是否为NULL。
返回bAux作为函数的结果。
总体而言，该函数用于执行一系列检查，以判断状态观测器算法是否收敛。它考虑估计的速度与期望速度的一致性、速度方差的可靠性等因素，根据设定的阈值判断算法是否收敛，并返回结果。
  *         can be considered reliable, false otherwise
  * @param  pHandle: handler of the current instance of the STO component
  * @param  hForcedMecSpeedUnit Mechanical speed in 0.1Hz unit as forced by VSS
  * @retval bool sensor reliability state
  */
__weak bool STO_PLL_IsObserverConverged(STO_PLL_Handle_t *pHandle, int16_t *phForcedMecSpeedUnit)
{
  bool bAux = false;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == pHandle) || (MC_NULL == phForcedMecSpeedUnit))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int16_t hEstimatedSpeedUnit;
    int16_t hUpperThreshold;
    int16_t hLowerThreshold;
    int32_t wAux;
    int32_t wtemp;

    if (true == pHandle->ForceConvergency2)
    {
      *phForcedMecSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;
    }

    if (true == pHandle->ForceConvergency)
    {
      bAux = true;
      pHandle->IsAlgorithmConverged = true;
      pHandle->_Super.bSpeedErrorNumber = 0U;
    }
    else
    {
      hEstimatedSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;

      wtemp = ((int32_t)hEstimatedSpeedUnit) * ((int32_t)*phForcedMecSpeedUnit);

      if (wtemp > 0)
      {
        if (hEstimatedSpeedUnit < 0)
        {
          hEstimatedSpeedUnit = -hEstimatedSpeedUnit;
        }

        if (*phForcedMecSpeedUnit < 0)
        {
          *phForcedMecSpeedUnit = -*phForcedMecSpeedUnit;
        }
        wAux = ((int32_t)*phForcedMecSpeedUnit) * ((int16_t)pHandle->SpeedValidationBand_H);
        hUpperThreshold = (int16_t)(wAux / ((int32_t)16));

        wAux = ((int32_t)*phForcedMecSpeedUnit) * ((int16_t)pHandle->SpeedValidationBand_L);
        hLowerThreshold = (int16_t)(wAux / ((int32_t)16));

        /* If the variance of the estimated speed is low enough...*/
        if (true == pHandle->IsSpeedReliable)
        {
          if ((uint16_t)hEstimatedSpeedUnit > pHandle->MinStartUpValidSpeed)
          {
            /*...and the estimated value is quite close to the expected value... */
            if (hEstimatedSpeedUnit >= hLowerThreshold)
            {
              if (hEstimatedSpeedUnit <= hUpperThreshold)
              {
                pHandle->ConsistencyCounter++;

                /*... for hConsistencyThreshold consecutive times... */
                if (pHandle->ConsistencyCounter >= pHandle->StartUpConsistThreshold)
                {

                  /* the algorithm converged.*/
                  bAux = true;
                  pHandle->IsAlgorithmConverged = true;
                  pHandle->_Super.bSpeedErrorNumber = 0U;
                }
              }
              else
              {
                pHandle->ConsistencyCounter = 0U;
              }
            }
            else
            {
              pHandle->ConsistencyCounter = 0U;
            }
          }
          else
          {
            pHandle->ConsistencyCounter = 0U;
          }
        }
        else
        {
          pHandle->ConsistencyCounter = 0U;
        }
      }
    }
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
  return (bAux);
}

/**
  * @brief  It exports estimated Bemf alpha-beta in alphabeta_t format
  * @param  pHandle: handler of the current instance of the STO component
  * @retval alphabeta_t Bemf alpha-beta
  */
__weak alphabeta_t STO_PLL_GetEstimatedBemf(STO_PLL_Handle_t *pHandle)//获取估计得反电动势
{
  alphabeta_t vaux;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    vaux.alpha = 0;
    vaux.beta = 0;
  }
  else
  {
#endif
    vaux.alpha = pHandle->hBemf_alfa_est;
    vaux.beta = pHandle->hBemf_beta_est;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
  return (vaux);
}


/**
  * @brief  It exports the stator current alpha-beta as estimated by state
  *         observer
  * @param  pHandle: handler of the current instance of the STO component
  * @retval alphabeta_t State observer estimated stator current Ialpha-beta
  */
__weak alphabeta_t STO_PLL_GetEstimatedCurrent(STO_PLL_Handle_t *pHandle)//得到估计的电流
{
  alphabeta_t iaux;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    iaux.alpha = 0;
    iaux.beta = 0;
  }
  else
  {
#endif
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
  iaux.alpha = (int16_t)(pHandle->Ialfa_est >> pHandle->F1LOG);
#else
  iaux.alpha = (int16_t)(pHandle->Ialfa_est / pHandle->hF1);
#endif

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
  iaux.beta = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);
#else
  iaux.beta = (int16_t)(pHandle->Ibeta_est / pHandle->hF1);
#endif
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
  return (iaux);
}

/**
  * @brief  It exports current observer gains through parameters hhC2 and hhC4
  * @param  pHandle: handler of the current instance of the STO component
  * @param  phC2 pointer to int16_t used to return parameters hhC2
函数开始时，检查pHandle、phC2和phC4是否为NULL（根据编译选项NULL_PTR_STO_PLL_SPD_POS_FDB）。如果其中任何一个为NULL，则函数不执行任何操作。
如果以上检查通过，将观测器的增益hC2和hC4分别赋值给*phC2和*phC4，以便将其返回。
根据编译选项（NULL_PTR_STO_PLL_SPD_POS_FDB），检查pHandle是否为NULL。
函数结束，没有返回值。
总体而言，该函数用于获取当前观测器的增益，并通过指针参数phC2和phC4返回这些增益值。如果传入的指针参数为NULL，则函数不执行任何操作。
  * @param  phC4 pointer to int16_t used to return parameters hhC4
  * @retval none
  */
__weak void STO_PLL_GetObserverGains(STO_PLL_Handle_t *pHandle, int16_t *phC2, int16_t *phC4)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == pHandle) || (MC_NULL == phC2) || (MC_NULL == phC4))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    *phC2 = pHandle->hC2;
    *phC4 = pHandle->hC4;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}


/**
  * @brief  It allows setting new values for observer gains
  * @param  pHandle: handler of the current instance of the STO component
  * @param  wK1 new value for observer gain hhC1
  * @param  wK2 new value for observer gain hhC2
  * @retval none
  */
__weak void STO_PLL_SetObserverGains(STO_PLL_Handle_t *pHandle, int16_t hhC1, int16_t hhC2)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hC2 = hhC1;
    pHandle->hC4 = hhC2;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  It exports current PLL gains through parameters pPgain and pIgain
  * @param  pHandle: handler of the current instance of the STO component
  * @param  pPgain pointer to int16_t used to return PLL proportional gain
  * @param  pIgain pointer to int16_t used to return PLL integral gain
  * @retval none
  */
__weak void STO_GetPLLGains(STO_PLL_Handle_t *pHandle, int16_t *pPgain, int16_t *pIgain)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == pHandle) || (MC_NULL == pPgain) || (MC_NULL == pIgain))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    *pPgain = PID_GetKP(& pHandle->PIRegulator);
    *pIgain = PID_GetKI(& pHandle->PIRegulator);
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}


/**
  * @brief  It allows setting new values for PLL gains
  * @param  pHandle: handler of the current instance of the STO component
  * @param  hPgain new value for PLL proportional gain
  * @param  hIgain new value for PLL integral gain
  * @retval none
  */
__weak void STO_SetPLLGains(STO_PLL_Handle_t *pHandle, int16_t hPgain, int16_t hIgain)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    PID_SetKP(&pHandle->PIRegulator, hPgain);
    PID_SetKI(&pHandle->PIRegulator, hIgain);
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}


/**
  * @brief  It could be used to set istantaneous information on rotor mechanical
  *         angle.
  *         Note: Mechanical angle management is not implemented in this
  *         version of State observer sensor class.
  * @param  pHandle: handler of the current instance of the STO component
  * @param  hMecAngle istantaneous measure of rotor mechanical angle
  * @retval none
  */
//cstat !RED-func-no-effect
__weak void STO_PLL_SetMecAngle(STO_PLL_Handle_t *pHandle, int16_t hMecAngle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == (void *)pHandle) || ((int16_t)0 == hMecAngle))
  {
    /* Nothing to do */
  }
  else
  {
    /* nothing to do */
  }
#endif
}

/**
  * @brief  It resets integral term of PLL during on-the-fly startup
  * @param  pHandle: handler of the current instance of the STO component
  * @retval none
  */
__weak void STO_OTF_ResetPLL(STO_Handle_t * pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    STO_PLL_Handle_t *pHdl = (STO_PLL_Handle_t *)pHandle->_Super; //cstat !MISRAC2012-Rule-11.3
    PID_SetIntegralTerm(&pHdl->PIRegulator, (int32_t)0);
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  It resets integral term of PLL
  * @param  pHandle: handler of the current instance of the STO component
  * @retval none
  */
__weak void STO_ResetPLL(STO_PLL_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    PID_SetIntegralTerm(&pHandle->PIRegulator, (int32_t)0);
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  It sends locking info for PLL
  * @param  pHandle: handler of the current instance of the STO component
  * @param  hElSpeedDpp:
  * @param  hElAngle:
  * @retval none
  */
__weak void STO_SetPLL(STO_PLL_Handle_t *pHandle, int16_t hElSpeedDpp, int16_t hElAngle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    PID_SetIntegralTerm(&pHandle->PIRegulator, ((int32_t)hElSpeedDpp)
                                              * (int32_t)(PID_GetKIDivisor(&pHandle->PIRegulator)));
    pHandle->_Super.hElAngle = hElAngle;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  It exports estimated Bemf squared level
  * @param  pHandle: handler of the current instance of the STO component
  * @retval int32_t
  */
__weak int32_t STO_PLL_GetEstimatedBemfLevel(STO_PLL_Handle_t *pHandle)//获取估计的电动势
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  return ((MC_NULL == pHandle) ? 0 : pHandle->Est_Bemf_Level);
#else
  return (pHandle->Est_Bemf_Level);
#endif
}

/**
  * @brief  It exports observed Bemf squared level
  * @param  pHandle: handler of the current instance of the STO component
  * @retval int32_t
  */
__weak int32_t STO_PLL_GetObservedBemfLevel(STO_PLL_Handle_t *pHandle)//获取观测得到的反电动势
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  return ((MC_NULL == pHandle) ? 0 : pHandle->Obs_Bemf_Level);
#else
  return (pHandle->Obs_Bemf_Level);
#endif
}

/**
  * @brief  It enables/disables the bemf consistency check
  * @param  pHandle: handler of the current instance of the STO component
  * @param  bSel boolean; true enables check; false disables check
  */
__weak void STO_PLL_BemfConsistencyCheckSwitch(STO_PLL_Handle_t *pHandle, bool bSel)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->EnableDualCheck = bSel;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  It returns the result of the Bemf consistency check
  * @param  pHandle: handler of the current instance of the STO component
  * @retval bool Bemf consistency state
  */
__weak bool STO_PLL_IsBemfConsistent(STO_PLL_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  return ((MC_NULL == pHandle) ? false : pHandle->IsBemfConsistent);
#else
  return (pHandle->IsBemfConsistent);
#endif
}

/**
  * @brief  It returns the result of the last variance check
  * @param  pHandle: handler of the current instance of the STO component
  * @retval bool Variance state
  */
__weak bool STO_PLL_IsVarianceTight(const STO_Handle_t *pHandle)
{
  bool tempStatus;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    tempStatus = false;
  }
  else
  {
#endif
    STO_PLL_Handle_t *pHdl = (STO_PLL_Handle_t *)pHandle->_Super; //cstat !MISRAC2012-Rule-11.3
    tempStatus = pHdl->IsSpeedReliable;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
  return (tempStatus);
}

/**
  * @brief  It forces the state-observer to declare convergency
  * @param  pHandle: handler of the current instance of the STO component
  */
__weak void STO_PLL_ForceConvergency1(STO_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    STO_PLL_Handle_t *pHdl = (STO_PLL_Handle_t *)pHandle->_Super; //cstat !MISRAC2012-Rule-11.3
    pHdl->ForceConvergency = true;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  It forces the state-observer to declare convergency
  * @param  pHandle: handler of the current instance of the STO component
  */
__weak void STO_PLL_ForceConvergency2(STO_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    STO_PLL_Handle_t *pHdl = (STO_PLL_Handle_t *)pHandle->_Super; //cstat !MISRAC2012-Rule-11.3
    pHdl->ForceConvergency2 = true;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Set the Absolute value of minimum mechanical speed (expressed in
  *         the unit defined by #SPEED_UNIT) required to validate the start-up.
  * @param  pHandle: handler of the current instance of the STO component
  * @param  hMinStartUpValidSpeed: Absolute value of minimum mechanical speed
  */
__weak void STO_SetMinStartUpValidSpeedUnit(STO_PLL_Handle_t *pHandle, uint16_t hMinStartUpValidSpeed)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->MinStartUpValidSpeed = hMinStartUpValidSpeed;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  forces the rotation direction
  * @param  direction: imposed direction
  */
__weak void STO_SetDirection(STO_PLL_Handle_t *pHandle, int8_t direction)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hForcedDirection = direction;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

//-------------------------反正切-------------------------------
static inline int16_t fanzhengqie(int32_t wBemf_alfa_est, int32_t wBemf_beta_est)//反正切求解角度
{

  /* Configure and call to CORDIC */
  WRITE_REG(CORDIC->CSR,CORDIC_CONFIG_PHASE);
  LL_CORDIC_WriteData(CORDIC, (uint32_t)wBemf_alfa_est);
  LL_CORDIC_WriteData(CORDIC, (uint32_t)wBemf_beta_est);

  /* Read computed angle */
  uint32_t result;
  result = LL_CORDIC_ReadData(CORDIC) >> 16U;
  return ((int16_t)result);

}



//-----------------------------------------滑膜观测器----------------------


int16_t update_E(SMO_Handle* Phandle, Observer_Inputs_t *pInputs)
{
	 int32_t wIalfa_est_Next;//定义的alfa电流估计值
   int32_t wIbeta_est_Next;//定义的beta电流估计值
   int32_t wBemf_alfa_est_Next;
   int32_t wBemf_beta_est_Next;
	int16_t speed;
	int sign_a,sign_b;
	 int h = 1000;
	 int32_t error_a,error_b;
	 error_a = pInputs->Ialfa_beta.alpha;
	 error_b = pInputs->Ialfa_beta.beta;
	
	 if(error_a>0u)
	 {
		sign_a = 1; 
	 }
	 else
	 {
		 sign_a = -1;
	 }
	 if(error_b>0u)
	 {
		sign_b = 1; 
	 }
	 else
	 {
		 sign_b = -1;
	 }
	 //求得估计的反电动势
	 wBemf_alfa_est_Next = h*sign_a;
	 wBemf_beta_est_Next = h*sign_b;
	 speed = SMO_ExecutePLL(Phandle,  wBemf_alfa_est_Next, - wBemf_beta_est_Next);//求解得到速度
	 
}
inline static int16_t SMO_ExecutePLL(SMO_Handle *pHandle, int16_t hBemf_alfa_est, int16_t hBemf_beta_est)//SMO滑膜观测器
{
  int32_t wAlfa_Sin_tmp;
  int32_t wBeta_Cos_tmp;
  Trig_Components Local_Components;
  int16_t hAux1;
  int16_t hAux2;
  int16_t hOutput;
	int32_t e;
  Local_Components = MCM_Trig_Functions(pHandle->_Super.hElAngle);

  /* Alfa & Beta BEMF multiplied by Cos & Sin*/
  wAlfa_Sin_tmp = ((int32_t )hBemf_alfa_est) * ((int32_t )Local_Components.hSin);
  wBeta_Cos_tmp = ((int32_t )hBemf_beta_est) * ((int32_t )Local_Components.hCos);

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  hAux1 = (int16_t)(wBeta_Cos_tmp >> 15); //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
  hAux1 = (int16_t)(wBeta_Cos_tmp / 32768);
#endif

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  hAux2 = (int16_t)(wAlfa_Sin_tmp >> 15); //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
  hAux2 = (int16_t)(wAlfa_Sin_tmp / 32768);
#endif
	e = (int32_t)sqrt(hAux1*hAux1 +hAux2 *hAux2  );//单位化
  /* Speed PI regulator */
 // hOutput = PI_Controller(& pHandle->PIRegulator, (int32_t)(hAux1 ) - hAux2);
  hOutput = PI_Controller(& pHandle->PIRegulator, e);
  return (hOutput);
}

/**
  * @}
  */

/**
  * @}
  */

/** @} */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
