/*
 * helper.h
 *
 *  Created on: Nov 8, 2024
 *      Author: ege
 */

#ifndef INC_HELPER_H_
#define INC_HELPER_H_


float custom_sqrtf(float x) {
    if (x < 0.0f) {
        // Handle negative input gracefully (return 0 or NaN, depending on your use case)
        return 0.0f; // or NAN if <math.h> is included
    }
    if (x == 0.0f || x == 1.0f) {
        return x; // Handle edge cases directly
    }

    // Initial guess (you can fine-tune this for your range of inputs)
    float guess = x / 2.0f;

    // Newton-Raphson iterations
    for (int i = 0; i < 10; i++) { // 5 iterations are usually sufficient for good precision
        guess = 0.5f * (guess + x / guess);
    }

    return guess;
}


float ratof(char *arr)
{
  float val = 0;
  int afterdot=0;
  float scale=1;
  int neg = 0;

  if (*arr == '-') {
    arr++;
    neg = 1;
  }
  while (*arr) {
    if (afterdot) {
      scale = scale/10;
      val = val + (*arr-'0')*scale;
    } else {
      if (*arr == '.')
    afterdot++;
      else
    val = val * 10.0 + (*arr - '0');
    }
    arr++;
  }
  if(neg) return -val;
  else    return  val;
}

static int isdigit_(char ch)
{
    return (ch >= '0' && ch <= '9') || (ch == '.') || (ch == '-');
}

static int isalpha_(char ch)
{
    return ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z'));
}

static void extractKeywordAndValue(const char *inputString, int length, char *keyword, char *value)
{
	int i = length - 1;
	int valueIndex = 0;
	int keywordIndex = 0;
	int hasFoundValue = 0;

	// Check the character before \n to determine if it is a letter or a number
	while (i >= 0 && (inputString[i] == '\n' || inputString[i] == ' '))
	{
		i--;
	}

	if (i >= 0 && isdigit_(inputString[i]))
	{
		// Found a numerical value, proceed to read the value
		while (i >= 0 && (inputString[i] != '\n' && inputString[i] != '\0' && inputString[i] != ' '))
		{
			if (isdigit_(inputString[i]))
			{
				value[valueIndex++] = inputString[i];
				hasFoundValue = 1;
			}

			i--;
		}

		// Null-terminate the value string and reverse it since it was read backwards
		if (hasFoundValue)
		{
			value[valueIndex] = '\0';
			for (int j = 0; j < valueIndex / 2; j++)
			{
				char temp = value[j];
				value[j] = value[valueIndex - j - 1];
				value[valueIndex - j - 1] = temp;
			}
		}

		// Check if a space character was found and proceed to find the keyword
		if (hasFoundValue && i >= 0 && inputString[i] == ' ')
		{
			i--;	// Move left to skip the space character
			// Traverse the string to find the keyword
			while (i >= 0 && (inputString[i] != '\n' && inputString[i] != '\0' && inputString[i] != ' '))
			{
				keyword[keywordIndex++] = inputString[i];
				i--;
			}

			// Null-terminate the keyword string and reverse it since it was read backwards
			if (keywordIndex > 0)
			{
				keyword[keywordIndex] = '\0';
				for (int j = 0; j < keywordIndex / 2; j++)
				{
					char temp = keyword[j];
					keyword[j] = keyword[keywordIndex - j - 1];
					keyword[keywordIndex - j - 1] = temp;
				}
			}
		}
	}
	else if (i >= 0 && isalpha_(inputString[i]))
	{
		// Found a single letter, proceed to read the keyword
		while (i >= 0 && (inputString[i] != '\n' && inputString[i] != '\0' && inputString[i] != ' '))
		{
			keyword[keywordIndex++] = inputString[i];
			i--;
		}

		// Null-terminate the keyword string and reverse it since it was read backwards
		if (keywordIndex > 0)
		{
			keyword[keywordIndex] = '\0';
			for (int j = 0; j < keywordIndex / 2; j++)
			{
				char temp = keyword[j];
				keyword[j] = keyword[keywordIndex - j - 1];
				keyword[keywordIndex - j - 1] = temp;
			}
		}
	}

}



/**
  * @brief  Starts the TIM Encoder Interface for both channels.
  * @param  htim TIM Encoder Interface handle
  * @retval HAL status
  */
static HAL_StatusTypeDef HAL_TIM_Encoder_Start_optimized_(TIM_HandleTypeDef *htim)
{
  HAL_TIM_ChannelStateTypeDef channel_1_state = TIM_CHANNEL_STATE_GET(htim, TIM_CHANNEL_1);
  HAL_TIM_ChannelStateTypeDef channel_2_state = TIM_CHANNEL_STATE_GET(htim, TIM_CHANNEL_2);
  HAL_TIM_ChannelStateTypeDef complementary_channel_1_state = TIM_CHANNEL_N_STATE_GET(htim, TIM_CHANNEL_1);
  HAL_TIM_ChannelStateTypeDef complementary_channel_2_state = TIM_CHANNEL_N_STATE_GET(htim, TIM_CHANNEL_2);

  /* Check the parameters */
  assert_param(IS_TIM_ENCODER_INTERFACE_INSTANCE(htim->Instance));

  /* Verify that all channels are ready */
  if ((channel_1_state != HAL_TIM_CHANNEL_STATE_READY) ||
      (channel_2_state != HAL_TIM_CHANNEL_STATE_READY) ||
      (complementary_channel_1_state != HAL_TIM_CHANNEL_STATE_READY) ||
      (complementary_channel_2_state != HAL_TIM_CHANNEL_STATE_READY))
  {
    return HAL_ERROR;
  }

  /* Set the channels to BUSY */
  TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_1, HAL_TIM_CHANNEL_STATE_BUSY);
  TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_2, HAL_TIM_CHANNEL_STATE_BUSY);
  TIM_CHANNEL_N_STATE_SET(htim, TIM_CHANNEL_1, HAL_TIM_CHANNEL_STATE_BUSY);
  TIM_CHANNEL_N_STATE_SET(htim, TIM_CHANNEL_2, HAL_TIM_CHANNEL_STATE_BUSY);

  /* Enable both channels */
  TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
  TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);

  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(htim);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Configure the TI3 as Input.
  * @param  TIMx to select the TIM peripheral
  * @param  TIM_ICPolarity The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPOLARITY_RISING
  *            @arg TIM_ICPOLARITY_FALLING
  *            @arg TIM_ICPOLARITY_BOTHEDGE
  * @param  TIM_ICSelection specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICSELECTION_DIRECTTI: TIM Input 3 is selected to be connected to IC3.
  *            @arg TIM_ICSELECTION_INDIRECTTI: TIM Input 3 is selected to be connected to IC4.
  *            @arg TIM_ICSELECTION_TRC: TIM Input 3 is selected to be connected to TRC.
  * @param  TIM_ICFilter Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  * @note TIM_ICFilter and TIM_ICPolarity are not used in INDIRECT mode as TI3FP4
  *       (on channel1 path) is used as the input signal. Therefore CCMR2 must be
  *        protected against un-initialized filter and polarity values.
  */
static void TIM_TI3_SetConfig_optimized(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection,
                              uint32_t TIM_ICFilter)
{
  uint32_t tmpccmr2;
  uint32_t tmpccer;

  /* Disable the Channel 3: Reset the CC3E Bit */
  tmpccer = TIMx->CCER;
  TIMx->CCER &= ~TIM_CCER_CC3E;
  tmpccmr2 = TIMx->CCMR2;

  /* Select the Input */
  tmpccmr2 &= ~TIM_CCMR2_CC3S;
  tmpccmr2 |= TIM_ICSelection;

  /* Set the filter */
  tmpccmr2 &= ~TIM_CCMR2_IC3F;
  tmpccmr2 |= ((TIM_ICFilter << 4U) & TIM_CCMR2_IC3F);

  /* Select the Polarity and set the CC3E Bit */
  tmpccer &= ~(TIM_CCER_CC3P | TIM_CCER_CC3NP);
  tmpccer |= ((TIM_ICPolarity << 8U) & (TIM_CCER_CC3P | TIM_CCER_CC3NP));

  /* Write to TIMx CCMR2 and CCER registers */
  TIMx->CCMR2 = tmpccmr2;
  TIMx->CCER = tmpccer;
}


/**
  * @brief  Configure the TI4 as Input.
  * @param  TIMx to select the TIM peripheral
  * @param  TIM_ICPolarity The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPOLARITY_RISING
  *            @arg TIM_ICPOLARITY_FALLING
  *            @arg TIM_ICPOLARITY_BOTHEDGE
  * @param  TIM_ICSelection specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICSELECTION_DIRECTTI: TIM Input 4 is selected to be connected to IC4.
  *            @arg TIM_ICSELECTION_INDIRECTTI: TIM Input 4 is selected to be connected to IC3.
  *            @arg TIM_ICSELECTION_TRC: TIM Input 4 is selected to be connected to TRC.
  * @param  TIM_ICFilter Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @note TIM_ICFilter and TIM_ICPolarity are not used in INDIRECT mode as TI4FP3
  *       (on channel1 path) is used as the input signal. Therefore CCMR2 must be
  *        protected against un-initialized filter and polarity values.
  * @retval None
  */
static void TIM_TI4_SetConfig_optimized(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection,
                              uint32_t TIM_ICFilter)
{
  uint32_t tmpccmr2;
  uint32_t tmpccer;

  /* Disable the Channel 4: Reset the CC4E Bit */
  tmpccer = TIMx->CCER;
  TIMx->CCER &= ~TIM_CCER_CC4E;
  tmpccmr2 = TIMx->CCMR2;

  /* Select the Input */
  tmpccmr2 &= ~TIM_CCMR2_CC4S;
  tmpccmr2 |= (TIM_ICSelection << 8U);

  /* Set the filter */
  tmpccmr2 &= ~TIM_CCMR2_IC4F;
  tmpccmr2 |= ((TIM_ICFilter << 12U) & TIM_CCMR2_IC4F);

  /* Select the Polarity and set the CC4E Bit */
  tmpccer &= ~(TIM_CCER_CC4P | TIM_CCER_CC4NP);
  tmpccer |= ((TIM_ICPolarity << 12U) & (TIM_CCER_CC4P | TIM_CCER_CC4NP));

  /* Write to TIMx CCMR2 and CCER registers */
  TIMx->CCMR2 = tmpccmr2;
  TIMx->CCER = tmpccer ;
}

/**
  * @brief  Initializes the TIM Input Capture Channels according to the specified
  *         parameters in the TIM_IC_InitTypeDef.
  * @param  htim TIM IC handle
  * @param  sConfig TIM Input Capture configuration structure
  * @param  Channel TIM Channel to configure
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
static HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel_optimized(TIM_HandleTypeDef *htim, const TIM_IC_InitTypeDef *sConfig, uint32_t Channel)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Check the parameters */
  assert_param(IS_TIM_CC1_INSTANCE(htim->Instance));
  assert_param(IS_TIM_IC_POLARITY(sConfig->ICPolarity));
  assert_param(IS_TIM_IC_SELECTION(sConfig->ICSelection));
  assert_param(IS_TIM_IC_PRESCALER(sConfig->ICPrescaler));
  assert_param(IS_TIM_IC_FILTER(sConfig->ICFilter));

  /* Process Locked */
  __HAL_LOCK(htim);

if (Channel == TIM_CHANNEL_3)
  {
    /* TI3 Configuration */
    assert_param(IS_TIM_CC3_INSTANCE(htim->Instance));

    TIM_TI3_SetConfig_optimized(htim->Instance,
                      sConfig->ICPolarity,
                      sConfig->ICSelection,
                      sConfig->ICFilter);

    /* Reset the IC3PSC Bits */
    htim->Instance->CCMR2 &= ~TIM_CCMR2_IC3PSC;

    /* Set the IC3PSC value */
    htim->Instance->CCMR2 |= sConfig->ICPrescaler;
  }
  else if (Channel == TIM_CHANNEL_4)
  {
    /* TI4 Configuration */
    assert_param(IS_TIM_CC4_INSTANCE(htim->Instance));

    TIM_TI4_SetConfig_optimized(htim->Instance,
                      sConfig->ICPolarity,
                      sConfig->ICSelection,
                      sConfig->ICFilter);

    /* Reset the IC4PSC Bits */
    htim->Instance->CCMR2 &= ~TIM_CCMR2_IC4PSC;

    /* Set the IC4PSC value */
    htim->Instance->CCMR2 |= (sConfig->ICPrescaler << 8U);
  }
  else
  {
    status = HAL_ERROR;
  }

  __HAL_UNLOCK(htim);

  return status;
}

/**
  * @brief  Timer Output Compare 3 configuration
  * @param  TIMx to select the TIM peripheral
  * @param  OC_Config The output configuration structure
  * @retval None
  */
static void TIM_OC3_SetConfig_optimized(TIM_TypeDef *TIMx, const TIM_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Get the TIMx CCER register value */
  tmpccer = TIMx->CCER;

  /* Disable the Channel 3: Reset the CC2E Bit */
  TIMx->CCER &= ~TIM_CCER_CC3E;

  /* Get the TIMx CR2 register value */
  tmpcr2 =  TIMx->CR2;

  /* Get the TIMx CCMR2 register value */
  tmpccmrx = TIMx->CCMR2;

  /* Reset the Output Compare mode and Capture/Compare selection Bits */
  tmpccmrx &= ~TIM_CCMR2_OC3M;
  tmpccmrx &= ~TIM_CCMR2_CC3S;
  /* Select the Output Compare Mode */
  tmpccmrx |= OC_Config->OCMode;

  /* Reset the Output Polarity level */
  tmpccer &= ~TIM_CCER_CC3P;
  /* Set the Output Compare Polarity */
  tmpccer |= (OC_Config->OCPolarity << 8U);

  if (IS_TIM_CCXN_INSTANCE(TIMx, TIM_CHANNEL_3))
  {
    assert_param(IS_TIM_OCN_POLARITY(OC_Config->OCNPolarity));

    /* Reset the Output N Polarity level */
    tmpccer &= ~TIM_CCER_CC3NP;
    /* Set the Output N Polarity */
    tmpccer |= (OC_Config->OCNPolarity << 8U);
    /* Reset the Output N State */
    tmpccer &= ~TIM_CCER_CC3NE;
  }

  if (IS_TIM_BREAK_INSTANCE(TIMx))
  {
    /* Check parameters */
    assert_param(IS_TIM_OCNIDLE_STATE(OC_Config->OCNIdleState));
    assert_param(IS_TIM_OCIDLE_STATE(OC_Config->OCIdleState));

    /* Reset the Output Compare and Output Compare N IDLE State */
    tmpcr2 &= ~TIM_CR2_OIS3;
    tmpcr2 &= ~TIM_CR2_OIS3N;
    /* Set the Output Idle state */
    tmpcr2 |= (OC_Config->OCIdleState << 4U);
    /* Set the Output N Idle state */
    tmpcr2 |= (OC_Config->OCNIdleState << 4U);
  }

  /* Write to TIMx CR2 */
  TIMx->CR2 = tmpcr2;

  /* Write to TIMx CCMR2 */
  TIMx->CCMR2 = tmpccmrx;

  /* Set the Capture Compare Register value */
  TIMx->CCR3 = OC_Config->Pulse;

  /* Write to TIMx CCER */
  TIMx->CCER = tmpccer;
}

/**
  * @brief  Timer Output Compare 4 configuration
  * @param  TIMx to select the TIM peripheral
  * @param  OC_Config The output configuration structure
  * @retval None
  */
static void TIM_OC4_SetConfig_optimized(TIM_TypeDef *TIMx, const TIM_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Get the TIMx CCER register value */
  tmpccer = TIMx->CCER;

  /* Disable the Channel 4: Reset the CC4E Bit */
  TIMx->CCER &= ~TIM_CCER_CC4E;

  /* Get the TIMx CR2 register value */
  tmpcr2 =  TIMx->CR2;

  /* Get the TIMx CCMR2 register value */
  tmpccmrx = TIMx->CCMR2;

  /* Reset the Output Compare mode and Capture/Compare selection Bits */
  tmpccmrx &= ~TIM_CCMR2_OC4M;
  tmpccmrx &= ~TIM_CCMR2_CC4S;

  /* Select the Output Compare Mode */
  tmpccmrx |= (OC_Config->OCMode << 8U);

  /* Reset the Output Polarity level */
  tmpccer &= ~TIM_CCER_CC4P;
  /* Set the Output Compare Polarity */
  tmpccer |= (OC_Config->OCPolarity << 12U);

  if (IS_TIM_BREAK_INSTANCE(TIMx))
  {
    /* Check parameters */
    assert_param(IS_TIM_OCIDLE_STATE(OC_Config->OCIdleState));

    /* Reset the Output Compare IDLE State */
    tmpcr2 &= ~TIM_CR2_OIS4;

    /* Set the Output Idle state */
    tmpcr2 |= (OC_Config->OCIdleState << 6U);
  }

  /* Write to TIMx CR2 */
  TIMx->CR2 = tmpcr2;

  /* Write to TIMx CCMR2 */
  TIMx->CCMR2 = tmpccmrx;

  /* Set the Capture Compare Register value */
  TIMx->CCR4 = OC_Config->Pulse;

  /* Write to TIMx CCER */
  TIMx->CCER = tmpccer;
}

/**
  * @brief  Initializes the TIM PWM  channels according to the specified
  *         parameters in the TIM_OC_InitTypeDef.
  * @param  htim TIM PWM handle
  * @param  sConfig TIM PWM configuration structure
  * @param  Channel TIM Channels to be configured
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  *            @arg TIM_CHANNEL_5: TIM Channel 5 selected
  *            @arg TIM_CHANNEL_6: TIM Channel 6 selected
  * @retval HAL status
  */
static HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel_optimized(TIM_HandleTypeDef *htim,
                                            const TIM_OC_InitTypeDef *sConfig,
                                            uint32_t Channel)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Check the parameters */
  assert_param(IS_TIM_CHANNELS(Channel));
  assert_param(IS_TIM_PWM_MODE(sConfig->OCMode));
  assert_param(IS_TIM_OC_POLARITY(sConfig->OCPolarity));
  assert_param(IS_TIM_FAST_STATE(sConfig->OCFastMode));

  /* Process Locked */
  __HAL_LOCK(htim);

  switch (Channel)
  {
    case TIM_CHANNEL_3:
    {
      /* Check the parameters */
      assert_param(IS_TIM_CC3_INSTANCE(htim->Instance));

      /* Configure the Channel 3 in PWM mode */
      TIM_OC3_SetConfig_optimized(htim->Instance, sConfig);

      /* Set the Preload enable bit for channel3 */
      htim->Instance->CCMR2 |= TIM_CCMR2_OC3PE;

      /* Configure the Output Fast mode */
      htim->Instance->CCMR2 &= ~TIM_CCMR2_OC3FE;
      htim->Instance->CCMR2 |= sConfig->OCFastMode;
      break;
    }
    case TIM_CHANNEL_4:
    {
      /* Check the parameters */
      assert_param(IS_TIM_CC4_INSTANCE(htim->Instance));

      /* Configure the Channel 4 in PWM mode */
      TIM_OC4_SetConfig_optimized(htim->Instance, sConfig);

      /* Set the Preload enable bit for channel4 */
      htim->Instance->CCMR2 |= TIM_CCMR2_OC4PE;

      /* Configure the Output Fast mode */
      htim->Instance->CCMR2 &= ~TIM_CCMR2_OC4FE;
      htim->Instance->CCMR2 |= sConfig->OCFastMode << 8U;
      break;
    }
    default:
      status = HAL_ERROR;
      break;
  }

  __HAL_UNLOCK(htim);

  return status;
}

/**
  * @brief  Starts the PWM signal generation.
  * @param  htim TIM handle
  * @param  Channel TIM Channels to be enabled
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  *            @arg TIM_CHANNEL_5: TIM Channel 5 selected
  *            @arg TIM_CHANNEL_6: TIM Channel 6 selected
  * @retval HAL status
  */
static HAL_StatusTypeDef HAL_TIM_PWM_Start_optimized(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  uint32_t tmpsmcr;

  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

  /* Check the TIM channel state */
  if (TIM_CHANNEL_STATE_GET(htim, Channel) != HAL_TIM_CHANNEL_STATE_READY)
  {
    return HAL_ERROR;
  }

  /* Set the TIM channel state */
  TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);

  /* Enable the Capture compare channel */
  TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);

  if (IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET)
  {
    /* Enable the main output */
    __HAL_TIM_MOE_ENABLE(htim);
  }

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TIM_SLAVE_INSTANCE(htim->Instance))
  {
    tmpsmcr = htim->Instance->SMCR & TIM_SMCR_SMS;
    if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __HAL_TIM_ENABLE(htim);
      //htim->Instance->CR1|=(TIM_CR1_CEN);
    }
  }
  else
  {
    __HAL_TIM_ENABLE(htim);
    //htim->Instance->CR1|=(TIM_CR1_CEN);
  }

  /* Return function status */
  return HAL_OK;
}



#endif /* INC_HELPER_H_ */
