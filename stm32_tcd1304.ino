#define CCDSize 3694
#define CCD_fm 2000000
#define SH_delay 12
#define ICG_delay 11
#define fm_delay 3

#define RESERVED_MASK           (uint32_t)0x0F7D0F7D
#define HIGH_ISR_MASK           (uint32_t)0x20000000
#define DMA_IT_TCIF0            ((uint32_t)0x10008020)
#define TRANSFER_IT_ENABLE_MASK (uint32_t)(DMA_SxCR_TCIE | DMA_SxCR_HTIE | \
                                           DMA_SxCR_TEIE | DMA_SxCR_DMEIE)

uint32_t SH_period = 25;
uint32_t ICG_period = 500000;

int apb1_freq;

volatile uint8_t pulse_counter = 0;
volatile  uint8_t CCD_flushed;
volatile  uint8_t transmit_data_flag;


volatile uint16_t aTxBuffer[CCDSize];

extern "C" {

  void TIM5_IRQHandler(void)
  {
     if(TIM5->SR & TIM_SR_UIF)
    { 
      /* Clear TIM5 update interrupt */
      TIM5->SR = ~TIM_SR_UIF;
      if (pulse_counter == 6)
      {
        /* Restart TIM4 as this gets the ADC running again */
        TIM4->CR1 |= TIM_CR1_CEN;
      }
      else if (pulse_counter == 3)
      {
        CCD_flushed = 1;
      }
      pulse_counter++;
      /* prevent overflow */
      if (pulse_counter > 10)
        pulse_counter = 10;
  
      /* Flash the led to the beat of ICG */
      //GPIOG->ODR ^= GPIO_PIN_14;
    }
  
  }

  void DMA2_Stream0_IRQHandler(void)
  {
        DMA2->LIFCR = (uint32_t)(DMA_IT_TCIF0 & RESERVED_MASK);
    
            /* Stop TIM4 and thus the ADC */
        TIM4->CR1 &= (uint16_t)~TIM_CR1_CEN;

        /* Set the transmit_data_flag */
        transmit_data_flag = 1;
     GPIOG->ODR ^= GPIO_PIN_14;
  }
  
}


void get_Timer_clocks(void)
{
  /* Get the apb-prescalers */
  int apb1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;

  /* APBx prescaler table */
  int apb2x[8] = {1,0,0,0,2,4,8,16};

  /* Calculate the timer clocks */
  apb1_freq = SystemCoreClock / (apb2x[apb1]*(apb2x[apb1]==1) + apb2x[apb1]/2*(apb2x[apb1]!=1));
}

void GPIO_conf(void){

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  GPIO_InitTypeDef     GPIO_InitStructure;
  
  /* fM GPIO Configuration: TIM3 CH4 (PC9) */ 
  GPIO_InitStructure.Pin = GPIO_PIN_9;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADC GPIO Configuration: TIM4 CH4 (PB9) */ 
  GPIO_InitStructure.Pin = GPIO_PIN_9;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* SH GPIO Configuration: TIM2 CH3 (PB10) */
  GPIO_InitStructure.Pin = GPIO_PIN_10;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* ICG GPIO Configuration: TIM5 CH4 (PA3) */
  GPIO_InitStructure.Pin = GPIO_PIN_3;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* ADC GPIO Configuration: ADC1 CH13 (PC3) */
  GPIO_InitStructure.Pin = GPIO_PIN_3;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* LEDS INIT */
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/* fM is served by TIM3 on PC9 */
void TIM_CCD_fM_conf(void)
{
   __HAL_RCC_TIM3_CLK_ENABLE();

  /* Select the Counter Mode */
  TIM3->CR1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS));
  TIM3->CR1 |= (uint32_t)TIM_COUNTERMODE_UP;

  /* Set the clock division */
  TIM3->CR1 &=  (uint16_t)(~TIM_CR1_CKD);
  TIM3->CR1 |= (uint32_t)TIM_CLOCKDIVISION_DIV1;

  /* Set the Autoreload value */
  TIM3->ARR = apb1_freq / CCD_fm - 1;
 
  /* Set the Prescaler value */
  TIM3->PSC = 0;
    
  /* Generate an update event to reload the Prescaler 
     and the repetition counter(only for TIM1 and TIM8) value immediately */
  TIM3->EGR = TIM_EVENTSOURCE_UPDATE;    
  
  /* Disable the Channel 4: Reset the CC4E Bit */
  TIM3->CCER &= (uint16_t)~TIM_CCER_CC4E;
    
  /* Reset the Output Compare mode and Capture/Compare selection Bits */
  TIM3->CCMR2 &= (uint16_t)~TIM_CCMR2_OC4M;
  TIM3->CCMR2 &= (uint16_t)~TIM_CCMR2_CC4S;
  
  /* Select the Output Compare Mode */
  TIM3->CCMR2 |= (uint16_t)(TIM_OCMODE_PWM1 << 8);
  
  /* Reset the Output Polarity level */
  TIM3->CCER &= (uint16_t)~TIM_CCER_CC4P;
  /* Set the Output Compare Polarity */
  TIM3->CCER |= (uint16_t)(TIM_OCPOLARITY_HIGH << 12);
  
  /* Set the Output State */
  TIM3->CCER |= (uint16_t)(TIM_OUTPUTSTATE_ENABLE << 12);
    
  /* Set the Capture Compare Register value */
  TIM3->CCR4 = apb1_freq / (2*CCD_fm);

  /* Reset the OC4PE Bit */
  TIM3->CCMR2 &= (~TIM_CCMR2_OC4PE);

  /* Enable or Disable the Output Compare Preload feature */
  TIM3->CCMR2 |= TIM_CCMR2_OC4PE;

  TIM3->CR1 |= TIM_AUTORELOAD_PRELOAD_ENABLE;

  TIM3->CR1 |= TIM_CR1_CEN;
}

/* ADC is paced by TIM4 (optional output on PB9) */
void TIM_ADC_conf(void)
{
   __HAL_RCC_TIM4_CLK_ENABLE();

  /* Select the Counter Mode */
  TIM4->CR1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS));
  TIM4->CR1 |= (uint32_t)TIM_COUNTERMODE_UP;

  /* Set the clock division */
  TIM4->CR1 &=  (uint16_t)(~TIM_CR1_CKD);
  TIM4->CR1 |= (uint32_t)TIM_CLOCKDIVISION_DIV1;

  /* Set the Autoreload value */
  TIM4->ARR = 4 * apb1_freq / CCD_fm - 1;
 
  /* Set the Prescaler value */
  TIM4->PSC = 0;
    
  /* Generate an update event to reload the Prescaler 
     and the repetition counter(only for TIM1 and TIM8) value immediately */
  TIM4->EGR = TIM_EVENTSOURCE_UPDATE;    
  
  /* Disable the Channel 4: Reset the CC4E Bit */
  TIM4->CCER &= (uint16_t)~TIM_CCER_CC4E;
    
  /* Reset the Output Compare mode and Capture/Compare selection Bits */
  TIM4->CCMR2 &= (uint16_t)~TIM_CCMR2_OC4M;
  TIM4->CCMR2 &= (uint16_t)~TIM_CCMR2_CC4S;
  
  /* Select the Output Compare Mode */
  TIM4->CCMR2 |= (uint16_t)(TIM_OCMODE_PWM1 << 8);
  
  /* Reset the Output Polarity level */
  TIM4->CCER &= (uint16_t)~TIM_CCER_CC4P;
  /* Set the Output Compare Polarity */
  TIM4->CCER |= (uint16_t)(TIM_OCPOLARITY_HIGH << 12);
  
  /* Set the Output State */
  TIM4->CCER |= (uint16_t)(TIM_OUTPUTSTATE_ENABLE << 12);
    
  /* Set the Capture Compare Register value */
  TIM4->CCR4 = apb1_freq / (2*CCD_fm);

  /* Reset the OC4PE Bit */
  TIM4->CCMR2 &= (~TIM_CCMR2_OC4PE);

  /* Enable or Disable the Output Compare Preload feature */
  TIM4->CCMR2 |= TIM_CCMR2_OC4PE;

  TIM4->CR1 |= TIM_AUTORELOAD_PRELOAD_ENABLE;

  //TIM4->CR1 |= TIM_CR1_CEN; 
}


void TIM_ICG_SH_conf(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM5_CLK_ENABLE();

  /*  Time base configuration */
  /*  The prescaler is set to period of TIM3 (apb1_freq / CCD_fm - 1),
    so TIM2 and TIM5's clocks are equal to the period of TIM3.
    Some numbers to consider for the period of the timers:  
      ICG (TIM5): t_read = Period / CCD_fm MHz
      SH (TIM2): t_int = Period / CCD_fm MHz  
    eg. integration time is:
      t_int = 20 / 2 MHz = 10 µs */

  /* Common for TIM2 & TIM5 */

  /* Select the Counter Mode */
  TIM5->CR1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS));
  TIM5->CR1 |= (uint32_t)TIM_COUNTERMODE_UP;

  /* Set the clock division */
  TIM5->CR1 &=  (uint16_t)(~TIM_CR1_CKD);
  TIM5->CR1 |= (uint32_t)TIM_CLOCKDIVISION_DIV1;

  /* Set the Autoreload value */
  TIM5->ARR = ICG_period - 1; 
 
  /* Set the Prescaler value */
  TIM5->PSC = apb1_freq / CCD_fm - 1;  
    
  /* Generate an update event to reload the Prescaler 
     and the repetition counter(only for TIM1 and TIM8) value immediately */
  TIM5->EGR = TIM_EVENTSOURCE_UPDATE;    

    /* Disable the Channel 4: Reset the CC4E Bit */
  TIM5->CCER &= (uint16_t)~TIM_CCER_CC4E;
    
  /* Reset the Output Compare mode and Capture/Compare selection Bits */
  TIM5->CCMR2 &= (uint16_t)~TIM_CCMR2_OC4M;
  TIM5->CCMR2 &= (uint16_t)~TIM_CCMR2_CC4S;
  
  /* Select the Output Compare Mode */
  TIM5->CCMR2 |= (uint16_t)(TIM_OCMODE_PWM1 << 8);
  
  /* Reset the Output Polarity level */
  TIM5->CCER &= (uint16_t)~TIM_CCER_CC4P;
  /* Set the Output Compare Polarity */
  TIM5->CCER |= (uint16_t)(TIM_OCPOLARITY_LOW << 12);
  
  /* Set the Output State */
  TIM5->CCER |= (uint16_t)(TIM_OUTPUTSTATE_ENABLE << 12);
    
  /* Set the Capture Compare Register value */
  TIM5->CCR4 = (5 * CCD_fm) / 1000000;

  /* Reset the OC4PE Bit */
  TIM5->CCMR2 &= (~TIM_CCMR2_OC4PE);

  /* Enable or Disable the Output Compare Preload feature */
  TIM5->CCMR2 |= TIM_CCMR2_OC4PE;

  TIM5->CR1 |= TIM_AUTORELOAD_PRELOAD_ENABLE;

  TIM5->SR = (uint16_t)~TIM_FLAG_UPDATE;

  TIM5->DIER |= TIM_IT_UPDATE;
  
  TIM5->CR1 |= TIM_CR1_CEN;

  /* Select the Counter Mode */
  TIM2->CR1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS));
  TIM2->CR1 |= (uint32_t)TIM_COUNTERMODE_UP;

  /* Set the clock division */
  TIM2->CR1 &=  (uint16_t)(~TIM_CR1_CKD);
  TIM2->CR1 |= (uint32_t)TIM_CLOCKDIVISION_DIV1;

  /* Set the Autoreload value */
  TIM2->ARR = SH_period - 1;  
 
  /* Set the Prescaler value */
  TIM2->PSC = apb1_freq / CCD_fm - 1;  
    
  /* Generate an update event to reload the Prescaler 
     and the repetition counter(only for TIM1 and TIM8) value immediately */
  TIM2->EGR = TIM_EVENTSOURCE_UPDATE; 

  /* Disable the Channel 4: Reset the CC4E Bit */
  TIM2->CCER &= (uint16_t)~TIM_CCER_CC3E;
    
  /* Reset the Output Compare mode and Capture/Compare selection Bits */
  TIM2->CCMR2 &= (uint16_t)~TIM_CCMR2_OC3M;
  TIM2->CCMR2 &= (uint16_t)~TIM_CCMR2_CC3S;
  
  /* Select the Output Compare Mode */
  TIM2->CCMR2 |= TIM_OCMODE_PWM1;
  
  /* Reset the Output Polarity level */
  TIM2->CCER &= (uint16_t)~TIM_CCER_CC3P;
  /* Set the Output Compare Polarity */
  TIM2->CCER |= (uint16_t)(TIM_OCPOLARITY_HIGH << 8);
  
  /* Set the Output State */
  TIM2->CCER |= (uint16_t)(TIM_OUTPUTSTATE_ENABLE << 8);
    
  /* Set the Capture Compare Register value */
  TIM2->CCR3 = (2 * CCD_fm) / 1000000;

  /* Reset the OC4PE Bit */
  TIM2->CCMR2 &= (~TIM_CCMR2_OC3PE);

  /* Enable or Disable the Output Compare Preload feature */
  TIM2->CCMR2 |= TIM_CCMR2_OC3PE;

  TIM2->CR1 |= TIM_AUTORELOAD_PRELOAD_ENABLE;

  TIM2->CR1 |= TIM_CR1_CEN;
  
  /*  Set parameters for TIM5 PWM1 Mode configuration: Channel2 */
  /*  The duty cycle should be 5 µs for ICG, so the pulse is:
    pulse = 5 µs * CCD_fm MHz
  Of course this is only accurate when CCD_fM in MHz is an integer.
  Change the Polarity to High if 74HC04 is in place */

  /*  Set parameters for TIM2 PWM1 Mode configuration: Channel3 */
  /*  The duty cycle should be 2 µs for SH, so the pulse is:
    pulse = 2 µs * CCD_fm MHz
  Of course this is only accurate when CCD_fM in MHz is an integer.
  Change the Polarity to Low if 74HC04 is in place */


/*  Set counters close to expiration, as the integration times may be very long. 
  (For example: with an ICG-period of 300s we'd have to wait 600s for two ICG-
  pulses if we don't cut the first one short.)
  The SH-period is slightly delayed to comply with the CCD's timing requirements. */
  TIM5->CNT = ICG_period - ICG_delay;
  TIM2->CNT = SH_period - SH_delay;
  TIM3->CNT = fm_delay;
}

/* ADC1 - Input on Ch13 (PC3)  - Triggered by TIM4 Ch4 - DMA2 Ch0 Stream0 */
void ADC1_conf() {
  
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  DMA2_Stream0->CR &= ((uint32_t)~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | \
                         DMA_SxCR_PL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | \
                         DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC | \
                         DMA_SxCR_DIR));
  
  DMA2_Stream0->CR |= DMA_CHANNEL_0 | DMA_PERIPH_TO_MEMORY | \
                      DMA_PINC_DISABLE | DMA_PINC_DISABLE | \
                      DMA_PDATAALIGN_HALFWORD | DMA_MDATAALIGN_HALFWORD | \
                      DMA_CIRCULAR | DMA_PRIORITY_HIGH | \
                      DMA_MBURST_SINGLE | DMA_PBURST_SINGLE;
                      
  DMA2_Stream0->FCR  &= (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);

  DMA2_Stream0->FCR |= DMA_FIFOMODE_DISABLE | DMA_FIFO_THRESHOLD_HALFFULL;

  /*------------------------- DMAy Streamx NDTR Configuration ----------------*/
  /* Write to DMAy Streamx NDTR register */
  DMA2_Stream0->NDTR = CCDSize;

  /*------------------------- DMAy Streamx PAR Configuration -----------------*/
  /* Write to DMAy Streamx PAR */
  DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;

  /*------------------------- DMAy Streamx M0AR Configuration ----------------*/
  /* Write to DMAy Streamx M0AR */
  DMA2_Stream0->M0AR = (uint32_t)&aTxBuffer;

  DMA2_Stream0->CR |= (uint32_t)DMA_SxCR_EN;
                      
  DMA2_Stream0->CR |= (uint32_t)(DMA_IT_TC & TRANSFER_IT_ENABLE_MASK);


 ADC->CCR &= ((uint32_t)0xFFFC30E0);

  ADC->CCR |= (uint32_t)0x00000000 | (uint32_t)0x00000000 | (uint32_t)0x00000000 | (uint32_t)0x00000000;

  ADC1->CR1 &= (uint32_t)0xFCFFFEFF;

  ADC1->CR1 |= (uint32_t)(((uint32_t)0x00000000 << 8) | \
                                   ADC_RESOLUTION_12B);

  ADC1->CR2 &= ((uint32_t)0xC0FFF7FD);

  ADC1->CR2 |= (uint32_t)(ADC_DATAALIGN_RIGHT | ADC_EXTERNALTRIGCONV_T4_CC4 | ADC_EXTERNALTRIGCONVEDGE_RISING | ((uint32_t)0x00000000 << 1));

  ADC1->SQR1 &= ((uint32_t)0xFF0FFFFF);

  uint8_t tmpreg2 = 0;
  tmpreg2 |= (uint8_t)(1 - (uint8_t)1);
  ADC1->SQR1 |= ((uint32_t)tmpreg2 << 20);

  ADC1->SMPR2 &= ~((uint32_t)0x00000007 << (3 * ADC_CHANNEL_13));
  
  ADC1->SMPR2 |= (uint32_t)ADC_SAMPLETIME_15CYCLES << (3 * ADC_CHANNEL_13);

  ADC1->SQR3 &= ~((uint32_t)0x0000001F << (5 * (1 - 1)));

  ADC1->SQR3 |= (uint32_t)ADC_CHANNEL_13 << (5 * (1 - 1));

  ADC1->CR2 |= (uint32_t)ADC_CR2_DDS;

  ADC1->CR2 |= (uint32_t)ADC_CR2_DMA;

  ADC1->CR2 |= (uint32_t)ADC_CR2_ADON;
  
}

void NVIC_conf(void)
{
  NVIC_SetPriority(TIM5_IRQn, 0);
  NVIC_EnableIRQ(TIM5_IRQn);

  NVIC_SetPriority(DMA2_Stream0_IRQn, 1);
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void flush_CCD()
{
  /* Set exposure very low */
  ICG_period = 15000;
  SH_period = 25;

  /*  Disable ICG (TIM5) and SH (TIM2) before reconfiguring*/
  TIM2->CR1 &= (uint16_t)~TIM_CR1_CEN;
  TIM5->CR1 &= (uint16_t)~TIM_CR1_CEN;

  /*  Reset flags and counters */
  CCD_flushed = 0;
  pulse_counter = 0;
  ADC1_conf();
  //Serial.println(pulse_counter);
  /*  Reconfigure TIM2 and TIM5 */
  TIM_ICG_SH_conf();
  //Serial.println(pulse_counter);
  /*  Block until CCD is properly flushed */
  while(CCD_flushed == 0);

}


void setup() {
  Serial.begin(115200);
  GPIO_conf();
  get_Timer_clocks();
  NVIC_conf();
  TIM_CCD_fM_conf();
  TIM_ADC_conf();
  ADC1_conf();
  TIM_ICG_SH_conf();
}

void loop() {
 if (transmit_data_flag) {
    transmit_data_flag = 0;
    for(int i = 0; i < 3694; i++){
      Serial.println(String(aTxBuffer[i], HEX));
    }
    flush_CCD();
 }

 
 //Serial.println(pulse_counter);
}
