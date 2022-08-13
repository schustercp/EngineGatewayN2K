/*! @name PLL_AUDIO_NUM - Numerator of Audio PLL Fractional Loop Divider Register */
/*! @{ */
#define CCM_ANALOG_PLL_AUDIO_NUM_A_MASK          (0x3FFFFFFFU)
#define CCM_ANALOG_PLL_AUDIO_NUM_A_SHIFT         (0U)
#define CCM_ANALOG_PLL_AUDIO_NUM_A(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_NUM_A_SHIFT)) & CCM_ANALOG_PLL_AUDIO_NUM_A_MASK)
/*! @} */

/*! @name PLL_AUDIO_DENOM - Denominator of Audio PLL Fractional Loop Divider Register */
/*! @{ */
#define CCM_ANALOG_PLL_AUDIO_DENOM_B_MASK        (0x3FFFFFFFU)
#define CCM_ANALOG_PLL_AUDIO_DENOM_B_SHIFT       (0U)
#define CCM_ANALOG_PLL_AUDIO_DENOM_B(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_DENOM_B_SHIFT)) & CCM_ANALOG_PLL_AUDIO_DENOM_B_MASK)

#define CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
//#define CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_MASK)

#define CCM_ANALOG_PLL_AUDIO_DIV_SELECT_MASK     (0x7FU)
#define CCM_ANALOG_PLL_AUDIO_DIV_SELECT_SHIFT    (0U)
//#define CCM_ANALOG_PLL_AUDIO_DIV_SELECT(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_AUDIO_DIV_SELECT_MASK)

#define CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_MASK (0x180000U)
#define CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_SHIFT (19U)
/*! POST_DIV_SELECT
 *  0b00..Divide by 4.
 *  0b01..Divide by 2.
 *  0b10..Divide by 1.
 *  0b11..Reserved
 */
//#define CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_MASK)

#define CCM_ANALOG_PLL_AUDIO_DIV_SELECT_MASK     (0x7FU)
#define CCM_ANALOG_PLL_AUDIO_POWERDOWN_MASK      (0x1000U)
#define CCM_ANALOG_PLL_AUDIO_ENABLE_MASK         (0x2000U)
#define CCM_ANALOG_MISC2_AUDIO_DIV_MSB_MASK      (0x800000U)
#define CCM_ANALOG_MISC2_AUDIO_DIV_LSB_MASK      (0x8000U)
#define CCM_ANALOG_MISC2_AUDIO_DIV_MSB_MASK      (0x800000U)
#define CCM_ANALOG_PLL_AUDIO_LOCK_MASK           (0x80000000U)
#define CCM_ANALOG_PLL_AUDIO_BYPASS_MASK         (0x10000U)

/*! @brief Define the SAI bus type */
typedef enum _sai_protocol
{
    kSAI_BusLeftJustified = 0x0U, /*!< Uses left justified format.*/
    kSAI_BusRightJustified,       /*!< Uses right justified format. */
    kSAI_BusI2S,                  /*!< Uses I2S format. */
    kSAI_BusPCMA,                 /*!< Uses I2S PCM A format.*/
    kSAI_BusPCMB                  /*!< Uses I2S PCM B format. */
} sai_protocol_t;

typedef enum _sai_sync_mode
{
    kSAI_ModeAsync = 0x0U,    /*!< Asynchronous mode */
    kSAI_ModeSync,            /*!< Synchronous mode (with receiver or transmit) */
    kSAI_ModeSyncWithOtherTx, /*!< Synchronous with another SAI transmit */
    kSAI_ModeSyncWithOtherRx  /*!< Synchronous with another SAI receiver */
} sai_sync_mode_t;

typedef enum _sai_mclk_source
{
    kSAI_MclkSourceSysclk = 0x0U, /*!< Master clock from the system clock */
    kSAI_MclkSourceSelect1,       /*!< Master clock from source 1 */
    kSAI_MclkSourceSelect2,       /*!< Master clock from source 2 */
    kSAI_MclkSourceSelect3        /*!< Master clock from source 3 */
} sai_mclk_source_t;

/*! @brief Bit clock source */
typedef enum _sai_bclk_source
{
    kSAI_BclkSourceBusclk = 0x0U, /*!< Bit clock using bus clock */
    /* General device bit source definition */
    kSAI_BclkSourceMclkOption1 = 0x1U, /*!< Bit clock MCLK option 1 */
    kSAI_BclkSourceMclkOption2 = 0x2U, /*!< Bit clock MCLK option2  */
    kSAI_BclkSourceMclkOption3 = 0x3U, /*!< Bit clock MCLK option3 */
    /* Kinetis device bit clock source definition */
    kSAI_BclkSourceMclkDiv = 0x1U,   /*!< Bit clock using master clock divider */
    kSAI_BclkSourceOtherSai0 = 0x2U, /*!< Bit clock from other SAI device  */
    kSAI_BclkSourceOtherSai1 = 0x3U  /*!< Bit clock from other SAI device */
} sai_bclk_source_t;

typedef enum _sai_master_slave
{
    kSAI_Master = 0x0U, /*!< Master mode */
    kSAI_Slave = 0x1U   /*!< Slave mode */
} sai_master_slave_t;

typedef struct _sai_config
{
    sai_protocol_t protocol;  /*!< Audio bus protocol in SAI */
    sai_sync_mode_t syncMode; /*!< SAI sync mode, control Tx/Rx clock sync */
    bool mclkOutputEnable; /*!< Master clock output enable, true means master clock divider enabled */
    sai_mclk_source_t mclkSource; /*!< Master Clock source */
    sai_bclk_source_t bclkSource;   /*!< Bit Clock source */
    sai_master_slave_t masterSlave; /*!< Master or slave */
} sai_config_t;



/*! @brief PLL configuration for AUDIO and VIDEO */
typedef struct _clock_audio_pll_config
{
    uint8_t loopDivider;  /*!< PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    uint8_t postDivider;  /*!< Divider after the PLL, should only be 1, 2, 4, 8, 16. */
    uint32_t numerator;   /*!< 30 bit numerator of fractional loop divider.*/
    uint32_t denominator; /*!< 30 bit denominator of fractional loop divider */
    uint8_t src;          /*!< Pll clock source, reference _clock_pll_clk_src */
} clock_audio_pll_config_t;

/*!
 * brief Initializes the Audio PLL.
 *
 * This function initializes the Audio PLL with specific settings
 *
 * param config Configuration to set to PLL.
 */
void CLOCK_InitAudioPll(const clock_audio_pll_config_t *config)
{
    uint32_t pllAudio;
    uint32_t misc2 = 0;

    /* Bypass PLL first */
    CCM_ANALOG_PLL_AUDIO = (CCM_ANALOG_PLL_AUDIO & (~CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_MASK)) |
                            CCM_ANALOG_PLL_AUDIO_BYPASS_MASK | CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC(config->src);

    CCM_ANALOG_PLL_AUDIO_NUM = CCM_ANALOG_PLL_AUDIO_NUM_A(config->numerator);
    CCM_ANALOG_PLL_AUDIO_DENOM = CCM_ANALOG_PLL_AUDIO_DENOM_B(config->denominator);

    /*
     * Set post divider:
     *
     * ------------------------------------------------------------------------
     * | config->postDivider | PLL_AUDIO[POST_DIV_SELECT]  | MISC2[AUDIO_DIV] |
     * ------------------------------------------------------------------------
     * |         1           |            2                |        0         |
     * ------------------------------------------------------------------------
     * |         2           |            1                |        0         |
     * ------------------------------------------------------------------------
     * |         4           |            2                |        3         |
     * ------------------------------------------------------------------------
     * |         8           |            1                |        3         |
     * ------------------------------------------------------------------------
     * |         16          |            0                |        3         |
     * ------------------------------------------------------------------------
     */
    pllAudio =
        (CCM_ANALOG->PLL_AUDIO & (~(CCM_ANALOG_PLL_AUDIO_DIV_SELECT_MASK | CCM_ANALOG_PLL_AUDIO_POWERDOWN_MASK))) |
        CCM_ANALOG_PLL_AUDIO_ENABLE_MASK | CCM_ANALOG_PLL_AUDIO_DIV_SELECT(config->loopDivider);

    switch (config->postDivider)
    {
        case 16:
            pllAudio |= CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(0);
            misc2 = CCM_ANALOG_MISC2_AUDIO_DIV_MSB_MASK | CCM_ANALOG_MISC2_AUDIO_DIV_LSB_MASK;
            break;

        case 8:
            pllAudio |= CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(1);
            misc2 = CCM_ANALOG_MISC2_AUDIO_DIV_MSB_MASK | CCM_ANALOG_MISC2_AUDIO_DIV_LSB_MASK;
            break;

        case 4:
            pllAudio |= CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(2);
            misc2 = CCM_ANALOG_MISC2_AUDIO_DIV_MSB_MASK | CCM_ANALOG_MISC2_AUDIO_DIV_LSB_MASK;
            break;

        case 2:
            pllAudio |= CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(1);
            break;

        default:
            pllAudio |= CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(2);
            break;
    }

    CCM_ANALOG_MISC2 =
        (CCM_ANALOG_MISC2 & ~(CCM_ANALOG_MISC2_AUDIO_DIV_LSB_MASK | CCM_ANALOG_MISC2_AUDIO_DIV_MSB_MASK)) | misc2;

    CCM_ANALOG_PLL_AUDIO = pllAudio;

    while ((CCM_ANALOG_PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_LOCK_MASK) == 0)
    {
    }

    /* Disable Bypass */
    CCM_ANALOG_PLL_AUDIO &= ~CCM_ANALOG_PLL_AUDIO_BYPASS_MASK;
}
