/*****************************************************************************
   FILE:          gnssapp.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 GPS library and application
------------------------------------------------------------------------------
   DESCRIPTION:   Module to run and test STA8090 GPS library
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Created by : Fulvio boggia
           on : 2007.07.25
*****************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/

// general
#include "macros.h"
#include "fixpoint.h"
#include "clibs.h"

// LLD for STA8090
#include "lld_gpio.h"

// OS related
#include "gpOS.h"

// GNSS library related
#include "gnss_const.h"
#include "gnss_defs.h"
#include "gnss_api.h"
#include "gnss_debug.h"
#include "gpsconfig.h"

// GNSS MSG related
#include "gnss_msg.h"
#include "gnss_msg_queue.h"

#if defined( NVM_NOR ) || defined( NVM_SQI )
#include "gps_nvm_swap_mgr.h"
#endif
#include "gps_nvm.h"

#include "platform.h"
#include "sw_config.h"
#include "frontend.h"

#include "gnssapp.h"
#include "gnssapp_plugins.h"

// Services related
#include "svc_mcu.h"
#include "svc_adc.h"
#include "svc_ssp.h"
#include "svc_i2c.h"
#include "svc_gpio.h"
#include "svc_pwr.h"
#include "svc_fsw.h"
#include "svc_can.h"

// Modules related
#include "in_out.h"
#include "gnss_events.h"
#include "nmea.h"

#if defined( STBIN_LINKED )
#include "stbin.h"
#endif

#if defined( SDLOG_LINKED )
#include "sdlog.h"
#endif

#if defined( SW_CONFIG_PRIVATE_BLOCK )
#include "sm_sensors_api.h"
#include "sw_config_private.h"
#endif
#if defined( DR_CODE_LINKED )
#include "dr_api.h"
#endif

#if defined( FREERTOS )
#include "FR_timei.h"
#endif

#include "lvd_mgmt.h"
#include "antenna_sensing.h"
#include "shutdn_ctrl.h"
/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/**< Version ID for NVM data */
#define NVM_VID_GLONASS             0x1U

#define NVM_VID_GALILEO             0x2U

#define NVM_VID_COMPASS             0x4U

#if defined( ST_AGPS )
#define NVM_VID_STAGPS              0x10U
#else
#define NVM_VID_STAGPS              0x0U
#endif

#if defined(DR_CODE_LINKED)
#define NVM_VID_DR                  0x20U
#else
#define NVM_VID_DR                  0x0U
#endif

/**< version for backup data */
#define NVM_VERSION_ID              (0x84100000U | NVM_VID_DR | NVM_VID_STAGPS | NVM_VID_COMPASS | NVM_VID_GALILEO | NVM_VID_GLONASS)

/**< suffix for all versions */
#define GNSSAPP_VERSION_SUFFIX      MCR_MACROTOSTRING(VERSION_SUFFIX)

/**< product versions  */
#define GNSSAPP_VERSION_STRING      MCR_MACROTOSTRING(VERSION_SDK) GNSSAPP_VERSION_SUFFIX
#if defined( VERSION_BINARY )
#define BINIMG_VERSION_STRING       MCR_MACROTOSTRING(VERSION_BINARY) GNSSAPP_VERSION_SUFFIX
#endif
#define GNSSAPP_VERSION_SIZE        (32U)

/* NVM memory sizes
 * Note: they are sized to 128kB for flash devices as they need to be aligned to the highest possible sector
 *       size available on the market.
 */

#if defined( NVM_NOR)
#define NVM_MEM_SIZE          (1 * 128 * 1024)  /**< NVM region size in NOR configuration*/
#elif defined( NVM_SQI)
#define NVM_MEM_SIZE          (1 * 128 * 1024)  /**< NVM region size in SQI configuration*/
#else
#define NVM_MEM_SIZE          (32 * 1024)       /**< NVM region size in backup configuration */
#endif

/* SAT list size: different lists in the GNSS lib are sized according to the number of potential tracked satellites */
/* The following constant cannot exceed 64, and should be sized according to the number of constellation supported by the build */

#define GNSSLIB_SAT_LIST_SIZE     VISIBLE_MAX_NUM_OF_SATS

#define GNSSAPP_MTU_CLOCK         24552U

#define DEFAULT_AMQ_MSG_SIZE      2U

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef struct gnssapp_handler_s
{
  gpOS_partition_t *          fast_part;

  gnss_debug_writeout_t       debug_output;
  gnss_debug_inout_t          debug_input;

  nmea_inout_t                nmea_input;
  nmea_inout_t                nmea_output;

  nmea_inout_t                rtcm_input;

#if defined( SDLOG_LINKED )
  sdlog_file_t *              debug_file_hdlr;
  sdlog_file_t *              nmea_file_hdlr;
#endif
} gnssapp_handler_t;

typedef struct gnssapp_lowpow_s
{
  gnss_app_lowpow_standby_type_t  Standby;       /**< Standby activated */
  gnss_low_power_cyclic_mode_t    cyclic;        /**< backup of low power config */
  gnss_low_power_periodic_mode_t  periodic;      /**< backup of low power config */
}gnssapp_lowpow_t;

typedef enum
{
  GNSSAPP_LOW_POWER_INIT,
  GNSSAPP_LOW_POWER_UPDATE
}gnss_app_lowpow_setup_type_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

// Version string
const tChar gnssapp_ver[GNSSAPP_VERSION_SIZE]         = MCR_VERSION_STRING( "GPSAPP", GNSSAPP_VERSION_STRING );
#if defined( VERSION_BINARY )
const tChar gnssapp_binimg_ver[GNSSAPP_VERSION_SIZE]  = MCR_VERSION_STRING( "BINIMG", BINIMG_VERSION_STRING );
#endif

tUInt spm;
tUInt spm_configuration;

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static gnssapp_handler_t *gnssapp_handler;
static gnssapp_startup_time_t gnssapp_startup_time;
static gpOS_semaphore_t *gnssapp_startup_time_sem;

static tUInt tcxo_config_selector = 0U;

extern tU8 adc_chan_to_read[8];
extern boolean_t adc_chan_read_mode_ON;

#pragma arm section zidata = "SRAM_STDBY_DATA"
SRAM_STDBY_DATA static gnssapp_lowpow_t gnssapp_lowpow;     /**< gnssapp low power backup */
#pragma arm section zidata

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/
gpOS_error_t gnssapp_gnss_service_start( gpOS_partition_t *);
gpOS_error_t gnssapp_gnss_amq_init( gpOS_partition_t *, boolean_t , tUInt );
gpOS_error_t gnssapp_gnss_lib_start( gpOS_partition_t *);
static tVoid  gnssapp_low_power_setup( gnss_app_lowpow_setup_type_t, gnss_app_lowpow_standby_type_t, gnss_low_power_cyclic_mode_t *, gnss_low_power_periodic_mode_t *);
/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief Initialize NVM module
 *
 * \param tVoid
 * \return gpOS_FAILURE if it fails
 *
 ***********************************************/
gpOS_error_t gnssapp_nvm_start( gpOS_partition_t *part )
{
  tUInt   nvm_primary_addr, nvm_secondary_addr;
  tUInt   nvm_memory_size = NVM_MEM_SIZE;

  // Check if any plugins needs more NVM memory and update size
  gnssapp_plugins_get_nvm_size( &nvm_memory_size );

  // Initialize NVM or backup RAM memory
  platform_gnss_get_nvm_config( &nvm_primary_addr, &nvm_secondary_addr, nvm_memory_size);

  if ( nvm_open_p( part, NVM_VERSION_ID, ( nvm_address_t )nvm_primary_addr, ( nvm_address_t )nvm_secondary_addr, nvm_memory_size, gnss_flash_erase_priority ) == NVM_ERROR )
  {
    return ( gpOS_FAILURE );
  }

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return gpOS_error_t
 *
 ***********************************************/
static gpOS_error_t gnssapp_swconfig_setup( gpOS_partition_t *part )
{
  tUInt gpio_cfg0, gpio_mode_AFSLA;
  tUInt gpio_cfg1, gpio_mode_AFSLB;
  tUInt gpio_set_to_high_mask;
  tUInt gpio_set_to_low_mask;
  tU8 cpu_clock_speed;
  tUInt pwr_high_low_cfg = 0;

  /**< Start sw config module */
  sw_config_init();

  sw_config_get_param( CURRENT_CONFIG_DATA, TCXO_CONFIG_SELECTOR_ID, &tcxo_config_selector );

  if(tcxo_config_selector == 10) // to be better implemneted
  {
    FE_def_write_data(5,0x4D);
    FE_def_write_data(6,0xB6);
    FE_def_write_data(7,0x93);

    platform_mcu_setspeed( PLATFORM_MCU_SPEED_48MHZ);
  }
  else if(tcxo_config_selector == 11) // to be better implemneted
  {
    FE_def_write_data(5,0x19);
    FE_def_write_data(6,0xB0);
    FE_def_write_data(7,0x87);

    platform_mcu_setspeed( PLATFORM_MCU_SPEED_55MHZ);
  }

  if ( gnss_set_tcxo_config( tcxo_config_selector ) == GNSS_ERROR )
  {
    return gpOS_FAILURE;
  }

  svc_fsw_setextfreq( TRACKER_CPU_TICKS_PER_SECOND);

  sw_config_get_param( CURRENT_CONFIG_DATA, LOW_POWER_CFG_PARAMS_6_ID, &pwr_high_low_cfg);
  platform_set_pwr_high_low_configs(pwr_high_low_cfg);

  sw_config_get_param( CURRENT_CONFIG_DATA, CPU_CLOCK_SPEED, &cpu_clock_speed);
  platform_set_cpu_clock_speed( (cpu_clock_speed & 0xF), ((cpu_clock_speed >> 4) & 0x3));

  /**< GPIO configuration parameters */
  sw_config_get_param( CURRENT_CONFIG_DATA, GPIO_PORT0_CFG0_ID, &gpio_cfg0 );
  sw_config_get_param( CURRENT_CONFIG_DATA, GPIO_PORT0_CFG1_ID, &gpio_cfg1 );
  sw_config_get_param( CURRENT_CONFIG_DATA, GPIO_PORT0_MODE_AFSLA_ID, &gpio_mode_AFSLA);
  sw_config_get_param( CURRENT_CONFIG_DATA, GPIO_PORT0_MODE_AFSLB_ID, &gpio_mode_AFSLB);
  gpio_set_to_high_mask = gpio_cfg0 & gpio_cfg1;
  gpio_set_to_low_mask = ~gpio_cfg0 & ~gpio_cfg1;
  if ( ( gpio_set_to_high_mask | gpio_set_to_low_mask ) != 0 )
  {
    tUInt mode_none_mask=0, mode_A_mask=0, mode_B_mask=0, mode_C_mask=0;

    mode_none_mask = ~(gpio_mode_AFSLA | gpio_mode_AFSLB);
    mode_A_mask = gpio_mode_AFSLA & ~(gpio_mode_AFSLB);
    mode_B_mask = ~(gpio_mode_AFSLA) & gpio_mode_AFSLB;
    mode_C_mask = gpio_mode_AFSLA & gpio_mode_AFSLB;

    LLD_GPIO_SetControlMode((LLD_GPIO_idTy)GPIO0_REG_START_ADDR,(LLD_GPIO_PinTy)((gpio_set_to_high_mask | gpio_set_to_low_mask) & mode_none_mask),LLD_GPIO_ALTERNATE_NONE);
    LLD_GPIO_SetControlMode((LLD_GPIO_idTy)GPIO0_REG_START_ADDR,(LLD_GPIO_PinTy)((gpio_set_to_high_mask | gpio_set_to_low_mask) & mode_A_mask),LLD_GPIO_ALTERNATE_MODE_A);
    LLD_GPIO_SetControlMode((LLD_GPIO_idTy)GPIO0_REG_START_ADDR,(LLD_GPIO_PinTy)((gpio_set_to_high_mask | gpio_set_to_low_mask) & mode_B_mask),LLD_GPIO_ALTERNATE_MODE_B);
    LLD_GPIO_SetControlMode((LLD_GPIO_idTy)GPIO0_REG_START_ADDR,(LLD_GPIO_PinTy)((gpio_set_to_high_mask | gpio_set_to_low_mask) & mode_C_mask),LLD_GPIO_ALTERNATE_MODE_C);

    LLD_GPIO_SetDirectionOutput((LLD_GPIO_idTy)GPIO0_REG_START_ADDR,(LLD_GPIO_PinTy)(gpio_set_to_high_mask | gpio_set_to_low_mask));
    LLD_GPIO_SetStateHigh((LLD_GPIO_idTy)GPIO0_REG_START_ADDR,(LLD_GPIO_PinTy)gpio_set_to_high_mask);
    LLD_GPIO_SetStateLow((LLD_GPIO_idTy)GPIO0_REG_START_ADDR,(LLD_GPIO_PinTy)gpio_set_to_low_mask);

    {
      tUInt i2C_gpio_set_to_high_mask, i2C_gpio_set_to_low_mask;
      tUInt i2c_gpio_AltFunc_mask;

      i2c_gpio_AltFunc_mask = LLD_GPIO_GetMask(LLD_GPIO_AF_I2C);
      i2C_gpio_set_to_high_mask = gpio_set_to_high_mask & i2c_gpio_AltFunc_mask;
      i2C_gpio_set_to_low_mask  = gpio_set_to_low_mask  & i2c_gpio_AltFunc_mask;

      if ( (tUInt)( i2C_gpio_set_to_high_mask | i2C_gpio_set_to_low_mask ) != 0x0U )
      {
        i2c_gpio_sw_config_used = TRUE;
      }
    }
  }
  sw_config_get_param( CURRENT_CONFIG_DATA, GPIO_PORT1_CFG0_ID, &gpio_cfg0 );
  sw_config_get_param( CURRENT_CONFIG_DATA, GPIO_PORT1_CFG1_ID, &gpio_cfg1 );
  sw_config_get_param( CURRENT_CONFIG_DATA, GPIO_PORT1_MODE_AFSLA_ID, &gpio_mode_AFSLA);
  sw_config_get_param( CURRENT_CONFIG_DATA, GPIO_PORT1_MODE_AFSLB_ID, &gpio_mode_AFSLB);

  gpio_set_to_high_mask = gpio_cfg0 & gpio_cfg1;
  gpio_set_to_low_mask = ~gpio_cfg0 & ~gpio_cfg1;
  if ( ( gpio_set_to_high_mask | gpio_set_to_low_mask ) != 0 )
  {
    tUInt mode_none_mask=0, mode_A_mask=0, mode_B_mask=0, mode_C_mask=0;

    mode_none_mask = ~(gpio_mode_AFSLA | gpio_mode_AFSLB);
    mode_A_mask = gpio_mode_AFSLA & ~(gpio_mode_AFSLB);
    mode_B_mask = ~(gpio_mode_AFSLA) & gpio_mode_AFSLB;
    mode_C_mask = gpio_mode_AFSLA & gpio_mode_AFSLB;

    LLD_GPIO_SetControlMode((LLD_GPIO_idTy)GPIO1_REG_START_ADDR,(LLD_GPIO_PinTy)((gpio_set_to_high_mask | gpio_set_to_low_mask) & mode_none_mask),LLD_GPIO_ALTERNATE_NONE);
    LLD_GPIO_SetControlMode((LLD_GPIO_idTy)GPIO1_REG_START_ADDR,(LLD_GPIO_PinTy)((gpio_set_to_high_mask | gpio_set_to_low_mask) & mode_A_mask),LLD_GPIO_ALTERNATE_MODE_A);
    LLD_GPIO_SetControlMode((LLD_GPIO_idTy)GPIO1_REG_START_ADDR,(LLD_GPIO_PinTy)((gpio_set_to_high_mask | gpio_set_to_low_mask) & mode_B_mask),LLD_GPIO_ALTERNATE_MODE_B);
    LLD_GPIO_SetControlMode((LLD_GPIO_idTy)GPIO1_REG_START_ADDR,(LLD_GPIO_PinTy)((gpio_set_to_high_mask | gpio_set_to_low_mask) & mode_C_mask),LLD_GPIO_ALTERNATE_MODE_C);

    LLD_GPIO_SetDirectionOutput( (LLD_GPIO_idTy)GPIO1_REG_START_ADDR, ( LLD_GPIO_PinTy )( gpio_set_to_high_mask | gpio_set_to_low_mask ) );
    LLD_GPIO_SetStateHigh( (LLD_GPIO_idTy)GPIO1_REG_START_ADDR, ( LLD_GPIO_PinTy )gpio_set_to_high_mask );
    LLD_GPIO_SetStateLow( (LLD_GPIO_idTy)GPIO1_REG_START_ADDR, ( LLD_GPIO_PinTy )gpio_set_to_low_mask );
  }

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t gnssapp_gnss_debug_start( gpOS_partition_t *part )
{
  gnss_debug_mode_t gnss_debug_mode = GNSS_DEBUG_ON;

  sw_config_get_param( CURRENT_CONFIG_DATA, GPS_DEBUG_MODE_ID, &gnss_debug_mode );

  gnss_debug_mode &= GNSS_DEBUG_MASK;

  if ( gnss_debug_init( part, gnss_debug_mode, gnssapp_handler->debug_output ) == gpOS_FAILURE )
  {
    return ( gpOS_FAILURE );
  }

#if defined( __ARMCC_VERSION)
  GPS_DEBUG_MSG( ( "\r\n\r\n\r\nARM C/C++ Compiler version: %d\r\n", __ARMCC_VERSION ) );
#endif

#if defined( __GNUC__)
  GPS_DEBUG_MSG( ( "\r\n\r\n\r\nGNU C/C++ Compiler version: %d%d%d\r\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__ ) );
#endif
  GPS_DEBUG_MSG( ( "%s\r\n%s\r\n", gnssapp_version(), gpOS_version() ) );

  //sw_config_print();

  return ( gpOS_SUCCESS );
}

/********************************************//**
 * \brief Start GPS Services
 *
 * \param tVoid
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t gnssapp_gnss_service_start( gpOS_partition_t *gps_part)
{
  gpOS_error_t error = gpOS_SUCCESS;

  /* Init SSP service for frontend */
  if( svc_ssp_init( gps_part, PLATFORM_BUSCLK_ID_MCLK) == gpOS_FAILURE)
  {
    error = gpOS_FAILURE;
  }

  /* Init I2C service */
  if( svc_i2c_init( gps_part, PLATFORM_BUSCLK_ID_MCLK) == gpOS_FAILURE)
  {
    error = gpOS_FAILURE;
  }

#if defined( DR_CODE_LINKED )
  /* Init CAN service */
  if( svc_can_init(NULL,PLATFORM_BUSCLK_ID_CAN_CLK) == gpOS_FAILURE)
  {
    error = gpOS_FAILURE;
  }
#endif // defined( DR_CODE_LINKED )

  /* Init GPIO service */
  if( svc_gpio_init( gps_part) == gpOS_FAILURE)
  {
    error = gpOS_FAILURE;
  }

  return error;
}

/********************************************//**
 * \brief Initialize the Advanced Message Queue
 *
 * \param gpOS_partition_t *gps_part
 * \param boolean_t sensors_presence indicates if sensors are present in the config
 * \param boolean_t sens_buf_size indicates the buffer size defined in the config for sensors
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t gnssapp_gnss_amq_init( gpOS_partition_t *part, boolean_t sensors_presence, tUInt sens_buf_size)
{
  gpOS_error_t return_error = gpOS_SUCCESS;

  if ( ( sensors_presence == TRUE) || ( sens_buf_size != 0x0U )  )
  {
    if( sens_buf_size == 0x0U )
    {
      /* use the default msg queue length */
      if (gnss_msg_queue_init(part, &gnss_msg_queue, MSG_QUEUE_LEN) == GNSS_ERROR)
      {
        ERROR_MSG( "[main]: AMQ Init failed\r\n" );
        return_error = gpOS_FAILURE;
      }

    }
    else
    {
      /* initialize the msg queue length with the sensor buf msg number. AMQ will receive sensors data + nav msgs. Nav msg size is minor */
      if (gnss_msg_queue_init(part, &gnss_msg_queue, (tU16)(sens_buf_size * 64U)) == GNSS_ERROR)
      {
        ERROR_MSG( "[main]: AMQ Init failed\r\n" );
        return_error = gpOS_FAILURE;
      }
    }
  }
  /* If sensors are not enabled*/
  else
  {
     /* amq size is minimum */
      if (gnss_msg_queue_init(part, &gnss_msg_queue, (tU16)((sizeof( nav_message_t )+sizeof(gnss_msg_header_t))*DEFAULT_AMQ_MSG_SIZE )) == GNSS_ERROR)
      {
        ERROR_MSG( "[main]: AMQ Init failed\r\n" );
        return_error = gpOS_FAILURE;
      }
  }

  return return_error;
}

/********************************************//**
 * \brief Start GPS libraries
 *
 * \param tVoid
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t gnssapp_gnss_lib_start( gpOS_partition_t *gps_part )
{
  gnss_sat_type_mask_t   orbit_list_selection = 0;
  gnss_sat_type_mask_t   constellation_usage_selection = 0;
  gnss_error_t err = GNSS_NO_ERROR;
  gpOS_error_t ret = gpOS_SUCCESS;
  tU8 pps_clock = 32;

  // Initialize frontend default values if needed
  {
    tU32 cnt;

    for ( cnt = 0; cnt < 25; cnt++ )
    {
      tU8 addr, data, func;

      err = sw_config_get_param( CURRENT_CONFIG_DATA, FE_A0_ID + (cnt * 2), &addr );
      err = sw_config_get_param( CURRENT_CONFIG_DATA, FE_D0_ID + (cnt * 2), &data );

      if ( addr != DEFAULT_NOT_USED_FE_REGISTER )
      {
        func = ( addr >> 6 ) & 0x3;
        addr = addr & 0x3F;
        if ( func > 0 )
        {
          tUInt read_data;

          FE_def_read_data( addr, &read_data );
          switch ( func )
          {
            case 1:
              data = read_data | data;
              break;

            case 2:
              data = read_data & data;
              break;
            default:
            /* Should never go there*/
            break;
          }
        }

        FE_def_write_data( addr, data );
      }
    }
  }

  /**< Initialize gnss section */

  // Configure orbit lists for tracking and positioning
  if( sw_config_get_software_switch_status( GPS_ON_OFF_SWITCH )!= FALSE )
  {
    MCR_SETBIT( orbit_list_selection, GNSS_SAT_TYPE_GPS );
  }
  if( sw_config_get_software_switch_status( GPS_USE_ON_OFF_SWITCH )!= FALSE )
  {
    MCR_SETBIT( constellation_usage_selection, GNSS_SAT_TYPE_GPS );
  }

  if( sw_config_get_software_switch_status( QZSS_ON_OFF_SWITCH ) != FALSE)
  {
    MCR_SETBIT( orbit_list_selection, GNSS_SAT_TYPE_QZSS_L1_CA );
  }
  if( sw_config_get_software_switch_status( QZSS_USE_ON_OFF_SWITCH )!= FALSE )
  {
    MCR_SETBIT( constellation_usage_selection, GNSS_SAT_TYPE_QZSS_L1_CA );
  }

  if( sw_config_get_software_switch_status( GLONASS_ON_OFF_SWITCH )!= FALSE )
  {
    MCR_SETBIT( orbit_list_selection, GNSS_SAT_TYPE_GLONASS );
  }
  if( sw_config_get_software_switch_status( GLONASS_USE_ON_OFF_SWITCH )!= FALSE )
  {
    MCR_SETBIT( constellation_usage_selection, GNSS_SAT_TYPE_GLONASS );
  }

  if (( boolean_t )sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, GALILEO_ON_OFF_SWITCH )!= FALSE)
  {
    MCR_SETBIT( orbit_list_selection, GNSS_SAT_TYPE_GALILEO );
  }
  if(( boolean_t )sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, GALILEO_USE_ON_OFF_SWITCH )!= FALSE)
  {
    MCR_SETBIT( constellation_usage_selection, GNSS_SAT_TYPE_GALILEO );
  }

  if (( boolean_t )sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, COMPASS_ON_OFF_SWITCH )!= FALSE)
  {
    MCR_SETBIT( orbit_list_selection, GNSS_SAT_TYPE_COMPASS );
  }
  if(( boolean_t )sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, COMPASS_USE_ON_OFF_SWITCH )!= FALSE)
  {
    MCR_SETBIT( constellation_usage_selection, GNSS_SAT_TYPE_COMPASS );
  }




  // Configure frontend registers based on SW config data
  err = sw_config_get_param( CURRENT_CONFIG_DATA, PPS_CLOCK_SETTING_ID, &pps_clock );

  platform_gnss_set_pps_clock( pps_clock );

  if ( platform_gnss_init() == gpOS_FAILURE )
  {
    return ( gpOS_FAILURE );
  }
  gnssapp_startup_time.gnss_lib_set_timer_clock = gpOS_time_now();

  {
    tInt min_week_n = 0, max_week_n = 0, utc_delta_time_default = 0;
    gnss_fix_config_t fix_config;

    err = sw_config_get_param( CURRENT_CONFIG_DATA, GPS_MIN_MAX_WEEK_NUMBER_ID, &min_week_n );

    max_week_n = ( min_week_n >> 16 ) & 0xFFFF;
    min_week_n = min_week_n & 0xFFFF;

    gnss_set_min_max_week_number( min_week_n, max_week_n );

    err = sw_config_get_param( CURRENT_CONFIG_DATA, GPS_UTC_DEFAULT_SETTING_ID , &utc_delta_time_default );

    gnss_set_gps_utc_delta_time_default( utc_delta_time_default );

    gnss_get_fix_config( &fix_config );

    fix_config.few_sats_pos_estimation_enable = ( boolean_t )sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, FEW_SATS_POS_EST_ON_OFF_SWITCH );

    gnss_set_fix_config( &fix_config );

  }

  if(sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, EXT_RTC_OSCI_ON_OFF_SWITCH ) != FALSE)
  {
    LLD_PRCC_SetOscillator(FALSE);
  }
  else
  {
    /* Internal oscillator is used to generate the RTC clock.*/
  }

  if(sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, RTC_USAGE_DISABLING_ON_OFF_SWITCH ) != FALSE)
  {
  /**< Turn off RTC and initialise */
    gnss_time_t gnss_time;

    /* calculate the gps week and time of week */
    /* year month day, hour, mins ,secs */
    //gnss_date_time_to_gnss_time( 2009, 06, 12, 13, 30, 00, &gnss_time);

    /* Initialize the RTC to off and tell it GPS time at a CPU time */
    gnss_time.week_n = 0;
    gnss_time.tow = 0.0;

    gnss_init_rtc( RTC_SWITCH_OFF, &gnss_time, gpOS_time_now(), 0xFFFFFFFFU );
  }
  /*}}}  */


  /**< Init GNSS event mechanism */
  if ( gnss_events_init_p( gps_part ) == gpOS_FAILURE )
  {
    return ( gpOS_FAILURE );
  }

  gnss_set_sat_list_size(GNSSLIB_SAT_LIST_SIZE);

  /**< Setup GPS engine */
  if( gnss_init_p( (gpOS_partition_t *)gps_part, orbit_list_selection, constellation_usage_selection) == GNSS_ERROR )
  {
    ERROR_MSG( "[main]: ERROR gnss_init failed\r\n" );
    return ( gpOS_FAILURE );
  }
  else
  {
    tInt trk_threshold = 0;
    tDouble fix_rate = 0.0;
    tUInt mask_angle = 0;
    tUInt wls_params = 0;

    //if (nmea_msg_list_check(NMEA_msg_list, NAV_NMEA_M) == TRUE)
    if (sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, NAVM_SUPPORT_ON_OFF_SWITCH ) != 0)
    {
      //gnss_error_t gnss_nav_dump_init( gpOS_partition_t *part);

      if( gnss_nav_dump_init( gps_part) == GNSS_ERROR)
      {
        ERROR_MSG(("GNSS_ERROR subframe dump init failed\r\n"));
        return ( gpOS_FAILURE );
      }
    }

    err = sw_config_get_param( CURRENT_CONFIG_DATA, GPS_MASK_ANGLE_ID, &mask_angle );
    err = sw_config_get_param( CURRENT_CONFIG_DATA, GPS_FIX_RATE_ID, &fix_rate );

    gnss_set_elevation_mask_angle( ( tDouble )mask_angle );
    gnss_set_fix_rate( fix_rate );

    err = sw_config_get_param( CURRENT_CONFIG_DATA, GPS_MASK_ANGLE_POSITIONING_ID, &mask_angle );
    gnss_set_elevation_mask_angle_positioning( ( tDouble )mask_angle );

    err = sw_config_get_param( CURRENT_CONFIG_DATA, WLS_CFG_PARAMS_1_ID, &wls_params);

    if((wls_params & 0x1) != 0)
    {
      tDouble par1, par2;

      par1 = (tDouble)((wls_params >> 8) & 0xFF) / 10;
      par2 = (tDouble)((wls_params >> 16) & 0xFF) / 10;

      gnss_set_wls_runtime(TRUE,par1,par2);
    }

    {
      tInt notch_filter_cfg = 0;

      err = sw_config_get_param( CURRENT_CONFIG_DATA, NOTCH_FILTER_CFG_ID, &notch_filter_cfg );

      if ( MCR_ISBITSET( notch_filter_cfg, GNSS_SAT_TYPE_GPS ) )
      {
        gnss_notch_filter_enable( GNSS_SAT_TYPE_GPS, 0, 1 );
      }

      if ( MCR_ISBITSET( notch_filter_cfg, GNSS_SAT_TYPE_GLONASS ) )
      {
        gnss_notch_filter_enable( GNSS_SAT_TYPE_GLONASS, 0, 1 );
      }

      if ( MCR_ISBITSET( notch_filter_cfg >> 2, GNSS_SAT_TYPE_GPS ) )
      {
        gnss_notch_filter_enable( GNSS_SAT_TYPE_GPS, 0, 2 );
      }

      if ( MCR_ISBITSET( notch_filter_cfg >> 2, GNSS_SAT_TYPE_GLONASS ) )
      {
        gnss_notch_filter_enable( GNSS_SAT_TYPE_GLONASS, 0, 2 );
      }
    }

    if ( sw_config_get_software_switch_status( ACQ_2_5_PPM_TCXO_SWITCH ) != FALSE )
    {
      gnss_init_2_5_ppm_support( TRUE );
      GPS_DEBUG_MSG( ( "2.5 ppm ACQ TCXO error support\r\n" ) );
    }

    {
      tUInt gnss_integrity_check_cfg = 0U;
      err = sw_config_get_param( CURRENT_CONFIG_DATA, GNSS_INTEGRITY_CHECK_CFG_ID, &gnss_integrity_check_cfg );
      gnss_init_integrity_check_cfg( gnss_integrity_check_cfg );
      GPS_DEBUG_MSG( ( "GNSS Integrity support %d\r\n",gnss_integrity_check_cfg) );
    }

    if( sw_config_get_software_switch_status( HIGH_DYNAMICS_ON_OFF_SWITCH )!=FALSE )
    {
      if(gnss_init_high_dynamics_mode(1) == GNSS_ERROR)
      {
        GPS_DEBUG_MSG(("gnss_init_high_dynamics_mode: ERROR\r\n"));
      }
    }
    else
    {
      tUInt dyn_params = 0;
      tUInt param;
      err = sw_config_get_param( CURRENT_CONFIG_DATA, DYNAMIC_MODE_CFG_PARAMS_ID, &dyn_params);

      if((dyn_params & 0xFU) == 4U)
      {
        dynamic_mng_data_t data;

        param = ((dyn_params >> 24U) & 0xFFU);
        data.hd_acc_th = (tDouble)param;
        data.hd_acc_th = data.hd_acc_th / 10.0;
        param = ((dyn_params >> 16U) & 0xFFU);
        data.ld_acc_th = (tDouble)param;
        data.ld_acc_th = data.ld_acc_th / 100.0;
        param = ((dyn_params >> 8U) & 0xFFU);
        data.hd_hysteresis_s = (tChar)param;
        param= ((dyn_params >> 4U) & 0xFU);
        data.ld_stabilization_s = (tChar)param;
        gnss_set_dynamic_mng_params(TRUE,&data);
      }
      else
      {
        param = (dyn_params & 0xFU);
        if(gnss_init_high_dynamics_mode((tInt)param) == GNSS_ERROR)
        {
          GPS_DEBUG_MSG(("gnss_init_high_dynamics_mode: ERROR\r\n"));
        }
      }
    }

    if ( sw_config_get_param( CURRENT_CONFIG_DATA, GPS_TRACKING_TH_ID, &trk_threshold ) == GNSS_NO_ERROR )
    {
      gnss_init_tracking_threshold( trk_threshold );
    }

    if ( sw_config_get_software_switch_status( QZSS_SLOW_ACQUISITION_MODE_SWITCH ) != FALSE )
    {
      gnss_acquisition_set_operational_mode( GNSS_SAT_TYPE_QZSS_L1_CA, 1 );
    }

    {
      tUInt dsp_config_options = 0U;
      err = sw_config_get_param( CURRENT_CONFIG_DATA, DSP_CONFIG_OPTIONS_ID, &dsp_config_options);
      gnss_set_multipath_mitigation_mode((tInt)(dsp_config_options & 0x3));
      gnss_set_tcxo_jump_detection_mode((tInt)((dsp_config_options & 0x4)>>2));
    }

    {
      tUInt param1, param2;
      gnss_lms_config_t lms_cfg;

      err = sw_config_get_param( CURRENT_CONFIG_DATA, LMS_ALGO_CFG_PARAMS_1_ID, &param1 );
      err = sw_config_get_param( CURRENT_CONFIG_DATA, LMS_ALGO_CFG_PARAMS_2_ID, &param2 );

      gnss_lms_get_config( &lms_cfg );

      lms_cfg.enable_2D_fixes = (boolean_t)( ( param1 >> 0U ) & 0x1U );
      lms_cfg.check_residual_hdop_product = (boolean_t)( ( param1 >> 1U ) & 0x1U );
      lms_cfg.lock_glonass_path_delay = (boolean_t)( ( param1 >> 2U ) & 0x1U );

      lms_cfg.position_residual_thr = ( tDouble )( ( param1 >> 8U ) & 0xFFU );
      lms_cfg.position_residual_thr_after_raim = ( tDouble )( ( param1 >> 16U ) & 0xFFU );

      lms_cfg.min_sat_gnss_fix = ( ( param2 >> 0U ) & 0xFFU );
      lms_cfg.min_sat_single_const_fix = ( ( param2 >> 8U ) & 0xFFU );
      lms_cfg.glonass_path_delay_init_value = ( tDouble )( ( tShort )( ( param2 >> 16U ) & 0xFFFFU ) ) / 10;

      gnss_lms_set_config( &lms_cfg );
    }

    // restore previous parameter after standby return
    if( (svc_pwr_StartupMode() != SVC_PWR_STARTUP_POWER_ON) )
    {
      GPS_DEBUG_MSG(( "[gnssapp pwr] low power mode reload backup\r\n"));
      gnssapp_low_power_setup(GNSSAPP_LOW_POWER_INIT, gnssapp_lowpow.Standby, &gnssapp_lowpow.cyclic, &gnssapp_lowpow.periodic );
    }
    else
    {
      if(sw_config_get_software_switch_status( LOW_POWER_ON_OFF_SWITCH )!=FALSE)
      {
        gnss_low_power_cyclic_mode_t cyclic;
        gnss_low_power_periodic_mode_t periodic;
        gnss_app_lowpow_standby_type_t Standby;

        tUInt param1,param4,param5;

        GPS_DEBUG_MSG(( "[gnssapp pwr] low power mode reload NVM\r\n"));

        err = sw_config_get_param( CURRENT_CONFIG_DATA, LOW_POWER_CFG_PARAMS_1_ID, &param1);
        err = sw_config_get_param( CURRENT_CONFIG_DATA, LOW_POWER_CFG_PARAMS_4_ID, &param4);
        err = sw_config_get_param( CURRENT_CONFIG_DATA, LOW_POWER_CFG_PARAMS_5_ID, &param5);

        /* Adaptive and Cyclic config bits */
        cyclic.reduced_type       = (boolean_t)((param1      ) & 0x01U);
        cyclic.duty_cycle_on_off  = (boolean_t)((param1 >> 1U) & 0x01U);
        /* 2 bits left */

        /* Adaptive and Cyclic settings */
        cyclic.ehpe_threshold     = (tU8)   ((param1 >> 4U) & 0xFFU);
        cyclic.N_sats_reduced     = (tU8)   ((param1 >> 12U) & 0xFFU);
        cyclic.duty_cycle_fixperiod = (tShort)((param1 >> 20U) & 0xFFFU);

        /* ConsteLLation applicable */
        cyclic.const_mask_init = constellation_usage_selection;

        /* Periodic config bits */
        periodic.NoFixTimeout      = (tU8)      ((param5      ) & 0xFFU);
        periodic.NoFixOffTime      = (tU16)     ((param5 >> 8U) & 0xFFFU);
        periodic.periodic_mode     = (boolean_t)((param4      ) & 0x01U);

        periodic.EPH_refresh      = (tU8)((param4 >> 2U) & 0x01U);
        periodic.RTC_refresh      = (tU8)((param4 >> 3U) & 0x01U);
        periodic.wakeup_pin_en    = (tU8)((param4 >> 4U) & 0x01U);
        /* 3 bits left */

        /* Periodic settings */
        periodic.fix_period       = (tU32)((param4 >> 8U) & 0x1FFFFU);
        periodic.fix_on_time      = (tU8) ((param4 >> 25U) & 0x7FU);

        if(((param4 >> 1U) & 0x01U) == 0x01U)
        {
          Standby = GNSSAPP_LOW_POWER_STANDBY_ENABLE;
        }
        else
        {
          Standby = GNSSAPP_LOW_POWER_STANDBY_DISABLE;
        }
        gnssapp_low_power_setup(GNSSAPP_LOW_POWER_INIT, Standby, &cyclic, &periodic );

        if( svc_pwr_get_standby_allowed() == TRUE )
        {
          GPS_DEBUG_MSG(( "[gnssapp pwr] NVM standby mode activated\r\n"));
        }
      }
      else
      {
        gnssapp_low_power_setup(GNSSAPP_LOW_POWER_INIT, GNSSAPP_LOW_POWER_STANDBY_DISABLE, NULL, NULL );
      }
    }

    if(sw_config_get_software_switch_status_by_id(APP_ON_OFF_2_ID,GNSS_FAST_CN0_MODE_ON_OFF_SWITCH) != FALSE)
    {
      gnss_set_fast_CN0_mode(TRUE);
    }
    else
    {
      gnss_set_fast_CN0_mode(FALSE);
    }

    if (sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, RTC_USAGE_DISABLING_ON_OFF_SWITCH) == 0U)
    {
      if(sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, RTC_CALIBRATION_ON_OFF_SWITCH) != 0U)
      {
        gnss_set_rtc_calibration_mode(TRUE);
      }
      else
      {
        gnss_set_rtc_calibration_mode(FALSE);
      }
    }
  }

#if 0
  /**< Invalidate NVM data to do a cold start */
  gnss_clear_all_almanacs();
  gnss_clear_all_ephems();
  gnss_test_invalidate_user_pos();
  gnss_test_invalidate_rtc();
#endif

#if 0
  /**< Force setting of NCO frequency and bandwidth */
  gnss_set_centre_freq( startup_nco_center );
  gnss_set_freq_range( startup_nco_max, startup_nco_min );
#endif

  /**< Starts GPS engine */
  gnss_set_diff_mode( DIFF_MODE_AUTO );
  gnss_diff_set_source_type( diff_source );

  /*PPS CLOCK SETTING STEP2*/
  gnss_pps_set_clock_speed( pps_clock ); //this call must be placed before the gnss_start() call

  /* Pulse Per Second Initialization and Configuration*/
  if( sw_config_get_software_switch_status( PPS_ON_OFF_SWITCH ) != FALSE)
  {
    tDouble pps_correction = 0.0, gps_pps_correction = 0.0, glonass_pps_correction = 0.0, compass_pps_correction = 0.0, galileo_pps_correction = 0.0;
    tDouble pps_pulse_duration = 5E-1;
    boolean_t pps_inverted_polarity = FALSE;

    gnss_pps_enable_control( TRUE );

    err = sw_config_get_param( CURRENT_CONFIG_DATA, RF_TIME_CORRECTION_ID, &pps_correction );
    err = sw_config_get_param( CURRENT_CONFIG_DATA, PPS_PULSE_DURATION_ID, &pps_pulse_duration );
    pps_inverted_polarity = (boolean_t)sw_config_get_software_switch_status( PPS_INVERTED_POLARITY_ON_OFF_SWITCH );

    gnss_pps_init( (gpOS_partition_t *)gps_part, pps_correction, pps_pulse_duration, pps_inverted_polarity );

    err = sw_config_get_param( CURRENT_CONFIG_DATA, GPS_RF_TIME_CORRECTION_ID, &gps_pps_correction );
    err = sw_config_get_param( CURRENT_CONFIG_DATA, GLONASS_RF_TIME_CORRECTION_ID, &glonass_pps_correction );
    err = sw_config_get_param( CURRENT_CONFIG_DATA, COMPASS_RF_TIME_CORRECTION_ID, &compass_pps_correction );
    err = sw_config_get_param( CURRENT_CONFIG_DATA, GALILEO_RF_TIME_CORRECTION_ID, &galileo_pps_correction );
    gnss_pps_set_rf_compensation( GNSS_SAT_TYPE_GPS, gps_pps_correction );
    gnss_pps_set_rf_compensation( GNSS_SAT_TYPE_GLONASS, glonass_pps_correction );
    gnss_pps_set_rf_compensation( GNSS_SAT_TYPE_COMPASS, compass_pps_correction );
    gnss_pps_set_rf_compensation( GNSS_SAT_TYPE_GALILEO, galileo_pps_correction );

    {
      tU8 hw_cfg;

      err = sw_config_get_param( CURRENT_CONFIG_DATA, HARDWARE_CONFIGURATION_ID, &hw_cfg );

      if ( hw_cfg == HW_SAL_CFG )
      {
        LLD_GPIO_SetControlMode( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, LLD_GPIO_PIN1, LLD_GPIO_ALTERNATE_MODE_B );
      }
    }

    if( sw_config_get_software_switch_status( POSITION_HOLD_ON_OFF_SWITCH ) != FALSE)
    {
      tDouble lat, lon, height;

      err = sw_config_get_param( CURRENT_CONFIG_DATA, POSITION_HOLD_LAT_ID, &lat );
      err = sw_config_get_param( CURRENT_CONFIG_DATA, POSITION_HOLD_LON_ID, &lon );
      err = sw_config_get_param( CURRENT_CONFIG_DATA, POSITION_HOLD_HEIGHT_ID, &height );

      gnss_pps_set_position_hold_llh_pos( lat, lon, height );
      gnss_pps_set_position_hold_status( TRUE );
    }

    if(sw_config_get_software_switch_status( TIMING_TRAIM_ON_OFF_SWITCH )!= FALSE)
    {
      tDouble traim_alarm = 15E-9;
      err = sw_config_get_param( CURRENT_CONFIG_DATA, TIMING_TRAIM_ALARM_ID, &traim_alarm );
      gnss_pps_enable_traim( traim_alarm );
    }

    {
      tUInt pps_op_mode_setting_1 = 0, pps_op_mode_setting_2 = 0, pps_auto_survey_samples = 0;

      err = sw_config_get_param( CURRENT_CONFIG_DATA, PPS_OPERATING_MODE_SETTING_1_ID, &pps_op_mode_setting_1 );
      err = sw_config_get_param( CURRENT_CONFIG_DATA, PPS_OPERATING_MODE_SETTING_2_ID, &pps_op_mode_setting_2 );
      err = sw_config_get_param( CURRENT_CONFIG_DATA, PPS_AUTO_SURVEY_SAMPLES_ID, &pps_auto_survey_samples );

      gnss_pps_set_reference_time( ( pps_reference_time_t )( ( pps_op_mode_setting_1 >> 4 ) & 0xF ) );
      gnss_pps_set_output_mode( ( pps_output_mode_t )( ( pps_op_mode_setting_1 >> 0 ) & 0xF ) );
      gnss_pps_set_fix_condition( ( fix_status_t )( ( pps_op_mode_setting_1 >> 8 ) & 0xF ) );
      gnss_pps_set_sat_threshold( ( tU8 )( ( pps_op_mode_setting_1 >> 16 ) & 0xFF ) );
      gnss_pps_set_elevation_mask( ( tInt )( ( pps_op_mode_setting_1 >> 24 ) & 0xFF ) );
      gnss_pps_set_constellation_mask( ( gnss_sat_type_mask_t )( ( pps_op_mode_setting_2 >> 0 ) & 0xFF ) );
      gnss_pps_set_auto_hold_samples( pps_auto_survey_samples );
    }

    gnss_pps_set_signal_on_off_status( TRUE );
  }

  /**< Starts SBAS */
  if( gnss_sbas_init( gps_part) == GNSS_ERROR )
  {
    ERROR_MSG( "[main]: ERROR waas_plugin_init failed\r\n" );
    return( gpOS_FAILURE );
  }

  {
    sbas_svc_t       sbas_service;
    sbas_sat_param_t sbas_sat1;
    sbas_sat_param_t sbas_sat2;

    err = sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_SERVICE_ID, &sbas_service );

    if( SBAS_SAT_ID_VALID(sbas_service) == TRUE)
    {
      /* In order to keep compatibility with legacy, check if a PRN has been set */
      /* In that case, set SBAS service auto                                     */
      sbas_service = SBAS_SVC_AUTO;
    }

    gnss_sbas_set_service( sbas_service );

    gnss_sbas_set_status(( gnss_sbas_status_t )sw_config_get_software_switch_status( WAAS_ON_OFF_SWITCH ) );

    err = sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_SATELLITE_PARAM_1_ID, &(sbas_sat1.param));

    GPS_DEBUG_MSG(( "sat_id_1 %d\r\n",sbas_sat1.SatParam.sat_id ) );
    if (WAAS_SAT_ID_VALID(sbas_sat1.SatParam.sat_id) == TRUE)
    {
      tDouble longitude = (tDouble)sbas_sat1.SatParam.longitude;
      if (sbas_sat1.SatParam.sense == 1)
      {
        longitude = -longitude;
      }
      GPS_DEBUG_MSG(( "sat_id_1 %d LON %d Sense %d Service %d \r\n",sbas_sat1.SatParam.sat_id,
                     sbas_sat1.SatParam.longitude, sbas_sat1.SatParam.sense, sbas_sat1.SatParam.service) );
      gnss_sbas_insert_sat(sbas_sat1.SatParam.sat_id, longitude, (sbas_svc_t)sbas_sat1.SatParam.service);
    }

    err = sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_SATELLITE_PARAM_2_ID, &(sbas_sat2.param));

    if (WAAS_SAT_ID_VALID(sbas_sat2.SatParam.sat_id) == TRUE)
    {
      tDouble longitude = (tDouble)sbas_sat2.SatParam.longitude;
      if (sbas_sat2.SatParam.sense == 1)
      {
        longitude = -longitude;
      }
      GPS_DEBUG_MSG(( "sat_id_2 %d LON %d Sense %d Service %d \r\n",sbas_sat2.SatParam.sat_id,
                     sbas_sat2.SatParam.longitude, sbas_sat2.SatParam.sense, sbas_sat2.SatParam.service) );
      gnss_sbas_insert_sat(sbas_sat2.SatParam.sat_id, longitude, (sbas_svc_t)sbas_sat2.SatParam.service);
    }
  }

  GPS_DEBUG_MSG( ( "2D Fix Algorithm Status: %d\r\n", gnss_fix_allow_large_2D_move ) );
  GPS_DEBUG_MSG( ( "FDE Algorithm Status: %d\r\n", gnss_get_fde_status() ) );
  GPS_DEBUG_MSG( ( "KF Tunnel Algo Setting: %d %d\r\n", GNSS_TUNNEL_SHORT_THR_S, GNSS_TUNNEL_LONG_THR_S ) );

  if(err == GNSS_NO_ERROR)
  {
    ret = gpOS_SUCCESS;
  }
  else
  {
    ret = gpOS_FAILURE;
  }
  return ret;
}

/********************************************//**
 * \brief Start optional GNSS modules
 *
 * \param gps_part gpOS_partition_t*
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t gnssapp_gnss_modules_start( gpOS_partition_t *gps_part )
{
  if ( gnssapp_plugins_init( gps_part ) == gpOS_FAILURE )
  {
    return ( gpOS_FAILURE );
  }

  return ( gpOS_SUCCESS );
}

/********************************************//**
 * \brief   Start NMEA application
 *
 * \param   tVoid
 * \return  gpOS_error_t
 *
 ***********************************************/
gpOS_error_t gnssapp_nmea_start( gpOS_partition_t *part )
{
  tInt nmea_msg_list;
  tDouble sleep_time = 0.0;
  tU8 debug_port_mode = 0;

  sw_config_get_param( CURRENT_CONFIG_DATA, NMEA_PORT_SLEEP_TIME_ID, &sleep_time );

  if ( sw_config_get_param( CURRENT_CONFIG_DATA, NMEA_PORT_MSGLIST_L_ID, &nmea_msg_list ) )
  {
    nmea_msg_list = ( (tInt)1 << GGA_NMEA_MSG ) | ( (tInt)1 << GSA_NMEA_MSG ) | ( (tInt)1 << GSV_NMEA_MSG ) | ( (tInt)1 << TG_NMEA_MSG ) | ( (tInt)1 << TS_NMEA_MSG ) | ( (tInt)1 << PA_NMEA_MSG ) | ( (tInt)1 << RMC_NMEA_MSG ) | ( (tInt)1 << RES_NMEA_MSG ) | ( (tInt)1 << WAAS_NMEA_MSG ); /* | DIFF_NMEA_MSG; */
  }

  if ( NMEA_start_p( part, nmea_msg_list, NMEA_TXMODE_AFTER_FIX, sleep_time, gnssapp_handler->nmea_input, gnssapp_handler->nmea_output ) == NMEA_ERROR )
  {
    ERROR_MSG( "[main]: Error - NMEA_start_p failed\r\n" );
    return ( gpOS_FAILURE );
  }

  sw_config_get_param( CURRENT_CONFIG_DATA, GPS_DEBUG_MODE_ID, &debug_port_mode );

  if ( debug_port_mode & SWCFG_NMEA_INPUT_ON_DEBUG )
  {
    if ( NMEA_second_input_start_p( part, gnssapp_handler->debug_input ) == NMEA_ERROR )
    {
      ERROR_MSG( "[main]: Error - NMEA_start 2 failed\r\n" );
      return ( gpOS_FAILURE );
    }
  }

#if defined( STBIN_LINKED )

  if(sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, STBIN_ON_OFF_SWITCH ) != FALSE)
  {
    tU32 stbin_msglist[2] = {0, 0};
    sw_config_get_param( CURRENT_CONFIG_DATA, STBIN_MSGLIST_L_ID, &stbin_msglist[0] );
    sw_config_get_param( CURRENT_CONFIG_DATA, STBIN_MSGLIST_H_ID, &stbin_msglist[1] );

    if ( stbin_init_p( part, gnssapp_handler->nmea_input, gnssapp_handler->nmea_output, stbin_msglist ) == GNSS_ERROR )
    {
      ERROR_MSG( "[main]: Error - STBIN_start failed\r\n" );
      return ( gpOS_FAILURE );
    }

    if(sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, NMEA_IN_OUT_INTERFACE_SWITCH ) != FALSE)
    {
      nmea_set_if_mode( NMEA_EXTERNAL_IF_MODE );
    }
  }

#endif
  return ( gpOS_SUCCESS );
}

/********************************************//**
 * \brief   Start LVD management
 *
 * \param   None
 * \return  gpOS_error_t
 *
 ***********************************************/
static gpOS_error_t gnssapp_lvd_start( tVoid )
{
  if ( sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, LVD_MONITOR_ON_OFF_SWITCH ) != 0 )
  {
    if ( lvd_mgmt_start() == gpOS_FAILURE )
    {
      return ( gpOS_FAILURE );
    } else {
      return gpOS_SUCCESS;
    }
  }

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param part gpOS_partition_t*
 * \return gpOS_error_t
 *
 ***********************************************/
static gpOS_error_t gnssapp_adc_channel_read_start( gpOS_partition_t *part )
{
  tU32 adc_chan_read_cfg_params;
  tU8 adc_chan_mask;
  tU16 adc_clk_div;
  tU32 i = 0;
  _clibs_div_t result;
  svc_adc_int_mode_cfg_t adc_mode =
  {
    (svc_adc_mode_cfg_t)ADC_NOINTERRUPT
  };

  sw_config_get_param( CURRENT_CONFIG_DATA, ADC_CHAN_READ_CFG_PARAMS_ID, &adc_chan_read_cfg_params );
  adc_chan_mask = (tU8)((adc_chan_read_cfg_params >> 1) & 0xFF);
  adc_clk_div = (tU16)((adc_chan_read_cfg_params >> 9) & 0xFF);

  if ( svc_adc_init( part, 0, gpOS_INTERRUPT_NOPRIORITY, ADC_8CHAN_AVAILABLE, adc_clk_div, &adc_mode ) == gpOS_FAILURE )
  {
    return gpOS_FAILURE;
  }

  result.quot = adc_chan_mask;

  while ( result.quot > 0 )
  {
    result = _clibs_div( result.quot, 2 );
    adc_chan_to_read[i++] = result.rem;
  }

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param part gpOS_partition_t*
 * \return gpOS_error_t
 *
 ***********************************************/
static gpOS_error_t gnssapp_antenna_sensing_start( gpOS_partition_t *part )
{
  tUInt adc_antenna_sensing_cfg_params = 0;
  tUInt adc_chan_read_cfg_params = 0;
  antenna_sensing_config_t cfg;
  tUInt switch_on_off_par;
  tBool switch_on_off;
  antenna_sensing_control_t adc_antenna_sensing_mode = ANTENNA_SENSING_CONTROL_OFF;

  if(GNSS_ERROR == sw_config_get_param( CURRENT_CONFIG_DATA, ADC_ANTENNA_SENSING_CFG_PARAMS_ID, &adc_antenna_sensing_cfg_params ))
  {
    ERROR_MSG("[main]: Error - Ant Sens get sw config params\r\n");
  }
  adc_antenna_sensing_mode = (antenna_sensing_control_t)(adc_antenna_sensing_cfg_params & 0x3U); /*lint !e9030 !e9033 !e9034*/

  switch_on_off_par = (tUInt)(adc_antenna_sensing_cfg_params & 0x8U);
  if( 8U == switch_on_off_par)
  {
    switch_on_off = TRUE;
  }
  else
  {
    switch_on_off = FALSE;
  }

  if ( adc_antenna_sensing_mode != ANTENNA_SENSING_CONTROL_OFF )
  {
    tU16 clk_div;

    clk_div = ( tU16 )( ( (tUInt)adc_antenna_sensing_cfg_params >> 4U ) & 0xFFU );

    antenna_sensing_set_mode( ANTENNA_SENSING_MODE_ON );

    if(  adc_antenna_sensing_mode == ANTENNA_SENSING_CONTROL_RF )
    {
      antenna_sensing_set_switch( switch_on_off );
    }
    else if ( adc_antenna_sensing_mode == ANTENNA_SENSING_CONTROL_ADC )
    {
      tInt adc_antenna_sensing_cfg_adc_in_param = 0;
      tU32 threshold_min = 0U, threshold_max = 0U;
      tInt adc_chan_mask;
      tInt adc_input_to_ant_sens[2];
      tInt i, j;

      if(GNSS_ERROR == sw_config_get_param( CURRENT_CONFIG_DATA, ANTENNA_SENSING_CFG_ADC_IN_ID, &adc_antenna_sensing_cfg_adc_in_param ))
      {
        ERROR_MSG("[main]: Error - Ant Sens get ADC_Input sw config params\r\n");
      }

      threshold_min = ( tU32 )( ( adc_antenna_sensing_cfg_params >> 12U ) & 0x3FFU );
      threshold_max = ( tU32 )( ( adc_antenna_sensing_cfg_params >> 22U ) & 0x3FFU );
      antenna_sensing_set_adc_thresholds(threshold_min,threshold_max);

      adc_input_to_ant_sens[0] = 0;
      adc_input_to_ant_sens[1] = 0;
      adc_chan_mask = (tInt)(adc_antenna_sensing_cfg_adc_in_param & 0xFF);

      i = 0;
      j = 0;

      for(i = 0; i < 8; i++)
      {
        if(0 != (adc_chan_mask & (1<<i)))
        {
          if(j<2)
          {
            adc_input_to_ant_sens[j] = i;
          }
          j++;
        }
      }

      if(j > 2)
      {
        ERROR_MSG("[main]: Error - ADC Ant Sens number of input invalid\r\n");
      }

      if( antenna_sensing_set_adc_input(adc_input_to_ant_sens[0], adc_input_to_ant_sens[1]) == gpOS_FAILURE)
      {
        ERROR_MSG("[main]: Error - ADC Ant Sens input invalid\r\n");
        antenna_sensing_set_mode( ANTENNA_SENSING_MODE_OFF);
      }

      antenna_sensing_set_switch(FALSE);
    }
    else if ( adc_antenna_sensing_mode == ANTENNA_SENSING_CONTROL_GPIO)
    {
      tU32 antenna_sensing_cfg_gpio_pin = 0U;
      tU32 antenna_sensing_cfg_gpio_mode = 0U;
      tU32 antenna_sensing_cfg_gpio_active_levels = 0U;
      tU32 antenna_sensing_gpio_pin[4], antenna_sensing_gpio_mode[4], antenna_sensing_gpio_active_levels[4];
      LLD_GPIO_ModeTy gpio_mode[4];
      LLD_GPIO_StateTy gpio_level[4];
      tU32 i;
      tInt index[4];

      index[0] = ANTENNA_SENSING_DIAG_ON;
      index[1] = ANTENNA_SENSING_SWITCH_CTRL;
      index[2] = ANTENNA_SENSING_EXT_DIAG_SHORT;
      index[3] = ANTENNA_SENSING_EXT_DIAG_OPEN;

      if( GNSS_ERROR == sw_config_get_param( CURRENT_CONFIG_DATA, ANTENNA_SENSING_CFG_GPIO_PIN_ID, &antenna_sensing_cfg_gpio_pin ))
      {
         ERROR_MSG("[main]: Error - Ant Sens get GPIO sw config params\r\n");
      }
      if( GNSS_ERROR == sw_config_get_param( CURRENT_CONFIG_DATA, ANTENNA_SENSING_CFG_GPIO_MODE_ID, &antenna_sensing_cfg_gpio_mode ))
      {
        ERROR_MSG("[main]: Error - Ant Sens get GPIO sw config params\r\n");
      }
      if( GNSS_ERROR == sw_config_get_param( CURRENT_CONFIG_DATA, ANTENNA_SENSING_CFG_ACTIVE_LEVELS_ID, &antenna_sensing_cfg_gpio_active_levels ))
      {
        ERROR_MSG("[main]: Error - Ant Sens get GPIO sw config params\r\n");
      }
      for(i = 0U; i < 4U; i++)
      {
        if(i == 0U)
        {
          antenna_sensing_gpio_pin[i] = ( antenna_sensing_cfg_gpio_pin & 0xFFU );
          antenna_sensing_gpio_mode[i] = ( antenna_sensing_cfg_gpio_mode & 0xFFU );
          antenna_sensing_gpio_active_levels[i] = ( antenna_sensing_cfg_gpio_active_levels & 0xFFU );
        }
        else
        {
          antenna_sensing_gpio_pin[i] = ( ( antenna_sensing_cfg_gpio_pin >> ( 8U * i ) ) & 0xFFU );
          antenna_sensing_gpio_mode[i] = ( ( antenna_sensing_cfg_gpio_mode >> ( 8U * i ) ) & 0xFFU );
          antenna_sensing_gpio_active_levels[i] = ( ( antenna_sensing_cfg_gpio_active_levels >> ( 8U * i ) ) & 0xFFU );
        }
        if(antenna_sensing_gpio_mode[i] == 0x0U)
        {
          gpio_mode[i] = LLD_GPIO_ALTERNATE_NONE;
        }
        else if(antenna_sensing_gpio_mode[i] == 0x01U)
        {
          gpio_mode[i] = LLD_GPIO_ALTERNATE_MODE_A;
        }
        else if(antenna_sensing_gpio_mode[i] == 0x02U)
        {
          gpio_mode[i] = LLD_GPIO_ALTERNATE_MODE_B;
        }
        else if(antenna_sensing_gpio_mode[i] == 0x03U)
        {
          gpio_mode[i] = LLD_GPIO_ALTERNATE_MODE_C;
        }
        if( antenna_sensing_gpio_active_levels[i] == 0U)/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */
        {
          gpio_level[i] = LLD_GPIO_LOW;
        }
        else if( antenna_sensing_gpio_active_levels[i] == 1U)
        {
          gpio_level[i] = LLD_GPIO_HIGH;
        }
        if( gpOS_FAILURE == antenna_sensing_set_gpio( index[i], (LLD_GPIO_ChanTy)antenna_sensing_gpio_pin[i], (LLD_GPIO_ModeTy)gpio_mode[i] )) /*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */
        {
          ERROR_MSG("[main]: Error - Ant Sens set GPIO failed\r\n");
        }
        if( gpOS_FAILURE == antenna_sensing_set_gpio_active_level( index[i], ( LLD_GPIO_StateTy )gpio_level[i] ))/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */
        {
          ERROR_MSG("[main]: Error - Ant Sens set GPIO active level failed\r\n");
        }
      }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */

      antenna_sensing_set_switch(switch_on_off);
    }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */

    cfg.clk_div = clk_div;
    cfg.ctrl = adc_antenna_sensing_mode;
    if (antenna_sensing_init( part, &cfg) == gpOS_FAILURE)
    {
      return gpOS_FAILURE;
    }
  }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */

  sw_config_get_param( CURRENT_CONFIG_DATA, ADC_CHAN_READ_CFG_PARAMS_ID, &adc_chan_read_cfg_params );
  adc_chan_read_mode_ON = ( boolean_t )( ( adc_chan_read_cfg_params >> 0 ) & 0x1 );

  if ( adc_chan_read_mode_ON )
  {
    if ( gnssapp_adc_channel_read_start( part ) == gpOS_FAILURE )
    {
      return gpOS_FAILURE;
    }
  }

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param part gpOS_partition_t*
 * \return gpOS_error_t
 *
 ***********************************************/
static gpOS_error_t gnssapp_sw_protection_setup( tVoid)
{
  // Check if Software Protection Mode is enabled
  sw_config_get_param( CURRENT_CONFIG_DATA, SPM_ID, &spm);
  sw_config_get_param( CURRENT_CONFIG_DATA, SPM_CONFIGURATION_ID, &spm_configuration);

  return platform_sw_protection_setup( spm, spm_configuration);
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief
 *
 * \param fast_part gpOS_partition_t*
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t gnssapp_init( gpOS_partition_t *fast_part )
{
  tUInt sens_buf_size=0;
  boolean_t sensors_presence=FALSE;
  svc_pwr_startup_mode_t startup_mode;

  gnss_debug_direct_uart_enable( UART0_REG_START_ADDR );

  if( gnssapp_handler != NULL)
  {
    return gpOS_FAILURE;
  }

  gnssapp_handler = gpOS_memory_allocate_p( fast_part, sizeof( gnssapp_handler_t ) );

  if ( gnssapp_handler == NULL )
  {
    return gpOS_FAILURE;
  }

  gnssapp_startup_time_sem = gpOS_semaphore_create_p( SEM_FIFO, fast_part, 1 );

  nmea_support_reset_nmea_restart_cpu_time();

  if ( gnssapp_startup_time_sem == NULL )
  {
    gpOS_semaphore_delete( gnssapp_startup_time_sem );
    return gpOS_FAILURE;
  }

  gnssapp_handler->fast_part              = fast_part;

  /**< Start NVM module */
  gnssapp_startup_time.MTU_timer_clock = GNSSAPP_MTU_CLOCK;  /* clock speed in KHz */
  gnssapp_startup_time.suspend_restart_support_time = 0;
  gnssapp_startup_time.running_time_ms = 0;
  gnssapp_startup_time.nvm_start_cpu_time = gpOS_time_now();

  if ( gnssapp_nvm_start( fast_part ) == gpOS_FAILURE )
  {
    return ( gpOS_FAILURE );
  }

  gnssapp_startup_time.sw_config_cpu_time = gpOS_time_now();

  /**< Load configuration from NVM */
  if ( gnssapp_swconfig_setup( fast_part ) == gpOS_FAILURE )
  {
    return ( gpOS_FAILURE );
  }

  gnssapp_startup_time.gnss_debug_cpu_time = gpOS_time_now();

  /**< Start lvd monitor */
  if ( gnssapp_lvd_start() == gpOS_FAILURE )
  {
    return ( gpOS_FAILURE );
  }

  if( gnssapp_gnss_service_start( fast_part) == gpOS_FAILURE)
  {
    return( gpOS_FAILURE);
  }

  /**> Start in_out module */
  if ( in_out_start( fast_part ) == gpOS_FAILURE )
  {
    return gpOS_FAILURE;
  }

  /**> Start shutdown management module */
  if ( shutdn_ctrl_start() == gpOS_FAILURE )
  {
    return gpOS_FAILURE;
  }

  in_out_get_nmea_cfg( &gnssapp_handler->nmea_input, &gnssapp_handler->nmea_output );
  in_out_get_debug_cfg( &gnssapp_handler->debug_input, &gnssapp_handler->debug_output );
  in_out_get_rtcm_cfg( &gnssapp_handler->rtcm_input );

  sensors_presence = gnssapp_sensors_presence( &sens_buf_size );

  gnssapp_startup_time.gnss_lib_start_cpu_time = gpOS_time_now();

  /* Initialize te AMQ with the right size */
  if ( gnssapp_gnss_amq_init(fast_part, sensors_presence, sens_buf_size) == gpOS_FAILURE)
  {
    return ( gpOS_FAILURE );
  }

  /**< Start GPS library */
  if( gnssapp_gnss_lib_start( fast_part) == gpOS_FAILURE)
  {
    return ( gpOS_FAILURE );
  }

  gnssapp_startup_time.gnss_module_start_cpu_time = gpOS_time_now();

  /**< Start optional modules */
  if ( gnssapp_gnss_modules_start( NULL ) == gpOS_FAILURE )
  {
    return ( gpOS_FAILURE );
  }


#if defined SW_CONFIG_PRIVATE_BLOCK
  /* If Sensors are not initialized, initialize them here */
  if (sensors_presence == TRUE)
  {
    sm_sensors_config_t *dr_config_ptr;

    dr_config_ptr = (sm_sensors_config_t *)((tUInt)sw_config_private_get_base_address() + sizeof(tInt));

    /* Without DR Plugin disable PSTM3DACC, PSTM3DGYRO, PSTMDRCAN messages */
    dr_config_ptr->en_3dgyro_log = 0x00;
    dr_config_ptr->en_3dacc_log = 0x00;
    dr_config_ptr->en_can_log =  0x00;

    if( sm_sensors_init_p( NULL, dr_config_ptr) == GNSS_ERROR)
    {
      ERROR_MSG( "[gnssapp]: ERROR sensors init\r\n" );
      //return gpOS_FAILURE; //if sensors are not available or not working the GNSS software must start anyway
    }
  }
#endif /* SW_CONFIG_PRIVATE_BLOCK */

  /* Start GNSS library */
  // Load startup mode
  startup_mode = svc_pwr_StartupMode();
  switch( startup_mode )
  {
    case SVC_PWR_STARTUP_WAKEUP_PIN:
    {
      gnss_low_power_start(GNSS_STARTUP_WAKEUP_PIN);
    }
    break;

    case SVC_PWR_STARTUP_WAKEUP_RTC:
    {
      gnss_low_power_start(GNSS_STARTUP_WAKEUP_RTC);
    }
    break;

    case SVC_PWR_STARTUP_POWER_ON:
    default:
    {
      gnss_start();
    }
    break;
  }

  gnssapp_startup_time.nmea_start_cpu_start = gpOS_time_now();

  /**< Start NMEA application */
  if ( gnssapp_nmea_start( NULL ) == gpOS_FAILURE )
  {
    return ( gpOS_FAILURE );
  }

  gnssapp_startup_time.as_start_cpu_start = gpOS_time_now();

  /**< Start antenna sensing application */
  if ( gnssapp_antenna_sensing_start( NULL ) == gpOS_FAILURE )
  {
    return gpOS_FAILURE;
  }

  gnssapp_startup_time.running_time_update_cpu_time = gpOS_time_now();

  gnssapp_startup_time.running_time_ms = ( tUInt )( (gnssapp_startup_time.gnss_lib_set_timer_clock / gnssapp_startup_time.MTU_timer_clock ) +
                                         ( gpOS_time_minus( gnssapp_startup_time.running_time_update_cpu_time, gnssapp_startup_time.gnss_lib_set_timer_clock )  / gpOS_timer_ticks_per_msec() ) );

  gnssapp_startup_time.gnssapp_init_end_cpu_time = gpOS_time_now();

  svc_pwr_StartupTime(gnssapp_startup_time.running_time_ms);

  #if defined( NVM_NOR ) || defined( NVM_SQI )
  if ( nvm_swap_mgr_init( NULL ) == NVM_ERROR ) // nvm swap manager must be initialized at the end of the startup phase when all nvm_create() calls have been performed.
  {
    ERROR_MSG( "[main]: LLD_ERROR NVM swap manager init failed\r\n" );
    return ( gpOS_FAILURE );
  }
  #endif

  /**< Start debug for GNSS application */
  if ( gnssapp_gnss_debug_start( fast_part ) == gpOS_FAILURE )
  {
    return ( gpOS_FAILURE );
  }

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return const tChar*
 *
 ***********************************************/
const tChar *gnssapp_version( tVoid )
{
  return gnssapp_ver;
}

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return const tChar*
 *
 ***********************************************/
const tChar *gnssapp_binimg_version( tVoid )
{
  #if defined( VERSION_BINARY )
  return gnssapp_binimg_ver;
  #else
  return NULL;
  #endif
}

/********************************************//**
 * \brief   Get SW config version
 *
 * \param   tVoid
 * \return  tUInt
 *
 ***********************************************/
tUInt gnssapp_swcfg_version( tVoid)
{
  return sw_config_get_tag_version();
}

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return tVoid
 *
 ***********************************************/
tVoid gnssapp_suspend( tVoid )
{
  if( GNSS_ENGINE_STATUS_RUNNING == gnss_get_engine_status())
  {
    gnssapp_plugins_suspend();

    gnss_sbas_set_status( WAAS_STATUS_OFF);

    while( gnss_sbas_suspend_done() == FALSE)
    {
      gpOS_task_delay( gpOS_timer_ticks_per_sec() / 10);
    }

    gnss_sbas_flush_msgs();

    gnss_flush_nav_dump_msg();

    #ifdef SW_CONFIG_PRIVATE_BLOCK
    sm_sensors_suspend();
    #endif /* SW_CONFIG_PRIVATE_BLOCK */

    gnss_suspend();

    #ifdef BACKUP_FILE_LINKED
    {
      gpOS_task_t *backup_task = BACKUP_get_task_ptr();
      backup_task_suspend();
      gpOS_task_set_priority( backup_task, gnss_navigate_task_priority );

      while ( backup_task_suspended() == FALSE )
      {
        gpOS_task_delay( gpOS_timer_ticks_per_sec() / 10U );
      }
    }
    #endif

    #if defined( NVM_NOR ) || defined( NVM_SQI )
    nvm_swap_manager_suspend();

    while ( nvm_swap_manager_suspend_done() == FALSE )
    {
      gpOS_task_delay( gpOS_timer_ticks_per_sec() / 10U );
    }
    #endif
  }
}

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return tVoid
 *
 ***********************************************/
tVoid gnssapp_restart( tVoid )
{
  if( GNSS_ENGINE_STATUS_SUSPENDED == gnss_get_engine_status())
  {
    gnss_restart();

  #ifdef BACKUP_FILE_LINKED
    {
      gpOS_task_t* backup_task = BACKUP_get_task_ptr();
      gpOS_task_set_priority( backup_task, backup_task_priority );
      backup_task_restart();
    }
  #endif

#ifdef SW_CONFIG_PRIVATE_BLOCK
    sm_sensors_resume();
#endif /*SW_CONFIG_PRIVATE_BLOCK */

    {
      sbas_svc_t       sbas_service;
      sbas_sat_param_t sbas_sat1;
      sbas_sat_param_t sbas_sat2;

      sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_SERVICE_ID, &sbas_service );

      if( SBAS_SAT_ID_VALID(sbas_service) == TRUE)
      {
        /* In order to keep compatibility with legacy, check if a PRN has been set */
        /* In that case, set SBAS service auto                                     */
        sbas_service = SBAS_SVC_AUTO;
      }

      gnss_sbas_set_service( sbas_service );

      gnss_sbas_set_status(( gnss_sbas_status_t )sw_config_get_software_switch_status( WAAS_ON_OFF_SWITCH ));

      sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_SATELLITE_PARAM_1_ID, &(sbas_sat1.param));

      if (WAAS_SAT_ID_VALID(sbas_sat1.SatParam.sat_id) == TRUE)
      {
        tDouble longitude = (tDouble)sbas_sat1.SatParam.longitude;
        if (sbas_sat1.SatParam.sense == 1)
        {
          longitude = -longitude;
        }
        gnss_sbas_insert_sat(sbas_sat1.SatParam.sat_id, longitude, (sbas_svc_t)sbas_sat1.SatParam.service);
      }

      sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_SATELLITE_PARAM_2_ID, &(sbas_sat2.param));

      if (WAAS_SAT_ID_VALID(sbas_sat2.SatParam.sat_id) == TRUE)
      {
        tDouble longitude = (tDouble)sbas_sat2.SatParam.longitude;

        if (sbas_sat2.SatParam.sense == 1)
        {
          longitude = -longitude;
        }

        gnss_sbas_insert_sat(sbas_sat2.SatParam.sat_id, longitude, (sbas_svc_t)sbas_sat2.SatParam.service);
      }
    }

    gnssapp_plugins_start();

    gnssapp_swconfig_reload();

  #if defined( NVM_NOR ) || defined( NVM_SQI )
    nvm_swap_manager_restart();
  #endif

    gnssapp_startup_time.gnssapp_init_end_cpu_time = gpOS_time_now();
  }
}

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t gnssapp_imuselftestcmd( tInt Imu_Cat, boolean_t *self_test_result )
{
  gpOS_error_t return_error= gpOS_SUCCESS;

  #ifdef SW_CONFIG_PRIVATE_BLOCK
  return_error=  sm_sensors_imuselftestcmd(Imu_Cat, self_test_result);
  #else
  return_error=  GNSS_ERROR;
  #endif /* SW_CONFIG_PRIVATE_BLOCK */

return return_error;

}

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return tVoid
 *
 ***********************************************/
tVoid gnssapp_swconfig_reload( tVoid )
{
  sw_config_reload();

  // Reload PPS configuration
  if( gnss_pps_started() == TRUE )
  {
    return;
  }

  if( sw_config_get_software_switch_status( PPS_ON_OFF_SWITCH ) != FALSE)
  {
    tDouble pps_correction = 0.0, gps_pps_correction = 0.0, glonass_pps_correction = 0.0;
    tDouble pps_pulse_duration = 5E-1;
    boolean_t pps_inverted_polarity = FALSE;

    sw_config_get_param( CURRENT_CONFIG_DATA, RF_TIME_CORRECTION_ID, &pps_correction );
    sw_config_get_param( CURRENT_CONFIG_DATA, PPS_PULSE_DURATION_ID, &pps_pulse_duration );
    pps_inverted_polarity = (boolean_t)sw_config_get_software_switch_status( PPS_INVERTED_POLARITY_ON_OFF_SWITCH );
    sw_config_get_param( CURRENT_CONFIG_DATA, GPS_RF_TIME_CORRECTION_ID, &gps_pps_correction );
    sw_config_get_param( CURRENT_CONFIG_DATA, GLONASS_RF_TIME_CORRECTION_ID, &glonass_pps_correction );

    gnss_pps_set_time_delay( pps_correction );
    gnss_pps_set_pulse_duration( pps_pulse_duration );
    gnss_pps_set_polarity( pps_inverted_polarity );

    gnss_pps_set_rf_compensation( GNSS_SAT_TYPE_GPS, gps_pps_correction );
    gnss_pps_set_rf_compensation( GNSS_SAT_TYPE_GLONASS, glonass_pps_correction );

    if(sw_config_get_software_switch_status( POSITION_HOLD_ON_OFF_SWITCH )!= FALSE)
    {
      tDouble lat, lon, height;

      sw_config_get_param( CURRENT_CONFIG_DATA, POSITION_HOLD_LAT_ID, &lat );
      sw_config_get_param( CURRENT_CONFIG_DATA, POSITION_HOLD_LON_ID, &lon );
      sw_config_get_param( CURRENT_CONFIG_DATA, POSITION_HOLD_HEIGHT_ID, &height );

      gnss_pps_set_position_hold_llh_pos( lat, lon, height );
      gnss_pps_set_position_hold_status( TRUE );
    }

    if(sw_config_get_software_switch_status( TIMING_TRAIM_ON_OFF_SWITCH )!= FALSE)
    {
      tDouble traim_alarm = 15E-9;
      sw_config_get_param( CURRENT_CONFIG_DATA, TIMING_TRAIM_ALARM_ID, &traim_alarm );
      gnss_pps_enable_traim( traim_alarm );
    }

    {
      tUInt pps_op_mode_setting_1 = 0, pps_op_mode_setting_2 = 0, pps_auto_survey_samples = 0;

      sw_config_get_param( CURRENT_CONFIG_DATA, PPS_OPERATING_MODE_SETTING_1_ID, &pps_op_mode_setting_1 );
      sw_config_get_param( CURRENT_CONFIG_DATA, PPS_OPERATING_MODE_SETTING_2_ID, &pps_op_mode_setting_2 );
      sw_config_get_param( CURRENT_CONFIG_DATA, PPS_AUTO_SURVEY_SAMPLES_ID, &pps_auto_survey_samples );

      gnss_pps_set_reference_time( ( pps_reference_time_t )( ( pps_op_mode_setting_1 >> 4 ) & 0xF ) );
      gnss_pps_set_output_mode( ( pps_output_mode_t )( ( pps_op_mode_setting_1 >> 0 ) & 0xF ) );
      gnss_pps_set_fix_condition( ( fix_status_t )( ( pps_op_mode_setting_1 >> 8 ) & 0xF ) );
      gnss_pps_set_sat_threshold( ( tU8 )( ( pps_op_mode_setting_1 >> 16 ) & 0xFF ) );
      gnss_pps_set_elevation_mask( ( tInt )( ( pps_op_mode_setting_1 >> 24 ) & 0xFF ) );
      gnss_pps_set_constellation_mask( ( gnss_sat_type_mask_t )( ( pps_op_mode_setting_2 >> 0 ) & 0xFF ) );
      gnss_pps_set_auto_hold_samples( pps_auto_survey_samples );
    }

    gnss_pps_set_signal_on_off_status( TRUE );
  }
  else
  {
    gnss_pps_set_signal_on_off_status( FALSE );
  }
}

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return gnssapp_startup_time_t*
 *
 ***********************************************/
gnssapp_startup_time_t* gnssapp_get_startup_time( tVoid )
{
  gnssapp_startup_time_t* temp = NULL;

  gpOS_semaphore_wait( gnssapp_startup_time_sem );

  temp = &gnssapp_startup_time;

  gpOS_semaphore_signal( gnssapp_startup_time_sem );

  return temp;
}
/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return tVoid
 *
 ***********************************************/
tVoid gnssapp_reset_startup_time( tVoid )
{
  gpOS_semaphore_wait( gnssapp_startup_time_sem );

  gnssapp_startup_time.gnss_lib_set_timer_clock = gpOS_time_now();
  gnssapp_startup_time.suspend_restart_support_time = gnssapp_startup_time.gnss_lib_set_timer_clock;

  gpOS_semaphore_signal( gnssapp_startup_time_sem );
}
/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return tVoid
 *
 ***********************************************/
tUInt gnssapp_update_running_time( tVoid )
{
  tDouble delta_time_ms;

  tUInt running_time = 0;

  gpOS_clock_t cpu_time = gpOS_time_now();

  delta_time_ms = ( tDouble )gpOS_time_minus( cpu_time, gnssapp_startup_time.running_time_update_cpu_time ) / gpOS_timer_ticks_per_msec();

  gpOS_semaphore_wait( gnssapp_startup_time_sem );

  gnssapp_startup_time.running_time_ms += ( tUInt )delta_time_ms;

  gnssapp_startup_time.running_time_update_cpu_time = cpu_time;

  running_time = gnssapp_startup_time.running_time_ms;

  gpOS_semaphore_signal( gnssapp_startup_time_sem );

  return running_time;
}

/********************************************//**
 * \brief
 *
 * \param tUInt sens_buf_size: out param; size of the sensor msg number per second
 * \return boolean_t mems_presence indicates if the Mems init is requested
 *
 ***********************************************/
boolean_t gnssapp_sensors_presence(tUInt *sens_buf_size)
{
#if defined SW_CONFIG_PRIVATE_BLOCK
  sm_sensors_config_t *dr_config_ptr;

  dr_config_ptr = (sm_sensors_config_t *)((tUInt)sw_config_private_get_base_address() + sizeof(tInt));

  *sens_buf_size = dr_config_ptr->sens_buf_size;

#if defined DR_CODE_LINKED
  /* In case the DRCAN messages are logged, add the length of CAN_buf_size in the AMQ size */
  if (dr_config_ptr->en_can_log == TRUE)
  {
      *sens_buf_size += dr_config_ptr->CAN_buf_size;
  }

  dr_initialize_Callbacks();
#endif /*DR_CODE_LINKED */

  /* if one of Mems is requested */
  if( ((tInt)dr_config_ptr->enable_3Dacc == 0x01) ||
      ((tInt)dr_config_ptr->enable_3Dgyro == 0x01) ||
      ((tInt)dr_config_ptr->enable_analog == 0x01) ||
      ((tInt)dr_config_ptr->enable_pressure == 0x01) ||
      ((tInt)dr_config_ptr->enable_magn == 0x01) )
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
#else /*SW_CONFIG_PRIVATE_BLOCK*/
  return FALSE;
#endif /*SW_CONFIG_PRIVATE_BLOCK*/
}

/********************************************//**
 * \brief
 *
 * \param void
 * \return gpOS_task_t *: pointer on sens_task identifier
 *
 ***********************************************/
gpOS_task_t * gnssapp_get_ll_sensor_process_id(void)
{
#if defined SW_CONFIG_PRIVATE_BLOCK
  return sm_get_ll_process_id() ;
#else
  return NULL;
#endif
}

 /********************************************//**
 * \brief
 *
 * \param boolean Init: Lowpower setup update
 * \param gnssapp_lowpow_t * : low power setup
 * \return none
 *
 ***********************************************/
tVoid gnssapp_low_power_setup_update( gnss_app_lowpow_standby_type_t Standby, gnss_low_power_cyclic_mode_t * cyclic, gnss_low_power_periodic_mode_t * periodic)
{
  gnssapp_low_power_setup(GNSSAPP_LOW_POWER_UPDATE, Standby, cyclic, periodic );
}

/********************************************//**
 * \brief
 *
 * \param boolean Init: Lowpower setup update or init
 * \param gnssapp_lowpow_t * : low power setup
 * \return none
 *
 ***********************************************/
static tVoid gnssapp_low_power_setup( gnss_app_lowpow_setup_type_t setup_type, gnss_app_lowpow_standby_type_t Standby, gnss_low_power_cyclic_mode_t * cyclic, gnss_low_power_periodic_mode_t * periodic )
{
  boolean_t Activated = FALSE;

  if( setup_type == GNSSAPP_LOW_POWER_INIT )
  {
    gnss_low_power_private_config_t low_power_prv;
    tUInt param2,param3,param5;

    sw_config_get_param( CURRENT_CONFIG_DATA, LOW_POWER_CFG_PARAMS_2_ID, &param2);
    sw_config_get_param( CURRENT_CONFIG_DATA, LOW_POWER_CFG_PARAMS_3_ID, &param3);
    sw_config_get_param( CURRENT_CONFIG_DATA, LOW_POWER_CFG_PARAMS_5_ID, &param5);

    /* Adaptive, cyclic and Periodic common private settings */
    low_power_prv.eph_limit                 = (tU8) ((param2     )  & 0xFFU);
    low_power_prv.timer_scan_glonass_eph    = (tU8) ((param2 >> 8)  & 0xFFU);
    low_power_prv.gnss_eph_refresh_interval = (tU16)((param2 >> 16) & 0xFFU) * 60U;

    /* Cyclic private settings */
    low_power_prv.ehpe_timer_in      = (tU8) ((param3     ) & 0xFFU);
    low_power_prv.ehpe_timer_out     = (tU8) ((param3 >> 8) & 0xFFU);

    low_power_prv.NoFixTimeout2      = (tU16)((param5 >> 20) & 0x1FFU);

    // Setup power config
    gnss_low_power_init_prv_config_params( &low_power_prv );
  }

  GPS_DEBUG_MSG(( "[gnssapp pwr] gnssapp_low_power_setup\r\n"));
  if( cyclic != NULL )
  {
    Activated |= (((cyclic->reduced_type == TRUE) || (cyclic->duty_cycle_on_off == TRUE))?TRUE:FALSE);
  }
  if( periodic != NULL)
  {
    Activated |= ((periodic->periodic_mode == TRUE)?TRUE:FALSE);
  }

  if( Activated == TRUE )
  {
    GPS_DEBUG_MSG(( "[gnssapp pwr] low power mode activated\r\n"));
  }
  // Apply status to gnss
  gnss_low_power_set_status(Activated);

  if( setup_type == GNSSAPP_LOW_POWER_INIT )
  {
    gnss_low_power_init_config_params( cyclic, periodic );
  }
  else
  {
    gnss_low_power_set_config_params( cyclic, periodic );
  }

  // update power service
  svc_pwr_set_lowpower_allowed( Activated );
  if(Standby == GNSSAPP_LOW_POWER_STANDBY_DISABLE)
  {
    svc_pwr_set_standby_allowed( FALSE );
  }
  else
  {
    svc_pwr_set_standby_allowed( TRUE );
  }

  // store configuration for next run
  if( cyclic != NULL )
  {
    _clibs_memcpy( &gnssapp_lowpow.cyclic, cyclic, sizeof(gnss_low_power_cyclic_mode_t));
  }
  else
  {
    _clibs_memset( &gnssapp_lowpow.cyclic, 0, sizeof(gnss_low_power_cyclic_mode_t));
  }
  if( periodic != NULL )
  {
    _clibs_memcpy( &gnssapp_lowpow.periodic, periodic, sizeof(gnss_low_power_periodic_mode_t));
  }
  else
  {
    _clibs_memset( &gnssapp_lowpow.periodic, 0, sizeof(gnss_low_power_periodic_mode_t));
  }
  gnssapp_lowpow.Standby = Standby;

}

 /********************************************//**
 * \brief
 *
 * \param boolean Lowpower request for a fix : Software FixOnDemand
 * \param none
 * \return none
 *
 ***********************************************/
gpOS_error_t gnssapp_low_power_fix_request( void)
{
  gpOS_error_t error;

  if(GNSS_NO_ERROR == gnss_low_power_start(GNSS_STARTUP_WAKEUP_SW))
  {
    error = gpOS_SUCCESS;
  }
  else
  {
    error = gpOS_FAILURE;
  }

  return error;
}

 /********************************************//**
 * \brief
 *
 * \param boolean Lowpower fix abort
 * \param none
 * \return none
 *
 ***********************************************/
void gnssapp_low_power_fix_abort( void)
{
  gnss_low_power_periodic_fix_abort();
}


