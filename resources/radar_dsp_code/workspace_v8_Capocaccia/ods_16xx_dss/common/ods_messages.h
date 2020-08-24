/**
 *   @file  ods_messages.h
 *
 *   @brief
 *      This is the main header file for the Millimeter Wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ODS_MESSAGES_H
#define ODS_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif
/* UART API */
#include <ti/demo/io_interface/mmw_output.h>
#include <ti/demo/io_interface/mmw_config.h>

/* Map all common MmmDemo_* structures to OdsDemo_* */
#define OdsDemo_ClutterRemovalCfg           MmwDemo_ClutterRemovalCfg
#define OdsDemo_ExtendedMaxVelocityCfg      MmwDemo_ExtendedMaxVelocityCfg
#define OdsDemo_PeakGroupingCfg             MmwDemo_PeakGroupingCfg
#define OdsDemo_GuiMonSel                   MmwDemo_GuiMonSel
#define OdsDemo_CfarCfg                     MmwDemo_CfarCfg
#define OdsDemo_CalibDcRangeSigCfg          MmwDemo_CalibDcRangeSigCfg
#define OdsDemo_MultiObjBeamFormingCfg      MmwDemo_MultiObjBeamFormingCfg
#define OdsDemo_ADCBufCfg                   MmwDemo_ADCBufCfg
#define OdsDemo_measureRxChannelBiasCfg_t   MmwDemo_measureRxChannelBiasCfg_t
#define OdsDemo_output_message_header       MmwDemo_output_message_header
#define OdsDemo_output_message_tl           MmwDemo_output_message_tl
#define OdsDemo_output_message_dataObjDescr MmwDemo_output_message_dataObjDescr
#define OdsDemo_NearFieldCorrectionCfg      MmwDemo_NearFieldCorrectionCfg
//#define OdsDemo_AnaMonitorCfg               MmwDemo_AnaMonitorCfg
#define OdsDemo_BpmCfg                      MmwDemo_BpmCfg
#define OdsDemo_LvdsStreamCfg               MmwDemo_LvdsStreamCfg
#define OdsDemo_compRxChannelBiasCfg_t      MmwDemo_compRxChannelBiasCfg_t
#define OdsDemo_AnaMonitorCfg               MmwDemo_AnaMonitorCfg

/* Map all common MMDEMO_* MACRO to ODSDEMO_* */
#define ODSDEMO_OUTPUT_MSG_SEGMENT_LEN      MMWDEMO_OUTPUT_MSG_SEGMENT_LEN
#define ODSDEMO_OUTPUT_MSG_DETECTED_POINTS  MMWDEMO_OUTPUT_MSG_DETECTED_POINTS
#define ODSDEMO_OUTPUT_MSG_STATS            MMWDEMO_OUTPUT_MSG_STATS
#define ODSDEMO_OUTPUT_MSG_RANGE_PROFILE    MMWDEMO_OUTPUT_MSG_RANGE_PROFILE
#define ODSDEMO_OUTPUT_MSG_NOISE_PROFILE    MMWDEMO_OUTPUT_MSG_NOISE_PROFILE
#define ODSDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP   MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP
#define ODSDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP   MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP
#define ODSDEMO_OUTPUT_MSG_MAX             MMWDEMO_OUTPUT_MSG_MAX

/**
 * @brief
 *  Message types used in Millimeter Wave Demo for Mailbox communication 
 * between MSS and DSS.
 *
 * @details
 *  The enum is used to hold all the messages types used for Mailbox communication
 * between MSS and DSS in ods Demo.
 */
typedef enum OdsDemo_message_type_e 
{
    /*! @brief   message types for MSS to DSS communication */
    ODSDEMO_MSS2DSS_GUIMON_CFG = 0xFEED0001,
    ODSDEMO_MSS2DSS_CFAR_RANGE_CFG,
    ODSDEMO_MSS2DSS_CFAR_DOPPLER_CFG,
    ODSDEMO_MSS2DSS_PEAK_GROUPING_CFG,
    ODSDEMO_MSS2DSS_MULTI_OBJ_BEAM_FORM,
    ODSDEMO_MSS2DSS_CALIB_DC_RANGE_SIG,
    ODSDEMO_MSS2DSS_DETOBJ_SHIPPED,
    ODSDEMO_MSS2DSS_SET_DATALOGGER,
    ODSDEMO_MSS2DSS_ADCBUFCFG,
    ODSDEMO_MSS2DSS_EXTENDED_MAX_VELOCITY,
    ODSDEMO_MSS2DSS_CLUTTER_REMOVAL,
    ODSDEMO_MSS2DSS_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE,
    ODSDEMO_MSS2DSS_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE,
    ODSDEMO_MSS2DSS_BPM_CFG,
    ODSDEMO_MSS2DSS_NEAR_FIELD_CFG,
    ODSDEMO_MSS2DSS_CQ_SATURATION_MONITOR,
    ODSDEMO_MSS2DSS_LVDSSTREAM_CFG,
    ODSDEMO_MSS2DSS_CQ_SIGIMG_MONITOR,
    ODSDEMO_MSS2DSS_ANALOG_MONITOR,
 
    /*! @brief   message types for DSS to MSS communication */
    ODSDEMO_DSS2MSS_CONFIGDONE = 0xFEED0100,
    ODSDEMO_DSS2MSS_DETOBJ_READY,
    ODSDEMO_DSS2MSS_STOPDONE,
    ODSDEMO_DSS2MSS_ASSERT_INFO,
    ODSDEMO_DSS2MSS_ISR_INFO_ADDRESS,
    ODSDEMO_DSS2MSS_MEASUREMENT_INFO

}OdsDemo_message_type;


/*! @brief Software interrupt number used by DSS to signal exception from DSS to MSS */
#define ODSDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_DSS    1
/*! @brief Software interrupt ID on MSS corresponding to 
           @ref ODSDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_DSS */
#define ODSDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_MSS    SOC_XWR16XX_MSS_DSS2MSS_SW1_INT

/** @defgroup DSS_TO_MSS_EXCEPTION_IDS DSS to MSS Exception IDs
 *
 * @brief
 *  Exception ID definitions for DSS to MSS urgent exception signalling through 
 *  software interrupt.
 *
 @{ */

 /*! @brief   DSS to MSS chirp processing deadline miss exception ID */
#define ODSDEMO_DSS2MSS_CHIRP_PROC_DEADLINE_MISS_EXCEPTION 0

/*! @brief   DSS to MSS frame processing deadline miss exception ID */
#define ODSDEMO_DSS2MSS_FRAME_PROC_DEADLINE_MISS_EXCEPTION 1

/** @}*/ /* end defgroup DSS_TO_MSS_EXCEPTION_IDS */

/**
 * @brief
 *  TLV part of the message from DSS to MSS on data path detection information.
 *
 * @details
 *  The structure describes the message item 
 */
typedef struct OdsDemo_msgTlv_t
{
    /*! @brief   Payload type */
    uint32_t    type;

    /*! @brief   Length in bytes */
    uint32_t    length;

    /*! @brief Address of the payload */
    uint32_t   address;
} OdsDemo_msgTlv;

/**
 * @brief
 *  Message for reporting detection information from data path to MSS
 *
 * @details
 *  The structure defines the message body for detection information from from data path to MSS.
 */
typedef struct OdsDemo_detObjMsg_t
{
    /*! @brief Header of the detection information message */
    OdsDemo_output_message_header header;

    /*! @brief TLVs of the detection information */
    OdsDemo_msgTlv   tlv[ODSDEMO_OUTPUT_MSG_MAX];
} OdsDemo_detInfoMsg;

#define ODSDEMO_MAX_FILE_NAME_SIZE 128
/**
 * @brief
 *  Message for reporting DSS assertion information
 *
 * @details
 *  The structure defines the message body for the information
 *  on a DSS exception that should be forwarded to the MSS.
 */
typedef struct OdsDemo_dssAssertInfoMsg_t
{
    /*! @brief file name */
    char     file[ODSDEMO_MAX_FILE_NAME_SIZE];

    /*! @brief line number */
    uint32_t line;
} OdsDemo_dssAssertInfoMsg;

/**
 * @brief
 *  Message body used in Millimeter Wave Demo for passing configuration from MSS
 * to DSS.
 *
 * @details
 *  The union defines the message body for various configuration messages. 
 */
typedef union OdsDemo_message_body_u 
{
    /*! @brief   Gui Monitor Selection */
    OdsDemo_GuiMonSel     guiMonSel;

    /*! @brief   CFAR Range/Doppler configuraiton */
    OdsDemo_CfarCfg       cfarCfg;

    /*! @brief   Peak grouping configuration */
    OdsDemo_PeakGroupingCfg peakGroupingCfg;
    
    /*! @brief   Multi object beam forming configuration */
    OdsDemo_MultiObjBeamFormingCfg multiObjBeamFormingCfg;
    
    /*! @brief   Calibrate DC (zero) range signature */
    OdsDemo_CalibDcRangeSigCfg calibDcRangeSigCfg;
    
    /*! @brief   Extended maximum velocity configuration */
    OdsDemo_ExtendedMaxVelocityCfg extendedMaxVelocityCfg;
    
     /*! @brief   Near field correction configuration */
     OdsDemo_NearFieldCorrectionCfg nearFieldCorrectionCfg;

    /*! @brief   Clutter removal configuration */
    OdsDemo_ClutterRemovalCfg clutterRemovalCfg;
    
    /*! @brief   Detection Information message */
    OdsDemo_detInfoMsg     detObj;

    /*! @brief   ADCBUF configuration */
    OdsDemo_ADCBufCfg       adcBufCfg;

    /*! @brief   ChirpQuality - RX saturation monitor */
    rlRxSatMonConf_t            cqSatMonCfg;

    /*! @brief   ChirpQuality - Signal and image band energy monitor */
    rlSigImgMonConf_t           cqSigImgMonCfg;

    /*! @brief   ChirpQuality - Signal and image band energy monitor */
    OdsDemo_AnaMonitorCfg       anaMonCfg;
    
    /*! @brief   Datapath output logger setting */
    uint8_t               dataLogger;

    /*! @brief   Configuration for compensation for range bias 
                 and rx channel phase offset */
    OdsDemo_compRxChannelBiasCfg_t compRxChanCfg;

    /*! @brief   Configuration for measurement of range bias
                 and rx channel phase offset */
    OdsDemo_measureRxChannelBiasCfg_t measureRxChanCfg;

    /*! @brief   BPM cfg */
    OdsDemo_BpmCfg bpmCfg;

    /*! @brief   DSS assertion information */
    OdsDemo_dssAssertInfoMsg  assertInfo;
    
    /*! @brief Address of DSS to MSS ISR information storage, typically in HSRAM */
    uint32_t  dss2mssISRinfoAddress;
    
    /*! @brief  LVDS stream configuration */
    OdsDemo_LvdsStreamCfg lvdsStreamCfg;
} OdsDemo_message_body;

/*! @brief For advanced frame config, below define means the configuration given is
 * global at frame level and therefore it is broadcast to all sub-frames.
 */
#define ODSDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG (-1)

/**
 * @brief
 *  DSS/MSS communication messages
 *
 * @details
 *  The structure defines the message structure used to commuincate between MSS
 * and DSS.
 */
typedef struct OdsDemo_message_t
{
    /*! @brief   message type */
    OdsDemo_message_type      type;

    /*! @brief   message length : PROC_TBA does body need to be pointer and not a union structure?*/
    //uint32_t                  len;

    /*! @brief   Subframe number for which this message is applicable. 
                 Valid only when advanced frame config is used.
                 When advanced frame config is not used, this field should
                 be set to ODSDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG.
    */
    int8_t                   subFrameNum;

    /*! @brief  message body */
    OdsDemo_message_body      body;

} OdsDemo_message;

#ifdef __cplusplus
}
#endif

#endif /* ODS_MESSAGES_H */
