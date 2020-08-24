/*
 *   @file  dss_main.c
 *
 *   @brief
 *     This is the implementation of the DSS ODS TI Design 
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/c64p/Cache.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#include <ti/sysbios/utils/Load.h>


/* MMWSDK Include Files. */
#include <ti/drivers/soc/soc.h>
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

#include <ti/demo/utils/mmwDemo_monitor.h>
#include "dss_ods.h"
#include "dss_data_path.h"
#include "../common/ods_messages.h"
#include "dss_lvds_stream.h"

/* C674x mathlib */
/* Suppress the mathlib.h warnings
 *  #48-D: incompatible redefinition of macro "TRUE"
 *  #48-D: incompatible redefinition of macro "FALSE" 
 */
#pragma diag_push
#pragma diag_suppress 48
#include <ti/mathlib/mathlib.h>
#pragma diag_pop

/* Related to linker copy table for copying from L3 to L1PSRAM for example */
#include <cpy_tbl.h>

/* These address offsets are in bytes, when configure address offset in hardware,
   these values will be converted to number of 128bits
   Buffer at offset 0x0U is reserved by BSS, hence offset starts from 0x800
 */
#define ODS_DEMO_CQ_SIGIMG_ADDR_OFFSET          0x800U
#define ODS_DEMO_CQ_RXSAT_ADDR_OFFSET           0x1000U

/* CQ data is at 16 bytes alignment for mulitple chirps */
#define ODS_DEMO_CQ_DATA_ALIGNMENT            16U

#define MAR             (0x01848000)
#define Cache_PC        (1)
#define Cache_WTE       (2)
#define Cache_PCX       (4)
#define Cache_PFX       (8)
#define CACHE_0KCACHE   0x0

/**************************************************************************
 *************************** OdsDemo External DSS Functions ******************
 **************************************************************************/


/**************************************************************************
 *************************** Global Definitions ********************************
 **************************************************************************/
/**
 * @brief
 *  DSS stores demo output and DSS to MSS ISR information (for fast exception 
 *  signalling) in HSRAM.
 */
/*!   */
typedef struct OdsDemo_HSRAM_t_ {
#define ODS_DATAPATH_DET_PAYLOAD_SIZE (SOC_XWR16XX_DSS_HSRAM_SIZE -  sizeof(uint8_t))
    /*! @brief data path processing/detection related message payloads, these
               messages are signalled through DSS to MSS mailbox */ 
    uint8_t  dataPathDetectionPayload[ODS_DATAPATH_DET_PAYLOAD_SIZE];

    /*! @brief Information relayed through DSS triggering software interrupt to
               MSS. It stores one of the exception IDs @ref DSS_TO_MSS_EXCEPTION_IDS */
    uint8_t  dss2MssIsrInfo;
} OdsDemo_HSRAM_t;

#pragma DATA_SECTION(gHSRAM, ".demoSharedMem");
#pragma DATA_ALIGN(gHSRAM, 4);
OdsDemo_HSRAM_t gHSRAM;

/* Data memory for CQ:Rx Saturation - valid for 16bit CQ data format. 
  rlRfRxSaturationCqData_t is not used here, because the CQ data length 
  per chirp varies with number of slices. 
 */
uint8_t gCQRxSatMonMemory[SYS_COMMON_CQ_MAX_CHIRP_THRESHOLD * (RL_NUM_MON_SLICES_MAX + 1)];

/* Data memory for CQ:signal & image band monitor - valid for 16bit CQ data format. 
  rlRfSigImgPowerCqData_t is not used here, because the CQ data length 
  per chirp varies with number of slices. 
 */
uint16_t gCQRxSigImgMemory[SYS_COMMON_CQ_MAX_CHIRP_THRESHOLD * (RL_NUM_MON_SLICES_MAX + 1)];

#pragma DATA_SECTION(gCQRxSatMonMemory, ".l2data");
#pragma DATA_ALIGN(gCQRxSatMonMemory, 4);

#pragma DATA_SECTION(gCQRxSigImgMemory, ".l2data");
#pragma DATA_ALIGN(gCQRxSigImgMemory, 4);

/*! @brief Azimuth FFT size */
#define ODS_NUM_ANGLE_BINS 64

/*! @brief Flag to enable/disable two peak detection in azimuth for same range and velocity */
#define OdsDemo_AZIMUTH_TWO_PEAK_DETECTION_ENABLE 1

/*! @brief Threshold for two peak detection in azimuth for same range and velocity,
 *         if 2nd peak heigth > first peak height * this scale then declare
 *         2nd peak as detected. The peaks are in @ref OdsDemo_DSS_DataPathObj::azimuthMagSqr */
#define OdsDemo_AZIMUTH_TWO_PEAK_THRESHOLD_SCALE  (0.5)


#define OdsDemo_SPEED_OF_LIGHT_IN_METERS_PER_SEC (3.0e8)

//#define DBG

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
OdsDemo_DSS_MCB    gOdsDssMCB;

volatile cycleLog_t gCycleLog;

/* copy table related */
extern far COPY_TABLE _OdsDemo_fastCode_L1PSRAM_copy_table;

/**************************************************************************
 ************************* OdsDemo Functions Prototype  **********************
 **************************************************************************/

/* Copy table related */
static void OdsDemo_edmaBlockCopy(EDMA_Handle handle, uint32_t loadAddr, 
                                  uint32_t runAddr, uint16_t size);
static void OdsDemo_copyTable(EDMA_Handle handle, COPY_TABLE *tp);

/* Internal DataPath Functions */
int32_t OdsDemo_dssDataPathInit(void);
static int32_t OdsDemo_dssDataPathConfig(void);
static int32_t OdsDemo_dssDataPathStart(bool doRFStart);
static int32_t OdsDemo_dssDataPathStop(void);
static int32_t OdsDemo_dssDataPathProcessEvents(UInt event);
static int32_t OdsDemo_dssDataPathReconfig(OdsDemo_DSS_DataPathObj *obj);
static void OdsDemo_measurementResultOutput(OdsDemo_DSS_DataPathObj *obj);

/* Internal MMWave Call back Functions */
static int32_t OdsDemo_dssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, 
                                                 uint16_t sbLen, uint8_t *payload);
static void OdsDemo_dssMmwaveConfigCallbackFxn(MMWave_CtrlCfg* ptrCtrlCfg);
static void OdsDemo_dssMmwaveStartCallbackFxn(MMWave_CalibrationCfg* ptrCalibrationCfg);
static void OdsDemo_dssMmwaveStopCallbackFxn(void);
static void OdsDemo_dssMmwaveOpenCallbackFxn(MMWave_OpenCfg* ptrOpenCfg);
static void OdsDemo_dssMmwaveCloseCallbackFxn(void);

/* Internal Interrupt handler */
static void OdsDemo_dssChirpIntHandler(uintptr_t arg);
static void OdsDemo_dssFrameStartIntHandler(uintptr_t arg);

/* Internal OdsDemo Tasks running on DSS */
static void OdsDemo_dssInitTask(UArg arg0, UArg arg1);
static void OdsDemo_dssDataPathTask(UArg arg0, UArg arg1);
static void OdsDemo_dssMMWaveCtrlTask(UArg arg0, UArg arg1);

/* Internal odsDemo function to trigger DSS to MSS ISR for urgent exception signalling */
static void OdsDemo_triggerDss2MssISR(uint8_t dss2MssIsrInfo);

/* external sleep function when in idle (used in .cfg file) */
void OdsDemo_sleep(void);

static int32_t OdsDemo_dssSendProcessOutputToMSS
(
    uint8_t           *ptrHsmBuffer,
    uint32_t           outputBufSize,
    OdsDemo_DSS_DataPathObj   *obj
);
void OdsDemo_dssDataPathOutputLogging(    OdsDemo_DSS_DataPathObj   * dataPathObj);

/**************************************************************************
 *************************** OdsDemo DSS Functions **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Interrupt handler callback for chirp available. It runs in the ISR context.
 *
 *  @retval
 *      Not Applicable.
 */
 #ifdef DBG
 #define NUM_CHIRP_TIME_STAMPS 128
 uint32_t gChirpTimeStamp[NUM_CHIRP_TIME_STAMPS];
 #endif

static void OdsDemo_dssChirpIntHandler(uintptr_t arg)
{
    OdsDemo_DSS_DataPathObj * dpObj = &gOdsDssMCB.dataPathObj[gOdsDssMCB.subFrameIndx];

    if((gOdsDssMCB.state == ODSDEMO_DSS_STATE_STOPPED) ||
       (gOdsDssMCB.dataPathContext.interFrameProcToken <= 0))
    {
        gOdsDssMCB.stats.chirpIntSkipCounter++;
        return;
    }

#ifdef DBG
    if (dpObj->chirpCount < NUM_CHIRP_TIME_STAMPS)
    {
        gChirpTimeStamp[dpObj->chirpCount] =
            Cycleprofiler_getTimeStamp();
    }
#endif

    if (dpObj->chirpCount == 0)
    {
        OdsDemo_DSS_DataPathObj * dpObjPrev = dpObj;
        uint8_t subFrameIndxPrev;
        if (gOdsDssMCB.numSubFrames > 1)
        {
            if (gOdsDssMCB.subFrameIndx == 0)
            {
                subFrameIndxPrev = gOdsDssMCB.numSubFrames - 1;
            }
            else
            {
                subFrameIndxPrev = gOdsDssMCB.subFrameIndx - 1;
            }
            dpObjPrev = &gOdsDssMCB.dataPathObj[subFrameIndxPrev];
        }

        /* Note: this is valid after the first frame */
        dpObjPrev->timingInfo.interFrameProcessingEndMargin =
            Cycleprofiler_getTimeStamp() - dpObjPrev->timingInfo.interFrameProcessingEndTime -
            dpObjPrev->timingInfo.subFrameSwitchingCycles;
    }
    else if (dpObj->chirpCount == dpObj->numChirpsPerChirpEvent)
    {
        dpObj->timingInfo.chirpProcessingEndMarginMin =
            Cycleprofiler_getTimeStamp() - dpObj->timingInfo.chirpProcessingEndTime;
        dpObj->timingInfo.chirpProcessingEndMarginMax =
            dpObj->timingInfo.chirpProcessingEndMarginMin;
    }
    else
    {
        uint32_t margin = Cycleprofiler_getTimeStamp() - dpObj->timingInfo.chirpProcessingEndTime;
        if (margin > dpObj->timingInfo.chirpProcessingEndMarginMax)
        {
            dpObj->timingInfo.chirpProcessingEndMarginMax = margin;
        }
        if (margin < dpObj->timingInfo.chirpProcessingEndMarginMin)
        {
            dpObj->timingInfo.chirpProcessingEndMarginMin = margin;
        }
    }
    /* Increment interrupt counter for debugging purpose */
    gOdsDssMCB.stats.chirpIntCounter++;

    /* Check if previous chirp processing has completed */
    if (gOdsDssMCB.dataPathContext.chirpProcToken != 0)
    {
        OdsDemo_triggerDss2MssISR(ODSDEMO_DSS2MSS_CHIRP_PROC_DEADLINE_MISS_EXCEPTION);
        DebugP_assert(0);
    }

    gOdsDssMCB.dataPathContext.chirpProcToken++;

    /* Post event to notify chirp available interrupt */
    Event_post(gOdsDssMCB.eventHandle, ODSDEMO_CHIRP_EVT); // HG. CHIRP_EVENT CREATED
}

/**
 *  @b Description
 *  @n
 *      Interrupt handler callback for frame start ISR.
 *
 *  @retval
 *      Not Applicable.
 */
static void OdsDemo_dssFrameStartIntHandler(uintptr_t arg)
{
    if(gOdsDssMCB.state == ODSDEMO_DSS_STATE_STOPPED)
    {
        gOdsDssMCB.stats.frameIntSkipCounter++;
        return;
    }

    /* Check if previous chirp processing has completed */
    if (gOdsDssMCB.dataPathContext.interFrameProcToken != 0)
    {
        OdsDemo_triggerDss2MssISR(ODSDEMO_DSS2MSS_FRAME_PROC_DEADLINE_MISS_EXCEPTION);
        DebugP_assert(0);
    }

    gOdsDssMCB.dataPathContext.interFrameProcToken++;

    /* Increment interrupt counter for debugging purpose */    
    if (gOdsDssMCB.subFrameIndx == 0)
    {
        gOdsDssMCB.stats.frameStartIntCounter++;
    }

    /* Post event to notify frame start interrupt */
    Event_post(gOdsDssMCB.eventHandle, ODSDEMO_FRAMESTART_EVT);
}

/**
 *  @b Description
 *  @n
 *      Registered event callback function on DSS which is invoked by MMWAVE library when an event from the
 *      BSS is received.
 *
 *  @param[in]  msgId
 *      Message Identifier
 *  @param[in]  sbId
 *      Subblock identifier
 *  @param[in]  sbLen
 *      Length of the subblock
 *  @param[in]  payload
 *      Pointer to the payload buffer
 *
 *  @retval
 *      Always return 0. [Pass the event to the remote domain]
 */
static int32_t OdsDemo_dssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

    /* Debug Message: */
    /*System_printf ("Debug: ODSDemoDSS received BSS Event MsgId: %d [Sub Block Id: %d Sub Block Length: %d]\n",
                    msgId, sbId, sbLen); */

    /* Process the received message: */
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            /* Received Asychronous Message: */
            switch (asyncSB)
            {
                case RL_RF_AE_CPUFAULT_SB:
                {
                    /* BSS fault */
                    OdsDemo_dssAssert(0);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    /* BSS fault */
                    OdsDemo_dssAssert(0);
                    break;
                }
                case RL_RF_AE_ANALOG_FAULT_SB:
                {
                    /* Analog Fault */
                    OdsDemo_dssAssert(0);
                    break;
                }
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    /* This event should be handled by mmwave internally, ignore the event here */
                    break;
                }

                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                {
                    /* Post event to datapath task notify BSS events */
                    Event_post(gOdsDssMCB.eventHandle, ODSDEMO_BSS_FRAME_TRIGGER_READY_EVT);

                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    /* Increment the statistics to reports that the calibration failed */
                    gOdsDssMCB.stats.numFailedTimingReports++;
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    /* Increment the statistics to indicate that a calibration report was received */
                    gOdsDssMCB.stats.numCalibrationReports++;
                    break;
                }
                case RL_RF_AE_FRAME_END_SB:
                {
                    gOdsDssMCB.stats.bssStopAsyncEvt++;
                    /*Received Frame Stop async event from BSS. Post event to datapath task.*/
                    Event_post(gOdsDssMCB.eventHandle, ODSDEMO_BSS_STOP_COMPLETE_EVT);
                    break;
                }
                default:
                {
                    System_printf ("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
                    break;
                }
            }
            break;
        }
        default:
        {
            System_printf ("Error: Asynchronous message %d is NOT handled\n", msgId);
            break;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Registered config callback function on DSS which is invoked by MMWAVE library when the remote side
 *  has finished configure mmWaveLink and BSS. The configuration need to be saved on DSS and used for DataPath.
 *
 *  @param[in]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
static void OdsDemo_dssMmwaveConfigCallbackFxn(MMWave_CtrlCfg* ptrCtrlCfg)
{
    /* Save the configuration */
    memcpy((void *)(&gOdsDssMCB.cfg.ctrlCfg), (void *)ptrCtrlCfg, sizeof(MMWave_CtrlCfg));

    gOdsDssMCB.stats.configEvt++;

    /* Post event to notify configuration is done */
    Event_post(gOdsDssMCB.eventHandle, ODSDEMO_CONFIG_EVT);

    return;
}

/**
 *  @b Description
 *  @n
 *      Registered open callback function which is invoked when the mmWave module
 *      has been opened on the MSS
 *
 *  @param[in]  ptrOpenCfg
 *      Pointer to the open configuration
 *
 *  @retval
 *      Not applicable
 */
static void OdsDemo_dssMmwaveOpenCallbackFxn(MMWave_OpenCfg* ptrOpenCfg)
{
    /* Save the configuration */
    memcpy((void *)(&gOdsDssMCB.cfg.openCfg), (void *)ptrOpenCfg, sizeof(MMWave_OpenCfg));
    gOdsDssMCB.stats.openEvt++;
    return;
}

/**
 *  @b Description
 *  @n
 *      Registered close callback function which is invoked when the mmWave module
 *      has been close on the MSS
 *
 *  @retval
 *      Not applicable
 */
static void OdsDemo_dssMmwaveCloseCallbackFxn(void)
{
    gOdsDssMCB.stats.closeEvt++;
    return;
}

/**
 *  @b Description
 *  @n
 *      Registered Start callback function on DSS which is invoked by MMWAVE library
 *    when the remote side has started mmWaveLink and BSS. This Callback function passes
 *    the event to DataPath task.
 *
 *  @param[in]  ptrCalibrationCfg
 *      Pointer to the calibration configuration
 *
 *  @retval
 *      Not applicable
 */
static void OdsDemo_dssMmwaveStartCallbackFxn(MMWave_CalibrationCfg* ptrCalibrationCfg)
{
    gOdsDssMCB.stats.startEvt++;

    /* Post event to start is done */
    Event_post(gOdsDssMCB.eventHandle, ODSDEMO_START_EVT);
}

/**
 *  @b Description
 *  @n
 *      Registered Start callback function on DSS which is invoked by MMWAVE library
 *    when the remote side has stop mmWaveLink and BSS. This Callback function passes
 *    the event to DataPath task.
 *
 *  @retval
 *      Not applicable
 */
static void OdsDemo_dssMmwaveStopCallbackFxn(void)
{
    gOdsDssMCB.stats.stopEvt++;

}

/**
 *  @b Description
 *  @n
 *      Function to send a message to peer through Mailbox virtural channel
 *
 *  @param[in]  message
 *      Pointer to the Captuere demo message.
 *
 *  @retval
 *      Success    - 0
 *      Fail       < -1
 */
static int32_t OdsDemo_mboxWrite(OdsDemo_message    *message)
{
    int32_t                  retVal = -1;

    retVal = Mailbox_write (gOdsDssMCB.peerMailbox, (uint8_t*)message, sizeof(OdsDemo_message));
    if (retVal == sizeof(OdsDemo_message))
    {
        retVal = 0;
    }
    return retVal;
}

static void OdsDemo_cfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum)
{    
    /* if subFrameNum undefined, broadcast to all sub-frames */
    if(subFrameNum == ODSDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
    {
        uint8_t  indx;
        for(indx = 0; indx < RL_MAX_SUBFRAMES; indx++)
        {
            memcpy((void *)((uint32_t) &gOdsDssMCB.cliCfg[indx] + offset), srcPtr, size);
        }
        
    }
    else
    {
        /* Apply configuration to specific subframe (or to position zero for the legacy case
           where there is no advanced frame config) */
        memcpy((void *)((uint32_t) &gOdsDssMCB.cliCfg[subFrameNum] + offset), srcPtr, size);
    }
}

/**
 *  @b Description
 *  @n
 *      Function that acts upon receiving message that BSS has stopped
 *      successfully.
 *
 *  @retval
 *      Not applicable
 */
static void OdsDemo_bssStopDone(void)
{    
    /*Change state to stop_pending*/
    gOdsDssMCB.state = ODSDEMO_DSS_STATE_STOP_PENDING;    
    
    if(gOdsDssMCB.dataPathContext.interFrameProcToken == 0)
    {
        /*BSS stop message received after inter-frame processing
          is completed (including sending out UART data).
          */
         Event_post(gOdsDssMCB.eventHandle, ODSDEMO_STOP_COMPLETE_EVT);       
    }
    else
    {
        /*BSS stop message received during inter-frame processing.
          Switch to stop pending state and stop once inter frame
          processing done. Nothing to be done here.
          */
        
    }

}

/**
 *  @b Description
 *  @n
 *      The Task is used to handle  the mmw demo messages received from Mailbox virtual channel.
 *
 *  @param[in]  arg0
 *      arg0 of the Task. Not used
 *  @param[in]  arg1
 *      arg1 of the Task. Not used
 *
 *  @retval
 *      Not Applicable.
 */
static void OdsDemo_mboxReadTask(UArg arg0, UArg arg1)
{
    OdsDemo_message      message;
    int32_t              retVal = 0;
    int8_t               subFrameNum;
    uint32_t             log2NumAvgChirpsTemp;

    /* wait for new message and process all the messsages received from the peer */
    while(1)
    {
        Semaphore_pend(gOdsDssMCB.mboxSemHandle, BIOS_WAIT_FOREVER);

        /* Read the message from the peer mailbox: We are not trying to protect the read
         * from the peer mailbox because this is only being invoked from a single thread */
        retVal = Mailbox_read(gOdsDssMCB.peerMailbox, (uint8_t*)&message, sizeof(OdsDemo_message));
        if (retVal < 0)
        {
            /* Error: Unable to read the message. Setup the error code and return values */
            System_printf ("Error: Mailbox read failed [Error code %d]\n", retVal);
        }
        else if (retVal == 0)
        {
            /* We are done: There are no messages available from the peer execution domain. */
            continue;
        }
        else // HG: This is where the message is processed
        {
            /* Flush out the contents of the mailbox to indicate that we are done with the message. This will
             * allow us to receive another message in the mailbox while we process the received message. */
            Mailbox_readFlush (gOdsDssMCB.peerMailbox);

            /* Process the received message: */
            subFrameNum = message.subFrameNum;                     

            switch (message.type)
            {
                case ODSDEMO_MSS2DSS_GUIMON_CFG:
                {
                    /* Save guimon configuration */
                    OdsDemo_cfgUpdate((void *)&message.body.guiMonSel, 
                                         offsetof(OdsDemo_CliCfg_t, guiMonSel),
                                         sizeof(OdsDemo_GuiMonSel), subFrameNum);
                    break;
                }
                case ODSDEMO_MSS2DSS_CFAR_RANGE_CFG:
                {
                    OdsDemo_cfgUpdate((void *)&message.body.cfarCfg, 
                                         offsetof(OdsDemo_CliCfg_t, cfarCfgRange),
                                         sizeof(OdsDemo_CfarCfg), subFrameNum);
                    break;
                }
                case ODSDEMO_MSS2DSS_CFAR_DOPPLER_CFG:
                {
                    /* Save cfarDoppler configuration */
                    OdsDemo_cfgUpdate((void *)&message.body.cfarCfg, 
                                         offsetof(OdsDemo_CliCfg_t, cfarCfgDoppler),
                                         sizeof(OdsDemo_CfarCfg), subFrameNum);
                    
                    break;
                }
                case ODSDEMO_MSS2DSS_PEAK_GROUPING_CFG:
                {
                    /* Save Peak grouping configuration */
                    OdsDemo_cfgUpdate((void *)&message.body.peakGroupingCfg, 
                                         offsetof(OdsDemo_CliCfg_t, peakGroupingCfg),
                                         sizeof(OdsDemo_PeakGroupingCfg), subFrameNum);       
                    break;
                }
                case ODSDEMO_MSS2DSS_MULTI_OBJ_BEAM_FORM:
                {
                    /* Save multi object beam forming configuration */
                    OdsDemo_cfgUpdate((void*)&message.body.multiObjBeamFormingCfg,
                                      offsetof(OdsDemo_CliCfg_t, multiObjBeamFormingCfg),
                                      sizeof(OdsDemo_MultiObjBeamFormingCfg),
                                      subFrameNum);
                    break;
                }
                case ODSDEMO_MSS2DSS_CALIB_DC_RANGE_SIG:
                {
                    /* Save  of DC range antenna signature configuration */
                    OdsDemo_cfgUpdate((void *)&message.body.calibDcRangeSigCfg, 
                                         offsetof(OdsDemo_CliCfg_t, calibDcRangeSigCfg),
                                         sizeof(OdsDemo_CalibDcRangeSigCfg), subFrameNum);
                                      
                    log2NumAvgChirpsTemp = OdsDemo_floorLog2(message.body.calibDcRangeSigCfg.numAvgChirps);
                    /* Save log2NumAvgChirps  */
                    if (subFrameNum == ODSDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
                    {
                        uint8_t indx;
                        for(indx = 0; indx < RL_MAX_SUBFRAMES; indx++)
                        {
                            gOdsDssMCB.dataPathObj[indx].dcRangeSigCalibCntr = 0;
                            gOdsDssMCB.dataPathObj[indx].log2NumAvgChirps = log2NumAvgChirpsTemp;
                        }
                    }
                    else
                    {
                        gOdsDssMCB.dataPathObj[subFrameNum].dcRangeSigCalibCntr = 0;
                        gOdsDssMCB.dataPathObj[subFrameNum].log2NumAvgChirps = log2NumAvgChirpsTemp;
                    }
                    break;
                }
                case ODSDEMO_MSS2DSS_EXTENDED_MAX_VELOCITY:
                {
                    /* Save  of extended velocity configuration */
                    OdsDemo_cfgUpdate((void *)&message.body.extendedMaxVelocityCfg, 
                                         offsetof(OdsDemo_CliCfg_t, extendedMaxVelocityCfg),
                                         sizeof(OdsDemo_ExtendedMaxVelocityCfg), subFrameNum);
                    break;
                }
                case ODSDEMO_MSS2DSS_NEAR_FIELD_CFG:
                {
                    /* Save  of near field correction configuration */
                    OdsDemo_cfgUpdate((void *)&message.body.nearFieldCorrectionCfg,
                                         offsetof(OdsDemo_CliCfg_t, nearFieldCorrectionCfg),
                                         sizeof(OdsDemo_NearFieldCorrectionCfg), subFrameNum);
                    break;
                }
                case ODSDEMO_MSS2DSS_CLUTTER_REMOVAL:
                {
                    /* Save  clutter removal configuration */
                    OdsDemo_cfgUpdate((void *)&message.body.clutterRemovalCfg, 
                                         offsetof(OdsDemo_CliCfg_t, clutterRemovalCfg),
                                         sizeof(OdsDemo_ClutterRemovalCfg), subFrameNum);
                    break;
                }
                case ODSDEMO_MSS2DSS_ADCBUFCFG:
                {
                    /* Save ADCBUF configuration */ 
                    OdsDemo_cfgUpdate((void *)&message.body.adcBufCfg, 
                                         offsetof(OdsDemo_CliCfg_t, adcBufCfg),
                                         sizeof(OdsDemo_ADCBufCfg), subFrameNum);
                    break;
                }
                case ODSDEMO_MSS2DSS_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE:
                {
                    /* Save range bias and Rx channels phase compensation */
                    // HG: This is the compensation
                    // HG: The coefficients are being sent here to MSS to post it in the CLI
                    memcpy((void *) &gOdsDssMCB.cliCommonCfg.compRxChanCfg, (void *)&message.body.compRxChanCfg, sizeof(OdsDemo_compRxChannelBiasCfg_t));
                    break;
                }
                case ODSDEMO_MSS2DSS_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE:
                {
                    /* Save range bias and Rx channels phase compensation */
                    memcpy((void *) &gOdsDssMCB.cliCommonCfg.measureRxChanCfg, (void *)&message.body.measureRxChanCfg, sizeof(OdsDemo_measureRxChannelBiasCfg_t));
                    break;
                }
                case ODSDEMO_MSS2DSS_BPM_CFG:
                {
                    /* Save BPM cfg configuration */
                    OdsDemo_cfgUpdate((void *)&message.body.bpmCfg, 
                                         offsetof(OdsDemo_CliCfg_t, bpmCfg),
                                         sizeof(OdsDemo_BpmCfg), subFrameNum);
                    break;
                }                
                case ODSDEMO_MSS2DSS_LVDSSTREAM_CFG:
                {
                    /* Save LVDS Stream configuration */
                    OdsDemo_cfgUpdate((void *)&message.body.lvdsStreamCfg, 
                                         offsetof(OdsDemo_CliCfg_t, lvdsStreamCfg),
                                         sizeof(OdsDemo_LvdsStreamCfg), subFrameNum);
                    break;
                }
                case ODSDEMO_MSS2DSS_CQ_SATURATION_MONITOR:
                {
                    uint8_t     profileIdx;
                    profileIdx = message.body.cqSatMonCfg.profileIndx;
                    if(profileIdx < RL_MAX_PROFILES_CNT)
                    {
                        /* Save CQ RX saturation monitor configuration */
                        memcpy((void *) &gOdsDssMCB.cliCommonCfg.cqSatMonCfg[profileIdx],
                                (void *)&message.body.cqSatMonCfg,
                                sizeof(rlRxSatMonConf_t));
                    }
                    break;
                }
                case ODSDEMO_MSS2DSS_CQ_SIGIMG_MONITOR:
                {
                    uint8_t     profileIdx;
                    profileIdx = message.body.cqSigImgMonCfg.profileIndx;
                    if(profileIdx < RL_MAX_PROFILES_CNT)
                    {
                        /* Save CQ RX signal & Image band monitor configuration */
                        memcpy((void *) &gOdsDssMCB.cliCommonCfg.cqSigImgMonCfg[profileIdx],
                                (void *)&message.body.cqSigImgMonCfg,
                                sizeof(rlSigImgMonConf_t));
                    }
                    break;
                }
                case ODSDEMO_MSS2DSS_ANALOG_MONITOR:
                {
                    /* Save analog monitor configuration */
                    memcpy((void *) &gOdsDssMCB.cliCommonCfg.anaMonCfg,
                            (void *)&message.body.anaMonCfg,
                            sizeof(OdsDemo_AnaMonitorCfg));
                    break;
                }
                case ODSDEMO_MSS2DSS_DETOBJ_SHIPPED:
                {
                    OdsDemo_DSS_DataPathObj *dataPathCurrent, *dataPathNext;

                    dataPathCurrent = &gOdsDssMCB.dataPathObj[gOdsDssMCB.subFrameIndx];
                    dataPathCurrent->timingInfo.transmitOutputCycles =
                        Cycleprofiler_getTimeStamp() - dataPathCurrent->timingInfo.interFrameProcessingEndTime;

                    gOdsDssMCB.subFrameIndx++;
                    if (gOdsDssMCB.subFrameIndx == gOdsDssMCB.numSubFrames)
                    {
                        gOdsDssMCB.subFrameIndx = 0;
                    }

                    dataPathNext = &gOdsDssMCB.dataPathObj[gOdsDssMCB.subFrameIndx];
                    
                    /* execute subframe switching related functions */
                    if (gOdsDssMCB.numSubFrames > 1)
                    {
                        volatile uint32_t startTime;
                        startTime = Cycleprofiler_getTimeStamp();

                        OdsDemo_dssDataPathReconfig(dataPathNext);

                        dataPathCurrent->timingInfo.subFrameSwitchingCycles = Cycleprofiler_getTimeStamp() -
                                                                           startTime;
                    }
                    else
                    {
                        dataPathCurrent->timingInfo.subFrameSwitchingCycles = 0;
                    }

                    OdsDemo_checkDynamicConfigErrors(dataPathNext);

                    gOdsDssMCB.dataPathContext.interFrameProcToken--;

                    gOdsDssMCB.loggingBufferAvailable = 1;

                    /* Post event to complete stop operation, if pending */
                    if ((gOdsDssMCB.state == ODSDEMO_DSS_STATE_STOP_PENDING) && (gOdsDssMCB.subFrameIndx == 0))
                    {
                        Event_post(gOdsDssMCB.eventHandle, ODSDEMO_STOP_COMPLETE_EVT);
                    }
                    break;
                }
                case ODSDEMO_MSS2DSS_SET_DATALOGGER:
                {
                    gOdsDssMCB.cfg.dataLogger = message.body.dataLogger;
                    break;
                }
                default:
                {
                    /* Message not support */
                    System_printf ("Error: unsupport Mailbox message id=%d\n", message.type);
                    OdsDemo_dssAssert(0);
                    break;
                }
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This function is a callback funciton that invoked when a message is received from the peer.
 *
 *  @param[in]  handle
 *      Handle to the Mailbox on which data was received
 *  @param[in]  peer
 *      Peer from which data was received

 *  @retval
 *      Not Applicable.
 */
void OdsDemo_mboxCallback
(
    Mbox_Handle  handle,
    Mailbox_Type    peer
)
{
    /* Message has been received from the peer endpoint. Wakeup the mmWave thread to process
     * the received message. */
    Semaphore_post (gOdsDssMCB.mboxSemHandle);
}

/**
 *  @b Description
 *  @n
 *      Function to send detected objects to MSS logger.
 *
 *  @param[in]  ptrHsmBuffer
 *      Pointer to the output buffer
 *  @param[in]  outputBufSize
 *      Size of the output buffer
 *  @param[in]  obj
 *      Handle to the Data Path Object
 *
 *  @retval
 *      =0    Success
 *      <0    Failed
 */

int32_t OdsDemo_dssSendProcessOutputToMSS
(
    uint8_t           *ptrHsmBuffer,
    uint32_t           outputBufSize,
    OdsDemo_DSS_DataPathObj   *obj
)
{
    uint32_t            i;
    uint8_t             *ptrCurrBuffer;
    uint32_t            totalHsmSize = 0;
    uint32_t            totalPacketLen = sizeof(OdsDemo_output_message_header);
    uint32_t            itemPayloadLen;
    int32_t             retVal = 0;
    OdsDemo_message     message;
    OdsDemo_GuiMonSel   *pGuiMonSel;
    uint32_t            tlvIdx = 0;

    /* Get Gui Monitor configuration */
    pGuiMonSel = &obj->cliCfg->guiMonSel;

    /* Validate input params */
    if(ptrHsmBuffer == NULL)
    {
        retVal = -1;
        goto Exit;
    }


    /* Clear message to MSS */
    memset((void *)&message, 0, sizeof(OdsDemo_message));
    message.type = ODSDEMO_DSS2MSS_DETOBJ_READY;
    /* Header: */
    message.body.detObj.header.platform = 0xA1642;
    message.body.detObj.header.magicWord[0] = 0x0102;
    message.body.detObj.header.magicWord[1] = 0x0304;
    message.body.detObj.header.magicWord[2] = 0x0506;
    message.body.detObj.header.magicWord[3] = 0x0708;
    message.body.detObj.header.numDetectedObj = obj->numDetObj;
    message.body.detObj.header.version =    MMWAVE_SDK_VERSION_BUILD |   //DEBUG_VERSION
                                            (MMWAVE_SDK_VERSION_BUGFIX << 8) |
                                            (MMWAVE_SDK_VERSION_MINOR << 16) |
                                            (MMWAVE_SDK_VERSION_MAJOR << 24);

    /* Set pointer to HSM buffer */
    ptrCurrBuffer = ptrHsmBuffer;

    /* Put detected Objects in HSM buffer: sizeof(OdsDemo_objOut_t) * numDetObj  */
    if ((pGuiMonSel->detectedObjects == 1) && (obj->numDetObj > 0))
    {
        /* Add objects descriptor */
        OdsDemo_output_message_dataObjDescr descr;
        descr.numDetetedObj = obj->numDetObj;
        descr.xyzQFormat = obj->xyzOutputQFormat;
        itemPayloadLen = sizeof(OdsDemo_output_message_dataObjDescr);
        totalHsmSize += itemPayloadLen;
        if(totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }
        memcpy(ptrCurrBuffer, (void *)&descr, itemPayloadLen);

        /* Add array of objects */
        itemPayloadLen = sizeof(OdsDemo_detectedObj) * obj->numDetObj;
        totalHsmSize += itemPayloadLen;
        if(totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }
        memcpy(&ptrCurrBuffer[sizeof(OdsDemo_output_message_dataObjDescr)], (void *)obj->detObj2D, itemPayloadLen);

        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen + sizeof(OdsDemo_output_message_dataObjDescr);
        message.body.detObj.tlv[tlvIdx].type = ODSDEMO_OUTPUT_MSG_DETECTED_POINTS;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) ptrCurrBuffer;
        tlvIdx++;

        /* Incrementing pointer to HSM buffer */
        ptrCurrBuffer += itemPayloadLen + sizeof(OdsDemo_output_message_dataObjDescr);
        totalPacketLen += sizeof(OdsDemo_output_message_tl) + itemPayloadLen + sizeof(OdsDemo_output_message_dataObjDescr);
    }

    /* Sending range profile:  2bytes * numRangeBins */
    if (pGuiMonSel->logMagRange == 1)
    {
        itemPayloadLen = sizeof(uint16_t) * obj->numRangeBins;
        totalHsmSize += itemPayloadLen;
        if(totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }

        uint16_t *ptrMatrix = (uint16_t *)ptrCurrBuffer;
        for(i = 0; i < obj->numRangeBins; i++)
        {
            ptrMatrix[i] = obj->detMatrix[i*obj->numDopplerBins];
        }

        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen;
        message.body.detObj.tlv[tlvIdx].type = ODSDEMO_OUTPUT_MSG_RANGE_PROFILE;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) ptrCurrBuffer;
        tlvIdx++;

        /* Incrementing pointer to HSM buffer */
        ptrCurrBuffer = (uint8_t *)((uint32_t)ptrHsmBuffer + totalHsmSize);
        totalPacketLen += sizeof(OdsDemo_output_message_tl) + itemPayloadLen;
   }

    /* Sending range profile:  2bytes * numRangeBins */
    if (pGuiMonSel->noiseProfile == 1)
    {
        uint32_t maxDopIdx = obj->numDopplerBins/2 -1;
        itemPayloadLen = sizeof(uint16_t) * obj->numRangeBins;
        totalHsmSize += itemPayloadLen;
        if(totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }

        uint16_t *ptrMatrix = (uint16_t *)ptrCurrBuffer;
        for(i = 0; i < obj->numRangeBins; i++)
        {
            ptrMatrix[i] = obj->detMatrix[i*obj->numDopplerBins + maxDopIdx];
        }

        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen;
        message.body.detObj.tlv[tlvIdx].type = ODSDEMO_OUTPUT_MSG_NOISE_PROFILE;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) ptrCurrBuffer;
        tlvIdx++;

        /* Incrementing pointer to HSM buffer */
        ptrCurrBuffer = (uint8_t *)((uint32_t)ptrHsmBuffer + totalHsmSize);
        totalPacketLen += sizeof(OdsDemo_output_message_tl) + itemPayloadLen;
   }

    /* Sending range Azimuth Heat Map */
    if (pGuiMonSel->rangeAzimuthHeatMap == 1)
    {
        itemPayloadLen = obj->numRangeBins * obj->numVirtualAntAzim * sizeof(cmplx16ImRe_t);
        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen;
        message.body.detObj.tlv[tlvIdx].type = ODSDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) obj->azimuthStaticHeatMap;
        tlvIdx++;

        totalPacketLen += sizeof(OdsDemo_output_message_tl) + itemPayloadLen;
    }


    /* Sending range Doppler Heat Map  */
    if (pGuiMonSel->rangeDopplerHeatMap == 1)
    {
        itemPayloadLen = obj->numRangeBins * obj->numDopplerBins * sizeof(uint16_t);
        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen;
        message.body.detObj.tlv[tlvIdx].type = ODSDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) obj->detMatrix;
        tlvIdx++;

        totalPacketLen += sizeof(OdsDemo_output_message_tl) + itemPayloadLen;
    }

    /* Sending stats information  */
    if (pGuiMonSel->statsInfo == 1)
    {
        OdsDemo_output_message_stats stats;
        itemPayloadLen = sizeof(OdsDemo_output_message_stats);
        totalHsmSize += itemPayloadLen;
        if(totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }

        stats.interChirpProcessingMargin = (uint32_t) (obj->timingInfo.chirpProcessingEndMarginMin/DSP_CLOCK_MHZ);
        stats.interFrameProcessingMargin = (uint32_t) (obj->timingInfo.interFrameProcessingEndMargin/DSP_CLOCK_MHZ);
        stats.interFrameProcessingTime = (uint32_t) (obj->timingInfo.interFrameProcCycles/DSP_CLOCK_MHZ);
        stats.transmitOutputTime = (uint32_t) (obj->timingInfo.transmitOutputCycles/DSP_CLOCK_MHZ);
        stats.activeFrameCPULoad = obj->timingInfo.activeFrameCPULoad;
        stats.interFrameCPULoad = obj->timingInfo.interFrameCPULoad;
        memcpy(ptrCurrBuffer, (void *)&stats, itemPayloadLen);

        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen;
        message.body.detObj.tlv[tlvIdx].type = ODSDEMO_OUTPUT_MSG_STATS;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) ptrCurrBuffer;;
        tlvIdx++;

        /* Incrementing pointer to HSM buffer */
        ptrCurrBuffer = (uint8_t *)((uint32_t)ptrHsmBuffer + totalHsmSize);
        totalPacketLen += sizeof(OdsDemo_output_message_tl) + itemPayloadLen;
    }

    if( retVal == 0)
    {
        message.body.detObj.header.numTLVs = tlvIdx;
        /* Round up packet length to multiple of ODSDEMO_OUTPUT_MSG_SEGMENT_LEN */
        message.body.detObj.header.totalPacketLen = ODSDEMO_OUTPUT_MSG_SEGMENT_LEN *
                ((totalPacketLen + (ODSDEMO_OUTPUT_MSG_SEGMENT_LEN-1))/ODSDEMO_OUTPUT_MSG_SEGMENT_LEN);
        message.body.detObj.header.timeCpuCycles =  Cycleprofiler_getTimeStamp();
        message.body.detObj.header.frameNumber = gOdsDssMCB.stats.frameStartIntCounter;
        message.body.detObj.header.subFrameNumber = gOdsDssMCB.subFrameIndx;
        if (OdsDemo_mboxWrite(&message) != 0)
        {
            retVal = -1;
        }
    }
Exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function to trigger DSS to MSS ISR for fast signalling
 *
 *  @retval
 *      Not Applicable.
 */
static void OdsDemo_triggerDss2MssISR(uint8_t dss2MssIsrInfo)
{
    int32_t errCode;

    gHSRAM.dss2MssIsrInfo = dss2MssIsrInfo;
    if (SOC_triggerDSStoMSSsoftwareInterrupt(gOdsDssMCB.socHandle, 
            ODSDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_DSS, &errCode) != 0)
    {
        System_printf("Failed to trigger software interrupt %d, error code = %d\n",
            ODSDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_DSS, errCode);
    }
}

/**
 *  @b Description
 *  @n
 *      Function to send data path detection output.
 *
 *  @retval
 *      Not Applicable.
 */
void OdsDemo_dssDataPathOutputLogging(OdsDemo_DSS_DataPathObj   * dataPathObj)
{
    int32_t errCode;
    
    /* Sending detected objects to logging buffer and shipped out from MSS UART */
    if (gOdsDssMCB.loggingBufferAvailable == 1)
    {
        /* Set the logging buffer available flag to be 0 */
        gOdsDssMCB.loggingBufferAvailable = 0;

        /*If LVDS user data streaming is enabled for this subframe, send user data through LVDS as well.*/ 
        if(dataPathObj->cliCfg->lvdsStreamCfg.isSwEnabled != 0) 
        {       
            /* Delete previous SW session if it exists. SW session is being
               reconfigured every frame/subframe because number of detected objects
               may change every frame/subframe which implies that the size of
               the streamed data may change. */        
            if(gOdsDssMCB.lvdsStream.swSessionHandle != NULL)
            {
                OdsDemo_LVDSStreamDeleteSwSession(gOdsDssMCB.lvdsStream.swSessionHandle);
            }    
            /* Configure SW session for this subframe */
            if (OdsDemo_LVDSStreamSwConfig(dataPathObj) < 0)
            {
                System_printf("Failed LVDS stream SW configuration\n");
                return;
            }
            /* Populate user data header that will be streamed out*/
            gOdsDssMCB.lvdsStream.userDataHeader.frameNum  = gOdsDssMCB.stats.frameStartEvt;
            gOdsDssMCB.lvdsStream.userDataHeader.detObjNum = dataPathObj->numDetObj;
            gOdsDssMCB.lvdsStream.userDataHeader.reserved  = 0xABCD;
            
            /* If SW LVDS stream is enabled, start the session here. User data will imediatelly
               start to stream over LVDS.*/
            if(CBUFF_activateSession (gOdsDssMCB.lvdsStream.swSessionHandle, &errCode) < 0)
            {
                System_printf("Failed to activate CBUFF session for LVDS stream SW. errCode=%d\n",errCode);
                return;
            }
        }    

        /* Save output in logging buffer - HSRAM memory and a message is sent to MSS to notify
           logging buffer is ready */
        if (OdsDemo_dssSendProcessOutputToMSS((uint8_t *)&gHSRAM.dataPathDetectionPayload[0],
                                             (uint32_t)ODS_DATAPATH_DET_PAYLOAD_SIZE,
                                             dataPathObj) < 0)
        {
                /* Increment logging error */
                gOdsDssMCB.stats.detObjLoggingErr++;
        }
    }
    else
    {
        /* Logging buffer is not available, skip saving detected objects to logging buffer */
        gOdsDssMCB.stats.detObjLoggingSkip++;
    }
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Initialization on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t OdsDemo_dataPathAdcBufInit(OdsDemo_DSS_dataPathContext_t *context)
{
    ADCBuf_Params       ADCBufparams;

    /*****************************************************************************
     * Initialize ADCBUF driver
     *****************************************************************************/
    ADCBuf_init();

    /* ADCBUF Params initialize */
    ADCBuf_Params_init(&ADCBufparams);
    ADCBufparams.chirpThresholdPing = 1;
    ADCBufparams.chirpThresholdPong = 1;
    ADCBufparams.continousMode  = 0;

    /* Open ADCBUF driver */
   context->adcbufHandle = ADCBuf_open(0, &ADCBufparams);
    if (context->adcbufHandle == NULL)
    {
        System_printf("Error: ODSDemoDSS Unable to open the ADCBUF driver\n");
        return -1;
    }
    System_printf("Debug: ODSDemoDSS ADCBUF Instance(0) %p has been opened successfully\n", 
        context->adcbufHandle);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Performs linker generated copy table copy using EDMA. Currently this is
 *      used to page in fast code from L3 to L1PSRAM.
 *  @param[in]  handle EDMA handle
 *  @param[in]  tp Pointer to copy table
 *
 *  @retval
 *      Not Applicable.
 */
static void OdsDemo_copyTable(EDMA_Handle handle, COPY_TABLE *tp)
{
    uint16_t i;
    COPY_RECORD crp;
    uint32_t loadAddr;
    uint32_t runAddr;

    for (i = 0; i < tp->num_recs; i++)
    {
        crp = tp->recs[i];
        loadAddr = (uint32_t)crp.load_addr;
        runAddr = (uint32_t)crp.run_addr;

        /* currently we use only one count of EDMA which is 16-bit so we cannot
           handle tables bigger than 64 KB */
        OdsDemo_dssAssert(crp.size <= 65536U);

        if (crp.size)
        {
            OdsDemo_edmaBlockCopy(handle, loadAddr, runAddr, crp.size);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Performs simple block copy using EDMA. Used for the purpose of copying
 *      linker table for L3 to L1PSRAM copy. memcpy cannot be used because there is
 *      no data bus access to L1PSRAM.
 *
 *  @param[in]  handle EDMA handle
 *  @param[in]  loadAddr load address
 *  @param[in]  runAddr run address
 *  @param[in]  size size in bytes
 *
 *  @retval
 *      Not Applicable.
 */
static void OdsDemo_edmaBlockCopy(EDMA_Handle handle, uint32_t loadAddr, uint32_t runAddr, uint16_t size)
{
    EDMA_channelConfig_t config;
    volatile bool isTransferDone;

    config.channelId = EDMA_TPCC0_REQ_FREE_0;
    config.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.paramId = (uint16_t)EDMA_TPCC0_REQ_FREE_0;
    config.eventQueueId = 0;

    config.paramSetConfig.sourceAddress = (uint32_t) SOC_translateAddress((uint32_t)loadAddr,
        SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    config.paramSetConfig.destinationAddress = (uint32_t) SOC_translateAddress((uint32_t)runAddr,
        SOC_TranslateAddr_Dir_TO_EDMA, NULL);

    config.paramSetConfig.aCount = size;
    config.paramSetConfig.bCount = 1U;
    config.paramSetConfig.cCount = 1U;
    config.paramSetConfig.bCountReload = 0U;

    config.paramSetConfig.sourceBindex = 0U;
    config.paramSetConfig.destinationBindex = 0U;

    config.paramSetConfig.sourceCindex = 0U;
    config.paramSetConfig.destinationCindex = 0U;

    config.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    config.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_A;
    config.paramSetConfig.transferCompletionCode = (uint8_t) EDMA_TPCC0_REQ_FREE_0;
    config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;

    /* don't care because of linear addressing modes above */
    config.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;

    config.paramSetConfig.isStaticSet = false;
    config.paramSetConfig.isEarlyCompletion = false;
    config.paramSetConfig.isFinalTransferInterruptEnabled = true;
    config.paramSetConfig.isIntermediateTransferInterruptEnabled = false;
    config.paramSetConfig.isFinalChainingEnabled = false;
    config.paramSetConfig.isIntermediateChainingEnabled = false;
    config.transferCompletionCallbackFxn = NULL;
    config.transferCompletionCallbackFxnArg = NULL;

    if (EDMA_configChannel(handle, &config, false) != EDMA_NO_ERROR)
    {
        OdsDemo_dssAssert(0);
    }

    if (EDMA_startDmaTransfer(handle, config.channelId) != EDMA_NO_ERROR)
    {
        OdsDemo_dssAssert(0);
    }

    /* wait until transfer done */
    do {
        if (EDMA_isTransferComplete(handle,
                config.paramSetConfig.transferCompletionCode,
                (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            OdsDemo_dssAssert(0);
        }
    } while (isTransferDone == false);

    /* make sure to disable channel so it is usable later */
    EDMA_disableChannel(handle, config.channelId, config.channelType);
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Initialization on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
int32_t OdsDemo_dssDataPathInit(void)
{
    int32_t retVal;
    SOC_SysIntListenerCfg  socIntCfg;
    int32_t errCode;
    OdsDemo_DSS_dataPathContext_t *context;
    uint32_t subFrameIndx;

    context = &gOdsDssMCB.dataPathContext;

    for(subFrameIndx = 0; subFrameIndx < RL_MAX_SUBFRAMES; subFrameIndx++)
    {
        OdsDemo_DSS_DataPathObj *obj;

        obj = &gOdsDssMCB.dataPathObj[subFrameIndx];
        OdsDemo_dataPathObjInit(obj,
                                context,
                                &gOdsDssMCB.cliCfg[subFrameIndx],
                                &gOdsDssMCB.cliCommonCfg,
                                &gOdsDssMCB.cfg);
        OdsDemo_dataPathInit1Dstate(obj);
    }

    retVal = OdsDemo_dataPathInitEdma(context);
    if (retVal < 0)
    {
        return -1;
    }

    /* Copy code from L3 to L1PSRAM, this code related to data path processing */
    OdsDemo_copyTable(context->edmaHandle[0], &_OdsDemo_fastCode_L1PSRAM_copy_table);

    retVal = OdsDemo_dataPathAdcBufInit(context);
    if (retVal < 0)
    {
        return -1;
    }

    /* Register chirp interrupt listener */
    socIntCfg.systemInterrupt  = SOC_XWR16XX_DSS_INTC_EVENT_CHIRP_AVAIL;
    socIntCfg.listenerFxn      = OdsDemo_dssChirpIntHandler; // HG. This is the function listening all the time for CHIRP_EVT, it can be from PING or PONG. Whenever each of them reach 8 chirps
    socIntCfg.arg              = (uintptr_t)NULL;

    if (SOC_registerSysIntListener(gOdsDssMCB.socHandle, &socIntCfg, &errCode) == NULL)
    {
        System_printf("Error: Unable to register chirp interrupt listener , error = %d\n", errCode);
        return -1;
    }

    /* Register frame start interrupt listener */
    socIntCfg.systemInterrupt  = SOC_XWR16XX_DSS_INTC_EVENT_FRAME_START;
    socIntCfg.listenerFxn      = OdsDemo_dssFrameStartIntHandler;
    socIntCfg.arg              = (uintptr_t)NULL;
    if (SOC_registerSysIntListener(gOdsDssMCB.socHandle, &socIntCfg, &errCode) == NULL)
    {
        System_printf("Error: Unable to register frame start interrupt listener , error = %d\n", errCode);
        return -1;
    }

    /* Initialize detected objects logging */
    gOdsDssMCB.loggingBufferAvailable = 1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to configure CQ.
 *
 *  @param[in] ptrDataPathObj Pointer to data path object.
 *
 *  @retval
 *      0 if no error, else error (there will be system prints for these).
 */
static int32_t OdsDemo_dssDataPathConfigCQ(OdsDemo_DSS_DataPathObj *ptrDataPathObj)
{
    OdsDemo_AnaMonitorCfg*      ptrAnaMonitorCfg;
    ADCBuf_CQConf               cqConfig;
    rlRxSatMonConf_t*           ptrSatMonCfg;
    rlSigImgMonConf_t*          ptrSigImgMonCfg;
    int32_t                     retVal;
    uint16_t                    cqChirpSize;

    /* Get analog monitor configuration */
    ptrAnaMonitorCfg = &ptrDataPathObj->cliCommonCfg->anaMonCfg;
    
    /* Config mmwaveLink to enable Saturation monitor - CQ2 */
    ptrSatMonCfg = &ptrDataPathObj->cliCommonCfg->cqSatMonCfg[ptrDataPathObj->validProfileIdx];
    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        retVal = mmwDemo_cfgRxSaturationMonitor(ptrSatMonCfg);
        if( retVal != 0)
        {
            System_printf ("Error: rlRfRxIfSatMonConfig returns error = %d for profile(%d)\n", retVal, ptrSatMonCfg->profileIndx);
        }
    }

    /* Config mmwaveLink to enable Saturation monitor - CQ1 */
    ptrSigImgMonCfg = &ptrDataPathObj->cliCommonCfg->cqSigImgMonCfg[ptrDataPathObj->validProfileIdx];
    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        retVal = mmwDemo_cfgRxSigImgMonitor(ptrSigImgMonCfg);
        if(retVal != 0)
        {
            System_printf ("Error: rlRfRxSigImgMonConfig returns error = %d for profile(%d)\n", retVal, ptrSigImgMonCfg->profileIndx);
        }
    }
    
    retVal = mmwDemo_cfgAnalogMonitor(ptrAnaMonitorCfg);
    if (retVal != 0)
    {
        System_printf ("Error: rlRfAnaMonConfig returns error = %d\n", retVal);

        OdsDemo_dssAssert(0);
    }
    
    if(ptrAnaMonitorCfg->rxSatMonEn || ptrAnaMonitorCfg->sigImgMonEn)
    {
        /* CQ driver config */
        memset((void *)&cqConfig, 0, sizeof(ADCBuf_CQConf));
        cqConfig.cqDataWidth = 0; /* 16bit for mmw demo */
        cqConfig.cq1AddrOffset = ODS_DEMO_CQ_SIGIMG_ADDR_OFFSET;      /* CQ1 starts from the beginning of the buffer */
        cqConfig.cq2AddrOffset = ODS_DEMO_CQ_RXSAT_ADDR_OFFSET;       /* Address shouldb be 16 bytes aligned */

        retVal = ADCBuf_control(ptrDataPathObj->context->adcbufHandle, ADCBufMMWave_CMD_CONF_CQ, (void *)&cqConfig);
        if (retVal < 0)
        {
            System_printf ("Error: MMWDemoDSS Unable to configure the CQ\n");
            return -1;
        }
    }

    /* Save config pointer */
    ptrDataPathObj->datapathCQ.rxSatMonCfg = ptrSatMonCfg;
    ptrDataPathObj->datapathCQ.sigImgMonCfg = ptrSigImgMonCfg;
    ptrDataPathObj->datapathCQ.anaMonCfg = ptrAnaMonitorCfg;

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        /* Save CQ-Signal & Image band energy info in datapath object */
        ptrDataPathObj->datapathCQ.sigImgMonAddr = ADCBUF_MMWave_getCQBufAddr(ptrDataPathObj->context->adcbufHandle,
                                                                                   ADCBufMMWave_CQType_CQ1,
                                                                                   &retVal);
        OdsDemo_dssAssert (ptrDataPathObj->datapathCQ.sigImgMonAddr != NULL);

        /* This is for 16bit format in mmw demo, signal/image band data has 2 bytes/slice
           For other format, please check DFP interface document 
         */
        cqChirpSize = (ptrSigImgMonCfg->numSlices + 1) * sizeof(uint16_t);
        cqChirpSize = (cqChirpSize + ODS_DEMO_CQ_DATA_ALIGNMENT -1U) /ODS_DEMO_CQ_DATA_ALIGNMENT * ODS_DEMO_CQ_DATA_ALIGNMENT;
        ptrDataPathObj->datapathCQ.sigImgMonDataSizePerChirp = cqChirpSize;
        ptrDataPathObj->datapathCQ.sigImgMonTotalSize = cqChirpSize * ptrDataPathObj->numChirpsPerChirpEvent;

        /* Found out CQ data memory address */
        ptrDataPathObj->datapathCQ.sigImgData = gCQRxSigImgMemory;

    }
    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        /* Save CQ-Rx Saturation info in datapath object */
        ptrDataPathObj->datapathCQ.satMonAddr = ADCBUF_MMWave_getCQBufAddr(ptrDataPathObj->context->adcbufHandle,
                                                                               ADCBufMMWave_CQType_CQ2,
                                                                               &retVal);
        OdsDemo_dssAssert    (ptrDataPathObj->datapathCQ.satMonAddr != NULL);

        /* This is for 16bit format in mmw demo, saturation data has one byter/slice
           For other format, please check DFP interface document 
         */
        cqChirpSize = (ptrSatMonCfg->numSlices + 1) * sizeof(uint8_t);
        cqChirpSize = (cqChirpSize + ODS_DEMO_CQ_DATA_ALIGNMENT -1U) /ODS_DEMO_CQ_DATA_ALIGNMENT * ODS_DEMO_CQ_DATA_ALIGNMENT;
        ptrDataPathObj->datapathCQ.satMonDataSizePerChirp = cqChirpSize;
         
        ptrDataPathObj->datapathCQ.satMonTotalSize = cqChirpSize * ptrDataPathObj->numChirpsPerChirpEvent;

        /* Found out CQ data memory address */
        ptrDataPathObj->datapathCQ.rxSatData = gCQRxSatMonMemory;

    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to configure ADCBUF driver based on CLI inputs.
 *  @param[in] ptrDataPathObj Pointer to data path object.
 *
 *  @retval
 *      0 if no error, else error (there will be system prints for these).
 */
static int32_t OdsDemo_dssDataPathConfigAdcBuf(OdsDemo_DSS_DataPathObj *ptrDataPathObj)
{
    ADCBuf_dataFormat   dataFormat;
    ADCBuf_RxChanConf   rxChanConf;
    int32_t             retVal;
    uint8_t             channel;
    uint8_t             numBytePerSample = 0;
    MMWave_OpenCfg*     ptrOpenCfg;
    uint32_t            chirpThreshold;
    uint32_t            rxChanMask = 0xF;
    OdsDemo_DSS_dataPathContext_t *context = ptrDataPathObj->context;
    OdsDemo_ADCBufCfg   *adcBufCfg = &ptrDataPathObj->cliCfg->adcBufCfg;

    /* Get data path object and control configuration */
    ptrOpenCfg = &gOdsDssMCB.cfg.openCfg;

    /*****************************************************************************
     * Data path :: ADCBUF driver Configuration
     *****************************************************************************/
    /* On XWR16xx only channel non-interleaved mode is supported */
    if(adcBufCfg->chInterleave != 1)
    {
        OdsDemo_dssAssert(0); /* Not supported */
    }

    /* Populate data format from configuration */
    dataFormat.adcOutFormat       = adcBufCfg->adcFmt;
    dataFormat.channelInterleave  = adcBufCfg->chInterleave;
    dataFormat.sampleInterleave   = adcBufCfg->iqSwapSel;

    /* Disable all ADCBuf channels */
    if ((retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_CHANNEL_DISABLE, (void *)&rxChanMask)) < 0)
    {
       System_printf("Error: Disable ADCBuf channels failed with [Error=%d]\n", retVal);
       return retVal;
    }

    retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_CONF_DATA_FORMAT, (void *)&dataFormat);
    if (retVal < 0)
    {
        System_printf ("Error: ODSDemoDSS Unable to configure the data formats\n");
        return -1;
    }

    memset((void*)&rxChanConf, 0, sizeof(ADCBuf_RxChanConf));

    chirpThreshold = ptrDataPathObj->numChirpsPerChirpEvent;
    numBytePerSample = ptrDataPathObj->numBytePerSample;

    /* Enable Rx Channels */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        if(ptrOpenCfg->chCfg.rxChannelEn & (0x1U << channel))
        {
            /* Populate the receive channel configuration: */
            rxChanConf.channel = channel;
            retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf);
            if (retVal < 0)
            {
                System_printf("Error: ODSDemoDSS ADCBuf Control for Channel %d Failed with error[%d]\n", channel, retVal);
                return -1;
            }
            rxChanConf.offset  += ptrDataPathObj->numAdcSamples * numBytePerSample * chirpThreshold;
        }
    }

    /* Set ping/pong chirp threshold: */
    retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_SET_PING_CHIRP_THRESHHOLD,
                            (void *)&chirpThreshold);
    if(retVal < 0)
    {
        System_printf("Error: ADCbuf Ping Chirp Threshold Failed with Error[%d]\n", retVal);
        return -1;
    }
    retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_SET_PONG_CHIRP_THRESHHOLD,
                            (void *)&chirpThreshold);
    if(retVal < 0)
    {
        System_printf("Error: ADCbuf Pong Chirp Threshold Failed with Error[%d]\n", retVal);
        return -1;
    }

    return 0;
}

static uint16_t OdsDemo_getChirpStartIdx(MMWave_CtrlCfg *cfg, uint8_t subFrameIndx)
{
    if (cfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(cfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].chirpStartIdx);
    }
    else 
    {
        return(cfg->u.frameCfg.frameCfg.chirpStartIdx);
    }
}

static uint16_t OdsDemo_getChirpEndIdx(MMWave_CtrlCfg *cfg, uint8_t subFrameIndx)
{
    if (cfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(cfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].chirpStartIdx +
              (cfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].numOfChirps - 1));
    }
    else 
    {
        return(cfg->u.frameCfg.frameCfg.chirpEndIdx);
    }
}

static uint16_t OdsDemo_getNumLoops(MMWave_CtrlCfg *cfg, uint8_t subFrameIndx)
{
    if (cfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(cfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].numLoops);
    }
    else 
    {
        return(cfg->u.frameCfg.frameCfg.numLoops);
    }
}

static MMWave_ProfileHandle OdsDemo_getProfileHandle(MMWave_CtrlCfg *cfg, uint32_t profileLoopIdx)
{
    if (cfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(cfg->u.advancedFrameCfg.profileHandle[profileLoopIdx]);
    }
    else 
    {
        return(cfg->u.frameCfg.profileHandle[profileLoopIdx]);
    }
}

/**
 *  @b Description
 *  @n
 *      parses Profile, Chirp and Frame config and extracts parameters
 *      needed for processing chain configuration
 */
bool OdsDemo_parseProfileAndChirpConfig(OdsDemo_DSS_DataPathObj *dataPathObj,
    MMWave_CtrlCfg* ptrCtrlCfg, uint8_t subFrameIndx)
{
    uint16_t        frameChirpStartIdx;
    uint16_t        frameChirpEndIdx;
    uint16_t        numLoops;
    int16_t         frameTotalChirps;
    int32_t         errCode;
    uint32_t        profileLoopIdx, chirpLoopIdx;
    bool            foundValidProfile = false;
    uint16_t        channelTxEn;
    uint8_t         channel;
    uint8_t         numRxChannels = 0;
    MMWave_OpenCfg* ptrOpenCfg;
    uint8_t         rxAntOrder [SYS_COMMON_NUM_RX_CHANNEL];
    uint8_t         txAntOrder [SYS_COMMON_NUM_TX_ANTENNAS];
    int32_t         i;
    int32_t         txIdx, rxIdx;

    /* Get data path object and control configuration */
    ptrOpenCfg = &gOdsDssMCB.cfg.openCfg;

    /* Get the Transmit channel enable mask: */
    channelTxEn = ptrOpenCfg->chCfg.txChannelEn;

    /* Find total number of Rx Channels */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        rxAntOrder[channel] = 0;
        if(ptrOpenCfg->chCfg.rxChannelEn & (0x1U << channel))
        {
            rxAntOrder[numRxChannels] = channel;
            /* Track the number of receive channels: */
            numRxChannels += 1;
        }
    }
    dataPathObj->numRxAntennas = numRxChannels;

    if (ptrCtrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        OdsDemo_dssAssert(
            ptrCtrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].numOfBurst == 1);
    }

    /* read frameCfg chirp start/stop*/
    frameChirpStartIdx = OdsDemo_getChirpStartIdx(ptrCtrlCfg, subFrameIndx);
    frameChirpEndIdx   = OdsDemo_getChirpEndIdx(ptrCtrlCfg, subFrameIndx);
    numLoops = OdsDemo_getNumLoops(ptrCtrlCfg, subFrameIndx);

    frameTotalChirps   = frameChirpEndIdx - frameChirpStartIdx + 1;

    /* since validChirpTxEnBits is static array of 32 */
    OdsDemo_dssAssert(frameTotalChirps <= 32);

    /* loop for profiles and find if it has valid chirps */
    /* we support only one profile in this processing chain */ // HG Important. The multiple chirp profile for different frames or chirp would not be supported by this code
    for (profileLoopIdx = 0;
        ((profileLoopIdx < MMWAVE_MAX_PROFILE) && (foundValidProfile == false));
        profileLoopIdx++)
    {
        uint32_t    mmWaveNumChirps = 0;
        bool        validProfileHasOneTxPerChirp = false;
        bool        validChirpHasOneTxPerChirp = false;
        uint16_t    validProfileTxEn = 0;
        uint16_t    validChirpTxEnBits[32] = {0};
        MMWave_ProfileHandle profileHandle;

        profileHandle = OdsDemo_getProfileHandle(ptrCtrlCfg, profileLoopIdx);
        if (profileHandle == NULL)
            continue; /* skip this profile */

        /* get numChirps for this profile; skip error checking */
        MMWave_getNumChirps(profileHandle, &mmWaveNumChirps, &errCode);
        /* loop for chirps and find if it has valid chirps for the frame
           looping around for all chirps in a profile, in case
           there are duplicate chirps
         */
        for (chirpLoopIdx = 1; chirpLoopIdx <= mmWaveNumChirps; chirpLoopIdx++)
        {
            MMWave_ChirpHandle chirpHandle;
            /* get handle and read ChirpCfg */
            if (MMWave_getChirpHandle(profileHandle, chirpLoopIdx, &chirpHandle, &errCode)==0)
            {
                rlChirpCfg_t chirpCfg;
                if (MMWave_getChirpCfg(chirpHandle, &chirpCfg, &errCode)==0)
                {
                    uint16_t chirpTxEn = chirpCfg.txEnable;
                    /* do chirps fall in range and has valid antenna enabled */
                    if ((chirpCfg.chirpStartIdx >= frameChirpStartIdx) &&
                        (chirpCfg.chirpEndIdx <= frameChirpEndIdx) &&
                        ((chirpTxEn & channelTxEn) > 0))
                    {
                        uint16_t idx = 0;
                        for (idx = (chirpCfg.chirpStartIdx - frameChirpStartIdx);
                             idx <= (chirpCfg.chirpEndIdx - frameChirpStartIdx); idx++)
                        {
                            validChirpTxEnBits[idx] = chirpTxEn;
                            foundValidProfile = true;
                        }

                    }
                }
            }
        }
        /* now loop through unique chirps and check if we found all of the ones
           needed for the frame and then determine the azimuth antenna
           configuration
         */
        if (foundValidProfile) {
            for (chirpLoopIdx = 0; chirpLoopIdx < frameTotalChirps; chirpLoopIdx++)
            {
                uint16_t chirpTxEn = validChirpTxEnBits[chirpLoopIdx];
                validChirpHasOneTxPerChirp = false;
                if (chirpTxEn == 0) {
                    /* this profile doesnt have all the needed chirps */
                    foundValidProfile = false;
                    break;
                }
               
                if(dataPathObj->cliCfg->bpmCfg.isEnabled)
                {
                    /*For configuration purposes, BPM should be treated as if it was TDM with 2 TX antennas
                      and one TX per chirp. This way, all logic below will follow through for BPM.
                      Just need to check here if all TX antennas are enabled for BPM.*/
                    validChirpHasOneTxPerChirp = true;
                    /* In case of BPM check if both TX antennas are enabled*/
                    if(chirpTxEn != 0x3)
                    {
                        /* This frame is configured as BPM but this chirp does not enable both TX antennas*/
                        foundValidProfile = false;
                        System_printf("Bad BPM configuration. chirpTxEn=%d for chirp %d \n",chirpTxEn,chirpLoopIdx);
                        break;
                    }
                }
                else
                {
                    validChirpHasOneTxPerChirp = ((chirpTxEn == 0x1) || (chirpTxEn == 0x2));
                }    
                /* if this is the first chirp, record the chirp's
                   MIMO config as profile's MIMO config. We dont handle intermix
                   at this point */
                if (chirpLoopIdx==0) {
                    validProfileHasOneTxPerChirp = validChirpHasOneTxPerChirp;
                }
                /* check the chirp's MIMO config against Profile's MIMO config */
                if (validChirpHasOneTxPerChirp != validProfileHasOneTxPerChirp)
                {
                    /* this profile doesnt have all chirps with same MIMO config */
                    foundValidProfile = false;
                    break;
                }
                /* save the antennas actually enabled in this profile */
                validProfileTxEn |= chirpTxEn;
            }
        }

        /* found valid chirps for the frame; mark this profile valid */
        if (foundValidProfile == true) {
            rlProfileCfg_t  profileCfg;
            uint32_t        numTxAntAzim = 0;
            uint32_t        numTxAntElev = 0;
            rlProfileCfg_t  ptrProfileCfg;

            /* Get profile id from profile config */
            if(MMWave_getProfileCfg(profileHandle, &ptrProfileCfg, &errCode) < 0)
            {
                OdsDemo_dssAssert(0);
            }
            dataPathObj->validProfileIdx = ptrProfileCfg.profileId;
            
            dataPathObj->numTxAntennas = 0;
            if (!validProfileHasOneTxPerChirp)
            {
                numTxAntAzim=1;
            }
            else
            {
                if (validProfileTxEn & 0x1)
                {
                    numTxAntAzim++;
                }
                if (validProfileTxEn & 0x2)
                {
                    numTxAntAzim++;
                }
            }
            //System_printf("Azimuth Tx: %d (MIMO:%d) \n",numTxAntAzim,validProfileHasMIMO);
            dataPathObj->numTxAntennas       = numTxAntAzim + numTxAntElev;
            dataPathObj->numVirtualAntAzim   = numTxAntAzim * dataPathObj->numRxAntennas;
            dataPathObj->numVirtualAntElev   = numTxAntElev * dataPathObj->numRxAntennas;
            dataPathObj->numVirtualAntennas  = dataPathObj->numVirtualAntAzim + dataPathObj->numVirtualAntElev;

            /* Sanity Check: Ensure that the number of antennas is within system limits */
            OdsDemo_dssAssert (dataPathObj->numVirtualAntennas > 0);
            OdsDemo_dssAssert (dataPathObj->numVirtualAntennas <= (SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL));

            /* Copy the Rx channel compensation coefficients from common area to data path structure */
            if (validProfileHasOneTxPerChirp)
            {
                for (i = 0; i < dataPathObj->numTxAntennas; i++)
                {
                    if(dataPathObj->cliCfg->bpmCfg.isEnabled)
                    {
                        /*If BPM is enabled, need to assume order of the antennas as it can not 
                          be derived from the chirp/profile configuration because both TX antennas
                          are enabled in both chirps. Will assume that the order is 0,1,...[numTxAntennas-1]*/
                        txAntOrder[i] = i;
                    }
                    else
                    {
                        txAntOrder[i] = OdsDemo_floorLog2(validChirpTxEnBits[i]);
                    }    
                }
                for (txIdx = 0; txIdx < dataPathObj->numTxAntennas; txIdx++)
                {
                    for (rxIdx = 0; rxIdx < dataPathObj->numRxAntennas; rxIdx++)
                    {
                        dataPathObj->compRxChanCfg.rxChPhaseComp[txIdx*dataPathObj->numRxAntennas + rxIdx] =
                                dataPathObj->cliCommonCfg->compRxChanCfg.rxChPhaseComp[txAntOrder[txIdx]*SYS_COMMON_NUM_RX_CHANNEL + rxAntOrder[rxIdx]];

                    }

                }
            }
            else
            {
                cmplx16ImRe_t one;
                one.imag = 0;
                one.real = 0x7fff;
                for (txIdx = 0; txIdx < dataPathObj->numTxAntennas; txIdx++)
                {
                    for (rxIdx = 0; rxIdx < dataPathObj->numRxAntennas; rxIdx++)
                    {
                        dataPathObj->compRxChanCfg.rxChPhaseComp[txIdx*dataPathObj->numRxAntennas + rxIdx] = one;
                    }

                }
            }
            /* Get the profile configuration: */
            if (MMWave_getProfileCfg (profileHandle, &profileCfg, &errCode) < 0)
            {
                System_printf ("Error: Unable to get the profile configuration [Error code %d]\n", errCode);
                OdsDemo_dssAssert (0);
                return -1;
            }

            /* multiplicity of 4 due to windowing library function requirement */
            if ((profileCfg.numAdcSamples % 4) != 0)
            {
                System_printf("Number of ADC samples must be multiple of 4\n");
                OdsDemo_dssAssert(0);
            }
            dataPathObj->numAdcSamples       = profileCfg.numAdcSamples;
            dataPathObj->numRangeBins        = OdsDemo_pow2roundup(dataPathObj->numAdcSamples);
            dataPathObj->numChirpsPerFrame   = (frameChirpEndIdx -frameChirpStartIdx + 1) *
                                               numLoops;
            dataPathObj->numAngleBins        = ODS_NUM_ANGLE_BINS;
            dataPathObj->numDopplerBins      = dataPathObj->numChirpsPerFrame/dataPathObj->numTxAntennas;

            /* Multiplicity of 4 due to windowing library function requirement.
               Minimum size of 16 due to DSPLib restriction - FFT size must be bigger than 16.*/
            if (((dataPathObj->numDopplerBins % 4) != 0) ||
                (dataPathObj->numDopplerBins < 16))
            {
                System_printf("Number of Doppler bins must be at least 16 and it must be a multiple of 4.\n");
                OdsDemo_dssAssert(0);
            }

            if (dataPathObj->cliCfg->extendedMaxVelocityCfg.enabled  && 
                dataPathObj->cliCfg->multiObjBeamFormingCfg.enabled)
            {
                System_printf("Simultaneous multi object beam forming and extended maximum velocity is not supported.\n");
                OdsDemo_dssAssert(0);
            }

            if (dataPathObj->cliCfg->extendedMaxVelocityCfg.enabled  && numTxAntAzim == 1)
            {
                System_printf("Extended maximum velocity technique is supported only in TDM MIMO\n");
                OdsDemo_dssAssert(0);
            }

#ifndef ODS_ENABLE_NEGATIVE_FREQ_SLOPE
            /* Check frequency slope */
            if (profileCfg.freqSlopeConst < 0)
            {
                System_printf("Frequency slope must be positive\n");
                OdsDemo_dssAssert(0);
            }
#endif
            dataPathObj->rangeResolution = OdsDemo_SPEED_OF_LIGHT_IN_METERS_PER_SEC * 
                profileCfg.digOutSampleRate * 1e3 / 
                (2 * profileCfg.freqSlopeConst * ((3.6*1e3*900) /
                 (1U << 26)) * 1e12 * dataPathObj->numRangeBins);
            dataPathObj->xyzOutputQFormat = (uint8_t) ceil(log10(16. /
                fabs(dataPathObj->rangeResolution))/log10(2));
        }
    }
    return foundValidProfile;
}

/**
 *  @b Description
 *  @n
 *      This function ADCBuf configuration and save info in datapath object to be used by 
 *   data path modules.
 */
void OdsDemo_parseAdcBufCfg(OdsDemo_DSS_DataPathObj *dataPathObj)
{
    uint8_t             numBytePerSample = 0;
    uint32_t            chirpThreshold;
    uint32_t            maxChirpThreshold;
    uint32_t            bytesPerChirp;
    OdsDemo_ADCBufCfg   *adcBufCfg = &dataPathObj->cliCfg->adcBufCfg;
    
    /* Check if ADC configuration is supported:*/
    /* ADC out bits: must be 16 Bits */
    OdsDemo_dssAssert(dataPathObj->cfg->openCfg.adcOutCfg.fmt.b2AdcBits == 2);
    
    /* ADC data format: must be complex */
    /*adcCfg command*/
    if((dataPathObj->cfg->openCfg.adcOutCfg.fmt.b2AdcOutFmt != 1) &&
       (dataPathObj->cfg->openCfg.adcOutCfg.fmt.b2AdcOutFmt != 2))
    {
        OdsDemo_dssAssert(0);
    }    
    /*adcbufCfg command*/
    OdsDemo_dssAssert(adcBufCfg->adcFmt == 0);
    
    /* ADC channel interleave mode: must be non-interleaved */
    OdsDemo_dssAssert(adcBufCfg->chInterleave == 1);
    
    /* Complex dataFormat has 4 bytes */
    numBytePerSample =  4;

    /* calculate max possible chirp threshold */
    bytesPerChirp = dataPathObj->numAdcSamples * dataPathObj->numRxAntennas * numBytePerSample;

    /* find maximum number of full chirps that can fit in the ADCBUF memory, while
       also being able to divide numChirpsPerFrame, we do not want remainder processing */
    maxChirpThreshold = SOC_XWR16XX_DSS_ADCBUF_SIZE / bytesPerChirp;
    
    /* There is a maximum of 8 CPs and CQs, so lets limit maxChirpThreshold to be that*/
    if(maxChirpThreshold > SYS_COMMON_CQ_MAX_CHIRP_THRESHOLD)
    {
        maxChirpThreshold = SYS_COMMON_CQ_MAX_CHIRP_THRESHOLD;
    }

    if (maxChirpThreshold >= dataPathObj->numChirpsPerFrame)
    {
        maxChirpThreshold = dataPathObj->numChirpsPerFrame;
    }
    else
    {
        /* find largest divisor of numChirpsPerFrame no bigger than maxChirpThreshold */
        while (dataPathObj->numChirpsPerFrame % maxChirpThreshold)
        {
            maxChirpThreshold--;
        }
    }

    /* ADCBuf control function requires argument alignment at 4 bytes boundary */
    chirpThreshold = adcBufCfg->chirpThreshold;

    /* if automatic, set to the calculated max */
    if (chirpThreshold == 0)
    {
        chirpThreshold = maxChirpThreshold;
    }
    else
    {
        if (chirpThreshold > maxChirpThreshold)
        {
            System_printf("Desired chirpThreshold %d higher than max possible of %d, setting to max\n",
                chirpThreshold, maxChirpThreshold);
            chirpThreshold = maxChirpThreshold;
        }
        else
        {
            /* check for divisibility of the user provided threshold */
            OdsDemo_dssAssert((dataPathObj->numChirpsPerFrame % chirpThreshold) == 0);
        }
    }

    /* Save info in data path Object */
    dataPathObj->numChirpsPerChirpEvent = chirpThreshold;
    dataPathObj->numBytePerSample = numBytePerSample;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Re-Configuration on DSS.
 *      This is used when switching between sub-frames.
 *
 *  @retval
 *      -1 if error, 0 otherwise.
 */
static int32_t OdsDemo_dssDataPathReconfig(OdsDemo_DSS_DataPathObj *obj)
{
    int32_t retVal;

    retVal = OdsDemo_dssDataPathConfigAdcBuf(obj);
    if (retVal < 0)
    {
        return -1;
    }

    OdsDemo_dataPathConfigFFTs(obj);

    /* must be after OdsDemo_dssDataPathConfigAdcBuf above as it calculates 
       numChirpsPerChirpEvent that is used in EDMA configuration */
    OdsDemo_dataPathConfigEdma(obj);

    /* Configure HW LVDS stream for this subframe? */
    if(obj->cliCfg->lvdsStreamCfg.dataFmt != 0) 
    {
        /* Delete previous CBUFF HW session if one was configured */
        if(gOdsDssMCB.lvdsStream.hwSessionHandle != NULL)
        {
            OdsDemo_LVDSStreamDeleteHwSession(gOdsDssMCB.lvdsStream.hwSessionHandle);
        }
        
        /* Configure HW session */    
        if (OdsDemo_LVDSStreamHwConfig(obj) < 0)
        {
            System_printf("Failed LVDS stream HW configuration\n");
            return -1;
        }
        
        /* If HW LVDS stream is enabled, start the session here so that ADC samples will be 
        streamed out as soon as the first chirp samples land on ADC*/
        if(CBUFF_activateSession (gOdsDssMCB.lvdsStream.hwSessionHandle, &retVal) < 0)
        {
            System_printf("Failed to activate CBUFF session for LVDS stream HW. errCode=%d\n",retVal);
            return -1;
        }
    }    

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Configuration on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t OdsDemo_dssDataPathConfig(void)
{
    int32_t             retVal = 0;
    MMWave_CtrlCfg      *ptrCtrlCfg;
    OdsDemo_DSS_DataPathObj *dataPathObj;
    uint8_t subFrameIndx;

    /* Get data path object and control configuration */
    ptrCtrlCfg   = &gOdsDssMCB.cfg.ctrlCfg;

    if (ptrCtrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        gOdsDssMCB.numSubFrames = 
            ptrCtrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.numOfSubFrames;
    }
    else
    {
        gOdsDssMCB.numSubFrames = 1;
    }

    for(subFrameIndx = 0; subFrameIndx < gOdsDssMCB.numSubFrames; subFrameIndx++)
    {
        dataPathObj  = &gOdsDssMCB.dataPathObj[subFrameIndx];
        dataPathObj->subFrameIndx = subFrameIndx;
        /*****************************************************************************
         * Data path :: Algorithm Configuration
         *****************************************************************************/

        /* Parse the profile and chirp configs and get the valid number of TX Antennas */
        if (OdsDemo_parseProfileAndChirpConfig(dataPathObj, ptrCtrlCfg, subFrameIndx) == true)
        {
            /* Data path configurations */
            OdsDemo_dataPathConfigBuffers(dataPathObj, SOC_XWR16XX_DSS_ADCBUF_BASE_ADDRESS);
            OdsDemo_dataPathComputeDerivedConfig(dataPathObj);

            /* Find out number of chirp per chirp interrupt 
               It will be used for both ADCBuf config and CQ config
             */
            OdsDemo_parseAdcBufCfg(dataPathObj); // HG. Number of chirpThreshold is defined here as 8, which is the maxChirpThreshold

            retVal = OdsDemo_dssDataPathConfigCQ(dataPathObj);
            if (retVal < 0)
            {
                return -1;
            }
            
            /* Below configurations are to be reconfigured every sub-frame so do only for first one */
            if (subFrameIndx == 0)
            {
                retVal = OdsDemo_dssDataPathReconfig(dataPathObj);
                if (retVal < 0)
                {
                    return -1;
                }
            }
        }
        else
        {
            /* no valid profile found - assert! */
            OdsDemo_dssAssert(0);
        }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to start Data Path on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t OdsDemo_dssDataPathStart(bool doRFStart)
{
    int32_t    errCode;
    MMWave_CalibrationCfg   calibrationCfg;

    gOdsDssMCB.dataPathContext.chirpProcToken = 0;
    gOdsDssMCB.dataPathContext.interFrameProcToken = 0;
    gOdsDssMCB.lvdsStream.hwFrameDoneCount = 0;
    gOdsDssMCB.lvdsStream.swFrameDoneCount = 0;

    if (doRFStart)
    {
        /* Initialize the calibration configuration: */
        memset ((void*)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

        /* Populate the calibration configuration: */
        calibrationCfg.dfeDataOutputMode                          = 
            gOdsDssMCB.cfg.ctrlCfg.dfeDataOutputMode;
        calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = true;
        calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = true;
        calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    /* Start the mmWave module: The configuration has been applied successfully. */
        if (MMWave_start (gOdsDssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
        {
            /* Error: Unable to start the mmWave control */
            System_printf ("Error: ODSDemoDSS mmWave Start failed [Error code %d]\n", errCode);
            return -1;
        }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to process Data Path events at runtime.
 *
 *  @param[in]  event
 *      Data Path Event
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t OdsDemo_dssDataPathProcessEvents(UInt event)
{
    OdsDemo_DSS_DataPathObj *dataPathObj;
    volatile uint32_t startTime;

    dataPathObj = &gOdsDssMCB.dataPathObj[gOdsDssMCB.subFrameIndx];

    /* Handle dataPath events */
    switch(event)
    {
        case ODSDEMO_CHIRP_EVT:
            /* The following commented calls to Clock_tickStart()/Stop() APIs are
               shown as an example of how to disable BIOS timer ticks
               to prevent the timer interrupt from disrupting the 1D chirp processing. 
               This may be necessary for very small chirp times
               when the CPU does not have enough MIPS to process the timer interrupt
               if it happens during the 1D processing.*/
            //Clock_tickStop();   
            /* Increment event stats */
            gOdsDssMCB.stats.chirpEvt++;

            /* Start CQ EDMA */
            OdsDemo_dssDataPathStartCQEdma(dataPathObj);

            {
                uint16_t chirpIndex;
                for (chirpIndex = 0; chirpIndex < dataPathObj->numChirpsPerChirpEvent; chirpIndex++)
                {
                    OdsDemo_processChirp(dataPathObj, (uint16_t) chirpIndex);
                }
            }
            //Clock_tickStart();
            gOdsDssMCB.dataPathContext.chirpProcToken--;
            dataPathObj->timingInfo.chirpProcessingEndTime = Cycleprofiler_getTimeStamp();

            if (dataPathObj->chirpCount == 0)
            {
                OdsDemo_waitEndOfChirps(dataPathObj);
                Load_update();
                dataPathObj->timingInfo.activeFrameCPULoad = Load_getCPULoad();

                dataPathObj->cycleLog.interChirpProcessingTime = gCycleLog.interChirpProcessingTime;
                dataPathObj->cycleLog.interChirpWaitTime = gCycleLog.interChirpWaitTime;
                gCycleLog.interChirpProcessingTime = 0;
                gCycleLog.interChirpWaitTime = 0;

                startTime = Cycleprofiler_getTimeStamp();
                OdsDemo_interFrameProcessing(dataPathObj);
                dataPathObj->timingInfo.interFrameProcCycles = (Cycleprofiler_getTimeStamp() - startTime);

                dataPathObj->cycleLog.interFrameProcessingTime = gCycleLog.interFrameProcessingTime;
                dataPathObj->cycleLog.interFrameWaitTime = gCycleLog.interFrameWaitTime;
                gCycleLog.interFrameProcessingTime = 0;
                gCycleLog.interFrameWaitTime = 0;

                /* Sending range bias and Rx channel phase offset measurements to MSS and from there to CLI */
                if(dataPathObj->cliCommonCfg->measureRxChanCfg.enabled)
                {
                    OdsDemo_measurementResultOutput (dataPathObj);
                }

                /* Sending detected objects to logging buffer */
                OdsDemo_dssDataPathOutputLogging (dataPathObj); // HG. The LVDS session is managed here
                dataPathObj->timingInfo.interFrameProcessingEndTime = Cycleprofiler_getTimeStamp();
            }
            break;

        case ODSDEMO_FRAMESTART_EVT:
            /* Increment event stats */
            gOdsDssMCB.stats.frameStartEvt++;
            Load_update();
            dataPathObj->timingInfo.interFrameCPULoad = Load_getCPULoad();
            OdsDemo_dssAssert(dataPathObj->chirpCount == 0);
            break;

        case ODSDEMO_BSS_FRAME_TRIGGER_READY_EVT:
            /* Increment event stats */
            gOdsDssMCB.stats.frameTrigEvt++;
            break;

        default:
            break;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to stop Data Path on DSS. Assume BSS has been stopped by mmWave already.
 *      This also sends the STOP done message back to MSS to signal the procssing
 *      chain has come to a stop.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t OdsDemo_dssDataPathStop(void)
{
    OdsDemo_message     message;
    uint32_t numFrameEvtsHandled = gOdsDssMCB.stats.frameStartEvt;
    int32_t errCode;

    /* move to stop state */
    gOdsDssMCB.state = ODSDEMO_DSS_STATE_STOPPED;

    /* Update stats */
    gOdsDssMCB.stats.chirpEvt = 0;
    gOdsDssMCB.stats.frameStartEvt = 0;
    gOdsDssMCB.stats.frameTrigEvt = 0;
    gOdsDssMCB.stats.numFailedTimingReports = 0;
    gOdsDssMCB.stats.numCalibrationReports = 0;

    /* Delete any active streaming session */
    if(gOdsDssMCB.lvdsStream.hwSessionHandle != NULL)
    {
        CBUFF_deactivateSession (gOdsDssMCB.lvdsStream.hwSessionHandle, &errCode);
        OdsDemo_LVDSStreamDeleteHwSession(gOdsDssMCB.lvdsStream.hwSessionHandle);
    }
    
    if(gOdsDssMCB.lvdsStream.swSessionHandle != NULL)
    {
        OdsDemo_LVDSStreamDeleteSwSession(gOdsDssMCB.lvdsStream.swSessionHandle);
    }
    
    /* send message back to MSS */
    message.type = ODSDEMO_DSS2MSS_STOPDONE;

    /* Waiting in a loop until the message is delivered to MSS */
    while(1)
    {
        errCode = OdsDemo_mboxWrite(&message);
        if (errCode < 0)
        {
            if(errCode == MAILBOX_ECHINUSE)
            {
                Task_sleep(1);
                continue;
            }
            System_printf ("Debug: Mailbox write bss done failed with error :%d \n",errCode);
            OdsDemo_dssAssert(0);
        }
        else
        {
            System_printf ("Debug: ODSDemoDSS Data Path stop succeeded stop%d,frames:%d \n",
                            gOdsDssMCB.stats.stopEvt,numFrameEvtsHandled);
            break;
        }
    }
    return 0;
}


/**
 *  @b Description
 *  @n
 *      The task is used to provide an execution context for the mmWave
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
static void OdsDemo_dssMMWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute (gOdsDssMCB.ctrlHandle, &errCode) < 0)
            System_printf ("Error: ODSDemoDSS mmWave control execution failed [Error code %d]\n", errCode);
    }
}

/**
 *  @b Description
 *  @n
 *      Data Path main task that handles events from remote and do dataPath processing.
 *  This task is created when MSS is responsible for the mmwave Link and DSS is responsible
 *  for data path processing.
 *
 *  @retval
 *      Not Applicable.
 */
static void OdsDemo_dssDataPathTask(UArg arg0, UArg arg1)
{
    int32_t       retVal = 0;
    UInt          event;

    /************************************************************************
     * Data Path :: Config
     ************************************************************************/

    /* Waiting for Config event from Remote - MSS */
    Event_pend(gOdsDssMCB.eventHandle, ODSDEMO_CONFIG_EVT, Event_Id_NONE, BIOS_WAIT_FOREVER);
    if ((retVal = OdsDemo_dssDataPathConfig()) < 0 ) // ADC Config, chirp threshold and all the rest values associated are initialized with this command
    {
        System_printf ("Debug: ODSDemoDSS Data Path config failed with Error[%d]\n",retVal);
        goto exit;
    }

    /************************************************************************
     * Data Path :: Start, mmwaveLink start will be triggered from DSS!
     ************************************************************************/
    if ((retVal = OdsDemo_dssDataPathStart(true)) < 0 )
    {
        System_printf ("Debug: ODSDemoDSS Data Path start failed with Error[%d]\n",retVal);
        goto exit;
    }
    gOdsDssMCB.state = ODSDEMO_DSS_STATE_STARTED;

    /************************************************************************
     * Data Path :: Main loop
     ************************************************************************/
    while (1) // HG: Important. This is the main DSS Loop
    {
        event = Event_pend(gOdsDssMCB.eventHandle,
                          Event_Id_NONE,
                          ODSDEMO_FRAMESTART_EVT | ODSDEMO_CHIRP_EVT |
                          ODSDEMO_BSS_STOP_COMPLETE_EVT | ODSDEMO_CONFIG_EVT |
                          ODSDEMO_STOP_COMPLETE_EVT | ODSDEMO_START_EVT, 
                          BIOS_WAIT_FOREVER);


        if(event & ODSDEMO_BSS_STOP_COMPLETE_EVT)
        {
            OdsDemo_bssStopDone();
        }

        /************************************************************************
         * Data Path process frame start event
         ************************************************************************/
        if(event & ODSDEMO_FRAMESTART_EVT)
        {
            if((gOdsDssMCB.state == ODSDEMO_DSS_STATE_STARTED) || (gOdsDssMCB.state == ODSDEMO_DSS_STATE_STOP_PENDING))
            {
                if ((retVal = OdsDemo_dssDataPathProcessEvents(ODSDEMO_FRAMESTART_EVT)) < 0 )
                {
                    System_printf ("Error: ODSDemoDSS Data Path process frame start event failed with Error[%d]\n",
                                  retVal);
                }
            }
        }

        /************************************************************************
         * Data Path process chirp event
         ************************************************************************/
        if(event & ODSDEMO_CHIRP_EVT)
        {
            if((gOdsDssMCB.state == ODSDEMO_DSS_STATE_STARTED) || (gOdsDssMCB.state == ODSDEMO_DSS_STATE_STOP_PENDING))
            {
                if ((retVal = OdsDemo_dssDataPathProcessEvents(ODSDEMO_CHIRP_EVT)) < 0 )
                {
                    System_printf ("Error: ODSDemoDSS Data Path process chirp event failed with Error[%d]\n",
                                  retVal);
                }
            }
        }

        /************************************************************************
         * Data Path re-config, only supported reconfiguration in stop state
         ************************************************************************/
        if(event & ODSDEMO_CONFIG_EVT)
        {
            if(gOdsDssMCB.state == ODSDEMO_DSS_STATE_STOPPED)
            {
                if ((retVal = OdsDemo_dssDataPathConfig()) < 0 )
                {
                    System_printf ("Debug: ODSDemoDSS Data Path config failed with Error[%d]\n",retVal);
                    goto exit;
                }

                /************************************************************************
                 * Data Path :: Start, mmwaveLink start will be triggered from DSS!
                 ************************************************************************/
                if ((retVal = OdsDemo_dssDataPathStart(true)) < 0 )
                {
                    System_printf ("Error: ODSDemoDSS Data Path start failed with Error[%d]\n",retVal);
                    goto exit;
                }
                gOdsDssMCB.state = ODSDEMO_DSS_STATE_STARTED;
            }
            else
            {
                System_printf ("Error: ODSDemoDSS Data Path config event in wrong state[%d]\n", gOdsDssMCB.state);
                goto exit;
            }
        }

        /************************************************************************
         * Quick start after stop
         ************************************************************************/
        if(event & ODSDEMO_START_EVT)
        {
            if(gOdsDssMCB.state == ODSDEMO_DSS_STATE_STOPPED)
            {
                /* RF start is done by MSS in this case; so just do DSS start */
                if ((retVal = OdsDemo_dssDataPathStart(false)) < 0 )
                {
                    System_printf ("Error: ODSDemoDSS Data Path start failed with Error[%d]\n",retVal);
                    goto exit;
                }
                gOdsDssMCB.state = ODSDEMO_DSS_STATE_STARTED;
            }
            else
            {
                System_printf ("Error: ODSDemoDSS Data Path config event in wrong state[%d]\n", gOdsDssMCB.state);
                goto exit;
            }
        }

        /************************************************************************
         * Data Path process frame start event
         ************************************************************************/
        if(event & ODSDEMO_STOP_COMPLETE_EVT)
        {
            if (gOdsDssMCB.state == ODSDEMO_DSS_STATE_STOP_PENDING)
            {
                /************************************************************************
                 * Local Data Path Stop
                 ************************************************************************/
                if ((retVal = OdsDemo_dssDataPathStop()) < 0 )
                {
                    System_printf ("Debug: ODSDemoDSS Data Path stop failed with Error[%d]\n",retVal);
                }
            }
        }
    }
exit:
    System_printf("Debug: ODSDemoDSS Data path exit\n");

}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
static void OdsDemo_dssInitTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    MMWave_InitCfg      initCfg;
    Task_Params         taskParams;
    Semaphore_Params    semParams;
    Mailbox_Config      mboxCfg;
    Error_Block         eb;
    OdsDemo_message     message;

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_DSS);

    /*****************************************************************************
     * Create mailbox Semaphore:
     *****************************************************************************/
    /* Create a binary semaphore which is used to handle mailbox interrupt. */
    Semaphore_Params_init(&semParams);
    semParams.mode             = Semaphore_Mode_BINARY;
    gOdsDssMCB.mboxSemHandle = Semaphore_create(0, &semParams, NULL);

    /* Setup the default mailbox configuration */
    Mailbox_Config_init(&mboxCfg);

    /* Setup the configuration: */
    mboxCfg.chType       = MAILBOX_CHTYPE_MULTI;
    mboxCfg.chId         = MAILBOX_CH_ID_0;
    mboxCfg.writeMode    = MAILBOX_MODE_BLOCKING;
    mboxCfg.readMode     = MAILBOX_MODE_CALLBACK;
    mboxCfg.readCallback = &OdsDemo_mboxCallback;

    gOdsDssMCB.peerMailbox = Mailbox_open(MAILBOX_TYPE_MSS, &mboxCfg, &errCode);
    if (gOdsDssMCB.peerMailbox == NULL)
    {
        /* Error: Unable to open the mailbox */
        System_printf("Error: Unable to open the Mailbox to the DSS [Error code %d]\n", errCode);
        return;
    }

     /* Debug Message: */
    /*System_printf("Debug: DSS Mailbox Handle %p\n", gOdsDssMCB.peerMailbox);*/

    /* Create task to handle mailbox messages */
    Task_Params_init(&taskParams);
    Task_create(OdsDemo_mboxReadTask, &taskParams, NULL);
    
    /* Initialize LVDS streaming components */
    if ((errCode = OdsDemo_LVDSStreamInit()) < 0 )
    {
        System_printf ("Error: ODSDemoDSS LVDS stream init failed with Error[%d]\n",errCode);
        return;
    }
    
    /*****************************************************************************
     * Create Event to handle mmwave callback and system datapath events
     *****************************************************************************/
    /* Default instance configuration params */
    Error_init(&eb);
    gOdsDssMCB.eventHandle = Event_create(NULL, &eb);
    if (gOdsDssMCB.eventHandle == NULL)
    {
        /* FATAL_TBA */
        System_printf("Error: ODSDemoDSS Unable to create an event handle\n");
        return ;
    }
    /*System_printf("Debug: MMWDemoDSS create event handle succeeded\n");*/

    /************************************************************************
     * mmwave library initialization
     ************************************************************************/

    /* Populate the init configuration for mmwave library: */
    initCfg.domain                      = MMWave_Domain_DSS;
    initCfg.socHandle                   = gOdsDssMCB.socHandle;
    initCfg.eventFxn                    = OdsDemo_dssMmwaveEventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver     = 1U;
    initCfg.linkCRCCfg.crcChannel       = CRC_Channel_CH1;
    initCfg.cfgMode                     = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode               = MMWave_ExecutionMode_COOPERATIVE;
    initCfg.cooperativeModeCfg.cfgFxn   = OdsDemo_dssMmwaveConfigCallbackFxn;
    initCfg.cooperativeModeCfg.startFxn = OdsDemo_dssMmwaveStartCallbackFxn;
    initCfg.cooperativeModeCfg.stopFxn  = OdsDemo_dssMmwaveStopCallbackFxn;
    initCfg.cooperativeModeCfg.openFxn  = OdsDemo_dssMmwaveOpenCallbackFxn;
    initCfg.cooperativeModeCfg.closeFxn = OdsDemo_dssMmwaveCloseCallbackFxn;

    /* Initialize and setup the mmWave Control module */
    gOdsDssMCB.ctrlHandle = MMWave_init (&initCfg, &errCode);
    if (gOdsDssMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        System_printf ("Error: ODSDemoDSS mmWave Control Initialization failed [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: ODSDemoDSS mmWave Control Initialization succeeded\n");

    /******************************************************************************
     * TEST: Synchronization
     * - The synchronization API always needs to be invoked.
     ******************************************************************************/
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = MMWave_sync (gOdsDssMCB.ctrlHandle , &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the mmWave control module */
            System_printf ("Error: ODSDemoDSS mmWave Control Synchronization failed [Error code %d]\n", errCode);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        Task_sleep(1);
    }

    /* Send the DSS to MSS signalling ISR payload address to the DSS. Note this
       should be done after both sides mailboxes have been opened, and because
       MMWave_sync above is a good one to check for synchronization, this is a good place */
    message.type = ODSDEMO_DSS2MSS_ISR_INFO_ADDRESS;
    message.body.dss2mssISRinfoAddress = (uint32_t) &gHSRAM.dss2MssIsrInfo;
    OdsDemo_mboxWrite(&message);

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priority than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority = 6;
    taskParams.stackSize = 4*1024;
    Task_create(OdsDemo_dssMMWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * Data path Startup
     *****************************************************************************/
    if ((errCode = OdsDemo_dssDataPathInit()) < 0 )
    {
        System_printf ("Error: ODSDemoDSS Data Path init failed with Error[%d]\n",errCode);
        return;
    }
    System_printf ("Debug: ODSDemoDSS Data Path init succeeded\n");
    gOdsDssMCB.state = ODSDEMO_DSS_STATE_INIT;

    /* Start data path task */
    Task_Params_init(&taskParams);
    taskParams.priority = 5;
    taskParams.stackSize = 4*1024;
    Task_create(OdsDemo_dssDataPathTask, &taskParams, NULL);

    System_printf("Debug: ODSDemoDSS initTask exit\n");
    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the DSP using IDLE instruction. When DSP has no work left to do,
 *     the BIOS will be in Idle thread and will call this function. The DSP will
 *     wake-up on any interrupt (e.g chirp interrupt).
 *
 *  @retval
 *      Not Applicable.
 */
void OdsDemo_sleep(void)
{
    /* issue IDLE instruction */
    asm(" IDLE ");
}

/**
 *  @b Description
 *  @n
 *      Sends DSS assert information to MSS
 *
 *  @retval
 *      Not Applicable.
 */
void _OdsDemo_dssAssert(int32_t expression,const char *file, int32_t line)
{
    OdsDemo_message  message;
    uint32_t         nameSize;

    if (!expression) 
    {
        message.type = ODSDEMO_DSS2MSS_ASSERT_INFO;
        nameSize = strlen(file);
        if(nameSize > ODSDEMO_MAX_FILE_NAME_SIZE)
            nameSize = ODSDEMO_MAX_FILE_NAME_SIZE;
            
        memcpy((void *) &message.body.assertInfo.file[0], (void *)file, nameSize);
        message.body.assertInfo.line = (uint32_t)line;
        if (OdsDemo_mboxWrite(&message) != 0)
        {
            System_printf ("Error: Failed to send exception information to MSS.\n");
        }
        
    }    
}        

/**
 *  @b Description
 *  @n
 *      Sends DSS range bias and rx channel phase offset measurement information to MSS
 *
 *  @retval
 *      Not Applicable.
 */
void OdsDemo_measurementResultOutput(OdsDemo_DSS_DataPathObj *obj)
{
    OdsDemo_message  message;

    message.type = ODSDEMO_DSS2MSS_MEASUREMENT_INFO;

    memcpy((void *) &message.body.compRxChanCfg, (void *)&obj->cliCommonCfg->compRxChanCfg, sizeof(OdsDemo_compRxChannelBiasCfg_t));
    if (OdsDemo_mboxWrite(&message) != 0)
    {
        System_printf ("Error: Failed to send measurement information to MSS.\n");
    }
}

#define L2ConfRegAddr  0x1840000
#define L1DWBINVRegAddr  0x1845044

void startClock()
{
    TSCL = 0;
}

void cache_setL2Size(int cacheConf)
{
    *((int *)L2ConfRegAddr) &= 0xFFFFFFF8;
    *((int *)L2ConfRegAddr) |= cacheConf;
}

void     cache_wbInvAllL2Wait()
{
    *((int *)L1DWBINVRegAddr) |= 0x1;
}

void cache_setMar(unsigned int * baseAddr, unsigned int byteSize, unsigned int value)
{
    unsigned int maxAddr;
    unsigned int firstMar, lastMar;
    unsigned int marNum;
    volatile unsigned int *marBase = (unsigned int *)MAR;

    /* caculate the maximum address */
    maxAddr = (unsigned int)baseAddr + (byteSize - 1);

    /* range of MAR's that need to be modified */
    firstMar = (unsigned int)baseAddr >> 24;
    lastMar = (unsigned int)maxAddr >> 24;

    /* write back invalidate all cached entries */
    cache_wbInvAllL2Wait();

    /* loop through the number of MAR registers affecting the address range */
    for (marNum = firstMar; marNum <= lastMar; marNum++) {
        /* set the MAR registers to the specified value */
        marBase[marNum] = value;
    }
}


/**
 *  @b Description
 *  @n
 *      Entry point into the test code.
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params    taskParams;
    SOC_Cfg        socCfg;
    int32_t        errCode;

    /* Initialize and populate the demo MCB */
    memset ((void*)&gOdsDssMCB, 0, sizeof(OdsDemo_DSS_MCB));

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_BYPASS_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    gOdsDssMCB.socHandle = SOC_init (&socCfg, &errCode);
    if (gOdsDssMCB.socHandle == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the DEMO configuration: */
    gOdsDssMCB.cfg.sysClockFrequency = DSS_SYS_VCLK;
    gOdsDssMCB.cfg.loggingBaudRate   = 921600;

    Cycleprofiler_init();

    /* Enable L3 cache */
    {
        cache_setL2Size(CACHE_0KCACHE);
        cache_setMar((unsigned int *)0x20000000, 0xa0000, Cache_PC | Cache_PFX);
        cache_setMar((unsigned int *)0x21080000, 0x8000, Cache_PC | Cache_PFX);
        startClock();
    }
    
    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 3*1024;
    Task_create(OdsDemo_dssInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}

