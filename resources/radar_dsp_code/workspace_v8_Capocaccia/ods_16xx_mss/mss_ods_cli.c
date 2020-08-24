/*
 *   @file  mss_ods_cli.c
 *
 *   @brief
 *      MSS Minimal CLI Implementation for the ODS TI Design
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

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/uart/UART.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>

/* Demo Include Files */
#include "mss_ods.h"
#include "common/ods_messages.h"


#define ODSDEMO_SATURATE_HIGH       MMWDEMO_SATURATE_HIGH
#define ODSDEMO_SATURATE_LOW        MMWDEMO_SATURATE_LOW

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/* CLI Extended Command Functions */
static int32_t OdsDemo_CLICfarCfg (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLIPeakGroupingCfg (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLIMultiObjBeamForming (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLICalibDcRangeSig (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLIExtendedMaxVelocity (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLIClutterRemoval (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLISensorStart (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLISensorStop (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLIGuiMonSel (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLISetDataLogger (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLIADCBufCfg (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLINearFieldCorrection (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLIChirpQualityRxSatMonCfg (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLIChirpQualitySigImgMonCfg (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLIAnalogMonitorCfg (int32_t argc, char* argv[]);
static int32_t OdsDemo_CLILvdsStreamCfg (int32_t argc, char* argv[]);

/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/

extern OdsDemo_MCB    gOdsMssMCB;
extern int32_t OdsDemo_mboxWrite(OdsDemo_message     * message);

/**************************************************************************
 *************************** CLI  Function Definitions **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor start command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLISensorStart (int32_t argc, char* argv[])
{
    bool doReconfig = true;
    if (argc==2)
    {
        doReconfig = (bool) atoi (argv[1]);
    }
    /* Post sensorSTart event to notify configuration is done */
    OdsDemo_notifySensorStart(doReconfig);
    /* Pend for completion */
    return (OdsDemo_waitSensorStartComplete());
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor stop command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLISensorStop (int32_t argc, char* argv[])
{
    /* Post sensorSTOP event to notify sensor stop command */
    OdsDemo_notifySensorStop();
    /* Pend for completion */
    OdsDemo_waitSensorStopComplete();
    return 0;
}

static int32_t OdsDemo_CLIGetSubframe (int32_t argc, char* argv[], int32_t expectedArgc, int8_t* subFrameNum)
{
    int8_t subframe;
    
    /* Sanity Check: Minimum argument check */
    if (argc != expectedArgc)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /*Subframe info is always in position 1*/
    subframe = (int8_t) atoi(argv[1]);

    if(subframe >= (int8_t)RL_MAX_SUBFRAMES)
    {
        CLI_write ("Error: Subframe number is invalid\n");
        return -1;
    }
    
    *subFrameNum = (int8_t)subframe;

    return 0;
}

static void OdsDemo_mssCfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum)
{    
    /* if subFrameNum undefined, broadcast to all sub-frames */
    if(subFrameNum == ODSDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
    {
        uint8_t  indx;
        for(indx = 0; indx < RL_MAX_SUBFRAMES; indx++)
        {
            memcpy((void *)((uint32_t) &gOdsMssMCB.cliCfg[indx] + offset), srcPtr, size);
        }
        
    }
    else
    {
        /* Apply configuration to specific subframe (or to position zero for the legacy case
           where there is no advanced frame config) */
        memcpy((void *)((uint32_t) &gOdsMssMCB.cliCfg[subFrameNum] + offset), srcPtr, size);
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for gui monitoring configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLIGuiMonSel (int32_t argc, char* argv[])
{
    OdsDemo_GuiMonSel   guiMonSel;
    OdsDemo_message     message;
    int8_t              subFrameNum;

    if(OdsDemo_CLIGetSubframe(argc, argv, 8, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize the guiMonSel configuration: */
    memset ((void *)&guiMonSel, 0, sizeof(OdsDemo_GuiMonSel));

    /* Populate configuration: */
    guiMonSel.detectedObjects           = atoi (argv[2]);
    guiMonSel.logMagRange               = atoi (argv[3]);
    guiMonSel.noiseProfile              = atoi (argv[4]);
    guiMonSel.rangeAzimuthHeatMap       = atoi (argv[5]);
    guiMonSel.rangeDopplerHeatMap       = atoi (argv[6]);
    guiMonSel.statsInfo                 = atoi (argv[7]);

    OdsDemo_mssCfgUpdate((void *)&guiMonSel, offsetof(OdsDemo_CliCfg_t, guiMonSel), 
        sizeof(OdsDemo_GuiMonSel), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(OdsDemo_message));

    message.type = ODSDEMO_MSS2DSS_GUIMON_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.guiMonSel, (void *)&guiMonSel, sizeof(OdsDemo_GuiMonSel));

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for CFAR configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLICfarCfg (int32_t argc, char* argv[])
{
    OdsDemo_CfarCfg     cfarCfg;
    OdsDemo_message     message;
    uint32_t            procDirection;
    int8_t              subFrameNum;

    if(OdsDemo_CLIGetSubframe(argc, argv, 9, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfarCfg, 0, sizeof(OdsDemo_CfarCfg));

    /* Populate configuration: */
    procDirection             = (uint32_t) atoi (argv[2]);
    cfarCfg.averageMode       = (uint8_t) atoi (argv[3]);
    cfarCfg.winLen            = (uint8_t) atoi (argv[4]);
    cfarCfg.guardLen          = (uint8_t) atoi (argv[5]);
    cfarCfg.noiseDivShift     = (uint8_t) atoi (argv[6]);
    cfarCfg.cyclicMode        = (uint8_t) atoi (argv[7]);
    cfarCfg.thresholdScale    = (uint16_t) atoi (argv[8]);

    /* Save Configuration to use later */     
    if (procDirection == 0)
    {
        OdsDemo_mssCfgUpdate((void *)&cfarCfg, offsetof(OdsDemo_CliCfg_t, cfarCfgRange),
            sizeof(OdsDemo_CfarCfg), subFrameNum);    
    }
    else
    {
        OdsDemo_mssCfgUpdate((void *)&cfarCfg, offsetof(OdsDemo_CliCfg_t, cfarCfgDoppler),
            sizeof(OdsDemo_CfarCfg), subFrameNum);    
    }

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(OdsDemo_message));
    if (procDirection == 0)
    {
        message.type = ODSDEMO_MSS2DSS_CFAR_RANGE_CFG;
    }
    else if (procDirection == 1)
    {
        message.type = ODSDEMO_MSS2DSS_CFAR_DOPPLER_CFG;
    }
    else
    {
        return -1;
    }

    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.cfarCfg, (void *)&cfarCfg, sizeof(OdsDemo_CfarCfg));

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;    
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for Peak grouping configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLIPeakGroupingCfg (int32_t argc, char* argv[])
{
    OdsDemo_PeakGroupingCfg peakGroupingCfg;
    OdsDemo_message         message;
    int8_t                  subFrameNum;

    if(OdsDemo_CLIGetSubframe(argc, argv, 7, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&peakGroupingCfg, 0, sizeof(OdsDemo_PeakGroupingCfg));

    /* Populate configuration: */
    peakGroupingCfg.scheme               = (uint8_t) atoi (argv[2]);
    peakGroupingCfg.inRangeDirectionEn   = (uint8_t) atoi (argv[3]);
    peakGroupingCfg.inDopplerDirectionEn = (uint8_t) atoi (argv[4]);
    peakGroupingCfg.minRangeIndex    = (uint16_t) atoi (argv[5]);
    peakGroupingCfg.maxRangeIndex    = (uint16_t) atoi (argv[6]);

    if (peakGroupingCfg.scheme != 1 && peakGroupingCfg.scheme != 2)
    {
        CLI_write ("Error: Invalid peak grouping scheme\n");
        return -1;
    }

    /* Save Configuration to use later */
    OdsDemo_mssCfgUpdate((void *)&peakGroupingCfg, offsetof(OdsDemo_CliCfg_t, peakGroupingCfg),
        sizeof(OdsDemo_PeakGroupingCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(OdsDemo_message));

    message.type = ODSDEMO_MSS2DSS_PEAK_GROUPING_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.peakGroupingCfg, (void *)&peakGroupingCfg, sizeof(OdsDemo_PeakGroupingCfg));

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for multi object beam forming configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLIMultiObjBeamForming (int32_t argc, char* argv[])
{
    OdsDemo_MultiObjBeamFormingCfg cfg;
    OdsDemo_message     message;
    int8_t              subFrameNum;

    if(OdsDemo_CLIGetSubframe(argc, argv, 4, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(OdsDemo_MultiObjBeamFormingCfg));

    /* Populate configuration: */
    cfg.enabled                     = (uint8_t) atoi (argv[2]);
    cfg.multiPeakThrsScal           = (float) atof (argv[3]);

    /* Save Configuration to use later */
    OdsDemo_mssCfgUpdate((void *)&cfg, offsetof(OdsDemo_CliCfg_t, multiObjBeamFormingCfg),
        sizeof(OdsDemo_MultiObjBeamFormingCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(OdsDemo_message));

    message.type = ODSDEMO_MSS2DSS_MULTI_OBJ_BEAM_FORM;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.multiObjBeamFormingCfg, (void *)&cfg, sizeof(OdsDemo_MultiObjBeamFormingCfg));

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

uint32_t log2Approx(uint32_t x)
{
    uint32_t idx, detectFlag = 0;

    if ( x < 2)
    {
        return (0);
    }

    idx = 32U;
    while((detectFlag==0U) || (idx==0U))
    {
        if(x & 0x80000000U)
        {
            detectFlag = 1;
        }
        x <<= 1U;
        idx--;
    }

    if(x != 0)
    {
        idx = idx + 1;
    }

    return(idx);
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for DC range calibration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLICalibDcRangeSig (int32_t argc, char* argv[])
{
    OdsDemo_CalibDcRangeSigCfg cfg;
    OdsDemo_message            message;
    uint32_t                   log2NumAvgChirps;
    int8_t                     subFrameNum;

    if(OdsDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(OdsDemo_CalibDcRangeSigCfg));

    /* Populate configuration: */
    cfg.enabled          = (uint16_t) atoi (argv[2]);
    cfg.negativeBinIdx   = (int16_t)  atoi (argv[3]);
    cfg.positiveBinIdx   = (int16_t)  atoi (argv[4]);
    cfg.numAvgChirps     = (uint16_t)  atoi (argv[5]);

    if (cfg.negativeBinIdx > 0)
    {
        CLI_write ("Error: Invalid negative bin index\n");
        return -1;
    }
    if ((cfg.positiveBinIdx - cfg.negativeBinIdx + 1) > DC_RANGE_SIGNATURE_COMP_MAX_BIN_SIZE)
    {
        CLI_write ("Error: Number of bins exceeds the limit\n");
        return -1;
    }
    log2NumAvgChirps = (uint32_t) log2Approx (cfg.numAvgChirps);
    if (cfg.numAvgChirps != (1 << log2NumAvgChirps))
    {
        CLI_write ("Error: Number of averaged chirps is not power of two\n");
        return -1;
    }

    /* Save Configuration to use later */
    OdsDemo_mssCfgUpdate((void *)&cfg, offsetof(OdsDemo_CliCfg_t, calibDcRangeSigCfg),
        sizeof(OdsDemo_CalibDcRangeSigCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(OdsDemo_message));

    message.type = ODSDEMO_MSS2DSS_CALIB_DC_RANGE_SIG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.calibDcRangeSigCfg, (void *)&cfg, sizeof(OdsDemo_CalibDcRangeSigCfg));

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for Velocity Disambiguation Configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLIExtendedMaxVelocity (int32_t argc, char* argv[])
{
    OdsDemo_ExtendedMaxVelocityCfg cfg;
    OdsDemo_message                message;
    int8_t                         subFrameNum;

    if(OdsDemo_CLIGetSubframe(argc, argv, 3, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(OdsDemo_ExtendedMaxVelocityCfg));

    /* Populate configuration: */
    cfg.enabled          = (uint16_t) atoi (argv[2]);


    /* Save Configuration to use later */
    OdsDemo_mssCfgUpdate((void *)&cfg, offsetof(OdsDemo_CliCfg_t, extendedMaxVelocityCfg),
        sizeof(OdsDemo_ExtendedMaxVelocityCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(OdsDemo_message));

    message.type = ODSDEMO_MSS2DSS_EXTENDED_MAX_VELOCITY;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.extendedMaxVelocityCfg, (void *)&cfg, sizeof(OdsDemo_ExtendedMaxVelocityCfg));

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for Near field correction Configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLINearFieldCorrection (int32_t argc, char* argv[])
{
    OdsDemo_NearFieldCorrectionCfg cfg;
    OdsDemo_message                message;
    int8_t                         subFrameNum;

    if(OdsDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for Near Field Correction */
    memset ((void *)&cfg, 0, sizeof(OdsDemo_NearFieldCorrectionCfg));

    /* Populate configuration: */
    cfg.enabled       = (uint8_t) atoi(argv[2]);
    cfg.startRangeIdx = (uint16_t) atoi(argv[3]);
    cfg.endRangeIdx   = (uint16_t) atoi(argv[4]);


    /* Save Configuration to use later */
    OdsDemo_mssCfgUpdate((void *)&cfg, offsetof(OdsDemo_CliCfg_t, nearFieldCorrectionCfg),
        sizeof(OdsDemo_NearFieldCorrectionCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(OdsDemo_message));

    message.type = ODSDEMO_MSS2DSS_NEAR_FIELD_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.nearFieldCorrectionCfg, (void *)&cfg, 
           sizeof(OdsDemo_NearFieldCorrectionCfg));

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}
/**
 *  @b Description
 *  @n
 *      Clutter removal Configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLIClutterRemoval (int32_t argc, char* argv[])
{
    OdsDemo_ClutterRemovalCfg cfg;
    OdsDemo_message     message;
    int8_t              subFrameNum;

    if(OdsDemo_CLIGetSubframe(argc, argv, 3, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for clutter removal */
    memset ((void *)&cfg, 0, sizeof(OdsDemo_ClutterRemovalCfg));

    /* Populate configuration: */
    cfg.enabled          = (uint16_t) atoi (argv[2]);


    /* Save Configuration to use later */
    OdsDemo_mssCfgUpdate((void *)&cfg, offsetof(OdsDemo_CliCfg_t, clutterRemovalCfg),
        sizeof(OdsDemo_ClutterRemovalCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(OdsDemo_message));

    message.type = ODSDEMO_MSS2DSS_CLUTTER_REMOVAL;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.clutterRemovalCfg, (void *)&cfg, sizeof(OdsDemo_ClutterRemovalCfg));

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for data logger set command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLISetDataLogger (int32_t argc, char* argv[])
{
    OdsDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }


    /* Save Configuration to use later */
    if (strcmp(argv[1], "mssLogger") == 0)  
        gOdsMssMCB.cfg.dataLogger = 0;
    else if (strcmp(argv[1], "dssLogger") == 0)  
        gOdsMssMCB.cfg.dataLogger = 1;
    else
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
       
    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(OdsDemo_message));

    message.type = ODSDEMO_MSS2DSS_SET_DATALOGGER;
    message.body.dataLogger = gOdsMssMCB.cfg.dataLogger;

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for data logger set command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLIADCBufCfg (int32_t argc, char* argv[])
{
    OdsDemo_ADCBufCfg   adcBufCfg;
    OdsDemo_message     message;
    int8_t              subFrameNum;

    if(OdsDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&adcBufCfg, 0, sizeof(OdsDemo_ADCBufCfg));

    /* Populate configuration: */
    adcBufCfg.adcFmt          = (uint8_t) atoi (argv[2]);
    adcBufCfg.iqSwapSel       = (uint8_t) atoi (argv[3]);
    adcBufCfg.chInterleave    = (uint8_t) atoi (argv[4]);
    adcBufCfg.chirpThreshold  = (uint8_t) atoi (argv[5]);

    /* Save Configuration to use later */
    OdsDemo_mssCfgUpdate((void *)&adcBufCfg, offsetof(OdsDemo_CliCfg_t, adcBufCfg),
        sizeof(OdsDemo_ADCBufCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(OdsDemo_message));
    message.type = ODSDEMO_MSS2DSS_ADCBUFCFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.adcBufCfg, (void *)&adcBufCfg, sizeof(OdsDemo_ADCBufCfg));

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for compensation of range bias and channel phase offsets
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    OdsDemo_compRxChannelBiasCfg_t   cfg;
    OdsDemo_message     message;
    int32_t Re, Im;
    int32_t argInd;
    int32_t i;

    /* Sanity Check: Minimum argument check */
    if (argc != (1+1+SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(OdsDemo_compRxChannelBiasCfg_t));

    /* Populate configuration: */
    cfg.rangeBias          = (float) atof (argv[1]);

    argInd = 2;
    for (i=0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        Re = (int32_t) (atof (argv[argInd++]) * 32768.);
        Re = ODSDEMO_SATURATE_HIGH(Re);
        Re = ODSDEMO_SATURATE_LOW(Re);
        cfg.rxChPhaseComp[i].real = (int16_t) Re;

        Im = (int32_t) (atof (argv[argInd++]) * 32768.);
        Im = ODSDEMO_SATURATE_HIGH(Im);
        Im = ODSDEMO_SATURATE_LOW(Im);
        cfg.rxChPhaseComp[i].imag = (int16_t) Im;

    }
    /* Save Configuration to use later */
    memcpy((void *) &gOdsMssMCB.cliCommonCfg.compRxChanCfg, &cfg, sizeof(OdsDemo_compRxChannelBiasCfg_t));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(OdsDemo_message));
    message.type = ODSDEMO_MSS2DSS_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE;
    memcpy((void *)&message.body.compRxChanCfg, (void *)&cfg, sizeof(OdsDemo_compRxChannelBiasCfg_t));

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for measurement configuration of range bias
 *      and channel phase offsets
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    OdsDemo_measureRxChannelBiasCfg_t   cfg;
    OdsDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(OdsDemo_measureRxChannelBiasCfg_t));

    /* Populate configuration: */
    cfg.enabled          = (uint8_t) atoi (argv[1]);
    cfg.targetDistance   = (float) atof (argv[2]);
    cfg.searchWinSize   = (float) atof (argv[3]);

    /* Save Configuration to use later */
    memcpy((void *) &gOdsMssMCB.cliCommonCfg.measureRxChanCfg, &cfg, sizeof(OdsDemo_measureRxChannelBiasCfg_t));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(OdsDemo_message));
    message.type = ODSDEMO_MSS2DSS_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE;
    memcpy((void *)&message.body.measureRxChanCfg, (void *)&cfg, sizeof(OdsDemo_measureRxChannelBiasCfg_t));

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for BPM configuration supported by the ods Demo
 *      Note that there is a generic BPM configuration command supported by
 *      utils/cli and mmwave. The generic BPM command is not supported by the
 *      demo as the ods demo assumes a specific BPM pattern for the TX antennas.
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLIBpmCfg (int32_t argc, char* argv[])
{

    int8_t           subFrameNum;
    OdsDemo_BpmCfg   cfg;
    OdsDemo_message  message;
    
    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    
    if(OdsDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(OdsDemo_BpmCfg));

    /* Populate configuration: */
    cfg.isEnabled = (bool) atoi(argv[2]) ;
    cfg.chirp0Idx = (uint16_t) atoi(argv[3]) ;
    cfg.chirp1Idx = (uint16_t) atoi(argv[4]) ;

    /* Save Configuration to use later */
    OdsDemo_mssCfgUpdate((void *)&cfg, offsetof(OdsDemo_CliCfg_t, bpmCfg),
        sizeof(OdsDemo_BpmCfg), subFrameNum);
        
    message.type = ODSDEMO_MSS2DSS_BPM_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.bpmCfg, (void *)&cfg, sizeof(OdsDemo_BpmCfg));

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for configuring CQ RX Saturation monitor
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLIChirpQualityRxSatMonCfg (int32_t argc, char* argv[])
{
    rlRxSatMonConf_t        cqSatMonCfg;
    OdsDemo_message         message;

    /* Sanity Check: Minimum argument check */
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cqSatMonCfg, 0, sizeof(rlRxSatMonConf_t));

    /* Populate configuration: */
    cqSatMonCfg.profileIndx                 = (uint8_t) atoi (argv[1]);
    
    if(cqSatMonCfg.profileIndx < RL_MAX_PROFILES_CNT)
    {
        
        cqSatMonCfg.satMonSel                   = (uint8_t) atoi (argv[2]);
        cqSatMonCfg.primarySliceDuration        = (uint16_t) atoi (argv[3]);
        cqSatMonCfg.numSlices                   = (uint16_t) atoi (argv[4]);
        cqSatMonCfg.rxChannelMask               = (uint8_t) atoi (argv[5]);
        
        /* Save Configuration to use later */
        memcpy((void *) &gOdsMssMCB.cliCommonCfg.cqSatMonCfg[cqSatMonCfg.profileIndx], 
                       &cqSatMonCfg, 
                       sizeof(rlRxSatMonConf_t));

        /* Send configuration to DSS */
        memset((void *)&message, 0, sizeof(OdsDemo_message));
        message.type = ODSDEMO_MSS2DSS_CQ_SATURATION_MONITOR;
        memcpy((void *)&message.body.cqSatMonCfg, (void *)&cqSatMonCfg, sizeof(rlRxSatMonConf_t));

        if (OdsDemo_mboxWrite(&message) == 0)
            return 0;
        else
            return -1;
    }
    else
    {
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for configuring CQ Singal & Image band monitor
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLIChirpQualitySigImgMonCfg (int32_t argc, char* argv[])
{
    rlSigImgMonConf_t       cqSigImgMonCfg;
    OdsDemo_message         message;

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cqSigImgMonCfg, 0, sizeof(rlSigImgMonConf_t));

    /* Populate configuration: */
    cqSigImgMonCfg.profileIndx              = (uint8_t) atoi (argv[1]);

    if(cqSigImgMonCfg.profileIndx < RL_MAX_PROFILES_CNT)
    {
    
        cqSigImgMonCfg.numSlices            = (uint8_t) atoi (argv[2]);
        cqSigImgMonCfg.timeSliceNumSamples  = (uint16_t) atoi (argv[3]);

        /* Save Configuration to use later */
        memcpy((void *) &gOdsMssMCB.cliCommonCfg.cqSigImgMonCfg[cqSigImgMonCfg.profileIndx], 
                &cqSigImgMonCfg, 
                sizeof(rlSigImgMonConf_t));

        /* Send configuration to DSS */
        memset((void *)&message, 0, sizeof(OdsDemo_message));
        message.type = ODSDEMO_MSS2DSS_CQ_SIGIMG_MONITOR;
        memcpy((void *)&message.body.cqSigImgMonCfg, (void *)&cqSigImgMonCfg, sizeof(rlSigImgMonConf_t));

        if (OdsDemo_mboxWrite(&message) == 0)
            return 0;
        else
            return -1;
    }
    else
    {
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for enabling analog monitors
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLIAnalogMonitorCfg (int32_t argc, char* argv[])
{
    OdsDemo_message     message;
    
    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Save Configuration to use later */
    gOdsMssMCB.cliCommonCfg.anaMonCfg.rxSatMonEn = atoi (argv[1]);
    gOdsMssMCB.cliCommonCfg.anaMonCfg.sigImgMonEn = atoi (argv[2]);
    
    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(OdsDemo_message));
    message.type = ODSDEMO_MSS2DSS_ANALOG_MONITOR;
    memcpy((void *)&message.body.anaMonCfg, 
            (void *)&gOdsMssMCB.cliCommonCfg.anaMonCfg, 
            sizeof(OdsDemo_AnaMonitorCfg));

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the High Speed Interface
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t OdsDemo_CLILvdsStreamCfg (int32_t argc, char* argv[])
{

    int8_t                  subFrameNum;
    OdsDemo_LvdsStreamCfg   cfg;
    OdsDemo_message         message;
    
    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    
    if(OdsDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(OdsDemo_LvdsStreamCfg));

    /* Populate configuration: */
    cfg.isHeaderEnabled = (bool) atoi(argv[2]) ;
    cfg.dataFmt         = (uint8_t) atoi(argv[3]) ;
    cfg.isSwEnabled     = (bool) atoi(argv[4]) ;

    /* Save Configuration to use later */
    OdsDemo_mssCfgUpdate((void *)&cfg, offsetof(OdsDemo_CliCfg_t, lvdsStreamCfg),
        sizeof(OdsDemo_LvdsStreamCfg), subFrameNum);
        
    message.type = ODSDEMO_MSS2DSS_LVDSSTREAM_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.lvdsStreamCfg, (void *)&cfg, sizeof(OdsDemo_LvdsStreamCfg));

    if (OdsDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
void OdsDemo_CLIInit (void)
{
    CLI_Cfg     cliCfg;
    char        demoBanner[256];
    uint32_t    cnt;

    /* Create Demo Banner to be printed out by CLI */
    sprintf(&demoBanner[0], 
                       "******************************************\n" \
                       "xWR16xx ODS Demo %02d.%02d.%02d.%02d\n"  \
                       "******************************************\n", 
                        MMWAVE_ODS_VERSION_MAJOR,
                        MMWAVE_ODS_VERSION_MINOR,
                        MMWAVE_ODS_VERSION_BUGFIX,
                        MMWAVE_ODS_VERSION_BUILD
            );

    /* Initialize the CLI configuration: */
    memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

    /* Populate the CLI configuration: */
    cliCfg.cliPrompt                    = "odsDemo:/>";
    cliCfg.cliBanner                    = demoBanner;
    cliCfg.cliUartHandle                = gOdsMssMCB.commandUartHandle;
    cliCfg.taskPriority                 = 3;
    cliCfg.mmWaveHandle                 = gOdsMssMCB.ctrlHandle;
    cliCfg.enableMMWaveExtension        = 1U;
    cliCfg.usePolledMode                = true;
    cnt=0;
    cliCfg.tableEntry[cnt].cmd            = "sensorStart";
    cliCfg.tableEntry[cnt].helpString     = "[doReconfig(optional, default:enabled)]";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLISensorStart;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "sensorStop";
    cliCfg.tableEntry[cnt].helpString     = "No arguments";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLISensorStop;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "guiMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <detectedObjects> <logMagRange> <noiseProfile> <rangeAzimuthHeatMap> <rangeDopplerHeatMap> <statsInfo>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLIGuiMonSel;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "cfarCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <procDirection> <averageMode> <winLen> <guardLen> <noiseDiv> <cyclicMode> <thresholdScale>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLICfarCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "peakGrouping";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <groupingMode> <rangeDimEn> <dopplerDimEn> <startRangeIdx> <endRangeIdx>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLIPeakGroupingCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "dataLogger";
    cliCfg.tableEntry[cnt].helpString     = "<mssLogger | dssLogger>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLISetDataLogger;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "multiObjBeamForming";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <threshold>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLIMultiObjBeamForming;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "calibDcRangeSig";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <negativeBinIdx> <positiveBinIdx> <numAvgFrames>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLICalibDcRangeSig;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "extendedMaxVelocity";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLIExtendedMaxVelocity;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "clutterRemoval";
    cliCfg.tableEntry[cnt].helpString     = "<enabled>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLIClutterRemoval;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "adcbufCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <adcOutputFmt> <SampleSwap> <ChanInterleave> <ChirpThreshold>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLIADCBufCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "compRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[cnt].helpString     = "<rangeBias> <Re00> <Im00> <Re01> <Im01> <Re02> <Im02> <Re03> <Im03> <Re10> <Im10> <Re11> <Im11> <Re12> <Im12> <Re13> <Im13> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLICompRangeBiasAndRxChanPhaseCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "measureRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[cnt].helpString     = "<enabled> <targetDistance> <searchWin>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "bpmCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <chirp0Idx> <chirp1Idx>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLIBpmCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "nearFieldCfg";
    cliCfg.tableEntry[cnt].helpString     = "<enabled> <startRangeIndex> <endRangeIndex>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLINearFieldCorrection;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "CQRxSatMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<profile> <satMonSel> <priSliceDuration> <numSlices> <rxChanMask>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLIChirpQualityRxSatMonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "CQSigImgMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<profile> <numSlices> <numSamplePerSlice>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLIChirpQualitySigImgMonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "analogMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<rxSaturation> <sigImgBand>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLIAnalogMonitorCfg;
    cnt++;
    
    cliCfg.tableEntry[cnt].cmd            = "lvdsStreamCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enableHeader> <dataFmt> <enableSW>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = OdsDemo_CLILvdsStreamCfg;
    cnt++;   
    
    /* Open the CLI: */
    if (CLI_open (&cliCfg) < 0)
    {
        System_printf ("Error: Unable to open the CLI\n");
        return;
    }
    System_printf ("Debug: CLI is operational\n");
    return;
}


