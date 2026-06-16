/************************************************************************
 * NASA Docket No. GSC-19,200-1, and identified as "cFS Draco"
 *
 * Copyright (c) 2023 United States Government as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License. You may obtain
 * a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ************************************************************************/

/**
 * \file
 *   This file contains the source code for the Gnc App Ground Command-handling functions
 */

/*
** Include Files:
*/
#include "gnc_app.h"
#include "gnc_app_cmds.h"
#include "gnc_app_msgids.h"
#include "gnc_app_eventids.h"
#include "gnc_app_version.h"
#include "gnc_app_tbl.h"
#include "gnc_app_utils.h"
#include "gnc_app_msg.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
CFE_Status_t GNC_APP_SendHkCmd(const GNC_APP_SendHkCmd_t *Msg)
{
    int i;

    /*
    ** Get command execution counters...
    */
    GNC_APP_Data.HkTlm.Payload.CommandErrorCounter = GNC_APP_Data.ErrCounter;
    GNC_APP_Data.HkTlm.Payload.CommandCounter      = GNC_APP_Data.CmdCounter;

    /*
    ** Send housekeeping telemetry packet...
    */
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(GNC_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(GNC_APP_Data.HkTlm.TelemetryHeader), true);

    /*
    ** Manage any pending table loads, validations, etc.
    */
    for (i = 0; i < GNC_APP_PLATFORM_NUMBER_OF_TABLES; i++)
    {
        CFE_TBL_Manage(GNC_APP_Data.TblHandles[i]);
    }

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* GNC NOOP commands                                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
CFE_Status_t GNC_APP_NoopCmd(const GNC_APP_NoopCmd_t *Msg)
{
    GNC_APP_Data.CmdCounter++;

    CFE_EVS_SendEvent(GNC_APP_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION, "GNC: NOOP command %s",
                      GNC_APP_VERSION);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function resets all the global counter variables that are     */
/*         part of the task telemetry.                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
CFE_Status_t GNC_APP_ResetCountersCmd(const GNC_APP_ResetCountersCmd_t *Msg)
{
    GNC_APP_Data.CmdCounter = 0;
    GNC_APP_Data.ErrCounter = 0;

    CFE_EVS_SendEvent(GNC_APP_RESET_INF_EID, CFE_EVS_EventType_INFORMATION, "GNC: RESET command");

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function Process Ground Station Command                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
CFE_Status_t GNC_APP_ProcessCmd(const GNC_APP_ProcessCmd_t *Msg)
{
    CFE_Status_t               Status;
    void *                     TblAddr;
    GNC_APP_ExampleTable_t *TblPtr;
    const char *               TableName = "GNC_APP.ExampleTable";

    /* Gnc Use of Example Table */
    GNC_APP_Data.CmdCounter++;
    Status = CFE_TBL_GetAddress(&TblAddr, GNC_APP_Data.TblHandles[0]);
    if (Status < CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Gnc App: Fail to get table address: 0x%08lx", (unsigned long)Status);
    }
    else
    {
        TblPtr = TblAddr;
        CFE_ES_WriteToSysLog("Gnc App: Example Table Value 1: %d  Value 2: %d", TblPtr->Int1, TblPtr->Int2);

        GNC_APP_GetCrc(TableName);

        Status = CFE_TBL_ReleaseAddress(GNC_APP_Data.TblHandles[0]);
        if (Status != CFE_SUCCESS)
        {
            CFE_ES_WriteToSysLog("Gnc App: Fail to release table address: 0x%08lx", (unsigned long)Status);
        }
    }

    return Status;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* A simple example command that displays a passed-in value                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
CFE_Status_t GNC_APP_DisplayParamCmd(const GNC_APP_DisplayParamCmd_t *Msg)
{
    GNC_APP_Data.CmdCounter++;
    CFE_EVS_SendEvent(GNC_APP_VALUE_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "GNC_APP: ValU32=%lu, ValI16=%d, ValStr=%s", (unsigned long)Msg->Payload.ValU32,
                      (int)Msg->Payload.ValI16, Msg->Payload.ValStr);

    return CFE_SUCCESS;
}
