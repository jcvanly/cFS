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
 *   This file contains the source code for the Adcs App Ground Command-handling functions
 */

/*
** Include Files:
*/
#include "adcs_app.h"
#include "adcs_app_cmds.h"
#include "adcs_app_msgids.h"
#include "adcs_app_eventids.h"
#include "adcs_app_version.h"
#include "adcs_app_tbl.h"
#include "adcs_app_utils.h"
#include "adcs_app_msg.h"

#include "nav_interface_app_fcncode_values.h"
#include "nav_interface_app_msgids.h"

typedef struct
{
    CFE_MSG_CommandHeader_t CommandHeader;
    struct
    {
        double TorqueX_B;
        double TorqueY_B;
        double TorqueZ_B;
    } Payload;
} ADCS_APP_NavTorqueReqCmd_t;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
CFE_Status_t ADCS_APP_SendHkCmd(const ADCS_APP_SendHkCmd_t *Msg)
{
    int i;

    /*
    ** Get command execution counters...
    */
    ADCS_APP_Data.HkTlm.Payload.CommandErrorCounter = ADCS_APP_Data.ErrCounter;
    ADCS_APP_Data.HkTlm.Payload.CommandCounter      = ADCS_APP_Data.CmdCounter;

    /*
    ** Send housekeeping telemetry packet...
    */
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(ADCS_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(ADCS_APP_Data.HkTlm.TelemetryHeader), true);

    /*
    ** Manage any pending table loads, validations, etc.
    */
    for (i = 0; i < ADCS_APP_PLATFORM_NUMBER_OF_TABLES; i++)
    {
        CFE_TBL_Manage(ADCS_APP_Data.TblHandles[i]);
    }

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* ADCS NOOP commands                                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
CFE_Status_t ADCS_APP_NoopCmd(const ADCS_APP_NoopCmd_t *Msg)
{
    ADCS_APP_Data.CmdCounter++;

    CFE_EVS_SendEvent(ADCS_APP_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION, "ADCS: NOOP command %s",
                      ADCS_APP_VERSION);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function resets all the global counter variables that are     */
/*         part of the task telemetry.                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
CFE_Status_t ADCS_APP_ResetCountersCmd(const ADCS_APP_ResetCountersCmd_t *Msg)
{
    ADCS_APP_Data.CmdCounter = 0;
    ADCS_APP_Data.ErrCounter = 0;

    CFE_EVS_SendEvent(ADCS_APP_RESET_INF_EID, CFE_EVS_EventType_INFORMATION, "ADCS: RESET command");

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function Process Ground Station Command                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
CFE_Status_t ADCS_APP_ProcessCmd(const ADCS_APP_ProcessCmd_t *Msg)
{
    CFE_Status_t               Status;
    void *                     TblAddr;
    ADCS_APP_ExampleTable_t *TblPtr;
    const char *               TableName = "ADCS_APP.ExampleTable";

    /* Adcs Use of Example Table */
    ADCS_APP_Data.CmdCounter++;
    Status = CFE_TBL_GetAddress(&TblAddr, ADCS_APP_Data.TblHandles[0]);
    if (Status < CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Adcs App: Fail to get table address: 0x%08lx", (unsigned long)Status);
    }
    else
    {
        TblPtr = TblAddr;
        CFE_ES_WriteToSysLog("Adcs App: Example Table Value 1: %d  Value 2: %d", TblPtr->Int1, TblPtr->Int2);

        ADCS_APP_GetCrc(TableName);

        Status = CFE_TBL_ReleaseAddress(ADCS_APP_Data.TblHandles[0]);
        if (Status != CFE_SUCCESS)
        {
            CFE_ES_WriteToSysLog("Adcs App: Fail to release table address: 0x%08lx", (unsigned long)Status);
        }
    }

    return Status;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* A simple example command that displays a passed-in value                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
CFE_Status_t ADCS_APP_DisplayParamCmd(const ADCS_APP_DisplayParamCmd_t *Msg)
{
    ADCS_APP_Data.CmdCounter++;
    CFE_EVS_SendEvent(ADCS_APP_VALUE_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "ADCS_APP: ValU32=%lu, ValI16=%d, ValStr=%s", (unsigned long)Msg->Payload.ValU32,
                      (int)Msg->Payload.ValI16, Msg->Payload.ValStr);

    return CFE_SUCCESS;
}

CFE_Status_t ADCS_APP_IngestNavCmd(const ADCS_APP_IngestNavCmd_t *Msg)
{
    ADCS_APP_Data.LatestNav.TimeNanos = Msg->Payload.TimeNanos;
    ADCS_APP_Data.LatestNav.SatId     = Msg->Payload.SatId;

    ADCS_APP_Data.LatestNav.r_BN_N[0] = Msg->Payload.PosX_N;
    ADCS_APP_Data.LatestNav.r_BN_N[1] = Msg->Payload.PosY_N;
    ADCS_APP_Data.LatestNav.r_BN_N[2] = Msg->Payload.PosZ_N;

    ADCS_APP_Data.LatestNav.v_BN_N[0] = Msg->Payload.VelX_N;
    ADCS_APP_Data.LatestNav.v_BN_N[1] = Msg->Payload.VelY_N;
    ADCS_APP_Data.LatestNav.v_BN_N[2] = Msg->Payload.VelZ_N;

    ADCS_APP_Data.NavPacketsReceived++;

    return CFE_SUCCESS;
}


CFE_Status_t ADCS_APP_TorqueReqCmd(const ADCS_APP_TorqueReqCmd_t *Msg)
{
    CFE_Status_t                Status = CFE_SUCCESS;
    ADCS_APP_NavTorqueReqCmd_t  Cmd;

    CFE_EVS_SendEvent(ADCS_APP_VALUE_INF_EID,
                    CFE_EVS_EventType_INFORMATION,
                    "ADCS APP: Torque request torque=(%.6f %.6f %.6f)",
                    Msg->Payload.TorqueX_B,
                    Msg->Payload.TorqueY_B,
                    Msg->Payload.TorqueZ_B);

    CFE_MSG_Init(CFE_MSG_PTR(Cmd.CommandHeader), CFE_SB_ValueToMsgId(NAV_INTERFACE_APP_CMD_MID), sizeof(Cmd));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(Cmd.CommandHeader), NAV_INTERFACE_APP_FunctionCode_TORQUE_REQ);

    Cmd.Payload.TorqueX_B = Msg->Payload.TorqueX_B;
    Cmd.Payload.TorqueY_B = Msg->Payload.TorqueY_B;
    Cmd.Payload.TorqueZ_B = Msg->Payload.TorqueZ_B;

    Status = CFE_SB_TransmitMsg(CFE_MSG_PTR(Cmd.CommandHeader), true);
    if (Status == CFE_SUCCESS)
    {
        ADCS_APP_Data.CmdCounter++;
    }
    else
    {
        ADCS_APP_Data.ErrCounter++;
        CFE_EVS_SendEvent(ADCS_APP_NAV_REQ_ERR_EID, CFE_EVS_EventType_ERROR,
                          "ADCS APP: Failed to publish torque request to NAV interface, RC=0x%08lX",
                          (unsigned long)Status);
    }

    return Status;
}
