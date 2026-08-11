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
 *   This file contains the source code for the Nav_interface App Ground Command-handling functions
 */

/*
** Include Files:
*/
#include "nav_interface_app.h"
#include "nav_interface_app_cmds.h"
#include "nav_interface_app_msgids.h"
#include "nav_interface_app_eventids.h"
#include "nav_interface_app_version.h"
#include "nav_interface_app_tbl.h"
#include "nav_interface_app_utils.h"
#include "nav_interface_app_msg.h"

#include <errno.h>
#include <string.h>
#include <sys/socket.h>

static CFE_Status_t NAV_INTERFACE_APP_SendCombinedCmd(void)
{
    CFE_Status_t                 Status = CFE_SUCCESS;
    NAV_INTERFACE_APP_BskCmdPacket_t Cmd;
    ssize_t                      Bytes;

    memset(&Cmd, 0, sizeof(Cmd));

    Cmd.TimeNanos = NAV_INTERFACE_APP_Data.LatestNav.TimeNanos;
    Cmd.SatId     = NAV_INTERFACE_APP_Data.LatestNav.SatId;

    Cmd.force_N[0] = NAV_INTERFACE_APP_Data.LatestForce_N[0];
    Cmd.force_N[1] = NAV_INTERFACE_APP_Data.LatestForce_N[1];
    Cmd.force_N[2] = NAV_INTERFACE_APP_Data.LatestForce_N[2];

    Cmd.torque_B[0] = NAV_INTERFACE_APP_Data.LatestTorque_B[0];
    Cmd.torque_B[1] = NAV_INTERFACE_APP_Data.LatestTorque_B[1];
    Cmd.torque_B[2] = NAV_INTERFACE_APP_Data.LatestTorque_B[2];

    Bytes = sendto(NAV_INTERFACE_APP_Data.UdpSocket, &Cmd, sizeof(Cmd), 0,
                   (struct sockaddr *)&NAV_INTERFACE_APP_Data.BasiliskAddr,
                   sizeof(NAV_INTERFACE_APP_Data.BasiliskAddr));

    if (Bytes == (ssize_t)sizeof(Cmd))
    {
        NAV_INTERFACE_APP_Data.UdpCmdPacketsSent++;
        NAV_INTERFACE_APP_Data.LatestCmd = Cmd;
    }
    else
    {
        NAV_INTERFACE_APP_Data.ErrCounter++;
        Status = CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
        CFE_EVS_SendEvent(NAV_INTERFACE_APP_UDP_TX_ERR_EID, CFE_EVS_EventType_ERROR,
                          "NAV_INTERFACE: Failed to send Basilisk cmd, errno=%d", errno);
    }

    return Status;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
CFE_Status_t NAV_INTERFACE_APP_SendHkCmd(const NAV_INTERFACE_APP_SendHkCmd_t *Msg)
{
    int i;

    /*
    ** Get command execution counters...
    */
    NAV_INTERFACE_APP_Data.HkTlm.Payload.CommandErrorCounter = NAV_INTERFACE_APP_Data.ErrCounter;
    NAV_INTERFACE_APP_Data.HkTlm.Payload.CommandCounter      = NAV_INTERFACE_APP_Data.CmdCounter;

    /*
    ** Send housekeeping telemetry packet...
    */
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(NAV_INTERFACE_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(NAV_INTERFACE_APP_Data.HkTlm.TelemetryHeader), true);

    /*
    ** Manage any pending table loads, validations, etc.
    */
    for (i = 0; i < NAV_INTERFACE_APP_PLATFORM_NUMBER_OF_TABLES; i++)
    {
        CFE_TBL_Manage(NAV_INTERFACE_APP_Data.TblHandles[i]);
    }

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* NAV_INTERFACE NOOP commands                                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
CFE_Status_t NAV_INTERFACE_APP_NoopCmd(const NAV_INTERFACE_APP_NoopCmd_t *Msg)
{
    NAV_INTERFACE_APP_Data.CmdCounter++;

    CFE_EVS_SendEvent(NAV_INTERFACE_APP_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION, "NAV_INTERFACE: NOOP command %s",
                      NAV_INTERFACE_APP_VERSION);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function resets all the global counter variables that are     */
/*         part of the task telemetry.                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
CFE_Status_t NAV_INTERFACE_APP_ResetCountersCmd(const NAV_INTERFACE_APP_ResetCountersCmd_t *Msg)
{
    NAV_INTERFACE_APP_Data.CmdCounter = 0;
    NAV_INTERFACE_APP_Data.ErrCounter = 0;

    CFE_EVS_SendEvent(NAV_INTERFACE_APP_RESET_INF_EID, CFE_EVS_EventType_INFORMATION, "NAV_INTERFACE: RESET command");

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function Process Ground Station Command                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
CFE_Status_t NAV_INTERFACE_APP_ProcessCmd(const NAV_INTERFACE_APP_ProcessCmd_t *Msg)
{
    CFE_Status_t               Status;
    void *                     TblAddr;
    NAV_INTERFACE_APP_ExampleTable_t *TblPtr;
    const char *               TableName = "NAV_INTERFACE_APP.ExampleTable";

    /* Nav_interface Use of Example Table */
    NAV_INTERFACE_APP_Data.CmdCounter++;
    Status = CFE_TBL_GetAddress(&TblAddr, NAV_INTERFACE_APP_Data.TblHandles[0]);
    if (Status < CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Nav_interface App: Fail to get table address: 0x%08lx", (unsigned long)Status);
    }
    else
    {
        TblPtr = TblAddr;
        CFE_ES_WriteToSysLog("Nav_interface App: Example Table Value 1: %d  Value 2: %d", TblPtr->Int1, TblPtr->Int2);

        NAV_INTERFACE_APP_GetCrc(TableName);

        Status = CFE_TBL_ReleaseAddress(NAV_INTERFACE_APP_Data.TblHandles[0]);
        if (Status != CFE_SUCCESS)
        {
            CFE_ES_WriteToSysLog("Nav_interface App: Fail to release table address: 0x%08lx", (unsigned long)Status);
        }

    }

    return Status;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* A simple example command that displays a passed-in value                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
CFE_Status_t NAV_INTERFACE_APP_DisplayParamCmd(const NAV_INTERFACE_APP_DisplayParamCmd_t *Msg)
{
    NAV_INTERFACE_APP_Data.CmdCounter++;
    CFE_EVS_SendEvent(NAV_INTERFACE_APP_VALUE_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "NAV_INTERFACE_APP: ValU32=%lu, ValI16=%d, ValStr=%s", (unsigned long)Msg->Payload.ValU32,
                      (int)Msg->Payload.ValI16, Msg->Payload.ValStr);

    return CFE_SUCCESS;
}

CFE_Status_t NAV_INTERFACE_APP_ForceReqCmd(const NAV_INTERFACE_APP_ForceReqCmd_t *Msg)
{
    NAV_INTERFACE_APP_Data.LatestForce_N[0] = Msg->Payload.ForceX_N;
    NAV_INTERFACE_APP_Data.LatestForce_N[1] = Msg->Payload.ForceY_N;
    NAV_INTERFACE_APP_Data.LatestForce_N[2] = Msg->Payload.ForceZ_N;

    NAV_INTERFACE_APP_Data.CmdCounter++;

    return NAV_INTERFACE_APP_SendCombinedCmd();
}

CFE_Status_t NAV_INTERFACE_APP_TorqueReqCmd(const NAV_INTERFACE_APP_TorqueReqCmd_t *Msg)
{
    NAV_INTERFACE_APP_Data.LatestTorque_B[0] = Msg->Payload.TorqueX_B;
    NAV_INTERFACE_APP_Data.LatestTorque_B[1] = Msg->Payload.TorqueY_B;
    NAV_INTERFACE_APP_Data.LatestTorque_B[2] = Msg->Payload.TorqueZ_B;

    NAV_INTERFACE_APP_Data.CmdCounter++;

    return NAV_INTERFACE_APP_SendCombinedCmd();
}
