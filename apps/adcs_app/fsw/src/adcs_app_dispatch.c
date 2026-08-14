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
 *   This file contains the source code for the Adcs App.
 */

/*
** Include Files:
*/
#include "adcs_app.h"
#include "adcs_app_dispatch.h"
#include "adcs_app_cmds.h"
#include "adcs_app_eventids.h"
#include "adcs_app_msgids.h"
#include "adcs_app_msg.h"

#include "nav_interface_app_msgids.h"
#include "nav_interface_app_msg.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* Verify command packet length                                               */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool ADCS_APP_VerifyCmdLength(const CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    bool              result       = true;
    size_t            ActualLength = 0;
    CFE_SB_MsgId_t    MsgId        = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_FcnCode_t FcnCode      = 0;

    CFE_MSG_GetSize(MsgPtr, &ActualLength);

    /*
    ** Verify the command packet length.
    */
    if (ExpectedLength != ActualLength)
    {
        CFE_MSG_GetMsgId(MsgPtr, &MsgId);
        CFE_MSG_GetFcnCode(MsgPtr, &FcnCode);

        CFE_EVS_SendEvent(ADCS_APP_CMD_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Invalid Msg length: ID = 0x%X,  CC = %u, Len = %u, Expected = %u",
                          (unsigned int)CFE_SB_MsgIdToValue(MsgId), (unsigned int)FcnCode, (unsigned int)ActualLength,
                          (unsigned int)ExpectedLength);

        result = false;

        ADCS_APP_Data.ErrCounter++;
    }

    return result;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* ADCS ground commands                                                     */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void ADCS_APP_ProcessGroundCommand(const CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    /*
    ** Process ADCS app ground commands
    */
    switch (CommandCode)
    {
        case ADCS_APP_NOOP_CC:
            if (ADCS_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(ADCS_APP_NoopCmd_t)))
            {
                ADCS_APP_NoopCmd((const ADCS_APP_NoopCmd_t *)SBBufPtr);
            }
            break;

        case ADCS_APP_RESET_COUNTERS_CC:
            if (ADCS_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(ADCS_APP_ResetCountersCmd_t)))
            {
                ADCS_APP_ResetCountersCmd((const ADCS_APP_ResetCountersCmd_t *)SBBufPtr);
            }
            break;

        case ADCS_APP_PROCESS_CC:
            if (ADCS_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(ADCS_APP_ProcessCmd_t)))
            {
                ADCS_APP_ProcessCmd((const ADCS_APP_ProcessCmd_t *)SBBufPtr);
            }
            break;

        case ADCS_APP_DISPLAY_PARAM_CC:
            if (ADCS_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(ADCS_APP_DisplayParamCmd_t)))
            {
                ADCS_APP_DisplayParamCmd((const ADCS_APP_DisplayParamCmd_t *)SBBufPtr);
            }
            break;

        case ADCS_APP_INGEST_NAV_CC:
            if (ADCS_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(ADCS_APP_IngestNavCmd_t)))
            {
                ADCS_APP_IngestNavCmd((const ADCS_APP_IngestNavCmd_t *)SBBufPtr);
            }
            break;

        case ADCS_APP_TORQUE_REQ_CC:
            if (ADCS_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(ADCS_APP_TorqueReqCmd_t)))
            {
                ADCS_APP_TorqueReqCmd((const ADCS_APP_TorqueReqCmd_t *)SBBufPtr);
            }
            break;

        /* default case already found during FC vs length test */
        default:
            CFE_EVS_SendEvent(ADCS_APP_CC_ERR_EID, CFE_EVS_EventType_ERROR, "Invalid ground command code: CC = %d",
                              CommandCode);
            break;
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the ADCS    */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void ADCS_APP_TaskPipe(const CFE_SB_Buffer_t *SBBufPtr)
{
    static CFE_SB_MsgId_t CMD_MID          = CFE_SB_MSGID_RESERVED;
    static CFE_SB_MsgId_t SEND_HK_MID      = CFE_SB_MSGID_RESERVED;
    static CFE_SB_MsgId_t NAV_STATE_TLM_MID = CFE_SB_MSGID_RESERVED;

    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    /* cache the local MID Values here, this avoids repeat lookups */
    if (!CFE_SB_IsValidMsgId(CMD_MID))
    {
        CMD_MID          = CFE_SB_ValueToMsgId(ADCS_APP_CMD_MID);
        SEND_HK_MID      = CFE_SB_ValueToMsgId(ADCS_APP_SEND_HK_MID);
        NAV_STATE_TLM_MID = CFE_SB_ValueToMsgId(NAV_INTERFACE_APP_NAV_STATE_TLM_MID);
    }

    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

    /* Process all SB messages */
    if (CFE_SB_MsgId_Equal(MsgId, SEND_HK_MID))
    {
        /* Housekeeping request */
        ADCS_APP_SendHkCmd((const ADCS_APP_SendHkCmd_t *)SBBufPtr);
    }
    else if (CFE_SB_MsgId_Equal(MsgId, CMD_MID))
    {
        /* Ground command */
        ADCS_APP_ProcessGroundCommand(SBBufPtr);
    }
    else if (CFE_SB_MsgId_Equal(MsgId, NAV_STATE_TLM_MID))
    {
        const NAV_INTERFACE_APP_NavStateTlm_t *NavState = (const NAV_INTERFACE_APP_NavStateTlm_t *)SBBufPtr;
        ADCS_APP_Data.LatestNav.TimeNanos = NavState->Payload.TimeNanos;
        ADCS_APP_Data.LatestNav.SatId     = NavState->Payload.SatId;
        ADCS_APP_Data.LatestNav.r_BN_N[0] = NavState->Payload.PosX_N;
        ADCS_APP_Data.LatestNav.r_BN_N[1] = NavState->Payload.PosY_N;
        ADCS_APP_Data.LatestNav.r_BN_N[2] = NavState->Payload.PosZ_N;
        ADCS_APP_Data.LatestNav.v_BN_N[0] = NavState->Payload.VelX_N;
        ADCS_APP_Data.LatestNav.v_BN_N[1] = NavState->Payload.VelY_N;
        ADCS_APP_Data.LatestNav.v_BN_N[2] = NavState->Payload.VelZ_N;
        ADCS_APP_Data.LatestNav.sigma_BN[0] = NavState->Payload.SigmaX_BN;
        ADCS_APP_Data.LatestNav.sigma_BN[1] = NavState->Payload.SigmaY_BN;
        ADCS_APP_Data.LatestNav.sigma_BN[2] = NavState->Payload.SigmaZ_BN;
        ADCS_APP_Data.LatestNav.omega_BN_B[0] = NavState->Payload.OmegaX_BN_B;
        ADCS_APP_Data.LatestNav.omega_BN_B[1] = NavState->Payload.OmegaY_BN_B;
        ADCS_APP_Data.LatestNav.omega_BN_B[2] = NavState->Payload.OmegaZ_BN_B;
        ADCS_APP_Data.NavPacketsReceived++;
        ADCS_APP_Data.NavStateValid = 1;
        ADCS_APP_Data.NavStateSequence++;
        ADCS_APP_Data.NavStateTimeNanos = NavState->Payload.TimeNanos;

        CFE_EVS_SendEvent(ADCS_APP_VALUE_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "ADCS received NAV state: time=%llu sat=%llu",
                          (unsigned long long)NavState->Payload.TimeNanos,
                          (unsigned long long)NavState->Payload.SatId);
    }
    else
    {
        /* Unknown command */
        CFE_EVS_SendEvent(ADCS_APP_MID_ERR_EID, CFE_EVS_EventType_ERROR, "ADCS: invalid command packet,MID = 0x%x",
                          (unsigned int)CFE_SB_MsgIdToValue(MsgId));
    }
}
