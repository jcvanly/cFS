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
#include <math.h>

#include "adcs_app.h"
#include "adcs_app_dispatch.h"
#include "adcs_app_cmds.h"
#include "adcs_app_eventids.h"
#include "adcs_app_msgids.h"
#include "adcs_app_msg.h"

#include "nav_interface_app_msgids.h"
#include "nav_interface_app_msg.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void ADCS_APP_Normalize3(const double v[3], double out[3])
{
    double mag = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    if (mag > 1.0e-12)
    {
        out[0] = v[0] / mag;
        out[1] = v[1] / mag;
        out[2] = v[2] / mag;
    }
    else
    {
        out[0] = 0.0;
        out[1] = 0.0;
        out[2] = 0.0;
    }
}

static void ADCS_APP_Cross3(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static double ADCS_APP_Dot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void ADCS_APP_MrpToDcm(const double sigma[3], double C[3][3])
{
    double s1 = sigma[0];
    double s2 = sigma[1];
    double s3 = sigma[2];

    double s2norm = s1 * s1 + s2 * s2 + s3 * s3;
    double denom   = (1.0 + s2norm) * (1.0 + s2norm);

    double S[3][3] = {
        {0.0, -s3, s2},
        {s3, 0.0, -s1},
        {-s2, s1, 0.0}
    };

    double SS[3][3] = {{0.0}};

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            for (int k = 0; k < 3; ++k)
            {
                SS[i][j] += S[i][k] * S[k][j];
            }
        }
    }

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            double identity = (i == j) ? 1.0 : 0.0;
            C[i][j] = identity + (8.0 * SS[i][j] - 4.0 * (1.0 - s2norm) * S[i][j]) / denom;
        }
    }
}

static void ADCS_APP_DcmToMrp(double C[3][3], double sigma[3])
{
    double tr = C[0][0] + C[1][1] + C[2][2];
    double denom = 1.0 + tr;

    if (denom > 1.0e-12)
    {
        sigma[0] = (C[2][1] - C[1][2]) / denom;
        sigma[1] = (C[0][2] - C[2][0]) / denom;
        sigma[2] = (C[1][0] - C[0][1]) / denom;
        return;
    }

    /* Fallback to the standard shadow-safe branch when trace is near -1. */
    int    idx = 0;
    double maxdiag = C[0][0];

    if (C[1][1] > maxdiag)
    {
        maxdiag = C[1][1];
        idx = 1;
    }
    if (C[2][2] > maxdiag)
    {
        maxdiag = C[2][2];
        idx = 2;
    }

    switch (idx)
    {
        case 0:
            denom = 1.0 + C[0][0] - C[1][1] - C[2][2];
            if (fabs(denom) > 1.0e-12)
            {
                sigma[0] = (C[0][1] + C[1][0]) / denom;
                sigma[1] = (C[0][2] + C[2][0]) / denom;
                sigma[2] = (C[1][2] - C[2][1]) / denom;
            }
            else
            {
                sigma[0] = 0.0;
                sigma[1] = 0.0;
                sigma[2] = 0.0;
            }
            break;

        case 1:
            denom = 1.0 - C[0][0] + C[1][1] - C[2][2];
            if (fabs(denom) > 1.0e-12)
            {
                sigma[0] = (C[0][1] + C[1][0]) / denom;
                sigma[1] = (C[1][2] + C[2][1]) / denom;
                sigma[2] = (C[2][0] - C[0][2]) / denom;
            }
            else
            {
                sigma[0] = 0.0;
                sigma[1] = 0.0;
                sigma[2] = 0.0;
            }
            break;

        default:
            denom = 1.0 - C[0][0] - C[1][1] + C[2][2];
            if (fabs(denom) > 1.0e-12)
            {
                sigma[0] = (C[2][0] - C[0][2]) / denom;
                sigma[1] = (C[1][2] + C[2][1]) / denom;
                sigma[2] = (C[2][0] + C[0][2]) / denom;
            }
            else
            {
                sigma[0] = 0.0;
                sigma[1] = 0.0;
                sigma[2] = 0.0;
            }
            break;
    }
}

static void ADCS_APP_ComputeNadirGuidance(void)
{
    const double *r = ADCS_APP_Data.LatestNav.r_BN_N;
    const double *v = ADCS_APP_Data.LatestNav.v_BN_N;
    const double *sigma = ADCS_APP_Data.LatestNav.sigma_BN;
    double        rmag = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    double        C_BN[3][3];
    double        C_RN[3][3];
    double        C_BR[3][3];
    double        x_temp[3];
    double        x_R_N[3];
    double        y_R_N[3];
    double        z_R_N[3];
    double        h_N[3];
    double        omega_RN_N[3];
    double        omega_RN_B[3];
    double        boresight_N[3];
    double        dot;
    double        clampDot;
    double        errorRad;
    double        errorDeg;
    static uint32_t EventCounter = 0;

    memset(&ADCS_APP_Data.Guidance, 0, sizeof(ADCS_APP_Data.Guidance));

    if (rmag <= 1.0e-9)
    {
        return;
    }

    z_R_N[0] = -r[0] / rmag;
    z_R_N[1] = -r[1] / rmag;
    z_R_N[2] = -r[2] / rmag;

    ADCS_APP_Normalize3(z_R_N, ADCS_APP_Data.Guidance.nadir_N);
    ADCS_APP_MrpToDcm(sigma, C_BN);

    boresight_N[0] = C_BN[2][0];
    boresight_N[1] = C_BN[2][1];
    boresight_N[2] = C_BN[2][2];
    ADCS_APP_Normalize3(boresight_N, ADCS_APP_Data.Guidance.boresight_N);

    dot = ADCS_APP_Dot3(ADCS_APP_Data.Guidance.nadir_N, ADCS_APP_Data.Guidance.boresight_N);
    if (dot > 1.0)
    {
        clampDot = 1.0;
    }
    else if (dot < -1.0)
    {
        clampDot = -1.0;
    }
    else
    {
        clampDot = dot;
    }

    errorRad = acos(clampDot);
    errorDeg = errorRad * 180.0 / M_PI;
    ADCS_APP_Data.Guidance.pointingErrorDeg = errorDeg;

    x_temp[0] = v[0] - ADCS_APP_Dot3(v, ADCS_APP_Data.Guidance.nadir_N) * ADCS_APP_Data.Guidance.nadir_N[0];
    x_temp[1] = v[1] - ADCS_APP_Dot3(v, ADCS_APP_Data.Guidance.nadir_N) * ADCS_APP_Data.Guidance.nadir_N[1];
    x_temp[2] = v[2] - ADCS_APP_Dot3(v, ADCS_APP_Data.Guidance.nadir_N) * ADCS_APP_Data.Guidance.nadir_N[2];

    if (sqrt(x_temp[0] * x_temp[0] + x_temp[1] * x_temp[1] + x_temp[2] * x_temp[2]) > 1.0e-12)
    {
        ADCS_APP_Normalize3(x_temp, x_R_N);
    }
    else
    {
        double ref[3] = {1.0, 0.0, 0.0};
        double tmp[3];

        ADCS_APP_Cross3(ref, ADCS_APP_Data.Guidance.nadir_N, tmp);
        if (sqrt(tmp[0] * tmp[0] + tmp[1] * tmp[1] + tmp[2] * tmp[2]) <= 1.0e-12)
        {
            ref[0] = 0.0;
            ref[1] = 1.0;
            ref[2] = 0.0;
            ADCS_APP_Cross3(ref, ADCS_APP_Data.Guidance.nadir_N, tmp);
        }
        ADCS_APP_Normalize3(tmp, x_R_N);
    }

    ADCS_APP_Cross3(ADCS_APP_Data.Guidance.nadir_N, x_R_N, y_R_N);
    ADCS_APP_Normalize3(y_R_N, y_R_N);

    C_RN[0][0] = x_R_N[0];
    C_RN[0][1] = x_R_N[1];
    C_RN[0][2] = x_R_N[2];
    C_RN[1][0] = y_R_N[0];
    C_RN[1][1] = y_R_N[1];
    C_RN[1][2] = y_R_N[2];
    C_RN[2][0] = ADCS_APP_Data.Guidance.nadir_N[0];
    C_RN[2][1] = ADCS_APP_Data.Guidance.nadir_N[1];
    C_RN[2][2] = ADCS_APP_Data.Guidance.nadir_N[2];

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            C_BR[i][j] = 0.0;
            for (int k = 0; k < 3; ++k)
            {
                C_BR[i][j] += C_BN[i][k] * C_RN[j][k];
            }
        }
    }

    ADCS_APP_DcmToMrp(C_BR, ADCS_APP_Data.Guidance.sigma_BR);

    h_N[0] = r[1] * v[2] - r[2] * v[1];
    h_N[1] = r[2] * v[0] - r[0] * v[2];
    h_N[2] = r[0] * v[1] - r[1] * v[0];

    double hmag = sqrt(h_N[0] * h_N[0] + h_N[1] * h_N[1] + h_N[2] * h_N[2]);
    if (hmag > 1.0e-12)
    {
        omega_RN_N[0] = h_N[0] / (rmag * rmag);
        omega_RN_N[1] = h_N[1] / (rmag * rmag);
        omega_RN_N[2] = h_N[2] / (rmag * rmag);
    }
    else
    {
        omega_RN_N[0] = 0.0;
        omega_RN_N[1] = 0.0;
        omega_RN_N[2] = 0.0;
    }

    omega_RN_B[0] = C_BN[0][0] * omega_RN_N[0] + C_BN[0][1] * omega_RN_N[1] + C_BN[0][2] * omega_RN_N[2];
    omega_RN_B[1] = C_BN[1][0] * omega_RN_N[0] + C_BN[1][1] * omega_RN_N[1] + C_BN[1][2] * omega_RN_N[2];
    omega_RN_B[2] = C_BN[2][0] * omega_RN_N[0] + C_BN[2][1] * omega_RN_N[1] + C_BN[2][2] * omega_RN_N[2];

    ADCS_APP_Data.Guidance.omega_BR_B[0] = ADCS_APP_Data.LatestNav.omega_BN_B[0] - omega_RN_B[0];
    ADCS_APP_Data.Guidance.omega_BR_B[1] = ADCS_APP_Data.LatestNav.omega_BN_B[1] - omega_RN_B[1];
    ADCS_APP_Data.Guidance.omega_BR_B[2] = ADCS_APP_Data.LatestNav.omega_BN_B[2] - omega_RN_B[2];

    ADCS_APP_Data.Guidance.valid = true;

    if ((EventCounter++ % 10U) == 0U)
    {
        CFE_EVS_SendEvent(ADCS_APP_NAV_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "ADCS NADIR: sat=%llu errorDeg=%.3f sigma_BR=[%.6f %.6f %.6f] omega_BR_B=[%.6f %.6f %.6f] nadir_N=[%.6f %.6f %.6f] boresight_N=[%.6f %.6f %.6f]",
                          (unsigned long long)ADCS_APP_Data.LatestNav.SatId,
                          ADCS_APP_Data.Guidance.pointingErrorDeg,
                          ADCS_APP_Data.Guidance.sigma_BR[0], ADCS_APP_Data.Guidance.sigma_BR[1], ADCS_APP_Data.Guidance.sigma_BR[2],
                          ADCS_APP_Data.Guidance.omega_BR_B[0], ADCS_APP_Data.Guidance.omega_BR_B[1], ADCS_APP_Data.Guidance.omega_BR_B[2],
                          ADCS_APP_Data.Guidance.nadir_N[0], ADCS_APP_Data.Guidance.nadir_N[1], ADCS_APP_Data.Guidance.nadir_N[2],
                          ADCS_APP_Data.Guidance.boresight_N[0], ADCS_APP_Data.Guidance.boresight_N[1], ADCS_APP_Data.Guidance.boresight_N[2]);
    }
}

static void ADCS_APP_ComputeNadirPointingError(void)
{
    ADCS_APP_ComputeNadirGuidance();
}

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

        ADCS_APP_ComputeNadirPointingError();

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
