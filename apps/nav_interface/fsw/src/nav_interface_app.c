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
 *   This file contains the source code for the Nav_interface App.
 */

/*
** Include Files:
*/
#include "nav_interface_app.h"
#include "nav_interface_app_cmds.h"
#include "nav_interface_app_utils.h"
#include "nav_interface_app_eventids.h"
#include "nav_interface_app_dispatch.h"
#include "nav_interface_app_tbl.h"
#include "nav_interface_app_version.h"

#include "adcs_app_fcncode_values.h"
#include "adcs_app_msgids.h"
#include "gnc_app_fcncode_values.h"
#include "gnc_app_msgids.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

typedef struct
{
    CFE_MSG_CommandHeader_t CommandHeader;
    struct
    {
        uint64 TimeNanos;
        uint64 SatId;
        double PosX_N;
        double PosY_N;
        double PosZ_N;
        double VelX_N;
        double VelY_N;
        double VelZ_N;
    } Payload;
} NAV_INTERFACE_APP_GncIngestNavCmd_t;

typedef struct
{
    CFE_MSG_CommandHeader_t CommandHeader;
    struct
    {
        uint64 TimeNanos;
        uint64 SatId;
        double PosX_N;
        double PosY_N;
        double PosZ_N;
        double VelX_N;
        double VelY_N;
        double VelZ_N;
    } Payload;
} NAV_INTERFACE_APP_AdcsIngestNavCmd_t;

static CFE_Status_t NAV_INTERFACE_APP_ForwardNavToGnc(const NAV_INTERFACE_APP_BskNavPacket_t *Nav)
{
    NAV_INTERFACE_APP_GncIngestNavCmd_t Cmd;

    CFE_MSG_Init(CFE_MSG_PTR(Cmd.CommandHeader), CFE_SB_ValueToMsgId(GNC_APP_CMD_MID), sizeof(Cmd));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(Cmd.CommandHeader), GNC_APP_FunctionCode_INGEST_NAV);

    Cmd.Payload.TimeNanos = Nav->TimeNanos;
    Cmd.Payload.SatId     = Nav->SatId;
    Cmd.Payload.PosX_N    = Nav->r_BN_N[0];
    Cmd.Payload.PosY_N    = Nav->r_BN_N[1];
    Cmd.Payload.PosZ_N    = Nav->r_BN_N[2];
    Cmd.Payload.VelX_N    = Nav->v_BN_N[0];
    Cmd.Payload.VelY_N    = Nav->v_BN_N[1];
    Cmd.Payload.VelZ_N    = Nav->v_BN_N[2];

    return CFE_SB_TransmitMsg(CFE_MSG_PTR(Cmd.CommandHeader), true);
}

static CFE_Status_t NAV_INTERFACE_APP_ForwardNavToAdcs(const NAV_INTERFACE_APP_BskNavPacket_t *Nav)
{
    NAV_INTERFACE_APP_AdcsIngestNavCmd_t Cmd;

    CFE_MSG_Init(CFE_MSG_PTR(Cmd.CommandHeader), CFE_SB_ValueToMsgId(ADCS_APP_CMD_MID), sizeof(Cmd));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(Cmd.CommandHeader), ADCS_APP_FunctionCode_INGEST_NAV);

    Cmd.Payload.TimeNanos = Nav->TimeNanos;
    Cmd.Payload.SatId     = Nav->SatId;
    Cmd.Payload.PosX_N    = Nav->r_BN_N[0];
    Cmd.Payload.PosY_N    = Nav->r_BN_N[1];
    Cmd.Payload.PosZ_N    = Nav->r_BN_N[2];
    Cmd.Payload.VelX_N    = Nav->v_BN_N[0];
    Cmd.Payload.VelY_N    = Nav->v_BN_N[1];
    Cmd.Payload.VelZ_N    = Nav->v_BN_N[2];

    return CFE_SB_TransmitMsg(CFE_MSG_PTR(Cmd.CommandHeader), true);
}

/*
** global data
*/
NAV_INTERFACE_APP_Data_t NAV_INTERFACE_APP_Data;

void NAV_INTERFACE_APP_Main(void)
{
    NAV_INTERFACE_Main();
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/*                                                                            */
/* Application entry point and main process loop                              */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void NAV_INTERFACE_Main(void)
{
    CFE_Status_t     status;
    CFE_SB_Buffer_t *SBBufPtr;

    /*
    ** Create the first Performance Log entry
    */
    CFE_ES_PerfLogEntry(NAV_INTERFACE_APP_PERF_ID);

    /*
    ** Perform application-specific initialization
    ** If the Initialization fails, set the RunStatus to
    ** CFE_ES_RunStatus_APP_ERROR and the App will not enter the RunLoop
    */
    status = NAV_INTERFACE_APP_Init();
    if (status != CFE_SUCCESS)
    {
        NAV_INTERFACE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    /*
    ** Nav_interface App Runloop
    */
    while (CFE_ES_RunLoop(&NAV_INTERFACE_APP_Data.RunStatus) == true)
    {
        /*
        ** Performance Log Exit Stamp
        */
        CFE_ES_PerfLogExit(NAV_INTERFACE_APP_PERF_ID);

        status = CFE_SB_ReceiveBuffer(&SBBufPtr, NAV_INTERFACE_APP_Data.CommandPipe, CFE_SB_POLL);

        /*
        ** Performance Log Entry Stamp
        */
        CFE_ES_PerfLogEntry(NAV_INTERFACE_APP_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            NAV_INTERFACE_APP_TaskPipe(SBBufPtr);
        }
        else if (status != CFE_SB_NO_MESSAGE)
        {
            CFE_EVS_SendEvent(NAV_INTERFACE_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "NAV_INTERFACE APP: SB Pipe Read Error, App Will Exit");

            NAV_INTERFACE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }

        NAV_INTERFACE_APP_ReadUdpNav();

        OS_TaskDelay(10);
    }

    /*
    ** Performance Log Exit Stamp
    */
    CFE_ES_PerfLogExit(NAV_INTERFACE_APP_PERF_ID);

    CFE_ES_ExitApp(NAV_INTERFACE_APP_Data.RunStatus);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* Initialization                                                             */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
CFE_Status_t NAV_INTERFACE_APP_Init(void)
{
    CFE_Status_t status;
    char         VersionString[NAV_INTERFACE_APP_CFG_MAX_VERSION_STR_LEN];

    /* Zero out the global data structure */
    memset(&NAV_INTERFACE_APP_Data, 0, sizeof(NAV_INTERFACE_APP_Data));

    NAV_INTERFACE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /*
    ** Register the events
    */
    status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Nav_interface App: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
    }
    else
    {
        /*
         ** Initialize housekeeping packet (clear user data area).
         */
        CFE_MSG_Init(CFE_MSG_PTR(NAV_INTERFACE_APP_Data.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(NAV_INTERFACE_APP_HK_TLM_MID),
                     sizeof(NAV_INTERFACE_APP_Data.HkTlm));

        /*
         ** Create Software Bus message pipe.
         */
        status = CFE_SB_CreatePipe(&NAV_INTERFACE_APP_Data.CommandPipe, NAV_INTERFACE_APP_PLATFORM_PIPE_DEPTH,
                                   NAV_INTERFACE_APP_PLATFORM_PIPE_NAME);
        if (status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(NAV_INTERFACE_APP_CR_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Nav_interface App: Error creating SB Command Pipe, RC = 0x%08lX", (unsigned long)status);
        }
    }

    if (status == CFE_SUCCESS)
    {
        /*
        ** Subscribe to Housekeeping request commands
        */
        status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(NAV_INTERFACE_APP_SEND_HK_MID), NAV_INTERFACE_APP_Data.CommandPipe);
        if (status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(NAV_INTERFACE_APP_SUB_HK_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Nav_interface App: Error Subscribing to HK request, RC = 0x%08lX", (unsigned long)status);
        }
    }

    if (status == CFE_SUCCESS)
    {
        /*
        ** Subscribe to ground command packets
        */
        status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(NAV_INTERFACE_APP_CMD_MID), NAV_INTERFACE_APP_Data.CommandPipe);
        if (status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(NAV_INTERFACE_APP_SUB_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Nav_interface App: Error Subscribing to Commands, RC = 0x%08lX", (unsigned long)status);
        }
    }

    if (status == CFE_SUCCESS)
    {
        /*
        ** Register Example Table(s)
        */
        status = CFE_TBL_Register(&NAV_INTERFACE_APP_Data.TblHandles[0], "ExampleTable", sizeof(NAV_INTERFACE_APP_ExampleTable_t),
                                  CFE_TBL_OPT_DEFAULT, NAV_INTERFACE_APP_TblValidationFunc);
        if (status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(NAV_INTERFACE_APP_TABLE_REG_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Nav_interface App: Error Registering Example Table, RC = 0x%08lX", (unsigned long)status);
        }
        else
        {
            status = CFE_TBL_Load(NAV_INTERFACE_APP_Data.TblHandles[0], CFE_TBL_SRC_FILE, NAV_INTERFACE_APP_PLATFORM_TABLE_FILE);
        }

        CFE_Config_GetVersionString(VersionString, NAV_INTERFACE_APP_CFG_MAX_VERSION_STR_LEN, "Nav_interface App", NAV_INTERFACE_APP_VERSION,
                                    NAV_INTERFACE_APP_BUILD_CODENAME, NAV_INTERFACE_APP_LAST_OFFICIAL);

        CFE_EVS_SendEvent(NAV_INTERFACE_APP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION, "Nav_interface App Initialized.%s",
                          VersionString);
    }

    if (status == CFE_SUCCESS)
    {
        NAV_INTERFACE_APP_Data.UdpSocket = socket(AF_INET, SOCK_DGRAM, 0);
        if (NAV_INTERFACE_APP_Data.UdpSocket < 0)
        {
            CFE_EVS_SendEvent(NAV_INTERFACE_APP_UDP_SETUP_ERR_EID, CFE_EVS_EventType_ERROR,
                              "NAV_INTERFACE: Failed to create UDP socket");
            status = CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
        }
    }

    if (status == CFE_SUCCESS)
    {
        int reuse = 1;
        struct sockaddr_in addr;

        if (setsockopt(NAV_INTERFACE_APP_Data.UdpSocket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
        {
            CFE_EVS_SendEvent(NAV_INTERFACE_APP_UDP_SETUP_ERR_EID, CFE_EVS_EventType_ERROR,
                              "NAV_INTERFACE: Failed SO_REUSEADDR, errno=%d", errno);
        }

        memset(&addr, 0, sizeof(addr));
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port        = htons(5001);

        if (bind(NAV_INTERFACE_APP_Data.UdpSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            CFE_EVS_SendEvent(NAV_INTERFACE_APP_UDP_SETUP_ERR_EID, CFE_EVS_EventType_ERROR,
                              "NAV_INTERFACE: Failed bind on UDP port 5001, errno=%d", errno);
            close(NAV_INTERFACE_APP_Data.UdpSocket);
            NAV_INTERFACE_APP_Data.UdpSocket = -1;
            status = CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
        }
    }

    if (status == CFE_SUCCESS)
    {
        int flags = fcntl(NAV_INTERFACE_APP_Data.UdpSocket, F_GETFL, 0);
        fcntl(NAV_INTERFACE_APP_Data.UdpSocket, F_SETFL, flags | O_NONBLOCK);
        CFE_EVS_SendEvent(NAV_INTERFACE_APP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "NAV_INTERFACE: UDP listener initialized on port 5001");
    }

    memset(&NAV_INTERFACE_APP_Data.BasiliskAddr, 0, sizeof(NAV_INTERFACE_APP_Data.BasiliskAddr));
    NAV_INTERFACE_APP_Data.BasiliskAddr.sin_family = AF_INET;
    NAV_INTERFACE_APP_Data.BasiliskAddr.sin_port   = htons(5101);

    if (inet_pton(AF_INET, "192.168.1.100", &NAV_INTERFACE_APP_Data.BasiliskAddr.sin_addr) != 1)
    {
        CFE_EVS_SendEvent(NAV_INTERFACE_APP_UDP_SETUP_ERR_EID, CFE_EVS_EventType_ERROR,
                          "NAV_INTERFACE: Invalid Basilisk IP address");
        status = CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    return status;
}

void NAV_INTERFACE_APP_ReadUdpNav(void)
{
    NAV_INTERFACE_APP_BskNavPacket_t Packet;
    ssize_t                          Bytes;
    CFE_Status_t                     Status;

    Bytes = recvfrom(NAV_INTERFACE_APP_Data.UdpSocket, &Packet, sizeof(Packet), 0, NULL, NULL);
    if (Bytes == (ssize_t)sizeof(Packet))
    {
        NAV_INTERFACE_APP_Data.LatestNav = Packet;
        NAV_INTERFACE_APP_Data.UdpPacketsReceived++;

        Status = NAV_INTERFACE_APP_ForwardNavToGnc(&Packet);
        if (Status != CFE_SUCCESS)
        {
            NAV_INTERFACE_APP_Data.ErrCounter++;
            CFE_EVS_SendEvent(NAV_INTERFACE_APP_FWD_GNC_ERR_EID, CFE_EVS_EventType_ERROR,
                              "NAV_INTERFACE: Failed to forward NAV packet to GNC, RC=0x%08lX",
                              (unsigned long)Status);
        }

        Status = NAV_INTERFACE_APP_ForwardNavToAdcs(&Packet);
        if (Status != CFE_SUCCESS)
        {
            NAV_INTERFACE_APP_Data.ErrCounter++;
            CFE_EVS_SendEvent(NAV_INTERFACE_APP_FWD_ADCS_ERR_EID, CFE_EVS_EventType_ERROR,
                              "NAV_INTERFACE: Failed to forward NAV packet to ADCS, RC=0x%08lX",
                              (unsigned long)Status);
        }
    }
    else if (Bytes > 0)
    {
        NAV_INTERFACE_APP_Data.UdpShortPackets++;
        CFE_EVS_SendEvent(NAV_INTERFACE_APP_UDP_RX_ERR_EID, CFE_EVS_EventType_ERROR,
                          "NAV_INTERFACE: Short NAV UDP packet (%ld bytes)", (long)Bytes);
    }
    else if (Bytes < 0 && errno != EWOULDBLOCK && errno != EAGAIN)
    {
        CFE_EVS_SendEvent(NAV_INTERFACE_APP_UDP_RX_ERR_EID, CFE_EVS_EventType_ERROR,
                          "NAV_INTERFACE: UDP receive error, errno=%d", errno);
    }
}
