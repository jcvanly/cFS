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
 *   This file contains the source code for the Gnc App.
 */

/*
** Include Files:
*/
#include "gnc_app.h"
#include "gnc_app_cmds.h"
#include "gnc_app_utils.h"
#include "gnc_app_eventids.h"
#include "gnc_app_dispatch.h"
#include "gnc_app_tbl.h"
#include "gnc_app_version.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
/*
** global data
*/
GNC_APP_Data_t GNC_APP_Data;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/*                                                                            */
/* Application entry point and main process loop                              */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void GNC_APP_Main(void)
{
    CFE_Status_t     status;
    CFE_SB_Buffer_t *SBBufPtr;

    /*
    ** Create the first Performance Log entry
    */
    CFE_ES_PerfLogEntry(GNC_APP_PERF_ID);

    /*
    ** Perform application-specific initialization
    ** If the Initialization fails, set the RunStatus to
    ** CFE_ES_RunStatus_APP_ERROR and the App will not enter the RunLoop
    */
    status = GNC_APP_Init();
    if (status != CFE_SUCCESS)
    {
        GNC_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    /*
    ** Gnc App Runloop
    */
    while (CFE_ES_RunLoop(&GNC_APP_Data.RunStatus) == true)
    {
        CFE_ES_PerfLogExit(GNC_APP_PERF_ID);

        status = CFE_SB_ReceiveBuffer(&SBBufPtr,
                                    GNC_APP_Data.CommandPipe,
                                    CFE_SB_POLL);

        CFE_ES_PerfLogEntry(GNC_APP_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            GNC_APP_TaskPipe(SBBufPtr);
        }
        else if (status != CFE_SB_NO_MESSAGE)
        {
            CFE_EVS_SendEvent(GNC_APP_PIPE_ERR_EID,
                            CFE_EVS_EventType_ERROR,
                            "GNC APP: SB Pipe Read Error, App Will Exit");

            GNC_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }

        GNC_APP_ReadUdpNav();

        OS_TaskDelay(10);
    }

    /*
    ** Performance Log Exit Stamp
    */
    CFE_ES_PerfLogExit(GNC_APP_PERF_ID);

    CFE_ES_ExitApp(GNC_APP_Data.RunStatus);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* Initialization                                                             */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
CFE_Status_t GNC_APP_Init(void)
{
    CFE_Status_t status;
    char         VersionString[GNC_APP_CFG_MAX_VERSION_STR_LEN];

    /* Zero out the global data structure */
    memset(&GNC_APP_Data, 0, sizeof(GNC_APP_Data));

    GNC_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /*
    ** Register the events
    */
    status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Gnc App: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
    }
    else
    {
        /*
         ** Initialize housekeeping packet (clear user data area).
         */
        CFE_MSG_Init(CFE_MSG_PTR(GNC_APP_Data.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(GNC_APP_HK_TLM_MID),
                     sizeof(GNC_APP_Data.HkTlm));

        /*
         ** Create Software Bus message pipe.
         */
        status = CFE_SB_CreatePipe(&GNC_APP_Data.CommandPipe, GNC_APP_PLATFORM_PIPE_DEPTH,
                                   GNC_APP_PLATFORM_PIPE_NAME);
        if (status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(GNC_APP_CR_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Gnc App: Error creating SB Command Pipe, RC = 0x%08lX", (unsigned long)status);
        }
    }

    if (status == CFE_SUCCESS)
    {
        /*
        ** Subscribe to Housekeeping request commands
        */
        status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GNC_APP_SEND_HK_MID), GNC_APP_Data.CommandPipe);
        if (status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(GNC_APP_SUB_HK_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Gnc App: Error Subscribing to HK request, RC = 0x%08lX", (unsigned long)status);
        }
    }

    if (status == CFE_SUCCESS)
    {
        /*
        ** Subscribe to ground command packets
        */
        status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(GNC_APP_CMD_MID), GNC_APP_Data.CommandPipe);
        if (status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(GNC_APP_SUB_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Gnc App: Error Subscribing to Commands, RC = 0x%08lX", (unsigned long)status);
        }
    }

    if (status == CFE_SUCCESS)
    {
        /*
        ** Register Example Table(s)
        */
        status = CFE_TBL_Register(&GNC_APP_Data.TblHandles[0], "ExampleTable", sizeof(GNC_APP_ExampleTable_t),
                                  CFE_TBL_OPT_DEFAULT, GNC_APP_TblValidationFunc);
        if (status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(GNC_APP_TABLE_REG_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Gnc App: Error Registering Example Table, RC = 0x%08lX", (unsigned long)status);
        }
        else
        {
            status = CFE_TBL_Load(GNC_APP_Data.TblHandles[0], CFE_TBL_SRC_FILE, GNC_APP_PLATFORM_TABLE_FILE);
        }

        CFE_Config_GetVersionString(VersionString, GNC_APP_CFG_MAX_VERSION_STR_LEN, "Gnc App", GNC_APP_VERSION,
                                    GNC_APP_BUILD_CODENAME, GNC_APP_LAST_OFFICIAL);

        CFE_EVS_SendEvent(GNC_APP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION, "Gnc App Initialized.%s",
                          VersionString);
    }

    if (status == CFE_SUCCESS)
    {
        GNC_APP_Data.UdpSocket = socket(AF_INET, SOCK_DGRAM, 0);
        if (GNC_APP_Data.UdpSocket < 0)
        {
            CFE_EVS_SendEvent(GNC_APP_INIT_INF_EID,
                            CFE_EVS_EventType_ERROR,
                            "GNC APP: Failed to create UDP socket");
            status = CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
        }
    }

    if (status == CFE_SUCCESS)
    {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));

        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(5001);

        if (bind(GNC_APP_Data.UdpSocket,
                (struct sockaddr *)&addr,
                sizeof(addr)) < 0)
        {
            CFE_EVS_SendEvent(GNC_APP_INIT_INF_EID,
                            CFE_EVS_EventType_ERROR,
                            "GNC APP: Failed to bind UDP socket on port 5001");

            close(GNC_APP_Data.UdpSocket);
            GNC_APP_Data.UdpSocket = -1;
            status = CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
        }
    }


    if (status == CFE_SUCCESS)
    {
        int flags = fcntl(GNC_APP_Data.UdpSocket, F_GETFL, 0);
        fcntl(GNC_APP_Data.UdpSocket, F_SETFL, flags | O_NONBLOCK);

        CFE_EVS_SendEvent(GNC_APP_INIT_INF_EID,
                        CFE_EVS_EventType_INFORMATION,
                        "GNC APP: UDP listener initialized on port 5001");
    }

    memset(&GNC_APP_Data.BasiliskAddr, 0, sizeof(GNC_APP_Data.BasiliskAddr));

    GNC_APP_Data.BasiliskAddr.sin_family = AF_INET;
    GNC_APP_Data.BasiliskAddr.sin_port = htons(5101); // this needs to be edited per satellite

    /* CHANGE THIS to the IP of the computer running Basilisk */
    if (inet_pton(AF_INET, "192.168.1.100", &GNC_APP_Data.BasiliskAddr.sin_addr) != 1)
    {
        CFE_EVS_SendEvent(GNC_APP_INIT_INF_EID,
                        CFE_EVS_EventType_ERROR,
                        "GNC APP: Invalid Basilisk IP address");
        status = CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }
    else
    {
        CFE_EVS_SendEvent(GNC_APP_INIT_INF_EID,
                        CFE_EVS_EventType_INFORMATION,
                        "GNC APP: Basilisk command output set to port 5101");
    }

    return status;
}

void GNC_APP_ReadUdpNav(void)
{
    GNC_APP_BskNavPacket_t pkt;

    ssize_t bytes = recvfrom(GNC_APP_Data.UdpSocket,
                             &pkt,
                             sizeof(pkt),
                             0,
                             NULL,
                             NULL);

    if (bytes == sizeof(pkt))
    {
        GNC_APP_Data.LatestNav = pkt;
        GNC_APP_Data.UdpPacketsReceived++;

        GNC_APP_SendBskCmd();

        CFE_EVS_SendEvent(GNC_APP_INIT_INF_EID,
                    CFE_EVS_EventType_INFORMATION,
                    "GNC APP: SAT %llu UDP t=%llu r=(%.3f %.3f %.3f) v=(%.3f %.3f %.3f)",
                    (unsigned long long)pkt.SatId,
                    (unsigned long long)pkt.TimeNanos,
                    pkt.r_BN_N[0],
                    pkt.r_BN_N[1],
                    pkt.r_BN_N[2],
                    pkt.v_BN_N[0],
                    pkt.v_BN_N[1],
                    pkt.v_BN_N[2]);
    }
    else if (bytes > 0)
    {
        GNC_APP_Data.UdpShortPackets++;
    }
}

void GNC_APP_SendBskCmd(void)
{
    GNC_APP_BskCmdPacket_t cmd;
    ssize_t bytes;

    memset(&cmd, 0, sizeof(cmd));

    cmd.TimeNanos = GNC_APP_Data.LatestNav.TimeNanos;
    cmd.SatId     = GNC_APP_Data.LatestNav.SatId;

    /* Test force first */
    cmd.force_N[0] = 0.001;
    cmd.force_N[1] = 0.0;
    cmd.force_N[2] = 0.0;

    cmd.torque_B[0] = 0.0;
    cmd.torque_B[1] = 0.0;
    cmd.torque_B[2] = 0.0;

    bytes = sendto(GNC_APP_Data.UdpSocket,
                   &cmd,
                   sizeof(cmd),
                   0,
                   (struct sockaddr *)&GNC_APP_Data.BasiliskAddr,
                   sizeof(GNC_APP_Data.BasiliskAddr));

    if (bytes == sizeof(cmd))
    {
        GNC_APP_Data.UdpCmdPacketsSent++;

        // CFE_EVS_SendEvent(GNC_APP_INIT_INF_EID,
        //                   CFE_EVS_EventType_INFORMATION,
        //                   "GNC APP: Sent Basilisk cmd #%lu force=(%.6f %.6f %.6f)",
        //                   (unsigned long)GNC_APP_Data.UdpCmdPacketsSent,
        //                   cmd.force_N[0],
        //                   cmd.force_N[1],
        //                   cmd.force_N[2]);
    }
    else
    {
        CFE_EVS_SendEvent(GNC_APP_INIT_INF_EID,
                          CFE_EVS_EventType_ERROR,
                          "GNC APP: Failed to send Basilisk cmd, errno=%d",
                          errno);
    }
}