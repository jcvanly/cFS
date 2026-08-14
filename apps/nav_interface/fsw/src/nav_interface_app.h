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
 * @file
 *
 * Main header file for the Nav_interface application
 */

#ifndef NAV_INTERFACE_APP_H
#define NAV_INTERFACE_APP_H

/*
** Required header files.
*/
#include "cfe.h"
#include "cfe_config.h"

#include "nav_interface_app_mission_cfg.h"
#include "nav_interface_app_platform_cfg.h"

#include "nav_interface_app_perfids.h"
#include "nav_interface_app_msgids.h"
#include "nav_interface_app_msg.h"

#include <netinet/in.h>

/************************************************************************
** Type Definitions
*************************************************************************/

/*
** Global Data
*/
typedef struct
{
    uint64 TimeNanos;
    uint64 SatId;
    double r_BN_N[3];
    double v_BN_N[3];
    double sigma_BN[3];
    double omega_BN_B[3];
} NAV_INTERFACE_APP_BskNavPacket_t;

typedef struct
{
    uint64 TimeNanos;
    uint64 SatId;
    double force_N[3];
    double torque_B[3];
} NAV_INTERFACE_APP_BskCmdPacket_t;

typedef struct
{
    /*
    ** Command interface counters...
    */
    uint8 CmdCounter;
    uint8 ErrCounter;

    /*
    ** Housekeeping telemetry packet...
    */
    NAV_INTERFACE_APP_HkTlm_t HkTlm;

    /*
    ** Run Status variable used in the main processing loop
    */
    uint32 RunStatus;

    /*
    ** Operational data (not reported in housekeeping)...
    */
    CFE_SB_PipeId_t CommandPipe;

    CFE_TBL_Handle_t TblHandles[NAV_INTERFACE_APP_PLATFORM_NUMBER_OF_TABLES];

    int UdpSocket;
    uint32 UdpPacketsReceived;
    uint32 UdpShortPackets;
    uint32 UdpCmdPacketsSent;

    NAV_INTERFACE_APP_BskNavPacket_t LatestNav;
    NAV_INTERFACE_APP_BskCmdPacket_t LatestCmd;
    double                            LatestForce_N[3];
    double                            LatestTorque_B[3];

    struct sockaddr_in BasiliskAddr;
} NAV_INTERFACE_APP_Data_t;

/*
** Global data structure
*/
extern NAV_INTERFACE_APP_Data_t NAV_INTERFACE_APP_Data;

/****************************************************************************/
/*
** Local function prototypes.
**
** Note: Except for the entry point (NAV_INTERFACE_APP_Main), these
**       functions are not called from any other source module.
*/
void         NAV_INTERFACE_APP_Main(void);
void         NAV_INTERFACE_Main(void);
CFE_Status_t NAV_INTERFACE_APP_Init(void);
void         NAV_INTERFACE_APP_ReadUdpNav(void);

#endif /* NAV_INTERFACE_APP_H */
