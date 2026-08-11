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
 *   Specification for the NAV_INTERFACE_APP command function codes
 *
 * @note
 *   This file should be strictly limited to the command/function code (CC)
 *   macro definitions.  Other definitions such as enums, typedefs, or other
 *   macros should be placed in the msgdefs.h or msg.h files.
 */
#ifndef NAV_INTERFACE_APP_FCNCODES_H
#define NAV_INTERFACE_APP_FCNCODES_H

#include "nav_interface_app_fcncode_values.h"

/************************************************************************
 * Macro Definitions
 ************************************************************************/

/*
** Nav_interface App command codes
*/
#define NAV_INTERFACE_APP_NOOP_CC           NAV_INTERFACE_APP_CCVAL(NOOP)
#define NAV_INTERFACE_APP_RESET_COUNTERS_CC NAV_INTERFACE_APP_CCVAL(RESET_COUNTERS)
#define NAV_INTERFACE_APP_PROCESS_CC        NAV_INTERFACE_APP_CCVAL(PROCESS)
#define NAV_INTERFACE_APP_DISPLAY_PARAM_CC  NAV_INTERFACE_APP_CCVAL(DISPLAY_PARAM)
#define NAV_INTERFACE_APP_FORCE_REQ_CC      NAV_INTERFACE_APP_CCVAL(FORCE_REQ)
#define NAV_INTERFACE_APP_TORQUE_REQ_CC     NAV_INTERFACE_APP_CCVAL(TORQUE_REQ)

#endif
