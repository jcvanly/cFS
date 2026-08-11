###########################################################
#
# NAV_INTERFACE_APP mission build setup
#
# This file is evaluated as part of the "prepare" stage
# and can be used to set up prerequisites for the build,
# such as generating header files
#
###########################################################

# The list of header files that control the NAV_INTERFACE_APP configuration
set(NAV_INTERFACE_APP_MISSION_CONFIG_FILE_LIST
  nav_interface_app_fcncode_values.h
  nav_interface_app_interface_cfg_values.h
  nav_interface_app_mission_cfg.h
  nav_interface_app_perfids.h
  nav_interface_app_msg.h
  nav_interface_app_msgdefs.h
  nav_interface_app_msgstruct.h
  nav_interface_app_tbl.h
  nav_interface_app_tbldefs.h
  nav_interface_app_tblstruct.h
  nav_interface_app_topicid_values.h
)

generate_configfile_set(${NAV_INTERFACE_APP_MISSION_CONFIG_FILE_LIST})

