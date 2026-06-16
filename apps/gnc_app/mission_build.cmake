###########################################################
#
# GNC_APP mission build setup
#
# This file is evaluated as part of the "prepare" stage
# and can be used to set up prerequisites for the build,
# such as generating header files
#
###########################################################

# The list of header files that control the GNC_APP configuration
set(GNC_APP_MISSION_CONFIG_FILE_LIST
  gnc_app_fcncode_values.h
  gnc_app_interface_cfg_values.h
  gnc_app_mission_cfg.h
  gnc_app_perfids.h
  gnc_app_msg.h
  gnc_app_msgdefs.h
  gnc_app_msgstruct.h
  gnc_app_tbl.h
  gnc_app_tbldefs.h
  gnc_app_tblstruct.h
  gnc_app_topicid_values.h
)

generate_configfile_set(${GNC_APP_MISSION_CONFIG_FILE_LIST})

