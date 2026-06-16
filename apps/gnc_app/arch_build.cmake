###########################################################
#
# GNC_APP platform build setup
#
# This file is evaluated as part of the "prepare" stage
# and can be used to set up prerequisites for the build,
# such as generating header files
#
###########################################################

# The list of header files that control the GNC_APP configuration
set(GNC_APP_PLATFORM_CONFIG_FILE_LIST
  gnc_app_internal_cfg_values.h
  gnc_app_platform_cfg.h
  gnc_app_perfids.h
  gnc_app_msgids.h
  gnc_app_msgid_values.h
)

generate_configfile_set(${GNC_APP_PLATFORM_CONFIG_FILE_LIST})

