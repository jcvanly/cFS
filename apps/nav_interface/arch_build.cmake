###########################################################
#
# NAV_INTERFACE_APP platform build setup
#
# This file is evaluated as part of the "prepare" stage
# and can be used to set up prerequisites for the build,
# such as generating header files
#
###########################################################

# The list of header files that control the NAV_INTERFACE_APP configuration
set(NAV_INTERFACE_APP_PLATFORM_CONFIG_FILE_LIST
  nav_interface_app_internal_cfg_values.h
  nav_interface_app_platform_cfg.h
  nav_interface_app_perfids.h
  nav_interface_app_msgids.h
  nav_interface_app_msgid_values.h
)

generate_configfile_set(${NAV_INTERFACE_APP_PLATFORM_CONFIG_FILE_LIST})

