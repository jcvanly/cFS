project(CFE_NAV_INTERFACE_APP C)

set(APP_SRC_FILES
  fsw/src/nav_interface_app.c
  fsw/src/nav_interface_app_cmds.c
  fsw/src/nav_interface_app_utils.c
)

if (CFE_EDS_ENABLED)
  list(APPEND APP_SRC_FILES
    fsw/src/nav_interface_app_eds_dispatch.c
  )
else()
  list(APPEND APP_SRC_FILES
    fsw/src/nav_interface_app_dispatch.c
  )
endif()

# Create the app module
add_cfe_app(nav_interface ${APP_SRC_FILES})

target_include_directories(nav_interface PUBLIC fsw/inc)
target_include_directories(nav_interface PRIVATE ../gnc_app/fsw/inc ../adcs_app/fsw/inc)

# Add table
add_cfe_tables(nav_interface fsw/tables/nav_interface_app_tbl.c)

# If UT is enabled, then add the tests from the subdirectory
# Note that this is an app, and therefore does not provide
# stub functions, as other entities would not typically make
# direct function calls into this application.
if (ENABLE_UNIT_TESTS)
  add_subdirectory(unit-test)
endif (ENABLE_UNIT_TESTS)
