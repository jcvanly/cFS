project(CFE_ADCS_APP C)

set(APP_SRC_FILES
  fsw/src/adcs_app.c
  fsw/src/adcs_app_cmds.c
  fsw/src/adcs_app_utils.c
)

if (CFE_EDS_ENABLED)
  list(APPEND APP_SRC_FILES
    fsw/src/adcs_app_eds_dispatch.c
  )
else()
  list(APPEND APP_SRC_FILES
    fsw/src/adcs_app_dispatch.c
  )
endif()

# Create the app module
add_cfe_app(adcs_app ${APP_SRC_FILES})

target_link_libraries(adcs_app m)
target_include_directories(adcs_app PUBLIC fsw/inc)
target_include_directories(adcs_app PRIVATE ../nav_interface/fsw/inc)

# Add table
add_cfe_tables(adcs_app fsw/tables/adcs_app_tbl.c)

# If UT is enabled, then add the tests from the subdirectory
# Note that this is an app, and therefore does not provide
# stub functions, as other entities would not typically make
# direct function calls into this application.
if (ENABLE_UNIT_TESTS)
  add_subdirectory(unit-test)
endif (ENABLE_UNIT_TESTS)
