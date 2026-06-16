project(CFE_GNC_APP C)

set(APP_SRC_FILES
  fsw/src/gnc_app.c
  fsw/src/gnc_app_cmds.c
  fsw/src/gnc_app_utils.c
)

if (CFE_EDS_ENABLED)
  list(APPEND APP_SRC_FILES
    fsw/src/gnc_app_eds_dispatch.c
  )
else()
  list(APPEND APP_SRC_FILES
    fsw/src/gnc_app_dispatch.c
  )
endif()

# Create the app module
add_cfe_app(gnc_app ${APP_SRC_FILES})

target_include_directories(gnc_app PUBLIC fsw/inc)

# Add table
add_cfe_tables(gnc_app fsw/tables/gnc_app_tbl.c)

# If UT is enabled, then add the tests from the subdirectory
# Note that this is an app, and therefore does not provide
# stub functions, as other entities would not typically make
# direct function calls into this application.
if (ENABLE_UNIT_TESTS)
  add_subdirectory(unit-test)
endif (ENABLE_UNIT_TESTS)
