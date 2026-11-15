# ##############################################################################
# A Logan Thomas Barnes project
# ##############################################################################

find_package( Git REQUIRED )

# Graphics API
find_package( Vulkan REQUIRED )

# ltb-vlk is a git submodule. When it is checked out, build against the local
# copy so changes made there are picked up immediately. Otherwise (e.g. this
# project is itself being fetched as a dependency) download the pinned commit.
if( NOT DEFINED CPM_LtbVlk_SOURCE AND EXISTS ${PROJECT_SOURCE_DIR}/ltb-vlk/CMakeLists.txt )
  set( CPM_LtbVlk_SOURCE ${PROJECT_SOURCE_DIR}/ltb-vlk )
endif()

if( DEFINED CPM_LtbVlk_SOURCE )
  message( STATUS "ltb-vlk source: ${CPM_LtbVlk_SOURCE}" )
  set( LTB_VLK_COMMIT_HASH "local" )
else()
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse HEAD:ltb-vlk
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE LTB_VLK_COMMIT_HASH
    RESULT_VARIABLE GIT_RESULT
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if( GIT_RESULT EQUAL 0 )
    message( STATUS "ltb-vlk commit hash: ${LTB_VLK_COMMIT_HASH}" )
  else()
    message( FATAL_ERROR "Failed to get ltb-vlk commit hash." )
  endif()
endif()

# Image writing (header only)
cpmaddpackage(
  NAME
  stb
  GITHUB_REPOSITORY
  nothings/stb
  GIT_TAG
  f1c79c02822848a9bed4315b12c8c8f3761e1296
  DOWNLOAD_ONLY
  TRUE
)

if( stb_ADDED )
  add_library(
    stb
    INTERFACE
  )
  add_library(
    stb::stb
    ALIAS
    stb
  )
  target_include_directories(
    stb
    SYSTEM
    INTERFACE
    # Mark external include directories as system includes to avoid warnings.
    $<BUILD_INTERFACE:${stb_SOURCE_DIR}>
  )
endif()

cpmaddpackage(
  NAME
  LtbVlk
  GITHUB_REPOSITORY
  LoganBarnes/ltb-vlk
  GIT_TAG
  ${LTB_VLK_COMMIT_HASH}
)
