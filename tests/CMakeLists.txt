# ##############################################################################
# A Logan Thomas Barnes project
# ##############################################################################
file(
  GLOB_RECURSE
  LtbFlew_TEST_SOURCE_FILES
  LIST_DIRECTORIES
  false
  CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_LIST_DIR}/*.cpp
)

add_executable(
  flew-tests
  ${LtbFlew_TEST_SOURCE_FILES}
)
target_link_libraries(
  flew-tests
  PRIVATE
  LtbFlew::LtbFlew
  GTest::gtest_main
)

gtest_discover_tests( flew-tests )
