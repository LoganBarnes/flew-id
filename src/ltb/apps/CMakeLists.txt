# ##############################################################################
# A Logan Thomas Barnes project
# ##############################################################################
function( ltb_make_app directory executable )
  file(
    GLOB_RECURSE
    ${directory}_app_SOURCE
    LIST_DIRECTORIES
    false
    CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/${directory}/*.cpp
  )

  add_executable(
    ${executable}
    ${${directory}_app_SOURCE}
  )
  target_link_libraries(
    ${executable}
    PRIVATE
    LtbFlew::LtbFlew
  )
endfunction()

ltb_make_app( flew_id flew-id )
ltb_make_app( flew_check flew-check )
