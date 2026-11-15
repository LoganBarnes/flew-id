# ##############################################################################
# A Logan Thomas Barnes project
# ##############################################################################
file(
  GLOB_RECURSE LtbFlew_SHADERS
  LIST_DIRECTORIES false
  CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_LIST_DIR}/*.vert
  ${CMAKE_CURRENT_LIST_DIR}/*.frag
  ${CMAKE_CURRENT_LIST_DIR}/*.comp
)
file(
  GLOB_RECURSE LtbFlew_SHADER_INCLUDES
  LIST_DIRECTORIES false
  CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_LIST_DIR}/*.glsl
)

file( MAKE_DIRECTORY ${LTB_FLEW_SPIRV_DIR} )

# Generate the SPIR-V file for each shader (in the build directory)
set( LtbFlew_SPIRV_FILES )
foreach( SHADER_FILE ${LtbFlew_SHADERS} )
  get_filename_component( SHADER_NAME ${SHADER_FILE} NAME )
  set( SPIRV_FILE ${LTB_FLEW_SPIRV_DIR}/${SHADER_NAME}.spv )

  add_custom_command(
    OUTPUT ${SPIRV_FILE}
    COMMAND $<TARGET_FILE:Vulkan::glslc>
    ARGS -I ${CMAKE_CURRENT_LIST_DIR} ${SHADER_FILE} -o ${SPIRV_FILE}
    DEPENDS ${SHADER_FILE} ${LtbFlew_SHADER_INCLUDES}
    COMMENT "Compiling ${SHADER_NAME}"
    VERBATIM
  )

  list( APPEND LtbFlew_SPIRV_FILES ${SPIRV_FILE} )
endforeach()

add_custom_target(
  ltb_flew_generate_spirv
  DEPENDS ${LtbFlew_SPIRV_FILES}
)
