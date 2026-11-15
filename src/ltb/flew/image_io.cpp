// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#include "ltb/flew/image_io.hpp"

// external
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

namespace ltb::flew
{

auto write_png(
    std::filesystem::path const&  file,
    glm::uvec2 const              size,
    std::span< uint8 const > const rgba
) -> utils::Result< void >
{
    constexpr auto channels = 4U;

    LTB_CHECK_VALID( rgba.size( ) == ( static_cast< size_t >( size.x ) * size.y * channels ) );

    auto const result = stbi_write_png(
        file.string( ).c_str( ),
        static_cast< int >( size.x ),
        static_cast< int >( size.y ),
        static_cast< int >( channels ),
        rgba.data( ),
        static_cast< int >( size.x * channels )
    );
    LTB_CHECK_VALID( 0 != result );

    return utils::success( );
}

} // namespace ltb::flew
