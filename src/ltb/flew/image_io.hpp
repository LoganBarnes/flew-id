// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#pragma once

// project
#include "ltb/utils/result.hpp"
#include "ltb/utils/types.hpp"

// external
#include <glm/glm.hpp>

// standard
#include <filesystem>
#include <span>

namespace ltb::flew
{

/// \brief Writes tightly packed 8-bit RGBA pixels (top row first) to a PNG file.
auto write_png( std::filesystem::path const& file, glm::uvec2 size, std::span< uint8 const > rgba )
    -> utils::Result< void >;

} // namespace ltb::flew
