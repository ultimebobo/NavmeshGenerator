includes("lib/nifly")

-- set project constants
set_project("navmesh-generator")
set_version("0.1.0")
set_license("GPL-3.0-or-later")
set_languages("c++23")
set_warnings("allextra")
-- Pin the static package used by both parser compression and fixture builders.
-- This keeps the test target independent of an ambient SDK zlib installation.
add_requires("zlib 1.3.2", { configs = { shared = false } })

-- add common rules
add_rules("mode.debug", "mode.releasedbg")
add_rules("plugin.vsxmake.autoupdate")

target("recast")
    set_kind("static")
    set_default(false)
    add_files("lib/recastnavigation/Recast/Source/*.cpp")
    add_includedirs("lib/recastnavigation/Recast/Include", {public = true})

target("navmesh-core")
    set_kind("static")
    set_default(false)
    add_deps("recast")
    add_files("src/core/**.cpp", "src/validation/**.cpp", "src/analysis/**.cpp")
    add_headerfiles("src/core/**.h", "src/validation/**.h", "src/analysis/**.h")
    add_includedirs("src", {public = true})

-- Reuse the bundled Dear ImGui core; backends are pinned to its matching release.
target("navmesh-ui-framework")
    set_kind("static")
    set_default(false)
    add_files("lib/recastnavigation/RecastDemo/Contrib/imgui/imgui.cpp",
              "lib/recastnavigation/RecastDemo/Contrib/imgui/imgui_draw.cpp",
              "lib/recastnavigation/RecastDemo/Contrib/imgui/imgui_tables.cpp",
              "lib/recastnavigation/RecastDemo/Contrib/imgui/imgui_widgets.cpp",
              "lib/recastnavigation/RecastDemo/Contrib/imgui/misc/cpp/imgui_stdlib.cpp",
              "third_party/imgui_backends/*.cpp")
    add_includedirs("lib/recastnavigation/RecastDemo/Contrib/imgui", "third_party/imgui_backends", {public = true})
    add_syslinks("d3d11", "dxgi", "d3dcompiler", "dwmapi", "imm32", {public = true})

target("navmesh-offline")
    set_kind("binary")
    set_default(true)
    add_deps("navmesh-core")
    add_deps("nifly")
    add_deps("navmesh-ui-framework")
    add_files("src/app/**.cpp", "src/ui/**.cpp", "src/cli/**.cpp", "src/skyrim/parser/**.cpp", "src/skyrim/mo2/**.cpp", "src/skyrim/extraction/asset_cache.cpp", "src/skyrim/extraction/geometry_extractor.cpp", "src/skyrim/extraction/terrain_extractor.cpp")
    add_headerfiles("src/app/**.h", "src/cli/**.h", "src/skyrim/parser/**.h")
    add_includedirs("src", "lib/nifly/external")
    add_packages("zlib")
    add_syslinks("user32", "gdi32", "shell32", "ole32", "uuid")

target("navmesh-tests")
    set_kind("binary")
    set_default(false)
    add_deps("navmesh-core")
    add_deps("nifly")
    add_files("tests/**.cpp", "src/app/candidate_cache.cpp", "src/app/candidate_artifacts.cpp", "src/app/geometry_pipeline.cpp", "src/skyrim/extraction/asset_cache.cpp", "src/skyrim/extraction/geometry_extractor.cpp", "src/skyrim/extraction/terrain_extractor.cpp", "src/skyrim/parser/**.cpp", "src/skyrim/mo2/**.cpp")
    add_includedirs("src", "lib/nifly/external")
    add_packages("zlib")
    add_files("src/ui/options_model.cpp")
