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

target("navmesh-offline")
    set_kind("binary")
    set_default(true)
    add_deps("navmesh-core")
    add_deps("nifly")
    add_files("src/app/**.cpp", "src/ui/**.cpp", "src/cli/**.cpp", "src/skyrim/parser/**.cpp", "src/skyrim/mo2/**.cpp", "src/skyrim/extraction/geometry_extractor.cpp", "src/skyrim/extraction/terrain_extractor.cpp")
    add_headerfiles("src/app/**.h", "src/cli/**.h", "src/skyrim/parser/**.h")
    add_includedirs("src", "lib/nifly/external")
    add_packages("zlib")
    add_syslinks("comctl32", "user32", "gdi32", "uxtheme")

target("navmesh-tests")
    set_kind("binary")
    set_default(false)
    add_deps("navmesh-core")
    add_deps("nifly")
    add_files("tests/**.cpp", "src/skyrim/extraction/geometry_extractor.cpp", "src/skyrim/extraction/terrain_extractor.cpp", "src/skyrim/parser/**.cpp", "src/skyrim/mo2/**.cpp")
    add_includedirs("src", "lib/nifly/external")
    add_packages("zlib")
