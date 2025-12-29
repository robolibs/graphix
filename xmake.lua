set_project("graphix")
set_version("0.0.0")
set_xmakever("2.7.0")

-- Set C++ standard
set_languages("c++20")

-- Add build options
add_rules("mode.debug", "mode.release")

-- Compiler warnings and flags
add_cxxflags("-Wall", "-Wextra", "-Wpedantic")
add_cxxflags("-Wno-reorder", "-Wno-narrowing", "-Wno-array-bounds")
add_cxxflags("-Wno-unused-variable", "-Wno-unused-parameter", "-Wno-stringop-overflow", "-Wno-unused-but-set-variable")

-- SIMD flags for optinum (AVX2 for modern x86_64)
add_cxxflags("-mavx2", "-mfma")

-- Add global search paths for packages in ~/.local
local home = os.getenv("HOME")
if home then
    add_includedirs(path.join(home, ".local/include"))
    add_linkdirs(path.join(home, ".local/lib"))
end

-- Add devbox/nix paths for system packages
local cmake_prefix = os.getenv("CMAKE_PREFIX_PATH")
if cmake_prefix then
    add_includedirs(path.join(cmake_prefix, "include"))
    add_linkdirs(path.join(cmake_prefix, "lib"))
end

local pkg_config = os.getenv("PKG_CONFIG_PATH")
if pkg_config then
    -- Split PKG_CONFIG_PATH by ':' and process each path
    for _, pkgconfig_path in ipairs(pkg_config:split(':')) do
        if os.isdir(pkgconfig_path) then
            -- PKG_CONFIG_PATH typically points to .../lib/pkgconfig
            -- We want to get the prefix (two levels up) to find include and lib
            local lib_dir = path.directory(pkgconfig_path)  -- .../lib
            local prefix_dir = path.directory(lib_dir)      -- .../
            local include_dir = path.join(prefix_dir, "include")

            if os.isdir(lib_dir) then
                add_linkdirs(lib_dir)
            end
            if os.isdir(include_dir) then
                add_includedirs(include_dir)
            end
        end
    end
end

-- Options
option("examples")
    set_default(false)
    set_showmenu(true)
    set_description("Build examples")
option_end()

option("tests")
    set_default(false)
    set_showmenu(true)
    set_description("Enable tests")
option_end()

option("rerun")
    set_default(false)
    set_showmenu(true)
    set_description("Enable rerun_sdk support")
option_end()

-- Define rerun_sdk package (from ~/.local installation)
package("rerun_sdk")
    set_kind("library", {headeronly = false})

    on_fetch(function (package)
        local home = os.getenv("HOME")
        if not home then
            return
        end

        local result = {}
        -- Link in correct order: rerun_sdk -> rerun_c -> arrow
        result.links = {"rerun_sdk", "rerun_c__linux_x64", "arrow", "arrow_bundled_dependencies"}
        result.linkdirs = {path.join(home, ".local/lib")}
        result.includedirs = {path.join(home, ".local/include")}

        -- Check if library exists
        local libpath = path.join(home, ".local/lib/librerun_sdk.a")
        if os.isfile(libpath) then
            return result
        end
    end)

    on_install(function (package)
        -- Already installed in ~/.local, nothing to do
        local home = os.getenv("HOME")
        package:addenv("PATH", path.join(home, ".local/bin"))
    end)
package_end()

-- Mandatory datapod dependency. Prefer local checkout at ../datapod, otherwise fetch from git.
local dp_dir = path.join(os.projectdir(), "..", "datapod")
local dp_source_dir = os.isdir(dp_dir) and dp_dir or path.join(os.projectdir(), "build/_deps/datapod-src")

package("datapod")
    set_sourcedir(dp_source_dir)

    on_fetch(function (package)
        local sourcedir = package:sourcedir()
        if sourcedir == dp_dir then
            return
        end
        if not os.isdir(sourcedir) then
            print("Fetching datapod from git...")
            os.mkdir(path.directory(sourcedir))
            os.execv("git", {"clone", "--quiet", "--depth", "1", "--branch", "0.0.15",
                            "-c", "advice.detachedHead=false",
                            "https://github.com/robolibs/datapod.git", sourcedir})
        end
    end)

    on_install(function (package)
        local configs = {}
        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:is_debug() and "Debug" or "Release"))
        import("package.tools.cmake").install(package, configs, {cmake_generator = "Unix Makefiles"})
    end)
package_end()

add_requires("datapod")

-- Mandatory optinum dependency. Prefer local checkout at ../optinum, otherwise fetch from git.
local op_dir = path.join(os.projectdir(), "..", "optinum")
local op_source_dir = os.isdir(op_dir) and op_dir or path.join(os.projectdir(), "build/_deps/optinum-src")

package("optinum")
    set_sourcedir(op_source_dir)

    on_fetch(function (package)
        local sourcedir = package:sourcedir()
        if sourcedir == op_dir then
            return
        end
        if not os.isdir(sourcedir) then
            print("Fetching optinum from git...")
            os.mkdir(path.directory(sourcedir))
            os.execv("git", {"clone", "--quiet", "--depth", "1", "--branch", "0.0.7",
                            "-c", "advice.detachedHead=false",
                            "https://github.com/robolibs/optinum.git", sourcedir})
        end
    end)

    on_install(function (package)
        local configs = {}
        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:is_debug() and "Debug" or "Release"))
        import("package.tools.cmake").install(package, configs, {cmake_generator = "Unix Makefiles"})
    end)
package_end()

add_requires("optinum")

-- Add required packages conditionally
if has_config("examples") then
    add_requires("rerun_sdk")
elseif has_config("rerun") then
    add_requires("rerun_sdk")
end

-- Main library target
target("graphix")
    set_kind("static")

    -- Add source files
    add_files("src/graphix/**.cpp")

    -- Add header files
    add_headerfiles("include/(graphix/**.hpp)")
    add_includedirs("include", {public = true})

    add_packages("datapod", {public = true})
    add_packages("optinum", {public = true})
    add_defines("SHORT_NAMESPACE", {public = true})

    -- Conditional rerun support (only if package is found)
    if has_config("examples") or has_config("rerun") then
        on_load(function (target)
            if target:pkg("rerun_sdk") then
                target:add("defines", "HAS_RERUN")
            end
        end)
        add_packages("rerun_sdk")
    end

    -- Set install files
    add_installfiles("include/(graphix/**.hpp)")
    on_install(function (target)
        local installdir = target:installdir()
        os.cp(target:targetfile(), path.join(installdir, "lib", path.filename(target:targetfile())))
    end)
target_end()

-- Examples (only build when graphix is the main project)
if has_config("examples") and os.projectdir() == os.curdir() then
    for _, filepath in ipairs(os.files("examples/*.cpp")) do
        local filename = path.basename(filepath)
        target(filename)
            set_kind("binary")
            add_files(filepath)
            add_deps("graphix")

            -- Always try to add rerun_sdk for examples (matching CMake behavior)
            on_load(function (target)
                if target:pkg("rerun_sdk") then
                    target:add("defines", "HAS_RERUN")
                end
            end)
            add_packages("rerun_sdk")

            add_includedirs("include")
        target_end()
    end
end

if has_config("tests") then
    add_requires("doctest")
end

-- Tests (only build when graphix is the main project)
if has_config("tests") and os.projectdir() == os.curdir() then
    for _, filepath in ipairs(os.files("test/**.cpp")) do
        local filename = path.basename(filepath)
        target(filename)
            set_kind("binary")
            add_files(filepath)
            add_deps("graphix")
            add_packages("doctest")
            add_includedirs("include")
            add_defines("DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN")

            -- Add as test
            add_tests("default", {rundir = os.projectdir()})
        target_end()
    end
end

-- Task to generate CMakeLists.txt
task("cmake")
    on_run(function ()
        import("core.project.config")

        -- Load configuration
        config.load()

        -- Generate CMakeLists.txt
        os.exec("xmake project -k cmakelists")

        print("CMakeLists.txt generated successfully!")
    end)

    set_menu {
        usage = "xmake cmake",
        description = "Generate CMakeLists.txt from xmake.lua",
        options = {}
    }
task_end()
