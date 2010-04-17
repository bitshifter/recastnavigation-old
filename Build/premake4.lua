solution "recastnavigation"
  configurations { "Debug", "Release" }
  
  -- default linux makefiles to the linux dir, otherwise use the action
  if _OS == "linux" and _ACTION == "gmake" then
    default_location = "linux"
  else
    default_location = _ACTION
  end
  location ( default_location )
  
  -- extra warnings, no exceptions or rtti
  flags { "ExtraWarnings", "NoExceptions", "NoRTTI" }
  
  -- debug configs
  configuration "Debug*"
    targetsuffix "D"
    defines { "DEBUG" }
    flags { "Symbols" }
 
 -- release configs
  configuration "Release*"
    defines { "NDEBUG" }
    flags { "Optimize" }
  
  -- windows specific
  configuration "windows"
    defines { "WIN32", "_WINDOWS", "_CRT_SECURE_NO_WARNINGS" }
    
project "DebugUtils"
  language "C++"
  kind     "StaticLib"
  location ( default_location )
  includedirs { "../DebugUtils/Include", "../Detour/Include", "../Recast/Include" }
  files { "../DebugUtils/Include/*.h", "../DebugUtils/Source/*.cpp" }
  targetdir ( default_location .. "/lib" )

project "Detour"
  language "C++"
  kind     "StaticLib"
  location ( default_location )
  includedirs { "../Detour/Include" }
  files { "../Detour/Include/*.h", "../Detour/Source/*.cpp" }
  targetdir ( default_location .. "/lib" )

project "Recast"
  language "C++"
  kind "StaticLib"
  location ( default_location )
  includedirs { "../Recast/Include" }
  files { "../Recast/Include/*.h", "../Recast/Source/*.cpp" }
  targetdir ( default_location .. "/lib" )
  
project "RecastDemo"
  language "C++"
  kind     "WindowedApp"
  location ( default_location )
  includedirs { "../RecastDemo/Include", "../RecastDemo/Contrib", "../DebugUtils/Include", "../Detour/Include", "../Recast/Include" }
  files  { "../RecastDemo/Include/*.h", "../RecastDemo/Source/*.cpp" }
  
  -- project dependencies
  links { "DebugUtils", "Detour", "Recast" }

  -- distribute executable in RecastDemo/Bin directory
  targetdir "../RecastDemo/Bin"
  
  -- linux library cflags and libs
  configuration { "linux", "gmake" }
    buildoptions { "`pkg-config --cflags sdl`", "`pkg-config --cflags gl`", "`pkg-config --cflags glu`" }
    linkoptions { "`pkg-config --libs sdl`", "`pkg-config --libs gl`", "`pkg-config --libs glu`" }
  
  -- windows library cflags and libs
  configuration { "windows" }
    includedirs { "../RecastDemo/Contrib/SDL/include" }
    libdirs { "../RecastDemo/Contrib/SDL/lib" }
    links { "opengl32", "glu32", "sdlmain", "sdl" }
 
