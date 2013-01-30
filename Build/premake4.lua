--
-- Use the --to=path option to control where the project files get generated. I use
-- this to create project files for each supported toolset, each in their own folder,
-- in preparation for deployment.
--
newoption {
	trigger = "to",
	value   = "path",
	description = "Set the output location for the generated files"
}

if _OPTIONS["to"] ~= nil then
	todir = _OPTIONS["to"]
else
	todir = "."
end

solution "recastnavigation"
	configurations { 
		"Debug",
		"Release"
	}
	location (todir)

	-- extra warnings, no exceptions or rtti
	flags { 
		"ExtraWarnings",
		"NoExceptions",
		"NoRTTI"
	}
	
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
	kind "StaticLib"
	location (todir)
	includedirs { 
		"../DebugUtils/Include",
		"../Detour/Include",
		"../DetourTileCache/Include",
		"../Recast/Include"
	}
	files { 
		"../DebugUtils/Include/*.h",
		"../DebugUtils/Source/*.cpp"
	}
	targetdir (todir .. "/lib")

project "Detour"
	language "C++"
	kind "StaticLib"
	location (todir)
	includedirs { 
		"../Detour/Include" 
	}
	files { 
		"../Detour/Include/*.h", 
		"../Detour/Source/*.cpp" 
	}
	targetdir (todir .. "/lib")

project "DetourCrowd"
	language "C++"
	kind "StaticLib"
	location (todir)
	includedirs {
		"../DetourCrowd/Include",
		"../Detour/Include",
		"../Recast/Include"
	}
	files {
		"../DetourCrowd/Include/*.h",
		"../DetourCrowd/Source/*.cpp"
	}
	targetdir (todir .. "/lib")

project "DetourTileCache"
	language "C++"
	kind "StaticLib"
	location (todir)
	includedirs {
		"../DetourTileCache/Include",
		"../Detour/Include",
		"../Recast/Include"
	}
	files {
		"../DetourTileCache/Include/*.h",
		"../DetourTileCache/Source/*.cpp"
	}
	targetdir (todir .. "/lib")

project "Recast"
	language "C++"
	kind "StaticLib"
	location (todir)
	includedirs { 
		"../Recast/Include" 
	}
	files { 
		"../Recast/Include/*.h",
		"../Recast/Source/*.cpp" 
	}
	targetdir (todir .. "/lib")
	
project "RecastDemo"
	language "C++"
	kind "WindowedApp"
	location (todir)
	includedirs { 
		"../RecastDemo/Include",
		"../RecastDemo/Contrib",
		"../RecastDemo/Contrib/fastlz",
		"../DebugUtils/Include",
		"../Detour/Include",
		"../DetourCrowd/Include",
		"../DetourTileCache/Include",
		"../Recast/Include"
	}
	files	{ 
		"../RecastDemo/Include/*.h",
		"../RecastDemo/Source/*.cpp",
		"../RecastDemo/Contrib/fastlz/*.h",
		"../RecastDemo/Contrib/fastlz/*.c"
	}
	
	-- project dependencies
	links { 
		"DebugUtils",
		"Detour",
		"DetourCrowd",
		"DetourTileCache",
		"Recast"
	}

	-- distribute executable in RecastDemo/Bin directory
	targetdir "../RecastDemo/Bin"
	
	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags sdl`",
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"`pkg-config --libs sdl`",
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
	
	-- windows library cflags and libs
	configuration { "windows" }
		includedirs { "../RecastDemo/Contrib/SDL/include" }
		libdirs { "../RecastDemo/Contrib/SDL/lib" }
		links { 
			"opengl32",
			"glu32",
			"sdlmain",
			"sdl"
		}
 
	 -- mac includes and libs
	 configuration { "macosx" }
		 includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		 buildoptions { "-Wreorder -Wsign-compare" }
		 links { 
			 "OpenGL.framework", 
			 "/Library/Frameworks/SDL.framework", 
			 "Cocoa.framework",
		 }
		 files {
			 "../RecastDemo/Include/SDLMain.h", 
			 "../RecastDemo/Source/SDLMain.m",
			 "Info.plist",
			 "Icon.icns",
			 "English.lproj/InfoPlist.strings",
			 "English.lproj/MainMenu.xib",
		 }

