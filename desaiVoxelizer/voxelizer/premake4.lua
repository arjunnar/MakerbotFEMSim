solution "Vox"
  include ("vecmath")
  configurations { "Debug", "Release" }

project "vox"
  language "C++"
  kind     "ConsoleApp"
  files  { "*.cpp", "src/**.cpp" }
  includedirs {"include","include/vecmath"}
  links {"vecmath"}
  
  configuration { "Debug*" }
    targetdir "debug"
    defines { "DEBUG" }
    flags   { "Symbols" }
    
  configuration { "Release*" }
    targetdir "release"
    defines { "NDEBUG" }
    flags   { "Optimize" } 
