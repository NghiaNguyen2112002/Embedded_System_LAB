# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/user/esp/esp-idf/components/bootloader/subproject"
  "E:/University/Embedded_System/Code/LAB_Source_Code/ES_LAB/build/bootloader"
  "E:/University/Embedded_System/Code/LAB_Source_Code/ES_LAB/build/bootloader-prefix"
  "E:/University/Embedded_System/Code/LAB_Source_Code/ES_LAB/build/bootloader-prefix/tmp"
  "E:/University/Embedded_System/Code/LAB_Source_Code/ES_LAB/build/bootloader-prefix/src/bootloader-stamp"
  "E:/University/Embedded_System/Code/LAB_Source_Code/ES_LAB/build/bootloader-prefix/src"
  "E:/University/Embedded_System/Code/LAB_Source_Code/ES_LAB/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/University/Embedded_System/Code/LAB_Source_Code/ES_LAB/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/University/Embedded_System/Code/LAB_Source_Code/ES_LAB/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
