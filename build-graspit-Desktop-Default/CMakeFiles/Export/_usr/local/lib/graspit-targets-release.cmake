#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "graspit" for configuration "Release"
set_property(TARGET graspit APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(graspit PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "/usr/lib/x86_64-linux-gnu/libQt3Support.so;/usr/lib/x86_64-linux-gnu/libQtGui.so;/usr/lib/x86_64-linux-gnu/libQtXml.so;/usr/lib/x86_64-linux-gnu/libQtSql.so;/usr/lib/x86_64-linux-gnu/libQtNetwork.so;/usr/lib/x86_64-linux-gnu/libQtCore.so;/usr/lib/x86_64-linux-gnu/libQt3Support.so;/usr/lib/x86_64-linux-gnu/libqhull.so;SoQt;QtOpenGL;QtGui;QtCore;Xmu;Xi;Coin;GL;Xext;SM;ICE;X11;dl;pthread;/usr/lib/liblapack.so;/usr/lib/libf77blas.so;/usr/lib/libatlas.so"
  IMPORTED_LOCATION_RELEASE "/usr/local/lib/libgraspit.so"
  IMPORTED_SONAME_RELEASE "libgraspit.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS graspit )
list(APPEND _IMPORT_CHECK_FILES_FOR_graspit "/usr/local/lib/libgraspit.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
