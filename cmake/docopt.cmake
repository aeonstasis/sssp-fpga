include(ExternalProject)
ExternalProject_Add(docopt
	GIT_REPOSITORY https://github.com/docopt/docopt.cpp.git
  BUILD_COMMAND make docopt_s
	INSTALL_COMMAND "")
ExternalProject_Get_Property(docopt binary_dir)
ExternalProject_Get_Property(docopt source_dir)
include_directories(${source_dir})
add_library(libdocopt STATIC IMPORTED GLOBAL)
add_dependencies(libdocopt docopt)
set_target_properties(libdocopt PROPERTIES
  IMPORTED_LOCATION ${binary_dir}/libdocopt.a
)
