add_library(logger logger.hpp)

find_package(spdlog REQUIRED)

target_link_libraries(logger spdlog::spdlog)
 set_target_properties(logger PROPERTIES LINKER_LANGUAGE CXX)