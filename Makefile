.DEFAULT_GOAL := debug

current_dir=$(CURDIR)

conan-install-debug:
	conan install . --build=missing --settings=build_type=Debug

conan-install-release:
	conan install . --build=missing --settings=build_type=Release

conan-install-reldbg:
	conan install . --build=missing --settings=build_type=RelWithDebInfo

configure-release: conan-install-release
	cmake -Hsrc -Bbuild/Release \
	-DCMAKE_C_COMPILER=clang \
	-DCMAKE_CXX_COMPILER=clang++ \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_TOOLCHAIN_FILE=$(current_dir)/build/Release/generators/conan_toolchain.cmake

configure-reldbg: conan-install-reldbg
	cmake -Hsrc -Bbuild/RelWithDebInfo \
	-DCMAKE_C_COMPILER=clang \
	-DCMAKE_CXX_COMPILER=clang++ \
	-DCMAKE_BUILD_TYPE=RelWithDebInfo \
	-DCMAKE_TOOLCHAIN_FILE=$(current_dir)/build/RelWithDebInfo/generators/conan_toolchain.cmake

configure-debug: conan-install-debug
	cmake -Hsrc -Bbuild/Debug \
	-DCMAKE_C_COMPILER=clang \
	-DCMAKE_CXX_COMPILER=clang++ \
	-DCMAKE_BUILD_TYPE=Debug \
	-DCMAKE_TOOLCHAIN_FILE=$(current_dir)/build/Debug/generators/conan_toolchain.cmake

release: configure-release
	make -j`$(nproc)` -C build/Release

reldbg: configure-reldbg
	make -j`$(nproc)` -C build/RelWithDebInfo

debug: configure-debug
	make -j`$(nproc)` -C build/Debug