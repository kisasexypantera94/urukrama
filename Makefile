.DEFAULT_GOAL := debug

current_dir=$(CURDIR)

conan-install-debug:
	conan install . --build=missing --settings=build_type=Debug

conan-install-release:
	conan install . --build=missing --settings=build_type=Release

conan-install-reldbg:
	conan install . --build=missing --settings=build_type=RelWithDebInfo

configure-release: conan-install-release
	cmake -Hsrc -Bbuild/release -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$(current_dir)/build/Release/generators/conan_toolchain.cmake

configure-reldbg: conan-install-reldbg
	cmake -Hsrc -Bbuild/reldbg -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_TOOLCHAIN_FILE=$(current_dir)/build/RelWithDebInfo/generators/conan_toolchain.cmake

configure-debug: conan-install-debug
	cmake -Hsrc -Bbuild/debug -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=$(current_dir)/build/Debug/generators/conan_toolchain.cmake

release: configure-release
	make -C build/release

reldbg: configure-reldbg
	make -C build/reldbg

debug: configure-debug
	make -C build/debug