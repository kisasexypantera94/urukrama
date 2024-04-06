.DEFAULT_GOAL := debug

configure-release:
	cmake -Hsrc -Bbuild/release -DCMAKE_BUILD_TYPE=Release

configure-debug:
	cmake -Hsrc -Bbuild/debug -DCMAKE_BUILD_TYPE=Debug

release: configure-release
	make -C build/release

debug: configure-debug
	make -C build/debug