#
# Makefile
# dephilia, 2021-04-27 16:31
#

all: init builds run

init:
	mkdir -p build; \
	cd build; \
	cmake .. -G "Ninja"

debugging:
	cmake --build build -DCMAKE_BUILD_TYPE=Debug

builds:
	cmake --build build

run:
	./build/dsMotor

clean:
	rm -rf build
