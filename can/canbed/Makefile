default: copy_arduino_lib

copy_arduino_lib: shared_lib/arduino_lib.hpp
	for d in */; do \
		if [ "$$d" != shared_lib/ ]; then \
			cp ./shared_lib/arduino_lib.hpp "$$d"; \
		fi; \
	done
