all: david.c david.h canshit.cpp canshit.hpp unpackers.hpp limits.hpp

david.c david.h: david.kcd
	cantools generate_c_source $^
	# Preserve reproducibility.
	sed -ie '1,/This file was generated/d' david.c david.h
	sed -ie '1d' david.c david.h

canshit.cpp canshit.hpp unpackers.hpp limits.hpp ../../ros/catkin_ws/motor_bridge/msg: genfromkcd.py david.kcd 
	python3 $<

clean:
	$(RM) -r david.c david.h canshit.cpp canshit.hpp unpackers.hpp limits.hpp ../../ros/catkin_ws/src/motor_bridge/msg

.PHONY: clean
