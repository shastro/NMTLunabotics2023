all: david.c david.h canshit.cpp

david.c david.h: david.kcd
	cantools generate_c_source $^
	# Preserve reproducibility.
	sed -ie '1,/This file was generated/d' david.c david.h
	sed -ie '1d' david.c david.h

canshit.cpp canshit.hpp unpackers.hpp limits.hpp: genfromkcd.py david.kcd
	python $< > canshit.cpp canshit.hpp unpackers.hpp limits.hpp

clean:
	$(RM) david.c david.h canshit.cpp canshit.hpp unpackers.hpp limits.hpp

.PHONY: clean
