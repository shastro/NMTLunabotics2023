all: david.c david.h

david.c david.h : david.kcd
	cantools generate_c_source $^
	# Preserve reproducibility.
	sed -ie '1,/This file was generated/d' david.c david.h
	sed -ie '1d' david.c david.h

canshit.cpp canshit.hpp unpackers.hpp limits.hpp : gen_from_kcd.py david.kcd
	python gen_from_kcd.py > canshit.cpp canshit.hpp unpackers.hpp limits.hpp

clean:
	$(RM) david.c david.h canshit.cpp canshit.hpp unpackers.hpp limits.hpp

.PHONY: clean
