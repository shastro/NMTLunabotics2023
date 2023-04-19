all: david.c david.h

david.c david.h &: david.kcd
	cantools generate_c_source $^
	# Preserve reproducibility.
	sed -ie '1,/This file was generated/d' david.c david.h
	sed -ie '1d' david.c david.h

clean:
	$(RM) david.c david.h

.PHONY: clean
