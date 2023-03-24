all: david.c david.h

david.c david.h &: david.kcd
	cantools generate_c_source $^

clean:
	$(RM) david.c david.h

.PHONY: clean
