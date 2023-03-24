all: david.c david.h

david.c david.h &: david.kcd
	cantools generate_c_source $^

# For arduino compiler we need david.h in each arduino project folder
# that needs the CAN message macros defined in david.h.
# So please add cp for each folder as needed.
clean:
	$(RM) david.c david.h
	cp david.h ../canbed/pitch_ctrl/david.h

.PHONY: clean
