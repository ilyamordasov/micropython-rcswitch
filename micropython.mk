RCSWITCH_MOD_DIR := $(USERMOD_DIR)

# Add all C files to SRC_USERMOD.
SRC_USERMOD += $(RCSWITCH_MOD_DIR)/rcswitch.c

# We can add our module folder to include paths if needed
# This is not actually needed in this example.
CFLAGS_USERMOD += -I$(RCSWITCH_MOD_DIR)
CRCSWITCH_MOD_DIR := $(USERMOD_DIR)
