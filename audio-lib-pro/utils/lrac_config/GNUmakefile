
BUILD_FOLDER = build
SOURCE_FOLDERS = source/com source
INC_FOLDER = $(SOURCE_FOLDERS)

INC_FOLDER_OPT=$(foreach d, $(INC_FOLDER), -I$d)

CC = gcc

EXECUTABLE = lrac_config.exe

CCFLAGS = -c $(INC_FOLDER_OPT) -g
LDFLAGS = -pthread -g

src = $(foreach dir,$(SOURCE_FOLDERS),$(wildcard $(dir)/*.c))
obj = $(addprefix $(BUILD_FOLDER)/, $(notdir $(src:.c=.o)))
dep = $(obj:.o=.d)

all: $(EXECUTABLE)

$(EXECUTABLE): $(obj)
	@echo Linking application $@
	@$(CC) $^ $(LDFLAGS) -o $@

# C rule macro
define c_compile_rule
$(BUILD_FOLDER)/%.o: $(1)/%.c
	@echo "Compiling '$$<'"
	@mkdir -p $$(@D)
	@$(CC) $$(CCFLAGS) -o $$@ -c $$<
endef

# Create a rule for every source folder
$(foreach dir, $(SOURCE_FOLDERS), $(eval $(call c_compile_rule, $(dir))))

.PHONY: clean getlibs
clean:
	rm -rf $(BUILD_FOLDER) $(EXECUTABLE)

# empty target in case MT IDE processing tries to work with this file for getlibs
getlibs:

-include $(dep)   # include all dep files in the makefile
