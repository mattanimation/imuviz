# ------------------------------------------------------------
# Variables – change only these if you move directories
# ------------------------------------------------------------
CC := gcc               # default compiler (Linux)
CFLAGS := -Wall -Werror
DEBUG_FLAGS := -g
BUILD := ./build
LINKING_FLAGS := -lX11 -lGL -lGLU -lm
NAME := imuviz


# ------------------------------------------------------------
# Build objects (single source -> single object)
# ------------------------------------------------------------
build_$(NAME):
	$(CC) $(CFLAGS) $(DEBUG_FLAGS) ./src/main.c $(LINKING_FLAGS) -o $(BUILD)/$(NAME)


# ------------------------------------------------------------
# Run helpers (useful for quick testing)
# ------------------------------------------------------------
run:
	$(BUILD)/$(NAME)

# ------------------------------------------------------------
# Cleaning
# ------------------------------------------------------------
clean:
	rm -rf $(BUILD)/*

