# Specify the name of the output binary
OUTPUT_LIBGPIOD = gpio

# List of source files
SRCS_LIBGPIOD = gpio.c

# Compiler and compiler flags
CC = gcc
CFLAGS = -Wall -Wextra

# Libraries to link (if any)
LIBS = -lgpiod

# Include directory for libgpiod header
INC_DIRS = -I/usr/include

# Targets to build both binaries
all: $(OUTPUT_LIBGPIOD)

# Compile the libgpiod program
$(OUTPUT_LIBGPIOD): $(SRCS_LIBGPIOD)
	$(CC) $(CFLAGS) $(INC_DIRS) -o $(OUTPUT_LIBGPIOD) $(SRCS_LIBGPIOD) $(LIBS)

# Clean up build artifacts
clean:
	rm -f $(OUTPUT_LIBGPIOD)

