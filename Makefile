CC      = gcc
CFLAGS  = -O2 -std=c17 -Wall -Wextra -Wpedantic \
          $(shell sdl2-config --cflags)
LDFLAGS = $(shell sdl2-config --libs) -lm
TARGET  = earth_orbit
SRC     = earth_orbit.c

.PHONY: all clean run distclean

all: $(TARGET)

# Generate Natural Earth map data (downloads ~2 MB npm tarball once)
map_data.h: gen_map_data.py
	python3 gen_map_data.py

$(TARGET): $(SRC) map_data.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

# Also remove downloaded/generated data files
distclean: clean
	rm -f map_data.h world-atlas-2.0.2.tgz *.geojson
