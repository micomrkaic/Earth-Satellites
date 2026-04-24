# Makefile — Earth Satellite Orbit Simulation
# Targets:
#   make             → native binary   ./earth_orbit
#   make run         → build + launch  (native)
#   make wasm        → WASM build      ./web/index.html
#   make serve       → local test server for WASM
#   make clean       → remove binaries
#   make distclean   → remove binaries, map_data.h, cached tarball

CC      ?= gcc
EMCC    ?= emcc
PYTHON  ?= python3

CFLAGS_NATIVE  = -O2 -std=c17 $(shell sdl2-config --cflags)
LIBS_NATIVE    = $(shell sdl2-config --libs) -lm
CFLAGS_WASM    = -O2 -std=gnu17 \
                 -s USE_SDL=2 \
                 -s ALLOW_MEMORY_GROWTH=1 \
                 -s INITIAL_MEMORY=67108864 \
                 -s ENVIRONMENT=web \
                 -s SINGLE_FILE=0 \
                 -s ASSERTIONS=0 \
                 --shell-file shell.html

SRC      = earth_orbit.c
BIN      = earth_orbit
WEB_DIR  = web
MAP_DATA = map_data.h

# ── Default: native binary ──────────────────────────────────────────────────
.PHONY: all
all: $(BIN)

$(MAP_DATA):
	$(PYTHON) gen_map_data.py

$(BIN): $(SRC) $(MAP_DATA)
	$(CC) $(CFLAGS_NATIVE) -o $@ $< $(LIBS_NATIVE)

.PHONY: run
run: $(BIN)
	./$(BIN)

# ── WASM build ──────────────────────────────────────────────────────────────
.PHONY: wasm
wasm: $(MAP_DATA)
	@command -v $(EMCC) >/dev/null 2>&1 || { \
	    echo "ERROR: emcc not found.  Run:"; \
	    echo "  source emsdk/emsdk_env.sh"; \
	    exit 1; }
	mkdir -p $(WEB_DIR)
	$(EMCC) $(SRC) $(CFLAGS_WASM) -o $(WEB_DIR)/index.html

.PHONY: serve
serve:
	@echo "Serving at http://localhost:8080/"
	cd $(WEB_DIR) && $(PYTHON) -m http.server 8080

.PHONY: zip
zip:
	cd $(WEB_DIR) && zip ../earth_orbit_web.zip index.html index.js *.wasm
	@echo "Created earth_orbit_web.zip — upload to itch.io"

# ── Housekeeping ─────────────────────────────────────────────────────────────
.PHONY: clean
clean:
	rm -f $(BIN)
	rm -rf $(WEB_DIR)

.PHONY: distclean
distclean: clean
	rm -f $(MAP_DATA) world-atlas-*.tgz
