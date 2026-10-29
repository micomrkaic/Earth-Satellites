# Makefile — Earth Satellite Orbit Simulation
# Targets:
#   make             → native binary   ./earth_orbit
#   make run         → build + launch  (native)
#   make install     → install binary + man page to ~/.local (or PREFIX=…)
#   make uninstall   → remove installed files
#   make wasm        → WASM build      ./web/index.html
#   make serve       → local test server for WASM
#   make pages       → copy WASM build to docs/ (for GitHub Pages)
#   make zip         → create earth_orbit_web.zip for itch.io
#   make clean       → remove binaries and web/
#   make distclean   → also remove map_data.h and cached tarball

CC      ?= gcc
EMCC    ?= emcc
PYTHON  ?= python3

PREFIX  ?= $(HOME)/.local
BINDIR   = $(PREFIX)/bin
MANDIR   = $(PREFIX)/share/man/man6

CFLAGS_NATIVE  = -O2 -std=c17 $(shell sdl2-config --cflags)
LIBS_NATIVE    = $(shell sdl2-config --libs) -lm
CFLAGS_WASM    = -O2 -std=gnu17 \
                 -s USE_SDL=2 \
                 -s ALLOW_MEMORY_GROWTH=1 \
                 -s INITIAL_MEMORY=67108864 \
                 -s ENVIRONMENT=web \
                 -s SINGLE_FILE=0 \
                 -s ASSERTIONS=0 \
                 -s MINIFY_HTML=0 \
                 -s TEXTDECODER=2 \
                 --shell-file shell.html

# GitHub Pages can't serve .wasm with the correct MIME type, so we bake
# the wasm into the JS as base64.  Larger download but no CORS/MIME issues.
CFLAGS_PAGES   = -O2 -std=gnu17 \
                 -s USE_SDL=2 \
                 -s INITIAL_MEMORY=67108864 \
                 -s ENVIRONMENT=web \
                 -s SINGLE_FILE=1 \
                 -s ASSERTIONS=0 \
                 -s MINIFY_HTML=0 \
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
zip: wasm
	cd $(WEB_DIR) && zip ../earth_orbit_web.zip index.html index.js *.wasm
	@echo "Created earth_orbit_web.zip — upload to itch.io"

# ── Install / uninstall (native binary + man page) ───────────────────────────
.PHONY: install
install: $(BIN)
	install -Dm755 $(BIN)          $(BINDIR)/$(BIN)
	install -Dm644 earth_orbit.6   $(MANDIR)/earth_orbit.6
	@echo "Installed binary : $(BINDIR)/$(BIN)"
	@echo "Installed man page: $(MANDIR)/earth_orbit.6"
	@echo "Ensure $(BINDIR) is in PATH and $(PREFIX)/share/man is in MANPATH."

.PHONY: uninstall
uninstall:
	rm -f $(BINDIR)/$(BIN)
	rm -f $(MANDIR)/earth_orbit.6
	@echo "Uninstalled."

# ── GitHub Pages: single-file build into docs/ (no .wasm MIME issues) ────────
.PHONY: pages
pages: $(MAP_DATA)
	@command -v $(EMCC) >/dev/null 2>&1 || { \
	    echo "ERROR: emcc not found.  Run:"; \
	    echo "  source emsdk/emsdk_env.sh"; \
	    exit 1; }
	mkdir -p docs
	$(EMCC) $(SRC) $(CFLAGS_PAGES) -o docs/index.html
	@echo "docs/ is ready — commit it and enable GitHub Pages (main branch, /docs)."

# ── Housekeeping ─────────────────────────────────────────────────────────────
.PHONY: clean
clean:
	rm -f $(BIN)
	rm -rf $(WEB_DIR)

.PHONY: distclean
distclean: clean
	rm -f $(MAP_DATA) world-atlas-*.tgz
