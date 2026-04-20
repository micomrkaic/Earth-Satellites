# Earth Satellite Orbit Simulation

A real-time 3D orbital mechanics simulation written in C with SDL2. Renders a
raycast Earth sphere with Lambert shading and a day/night terminator, eight
named satellites spanning the full range of orbit families, an optional full
24-satellite GPS constellation, satellite coverage footprint circles, and a
Mercator ground track monitor rendered from Natural Earth 1:50m geodata.

![Simulation screenshot placeholder](screenshot.png)

---

## Features

- **Raycast Earth renderer** — per-pixel sphere with land/ocean distinction,
  polar ice caps, lat/lon grid, atmospheric limb glow, and a physically correct
  day/night terminator (sun fixed at ECI +X).

- **Mercator ground track monitor** (`P`) — full-screen mission-control style
  map built from Natural Earth 1:50m coastlines and country borders. Shows each
  satellite's current sub-satellite point and accumulated ground track history
  with age-based alpha fade. Pressing `P` again returns to the 3D sphere view.

- **Eight named orbit families** simultaneously simulated with verified
  real-world parameters:

  | # | Name | a (km) | e | i | Ω | Notes |
  |---|------|--------|---|---|---|-------|
  | 1 | GEO | 42 164 | 0 | 0° | 0° | Geostationary, equatorial |
  | 2 | GEOSYNC | 42 164 | 0 | 28° | 45° | Inclined geosynchronous, figure-8 ground track |
  | 3 | MOLNIYA | 26 560 | 0.74 | 63.4° | 40° | Apogee over eastern Siberia (~130°E), 12 h period |
  | 4 | ISS | 6 778 | 0 | 51.6° | 45° | ~407 km, 92.5 min period |
  | 5 | POLAR | 7 000 | 0 | 90° | 90° | True polar LEO, 629 km |
  | 6 | MEO/GPS | 26 560 | 0 | 55° | 160° | GPS representative, ~12 h period |
  | 7 | TUNDRA | 42 164 | 0.27 | 63.4° | 180° | Apogee over North America (~90°W) |
  | 8 | SSO | 7 178 | 0 | 98.7° | 130° | Sun-synchronous, 807 km |

- **Full GPS constellation** (`C`) — 24-satellite Walker 24/6/2 configuration:
  6 orbital planes (Ω = 0°–300° in 60° steps), 4 satellites per plane spaced
  90° apart, inter-plane phase offset 30°. All at 20,200 km, i=55°, circular.
  Each plane rendered in a distinct colour. Uses a lightweight struct with no
  trail or ground track history.

- **Coverage footprint circles** (`F`) — for every visible named satellite and
  (when `C` is on) all 24 GPS satellites, draws the boundary of the region
  where the satellite is above 5° elevation. Coverage half-angle is computed
  per satellite from its current altitude, so GEO footprints (~76°) and ISS
  footprints (~16°) are automatically sized correctly. Enabling `C + F`
  together shows the full GPS global coverage mesh.

- **Velocity Verlet integrator** at 20 s physics timestep with adaptive
  sub-stepping near perigee — Molniya perigee uses 8 sub-steps of 2.5 s
  (force change < 0.7% per step). All satellites integrate continuously
  regardless of visibility, so toggling display never causes position jumps.

- **J2 oblateness perturbation** (`J`) — toggleable zonal harmonic
  acceleration; makes SSO precess correctly, LEO nodes regress, and Molniya
  apogee rotate.

- **Ground track display** (`G`, 3D mode) — sub-satellite point on sphere
  surface with nadir line.

- **Ground track history** (map mode) — 1 500-point ring buffer per satellite
  (~41 h at default sampling). Break sentinels prevent jump lines when
  satellites are re-enabled.

- **Orbit path overlay** — analytically computed Keplerian ellipses at 7 200
  points per orbit (sub-2 px segments at maximum zoom).

- **Fading 3D trails** — 500-point ECI ring buffer, sampled every 5 physics
  steps (~100 s sim time).

- **MP4 video capture** (`V`) — pipes raw RGB24 frames to `ffmpeg`; timestamped
  output file. Works in both 3D and map modes.

- **Full 3D mouse rotation and zoom** — scroll out to see all orbits including
  Tundra/GEO; scroll in to examine LEO details.

- **Embedded 5×7 bitmap font** — no font library dependency.

---

## Requirements

| Dependency | Notes |
|------------|-------|
| GCC ≥ 9 (C17) | or any C17-compliant compiler |
| SDL2 | `libsdl2-dev` on Debian/Ubuntu |
| `sdl2-config` | included with `libsdl2-dev` |
| `libm` | standard math library |
| Python 3 | for `gen_map_data.py` (run once at build time) |
| `ffmpeg` | **optional** — required only for video capture (`V` key) |

---

## Building

```bash
make            # builds ./earth_orbit  (runs gen_map_data.py on first build)
make run        # builds and launches
make clean      # removes the binary
make distclean  # removes binary, map_data.h, and downloaded tarball
```

On the first build, `make` runs `gen_map_data.py` which downloads the
`world-atlas@2.0.2` npm package (~2 MB, cached as `world-atlas-2.0.2.tgz`)
and generates `map_data.h` (~1.8 MB) containing Natural Earth 1:50m coastline
and country border coordinates as flat C arrays. Subsequent builds skip this
step entirely.

Direct compile (requires `map_data.h` to already exist):

```bash
gcc -O2 -std=c17 -o earth_orbit earth_orbit.c $(sdl2-config --cflags --libs) -lm
```

---

## Controls

| Input | Action |
|-------|--------|
| Left-drag | Rotate view (3D mode) |
| Scroll wheel | Zoom in / out (3D mode) |
| `+` / `=` | Double simulation speed |
| `-` | Halve simulation speed |
| `1` – `8` | Toggle individual named satellite on/off |
| `R` | Reset view to default zoom and angle |
| `Space` | Pause / resume |
| `J` | Toggle J2 oblateness perturbation |
| `G` | Toggle ground track display (3D mode only) |
| `C` | Toggle full 24-satellite GPS constellation |
| `F` | Toggle coverage footprint circles |
| `P` | Toggle Mercator ground track map / 3D sphere view |
| `V` | Start / stop MP4 video capture |
| `Q` / `Esc` | Quit |

Speed range: 1× – 1 000 000×. At the default 800×, one GEO orbit takes about
13 s of wall time.

---

## Physics

### Integrator

Velocity Verlet at a fixed 20 s outer timestep with adaptive sub-stepping.
A fractional accumulator converts real elapsed time to physics steps:

```
sim_acc += time_scale * dt_wall
while sim_acc >= DT: step(); sim_acc -= DT
```

Up to 600 steps are taken per frame. Any excess accumulator is discarded to
prevent jump-on-resume. All satellites — including GPS and invisible ones —
are integrated every step.

### Adaptive sub-stepping

Near perigee of highly eccentric orbits the gravitational force changes
significantly per timestep. Sub-steps are chosen so force change per sub-step
stays below ~1%:

| Altitude | Sub-steps | Sub-step size | Force Δ/step |
|----------|-----------|---------------|--------------|
| < 1 000 km | 8 | 2.5 s | ~0.7% |
| < 4 000 km | 4 | 5 s | ~1.4% |
| < 10 000 km | 2 | 10 s | ~0.7% |
| ≥ 10 000 km | 1 | 20 s | < 0.6% |

Molniya perigee (535 km) uses 8 sub-steps; all GPS and GEO orbits use 1.

### Gravity

Two-body point-mass gravity (always active):

```
a = −(GM / r³) · r⃗
```

With J2 enabled, the zonal harmonic correction is added:

```
factor = (3/2) · J₂ · GM · Rₑ² / r⁵

aₓ += factor · x · (1 − 5z²/r²)
aᵧ += factor · y · (1 − 5z²/r²)
a_z += factor · z · (3 − 5z²/r²)
```

where J₂ = 1.08263 × 10⁻³, Rₑ = 6 371 000 m, GM = 3.986004418 × 10¹⁴ m³/s².

### Initial conditions

Satellites are initialised from classical orbital elements via the Keplerian
solution: Kepler's equation solved by Newton's method (convergence to 10⁻¹³
rad), then transformed to ECI position and velocity through the perifocal frame
rotation (Ω, i, ω).

### Orbit placement rationale

Molniya and Tundra both use ω = 270° (apogee north). The ECI apogee longitude
is 90° + Ω, so:

- **Molniya** Ω = 40° → apogee at 130°E (eastern Siberia), matching Soviet-era
  operational coverage.
- **Tundra** Ω = 180° → apogee at 270°E = 90°W (North America), matching
  Sirius XM Radio orbital geometry.

### Coverage footprint

For a satellite at altitude h with minimum elevation angle ε = 5°, the Earth
central half-angle of coverage is:

```
η      = arcsin((Rₑ / (Rₑ + h)) · cos ε)
λ      = 90° − ε − η
```

Representative values at ε = 5°:

| Orbit | Altitude | λ (coverage radius) |
|-------|----------|---------------------|
| ISS | 407 km | ~16° |
| SSO | 807 km | ~20° |
| GPS | 20 200 km | ~71° |
| GEO | 35 793 km | ~76° |

### Earth rotation

`θ += Ω_E · dt` per physics step (Ω_E = 7.292115 × 10⁻⁵ rad/s), keeping
ground tracks and the Mercator map in sync with inertial satellite positions.

---

## Rendering pipeline

### 3D sphere mode

```
Per frame:
  1. Clear to black
  2. Draw star field (250 points)
  3. Lock streaming ARGB8888 texture → raycast Earth sphere
       pixel → view-space normal → ECI → Earth-fixed → lat/lon
       → land mask lookup → Lambert shading + ice/grid/limb glow
  4. Unlock + RenderCopy texture
  5. Draw orbit path ellipses (7 200 analytical Keplerian points each)
  6. [G] Draw ground track dots + nadir lines on sphere surface
  7. [F] Draw footprint circles on Earth surface (main sats + GPS if C)
  8. [C] Draw GPS constellation dots (diamond markers, colour per plane)
  9. Draw named satellite trails + star-shaped markers
 10. Draw HUD (legend, status bar, controls hint)
 11. [V] SDL_RenderReadPixels → write RGB24 frame to ffmpeg pipe
 12. SDL_RenderPresent
```

### Mercator map mode (P)

```
Startup (once):
  1. Fill MAP_W × WIN_H pixel buffer with navy background + lat/lon grid
  2. Draw Natural Earth 1:50m country borders (362 arcs, 1 px, dim green)
  3. Draw Natural Earth 1:50m coastlines (1 597 arcs, 2 px, bright green)
  4. Upload to static SDL_Texture (never redrawn)

Per frame:
  1. Blit static map texture into left 1 000 px (panel excluded)
  2. Draw ground track history with age-based alpha fade
     — GT_BREAK sentinels prevent lines across toggle-off gaps
     — antimeridian-crossing segments skipped
  3. [F] Draw footprint circles (lat/lon polylines with antimeridian breaks)
  4. [C] Draw GPS satellite positions (small coloured squares, colour per plane)
  5. Draw named satellite current position markers + crosshairs + labels
  6. Draw lat/lon axis tick labels
  7. Draw right panel (legend with live lat/lon per satellite, toggle states)
  8. Draw status bar (sim time, speed, pause, REC indicator)
```

---

## GPS Constellation

The full GPS constellation is implemented as a Walker 24/6/2 configuration,
matching the operational GPS Block II/III constellation geometry:

- 6 orbital planes labelled A–F, RAAN spaced 60° apart
- 4 satellites per plane, in-plane spacing 90°
- Inter-plane phase factor F=2, giving a 30° mean-anomaly offset between
  adjacent planes for uniform global coverage
- a = 26 559.75 km (20 200 km altitude), e = 0, i = 55°

Each plane is rendered in a distinct dim colour so the plane structure is
visible in both the 3D and map views. The GPS satellites use a lightweight
struct (no trail or ground track history) — memory overhead for all 24 is
under 2 KB. They are always integrated regardless of whether `C` is toggled,
so enabling the display never causes position discontinuities.

---

## Map data

The Mercator background is generated at first build from Natural Earth 1:50m
data via the `world-atlas@2.0.2` npm package (ISC licence / public domain
geodata). `gen_map_data.py` decodes the TopoJSON arc format, splits arcs into
coastlines (arcs bounding the unified `land` object) and internal country
borders (arcs in `countries` but not in `land`), and outputs `map_data.h` with
flat `float[N][2]` arrays ready for direct `#include`.

---

## Video capture

Press `V` to start recording. A timestamped `.mp4` is created in the working
directory. Press `V` again to stop and finalize. The blinking `REC` indicator
appears bottom-right while recording. Works in both 3D and map modes.

```bash
# Internal ffmpeg command:
ffmpeg -y -f rawvideo -pixel_format rgb24 -video_size 1200x780 \
  -framerate 60 -i pipe:0 \
  -c:v libx264 -preset fast -crf 18 -pix_fmt yuv420p \
  earth_orbit_YYYYMMDD_HHMMSS.mp4

# Playback:
ffplay earth_orbit_YYYYMMDD_HHMMSS.mp4
```

---

## File structure

```
earth_orbit.c         — complete simulation (~1 510 lines, single translation unit)
gen_map_data.py       — downloads world-atlas tarball, generates map_data.h
Makefile              — build rules, map_data.h dependency
README.md             — this file

Generated at build time (not committed):
map_data.h            — Natural Earth 1:50m C arrays (~1.8 MB, ~20 000 lines)
world-atlas-2.0.2.tgz — cached npm tarball (~2 MB)
```

---

## Constants

| Symbol | Value | Description |
|--------|-------|-------------|
| `RE` | 6 371 000 m | Earth mean radius |
| `GM` | 3.986004418 × 10¹⁴ m³/s² | Gravitational parameter |
| `Ω_E` | 7.2921150 × 10⁻⁵ rad/s | Earth rotation rate |
| `J₂` | 1.08263 × 10⁻³ | Second zonal harmonic |
| `DT_PHYS` | 20 s | Verlet outer timestep |
| `GT_LEN` | 1 500 points | Ground track history per named satellite |
| `TRAIL_LEN` | 500 points | 3D ECI trail per named satellite |
| `ORBIT_PTS` | 7 200 | Analytical orbit path resolution |
| `FP_PTS` | 90 | Points on each footprint circle |
| `GPS_N` | 24 | GPS constellation size |
| `MAP_LAT_CLIP` | ±82° | Mercator latitude cutoff |

---

## License

Released into the public domain. Do whatever you like with it.
The Natural Earth geodata embedded in `map_data.h` is public domain.
The `world-atlas` npm package is ISC licensed.
