# Earth Satellite Orbit Simulation

A real-time 3D orbital mechanics simulation written in C with SDL2. Renders a
raycast Earth sphere with a precomputed land mask, Lambert shading, day/night
terminator, and eight satellites spanning the full range of orbit families —
from LEO to GEO, Molniya, and beyond.

![Simulation screenshot placeholder](screenshot.png)

---

## Features

- **Raycast Earth renderer** — per-pixel sphere with land/ocean distinction,
  polar ice caps, lat/lon grid, atmospheric limb glow, and a physically correct
  day/night terminator (sun fixed at ECI +X).
- **Eight orbit families** simultaneously simulated:
  | # | Name | Regime | Notes |
  |---|------|--------|-------|
  | 1 | GEO | Geostationary | equatorial, e=0 |
  | 2 | GEOSYNC | Geosynchronous inclined | i=28°, figure-8 ground track |
  | 3 | MOLNIYA | Molniya | e=0.74, i=63.4°, 12 h period |
  | 4 | ISS | Low Earth | i=51.6°, ~400 km |
  | 5 | POLAR | Polar LEO | i=90° |
  | 6 | MEO/GPS | Medium Earth | i=55°, ~20 200 km |
  | 7 | TUNDRA | Tundra | e=0.27, i=63.4°, GEO altitude |
  | 8 | SSO | Sun-synchronous | i=98.7°, RAAN-precessing |
- **Velocity Verlet integrator** at 20 s physics timestep with fractional
  accumulator — speed multiplier is exact regardless of frame rate.
- **J2 oblateness perturbation** — toggleable; adds the zonal harmonic
  acceleration term so SSO precesses, LEO nodes regress, and Molniya apogee
  rotates correctly.
- **Ground track display** — sub-satellite point projected onto the sphere
  surface for each visible satellite, with a nadir line from satellite to
  ground.
- **Orbit path overlay** — analytically computed Keplerian ellipses drawn
  separately from the Verlet state; behind-Earth segments dimmed.
- **Fading trails** — 500-point ring buffer per satellite, sampled every 5
  physics steps (~100 s sim time).
- **MP4 video capture** — pipes raw RGB24 frames to `ffmpeg` on demand;
  output filename is timestamped.
- **Full 3D mouse rotation** and zoom covering all orbit sizes (scroll out to
  see Tundra/GEO simultaneously; scroll in to examine LEO details).
- **Embedded 5×7 bitmap font** — no font library dependency.

---

## Requirements

| Dependency | Notes |
|------------|-------|
| GCC ≥ 9 (C17) | or any C17-compliant compiler |
| SDL2 | `libsdl2-dev` on Debian/Ubuntu |
| `sdl2-config` | included with `libsdl2-dev` |
| `libm` | standard math library |
| `ffmpeg` | **optional** — required only for video capture (`V` key) |

---

## Building

```bash
make          # builds ./earth_orbit
make run      # builds and launches
make clean    # removes the binary
```

Or directly:

```bash
gcc -O2 -std=c17 -o earth_orbit earth_orbit.c $(sdl2-config --cflags --libs) -lm
```

No external libraries beyond SDL2 and libm. The land mask (~260 000 point-in-polygon
tests) is computed at startup and takes roughly 0.1 s on modern hardware.

---

## Controls

| Input | Action |
|-------|--------|
| Left-drag | Rotate view (yaw + pitch) |
| Scroll wheel | Zoom in / out |
| `+` / `=` | Double simulation speed |
| `-` | Halve simulation speed |
| `1` – `8` | Toggle individual satellite on/off |
| `R` | Reset view to default zoom and angle |
| `Space` | Pause / resume |
| `J` | Toggle J2 oblateness perturbation |
| `G` | Toggle ground track display |
| `V` | Start / stop MP4 video capture |
| `Q` / `Esc` | Quit |

Speed range: 1× – 1 000 000×. At the default 800× one GEO orbit takes about
13 s of wall time.

---

## Physics

### Integrator

Velocity Verlet at a fixed 20 s timestep. A fractional accumulator converts
real elapsed time to physics steps:

```
sim_acc += time_scale * dt_wall
while sim_acc >= DT: step(); sim_acc -= DT
```

Up to 600 steps are taken per frame to keep up at high speed multipliers.

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

### Earth rotation

Earth rotation angle advances as `θ += Ω_E · dt` per physics step
(Ω_E = 7.292115 × 10⁻⁵ rad/s), keeping the land mask and ground tracks in
sync with the inertial satellite positions.

---

## Rendering pipeline

```
Per frame:
  1. Clear to black
  2. Draw star field (250 points, fixed positions)
  3. Lock streaming ARGB8888 texture → raycast Earth sphere
       pixel → view-space normal → ECI → Earth-fixed → lat/lon
       → land mask lookup → Lambert shading + ice/grid/limb
  4. Unlock + RenderCopy texture
  5. Draw orbit path ellipses (360 analytical Keplerian points each)
  6. [G] Draw ground track dots + nadir lines
  7. Draw satellite trails + star markers
  8. Draw HUD (legend, status bar, controls hint)
  9. [V] SDL_RenderReadPixels → write RGB24 frame to ffmpeg pipe
 10. SDL_RenderPresent
```

---

## Video capture

Press `V` to start recording. A timestamped `.mp4` file is created in the
working directory. Press `V` again to stop and finalize. The `REC` indicator
in the bottom-right corner blinks while recording.

The ffmpeg command used internally:

```
ffmpeg -y -f rawvideo -pixel_format rgb24 -video_size 1200x780 \
  -framerate 60 -i pipe:0 \
  -c:v libx264 -preset fast -crf 18 -pix_fmt yuv420p \
  earth_orbit_YYYYMMDD_HHMMSS.mp4
```

`ffmpeg` must be available in `$PATH`. If it is not found, a message is printed
to stderr and recording does not start.

---

## File structure

```
earth_orbit.c   — complete simulation (~930 lines, single translation unit)
Makefile        — build rules
README.md       — this file
```

---

## Constants

| Symbol | Value | Description |
|--------|-------|-------------|
| `RE` | 6 371 000 m | Earth mean radius |
| `GM` | 3.986004418 × 10¹⁴ m³/s² | Gravitational parameter |
| `Ω_E` | 7.2921150 × 10⁻⁵ rad/s | Earth rotation rate |
| `J₂` | 1.08263 × 10⁻³ | Second zonal harmonic |
| `DT_PHYS` | 20 s | Verlet timestep |

---

## License

Released into the public domain. Do whatever you like with it.
