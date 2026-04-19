// earth_orbit.c  —  Satellite Orbital Mechanics Simulation
// Build:  gcc -O2 -std=c17 -o earth_orbit earth_orbit.c $(sdl2-config --cflags --libs) -lm
//
// Controls:
//   Left-drag    rotate view
//   Scroll       zoom in/out
//   +/=          faster
//   -            slower
//   1-8          toggle satellite
//   R            reset view
//   Space        pause / resume
//   J            toggle J2 oblateness perturbation
//   G            toggle ground track display
//   V            start/stop MP4 video capture  (requires ffmpeg in PATH)
//   Q / Esc      quit

#include <SDL2/SDL.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

/* ======================================================================
   Constants
   ====================================================================== */
#define WIN_W    1200
#define WIN_H     780
#define CX        480          /* earth disc centre X (pixels) */
#define CY        390          /* earth disc centre Y (pixels) */
#define BASE_R    270.0        /* earth radius at zoom = 1.0 (pixels) */
#define ZOOM_INIT   0.15       /* initial zoom — shows all orbits incl. GEO/Tundra */
#define ZOOM_MIN    0.04
#define ZOOM_MAX    8.0

#define RE        6371000.0       /* Earth radius (m) */
#define GM        3.986004418e14  /* gravitational parameter (m^3/s^2) */
#define OMEGA_E   7.2921150e-5    /* Earth rotation rate (rad/s) */
#define J2_COEFF  1.08263e-3      /* Earth J2 zonal harmonic */
#define PI        3.14159265358979323846
#define DEG       (PI/180.0)

#define DT_PHYS   20.0         /* Verlet physics timestep (s) */
#define NUM_SATS   8
#define TRAIL_LEN  500
#define TRAIL_STEP   5
#define ORBIT_PTS  360

/* ======================================================================
   5x7 bitmap font
   ====================================================================== */
static const uint8_t FONT[59][7] = {
/* 32 ' ' */{0x00,0x00,0x00,0x00,0x00,0x00,0x00},
/* 33 '!' */{0x04,0x04,0x04,0x04,0x04,0x00,0x04},
/* 34 '"' */{0x0A,0x0A,0x00,0x00,0x00,0x00,0x00},
/* 35     */{0x00,0x00,0x00,0x00,0x00,0x00,0x00},
/* 36     */{0x00,0x00,0x00,0x00,0x00,0x00,0x00},
/* 37     */{0x00,0x00,0x00,0x00,0x00,0x00,0x00},
/* 38     */{0x00,0x00,0x00,0x00,0x00,0x00,0x00},
/* 39     */{0x00,0x00,0x00,0x00,0x00,0x00,0x00},
/* 40 '(' */{0x04,0x08,0x10,0x10,0x10,0x08,0x04},
/* 41 ')' */{0x04,0x02,0x01,0x01,0x01,0x02,0x04},
/* 42 '*' */{0x00,0x11,0x0E,0x1F,0x0E,0x11,0x00},
/* 43 '+' */{0x00,0x04,0x04,0x1F,0x04,0x04,0x00},
/* 44 ',' */{0x00,0x00,0x00,0x00,0x00,0x04,0x08},
/* 45 '-' */{0x00,0x00,0x00,0x1F,0x00,0x00,0x00},
/* 46 '.' */{0x00,0x00,0x00,0x00,0x00,0x04,0x04},
/* 47 '/' */{0x01,0x01,0x02,0x04,0x08,0x10,0x10},
/* 48 '0' */{0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
/* 49 '1' */{0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
/* 50 '2' */{0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},
/* 51 '3' */{0x0E,0x11,0x01,0x06,0x01,0x11,0x0E},
/* 52 '4' */{0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
/* 53 '5' */{0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
/* 54 '6' */{0x0E,0x10,0x10,0x1E,0x11,0x11,0x0E},
/* 55 '7' */{0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
/* 56 '8' */{0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
/* 57 '9' */{0x0E,0x11,0x11,0x0F,0x01,0x11,0x0E},
/* 58 ':' */{0x00,0x04,0x04,0x00,0x04,0x04,0x00},
/* 59     */{0x00,0x00,0x00,0x00,0x00,0x00,0x00},
/* 60 '<' */{0x02,0x04,0x08,0x10,0x08,0x04,0x02},
/* 61 '=' */{0x00,0x00,0x1F,0x00,0x1F,0x00,0x00},
/* 62 '>' */{0x08,0x04,0x02,0x01,0x02,0x04,0x08},
/* 63 '?' */{0x0E,0x11,0x01,0x06,0x04,0x00,0x04},
/* 64 '@' */{0x00,0x00,0x00,0x00,0x00,0x00,0x00},
/* 65 'A' */{0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},
/* 66 'B' */{0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
/* 67 'C' */{0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
/* 68 'D' */{0x1E,0x11,0x11,0x11,0x11,0x11,0x1E},
/* 69 'E' */{0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},
/* 70 'F' */{0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
/* 71 'G' */{0x0E,0x11,0x10,0x13,0x11,0x11,0x0E},
/* 72 'H' */{0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
/* 73 'I' */{0x1F,0x04,0x04,0x04,0x04,0x04,0x1F},
/* 74 'J' */{0x1F,0x02,0x02,0x02,0x02,0x12,0x0C},
/* 75 'K' */{0x11,0x12,0x14,0x18,0x14,0x12,0x11},
/* 76 'L' */{0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
/* 77 'M' */{0x11,0x1B,0x15,0x11,0x11,0x11,0x11},
/* 78 'N' */{0x11,0x19,0x15,0x13,0x11,0x11,0x11},
/* 79 'O' */{0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
/* 80 'P' */{0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
/* 81 'Q' */{0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},
/* 82 'R' */{0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
/* 83 'S' */{0x0E,0x11,0x10,0x0E,0x01,0x11,0x0E},
/* 84 'T' */{0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
/* 85 'U' */{0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
/* 86 'V' */{0x11,0x11,0x11,0x11,0x11,0x0A,0x04},
/* 87 'W' */{0x11,0x11,0x11,0x15,0x15,0x1B,0x11},
/* 88 'X' */{0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
/* 89 'Y' */{0x11,0x11,0x0A,0x04,0x04,0x04,0x04},
/* 90 'Z' */{0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},
};

static void draw_char(SDL_Renderer *ren, int x, int y, char c,
                      uint8_t r, uint8_t g, uint8_t b, int sc)
{
    if (c < 32 || c > 90) return;
    const uint8_t *gl = FONT[(uint8_t)(c - 32)];
    SDL_SetRenderDrawColor(ren, r, g, b, 255);
    for (int row = 0; row < 7; row++)
        for (int col = 0; col < 5; col++)
            if (gl[row] & (0x10 >> col)) {
                SDL_Rect rc = {x + col*sc, y + row*sc, sc, sc};
                SDL_RenderFillRect(ren, &rc);
            }
}

static void draw_str(SDL_Renderer *ren, int x, int y, const char *s,
                     uint8_t r, uint8_t g, uint8_t b, int sc)
{
    for (; *s; s++, x += 6*sc)
        draw_char(ren, x, y, *s, r, g, b, sc);
}

/* ======================================================================
   Vec3 math
   ====================================================================== */
typedef struct { double x, y, z; } V3;

static inline V3     v3add(V3 a, V3 b)    { return (V3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline V3     v3scl(V3 a, double s){ return (V3){a.x*s, a.y*s, a.z*s}; }
static inline double v3dot(V3 a, V3 b)    { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline double v3len(V3 a)           { return sqrt(v3dot(a,a)); }
static inline V3     v3norm(V3 a)         { double l = v3len(a); return v3scl(a,1.0/l); }

static V3 rx(V3 v, double a) {
    double c = cos(a), s = sin(a);
    return (V3){v.x, c*v.y - s*v.z, s*v.y + c*v.z};
}
static V3 ry(V3 v, double a) __attribute__((unused));
static V3 ry(V3 v, double a) {
    double c = cos(a), s = sin(a);
    return (V3){c*v.x + s*v.z, v.y, -s*v.x + c*v.z};
}
static V3 rz(V3 v, double a) {
    double c = cos(a), s = sin(a);
    return (V3){c*v.x - s*v.y, s*v.x + c*v.y, v.z};
}

/* P_view = Rx(-pitch) * Rz(-yaw) * P_eci */
static V3 eci_to_view(V3 p, double yaw, double pitch) {
    return rx(rz(p, -yaw), -pitch);
}
static V3 view_to_eci(V3 p, double yaw, double pitch) {
    return rz(rx(p, pitch), yaw);
}

static int px_x(V3 v, double Rpx) { return (int)(CX + Rpx * v.x / RE); }
static int px_y(V3 v, double Rpx) { return (int)(CY - Rpx * v.y / RE); }

static int occluded(V3 v, double Rpx) {
    (void)Rpx;
    double nx = v.x/RE, ny = v.y/RE;
    double r2 = nx*nx + ny*ny;
    if (r2 >= 1.0) return 0;
    return v.z < sqrt(1.0 - r2) * RE;
}

/* ======================================================================
   Continent polygons  (lon, lat)
   ====================================================================== */
#define POLY(name) static const double name[][2] =

POLY(NA) {
    {-168,71},{-152,71},{-135,63},{-115,60},{-115,49},
    {-125,48},{-117,33},{-105,22},{-90,15},{-78,8},
    {-83,10},{-87,16},{-87,21},{-93,19},{-97,26},
    {-90,30},{-80,25},{-80,30},{-75,35},{-74,41},
    {-67,45},{-60,47},{-55,52},{-65,58},{-64,63},
    {-83,63},{-168,71}
};
#define NA_N 27

POLY(SA) {
    {-78,8},{-80,-2},{-80,-5},{-75,-14},{-76,-18},
    {-70,-33},{-72,-50},{-68,-55},{-58,-38},{-52,-33},
    {-42,-23},{-35,-5},{-35,-10},{-44,0},{-50,5},
    {-60,8},{-63,11},{-72,12},{-75,10},{-78,8}
};
#define SA_N 20

POLY(EU) {
    {-10,36},{5,36},{15,38},{28,37},{40,38},
    {42,42},{40,48},{28,56},{25,65},{28,70},
    {18,72},{14,70},{5,58},{-2,57},{-8,44},
    {-10,38},{-10,36}
};
#define EU_N 17

POLY(AF) {
    {-18,15},{-18,5},{-5,5},{10,5},{10,-5},
    {14,-12},{18,-25},{26,-35},{18,-35},{10,-28},
    {5,-18},{2,-8},{10,2},{40,12},{44,11},
    {48,12},{43,22},{38,22},{33,30},{25,31},
    {10,33},{0,37},{-5,35},{-5,30},{-18,21},{-18,15}
};
#define AF_N 26

POLY(AS) {
    {26,42},{36,37},{50,30},{57,22},{68,23},
    {80,22},{91,22},{97,17},{100,2},{104,-5},
    {110,-7},{120,-10},{122,15},{122,22},{122,30},
    {130,32},{140,35},{140,38},{135,48},{140,52},
    {160,60},{163,67},{168,55},{150,46},{145,44},
    {142,56},{140,72},{120,72},{100,73},{80,73},
    {60,73},{55,70},{48,70},{40,72},{32,70},
    {28,70},{26,68},{22,65},{28,57},{26,42}
};
#define AS_N 40

POLY(AU) {
    {114,-22},{124,-14},{136,-12},{142,-11},
    {150,-23},{153,-28},{151,-34},{148,-38},
    {140,-36},{135,-35},{125,-34},{114,-22}
};
#define AU_N 12

POLY(GR) {
    {-44,83},{-14,83},{-14,76},{-20,65},
    {-38,60},{-44,60},{-54,68},{-58,77},{-44,83}
};
#define GR_N 9

POLY(UK) {
    {-6,50},{0,51},{2,53},{0,55},{-2,55},
    {-5,58},{-6,57},{-3,55},{-5,52},{-6,50}
};
#define UK_N 10

/* ======================================================================
   Land mask (720x360, 0.5 deg/cell)
   ====================================================================== */
#define LM_LAT 360
#define LM_LON 720
static uint8_t land_mask[LM_LAT][LM_LON];

static int pip(double lon, double lat, const double poly[][2], int n) {
    int inside = 0;
    for (int i = 0, j = n-1; i < n; j = i++) {
        double xi = poly[i][0], yi = poly[i][1];
        double xj = poly[j][0], yj = poly[j][1];
        if (((yi > lat) != (yj > lat)) &&
            lon < (xj - xi) * (lat - yi) / (yj - yi) + xi)
            inside = !inside;
    }
    return inside;
}

static int point_on_land(double lon, double lat) {
    if (lat < -65.0) return 1;
    if (pip(lon,lat,NA,NA_N)) return 1;
    if (pip(lon,lat,SA,SA_N)) return 1;
    if (pip(lon,lat,EU,EU_N)) return 1;
    if (pip(lon,lat,AF,AF_N)) return 1;
    if (pip(lon,lat,AS,AS_N)) return 1;
    if (pip(lon,lat,AU,AU_N)) return 1;
    if (pip(lon,lat,GR,GR_N)) return 1;
    if (pip(lon,lat,UK,UK_N)) return 1;
    return 0;
}

static void build_land_mask(void) {
    for (int la = 0; la < LM_LAT; la++) {
        double lat = (la + 0.5) * 0.5 - 90.0;
        for (int lo = 0; lo < LM_LON; lo++) {
            double lon = (lo + 0.5) * 0.5 - 180.0;
            land_mask[la][lo] = (uint8_t)point_on_land(lon, lat);
        }
    }
}

static uint8_t lm_lookup(double lat_rad, double lon_rad) {
    double lat = lat_rad * (180.0/PI);
    double lon = lon_rad * (180.0/PI);
    int la = (int)((lat + 90.0) * 2.0);
    int lo = (int)((lon + 180.0) * 2.0);
    if (la <       0) la = 0;
    if (la >= LM_LAT) la = LM_LAT-1;
    if (lo <       0) lo = 0;
    if (lo >= LM_LON) lo = LM_LON-1;
    return land_mask[la][lo];
}

/* ======================================================================
   Orbital elements
   ====================================================================== */
typedef struct {
    double a, e, inc, Omega, omega, M0;
    const char *name;
    uint8_t cr, cg, cb;
} OrbEl;

static double kepler(double M, double e) {
    double E = M;
    for (int i = 0; i < 60; i++) {
        double dE = (M - E + e*sin(E)) / (1.0 - e*cos(E));
        E += dE;
        if (fabs(dE) < 1e-13) break;
    }
    return E;
}

static V3 orbit_pos_eci(const OrbEl *el, double nu) {
    double cO = cos(el->Omega), sO = sin(el->Omega);
    double co = cos(el->omega), so = sin(el->omega);
    double ci = cos(el->inc),   si = sin(el->inc);
    double Px =  cO*co - sO*so*ci,  Py = sO*co + cO*so*ci,  Pz = so*si;
    double Qx = -cO*so - sO*co*ci,  Qy =-sO*so + cO*co*ci,  Qz = co*si;
    double p   = el->a * (1.0 - el->e*el->e);
    double r   = p / (1.0 + el->e*cos(nu));
    double xo  = r*cos(nu), yo = r*sin(nu);
    return (V3){Px*xo+Qx*yo, Py*xo+Qy*yo, Pz*xo+Qz*yo};
}

static void el_to_state(const OrbEl *el, V3 *pos, V3 *vel) {
    double E  = kepler(el->M0, el->e);
    double nu = 2.0*atan2(sqrt(1.0+el->e)*sin(E/2.0),
                           sqrt(1.0-el->e)*cos(E/2.0));
    double cO = cos(el->Omega), sO = sin(el->Omega);
    double co = cos(el->omega), so = sin(el->omega);
    double ci = cos(el->inc),   si = sin(el->inc);
    double Px =  cO*co - sO*so*ci,  Py = sO*co + cO*so*ci,  Pz = so*si;
    double Qx = -cO*so - sO*co*ci,  Qy =-sO*so + cO*co*ci,  Qz = co*si;
    double p   = el->a * (1.0 - el->e*el->e);
    double r   = p / (1.0 + el->e*cos(nu));
    double xo  = r*cos(nu), yo = r*sin(nu);
    *pos = (V3){Px*xo+Qx*yo, Py*xo+Qy*yo, Pz*xo+Qz*yo};
    double h   = sqrt(GM * p);
    double vr  = (GM/h) * el->e * sin(nu);
    double vt  = (GM/h) * (1.0 + el->e*cos(nu));
    double vxo = vr*cos(nu) - vt*sin(nu);
    double vyo = vr*sin(nu) + vt*cos(nu);
    *vel = (V3){Px*vxo+Qx*vyo, Py*vxo+Qy*vyo, Pz*vxo+Qz*vyo};
}

/* ======================================================================
   Satellite state + Velocity Verlet with optional J2
   ====================================================================== */
typedef struct {
    V3  pos, vel;
    const OrbEl *el;
    V3  trail[TRAIL_LEN];
    int trail_head, trail_cnt, trail_ctr;
    int visible;
} Sat;

static void sat_init(Sat *s, const OrbEl *el) {
    s->el         = el;
    el_to_state(el, &s->pos, &s->vel);
    s->trail_head = 0;
    s->trail_cnt  = 0;
    s->trail_ctr  = 0;
    s->visible    = 1;
}

/* Gravitational acceleration with optional J2 zonal harmonic */
static V3 gravity(V3 pos, int use_j2) {
    double r2 = v3dot(pos, pos);
    double r  = sqrt(r2);
    V3 a      = v3scl(pos, -GM / (r2 * r));
    if (use_j2) {
        double z2r2 = (pos.z * pos.z) / r2;
        double fac  = 1.5 * J2_COEFF * GM * RE*RE / (r2 * r2 * r);
        a.x += fac * pos.x * (1.0 - 5.0*z2r2);
        a.y += fac * pos.y * (1.0 - 5.0*z2r2);
        a.z += fac * pos.z * (3.0 - 5.0*z2r2);
    }
    return a;
}

static void sat_step(Sat *s, double dt, int use_j2) {
    /* Velocity Verlet */
    V3 a0  = gravity(s->pos, use_j2);
    s->vel = v3add(s->vel, v3scl(a0, 0.5*dt));
    s->pos = v3add(s->pos, v3scl(s->vel, dt));
    V3 a1  = gravity(s->pos, use_j2);
    s->vel = v3add(s->vel, v3scl(a1, 0.5*dt));

    /* Trail sample */
    if (++s->trail_ctr >= TRAIL_STEP) {
        s->trail_ctr = 0;
        s->trail[s->trail_head] = s->pos;
        s->trail_head = (s->trail_head + 1) % TRAIL_LEN;
        if (s->trail_cnt < TRAIL_LEN) s->trail_cnt++;
    }
}

/* ======================================================================
   Stars
   ====================================================================== */
#define N_STARS 250
static int     star_x[N_STARS], star_y[N_STARS];
static uint8_t star_b[N_STARS];

static void build_stars(void) {
    srand(42);
    for (int i = 0; i < N_STARS; i++) {
        int x, y;
        do {
            x = rand() % WIN_W;
            y = rand() % WIN_H;
            double dx = x-CX, dy = y-CY;
            if (dx*dx + dy*dy > (BASE_R+30)*(BASE_R+30)) break;
        } while (1);
        star_x[i] = x;  star_y[i] = y;
        star_b[i]  = (uint8_t)(80 + rand() % 176);
    }
}

/* ======================================================================
   Orbit definitions
   ====================================================================== */
static const OrbEl ORBITS[NUM_SATS] = {
    { 42164000.0, 0.000,  0.0*DEG,   0.0*DEG,   0.0*DEG,   0.0*DEG, "GEO",     0,  255, 255 },
    { 42164000.0, 0.000, 28.0*DEG,  45.0*DEG,   0.0*DEG, 120.0*DEG, "GEOSYNC", 255, 255, 255 },
    { 26560000.0, 0.740, 63.4*DEG,   0.0*DEG, 270.0*DEG, 180.0*DEG, "MOLNIYA", 255,  50,  50 },
    {  6778000.0, 0.000, 51.6*DEG,  45.0*DEG,   0.0*DEG,  90.0*DEG, "ISS",     255, 255,   0 },
    {  7000000.0, 0.000, 90.0*DEG,  90.0*DEG,   0.0*DEG,   0.0*DEG, "POLAR",    50, 255,  50 },
    { 26560000.0, 0.010, 55.0*DEG, 160.0*DEG,   0.0*DEG, 240.0*DEG, "MEO/GPS", 255,  50, 255 },
    { 42164000.0, 0.270, 63.4*DEG,   0.0*DEG, 270.0*DEG, 270.0*DEG, "TUNDRA",  255, 165,   0 },
    {  7178000.0, 0.000, 98.7*DEG, 130.0*DEG,   0.0*DEG, 180.0*DEG, "SSO",     255, 150, 200 },
};

/* ======================================================================
   Earth sphere rendering (per-pixel raycast into streaming texture)
   ====================================================================== */
static void render_earth(uint32_t *px, int stride,
                          double Rpx, double view_yaw, double view_pitch,
                          double earth_angle)
{
    int iy0 = (int)(CY - Rpx - 2), iy1 = (int)(CY + Rpx + 2);
    int ix0 = (int)(CX - Rpx - 2), ix1 = (int)(CX + Rpx + 2);
    if (iy0 <      0) iy0 = 0;
    if (iy1 >= WIN_H) iy1 = WIN_H-1;
    if (ix0 <      0) ix0 = 0;
    if (ix1 >= WIN_W) ix1 = WIN_W-1;

    for (int py = iy0; py <= iy1; py++) {
        double ndy = (CY - py) / Rpx;
        for (int pxx = ix0; pxx <= ix1; pxx++) {
            double ndx = (pxx - CX) / Rpx;
            double r2  = ndx*ndx + ndy*ndy;
            uint32_t pixel = 0xFF000000;

            if (r2 <= 1.0) {
                double nz      = sqrt(1.0 - r2);
                V3 N_view      = {ndx, ndy, nz};
                V3 N_eci       = view_to_eci(N_view, view_yaw, view_pitch);
                V3 N_earth     = rz(N_eci, -earth_angle);
                double lat     = asin(fmax(-1.0, fmin(1.0, N_earth.z)));
                double lon     = atan2(N_earth.y, N_earth.x);
                int is_land    = lm_lookup(lat, lon);
                double diffuse = fmax(0.0, N_eci.x);
                double light   = 0.22 + 0.78 * diffuse;
                double lat_deg = lat * (180.0/PI);
                double abslat  = fabs(lat_deg);
                double fr, fg, fb;

                if (is_land) {
                    if (abslat > 65.0) {
                        fr = 220; fg = 225; fb = 240;
                    } else {
                        double t = fmax(0.0, 1.0 - abslat/65.0);
                        fr = 35 + 20*(1-t); fg = 100 + 30*t; fb = 20;
                    }
                } else {
                    fr = 5; fg = 40; fb = 130;
                }

                fr *= light; fg *= light; fb *= light;

                if (abslat > 70.0) {
                    double f = (abslat - 70.0) / 20.0;
                    fr += f*(230-fr); fg += f*(235-fg); fb += f*(245-fb);
                }

                double lo30 = fmod(fabs(lon*(180.0/PI)), 30.0);
                double la30 = fmod(abslat, 30.0);
                if (la30 < 0.7 || lo30 < 0.7) {
                    fr *= 0.75; fg *= 0.75; fb = fb*0.75 + 15;
                }

                double glow = (1.0 - nz);
                glow = glow*glow*glow * 0.18;
                fb += glow*180; fg += glow*80; fr += glow*20;

                pixel = 0xFF000000
                      | ((uint32_t)(uint8_t)fmin(255,fr) << 16)
                      | ((uint32_t)(uint8_t)fmin(255,fg) <<  8)
                      |  (uint32_t)(uint8_t)fmin(255,fb);

            } else if (r2 <= 1.055) {
                double rl   = sqrt(r2);
                double glow = (1.055 - rl) / 0.055;
                glow = glow*glow;
                pixel = 0xFF000000
                      | ((uint32_t)(uint8_t)(glow*10) << 16)
                      | ((uint32_t)(uint8_t)(glow*35) <<  8)
                      |  (uint32_t)(uint8_t)(glow*70);
            }

            px[py * stride + pxx] = pixel;
        }
    }
}

/* ======================================================================
   Orbit path (analytical)
   ====================================================================== */
static void render_orbit(SDL_Renderer *ren, const OrbEl *el,
                          double Rpx, double view_yaw, double view_pitch)
{
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    int psx = 0, psy = 0, pocc = 0;
    for (int j = 0; j <= ORBIT_PTS; j++) {
        double nu  = j * (2.0*PI / ORBIT_PTS);
        V3 pv      = eci_to_view(orbit_pos_eci(el, nu), view_yaw, view_pitch);
        int sx     = px_x(pv, Rpx);
        int sy     = px_y(pv, Rpx);
        int occ    = occluded(pv, Rpx);
        if (j > 0) {
            SDL_SetRenderDrawColor(ren, el->cr, el->cg, el->cb,
                                   (uint8_t)(occ && pocc ? 40 : 130));
            SDL_RenderDrawLine(ren, psx, psy, sx, sy);
        }
        psx = sx; psy = sy; pocc = occ;
    }
}

/* ======================================================================
   Ground track: sub-satellite point on sphere + nadir line
   ====================================================================== */
static void render_ground_tracks(SDL_Renderer *ren,
                                  const Sat sats[NUM_SATS],
                                  double Rpx,
                                  double view_yaw, double view_pitch,
                                  double earth_angle)
{
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    for (int i = 0; i < NUM_SATS; i++) {
        if (!sats[i].visible) continue;

        /* Sub-satellite point: satellite ECI pos → Earth-fixed unit → back to ECI at RE */
        V3 ef   = rz(v3norm(sats[i].pos), -earth_angle);
        V3 surf = v3scl(rz(ef, earth_angle), RE);
        V3 sv   = eci_to_view(surf, view_yaw, view_pitch);

        /* Only draw if ground point faces camera */
        if (sv.z <= 0.0) continue;

        int gx = px_x(sv, Rpx);
        int gy = px_y(sv, Rpx);
        if (gx < 3 || gx >= WIN_W-3 || gy < 3 || gy >= WIN_H-3) continue;

        const OrbEl *e = sats[i].el;

        /* Nadir line */
        V3 satv = eci_to_view(sats[i].pos, view_yaw, view_pitch);
        if (!occluded(satv, Rpx)) {
            SDL_SetRenderDrawColor(ren, e->cr, e->cg, e->cb, 55);
            SDL_RenderDrawLine(ren, px_x(satv, Rpx), px_y(satv, Rpx), gx, gy);
        }

        /* Ground track dot (coloured ring) */
        SDL_SetRenderDrawColor(ren, e->cr, e->cg, e->cb, 220);
        SDL_Rect outer = {gx-3, gy-3, 7, 7};
        SDL_RenderFillRect(ren, &outer);
        SDL_SetRenderDrawColor(ren, 0, 0, 0, 220);
        SDL_Rect inner = {gx-1, gy-1, 3, 3};
        SDL_RenderFillRect(ren, &inner);
    }
}

/* ======================================================================
   Trail + satellite dot
   ====================================================================== */
static void render_satellite(SDL_Renderer *ren, const Sat *s,
                              double Rpx, double view_yaw, double view_pitch)
{
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    for (int k = 0; k < s->trail_cnt; k++) {
        int idx = (s->trail_head - 1 - k + TRAIL_LEN) % TRAIL_LEN;
        V3 tv = eci_to_view(s->trail[idx], view_yaw, view_pitch);
        if (occluded(tv, Rpx)) continue;
        int tsx = px_x(tv, Rpx), tsy = px_y(tv, Rpx);
        if (tsx < 0 || tsx >= WIN_W || tsy < 0 || tsy >= WIN_H) continue;
        float f = 1.0f - (float)k / (float)s->trail_cnt;
        SDL_SetRenderDrawColor(ren,
            (uint8_t)(s->el->cr * f),
            (uint8_t)(s->el->cg * f),
            (uint8_t)(s->el->cb * f),
            (uint8_t)(f*f*180));
        SDL_RenderDrawPoint(ren, tsx, tsy);
    }

    V3 sv = eci_to_view(s->pos, view_yaw, view_pitch);
    if (occluded(sv, Rpx)) return;
    int sx = px_x(sv, Rpx), sy = px_y(sv, Rpx);
    if (sx < 4 || sx >= WIN_W-4 || sy < 4 || sy >= WIN_H-4) return;

    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_NONE);
    SDL_SetRenderDrawColor(ren, s->el->cr, s->el->cg, s->el->cb, 255);
    SDL_RenderDrawLine(ren, sx-4, sy,   sx+4, sy);
    SDL_RenderDrawLine(ren, sx,   sy-4, sx,   sy+4);
    SDL_RenderDrawLine(ren, sx-3, sy-3, sx+3, sy+3);
    SDL_RenderDrawLine(ren, sx+3, sy-3, sx-3, sy+3);
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
    SDL_Rect dot = {sx-1, sy-1, 3, 3};
    SDL_RenderFillRect(ren, &dot);
}

/* ======================================================================
   HUD
   ====================================================================== */
static void render_hud(SDL_Renderer *ren,
                        const Sat sats[NUM_SATS],
                        double time_scale, double sim_time,
                        int paused, int use_j2,
                        int show_gt, int recording,
                        uint32_t frame_count)
{
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(ren, 0, 0, 0, 180);
    SDL_Rect bg = {WIN_W-200, 10, 190, NUM_SATS*26 + 72};
    SDL_RenderFillRect(ren, &bg);

    draw_str(ren, WIN_W-196, 14, "ORBITS", 200, 200, 200, 1);

    for (int i = 0; i < NUM_SATS; i++) {
        int y = 28 + i*26;
        const OrbEl *e = sats[i].el;
        SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_NONE);
        SDL_SetRenderDrawColor(ren,
            sats[i].visible ? e->cr : 60,
            sats[i].visible ? e->cg : 60,
            sats[i].visible ? e->cb : 60, 255);
        SDL_Rect sw = {WIN_W-196, y+1, 12, 9};
        SDL_RenderFillRect(ren, &sw);
        char label[32];
        snprintf(label, sizeof(label), "%d %s", i+1, e->name);
        draw_str(ren, WIN_W-180, y,
                 label,
                 sats[i].visible ? e->cr : 80,
                 sats[i].visible ? e->cg : 80,
                 sats[i].visible ? e->cb : 80, 1);
    }

    /* Toggle status row */
    int ty = 28 + NUM_SATS*26 + 6;
    draw_str(ren, WIN_W-196, ty, "J2:", 150, 150, 150, 1);
    draw_str(ren, WIN_W-172, ty,
             use_j2 ? "ON " : "OFF",
             use_j2 ? 80  : 150,
             use_j2 ? 210 : 150,
             use_j2 ? 80  : 150, 1);
    draw_str(ren, WIN_W-140, ty, "GT:", 150, 150, 150, 1);
    draw_str(ren, WIN_W-116, ty,
             show_gt ? "ON " : "OFF",
             show_gt ? 80  : 150,
             show_gt ? 210 : 150,
             show_gt ? 80  : 150, 1);

    /* Status bar */
    char buf[160];
    double days  = sim_time / 86400.0;
    double hours = fmod(sim_time, 86400.0) / 3600.0;
    snprintf(buf, sizeof(buf), "SPEED: %gX", time_scale);
    draw_str(ren, 10, WIN_H-22, buf, 180, 220, 180, 1);
    snprintf(buf, sizeof(buf), "SIM: %4.0fD %05.2fH", days, hours);
    draw_str(ren, 160, WIN_H-22, buf, 180, 180, 220, 1);

    if (paused) draw_str(ren, 420, WIN_H-22, "PAUSED", 255, 200, 50, 2);

    /* Blinking REC indicator */
    if (recording) {
        int blink = (frame_count / 30) & 1;
        draw_str(ren, WIN_W-200, WIN_H-22, "REC",
                 255, blink ? 50 : 0, blink ? 50 : 0, 1);
    }

    /* Controls hints */
    draw_str(ren, 10, WIN_H-50,
             "+/- SPEED   R RESET   SPC PAUSE   1-8 TOGGLE   SCROLL ZOOM",
             85, 85, 85, 1);
    draw_str(ren, 10, WIN_H-38,
             "J J2   G GROUND TRACK   V VIDEO CAPTURE   Q/ESC QUIT",
             85, 85, 85, 1);
}

/* ======================================================================
   ffmpeg video capture
   ====================================================================== */
static FILE   *ffmpeg_pipe = NULL;
static uint8_t frame_buf[WIN_W * WIN_H * 4];   /* ARGB read buffer */

static int video_start(void) {
    char fname[80], cmd[512];
    time_t t = time(NULL);
    struct tm *tm = localtime(&t);
    strftime(fname, sizeof(fname), "earth_orbit_%Y%m%d_%H%M%S.mp4", tm);
    snprintf(cmd, sizeof(cmd),
        "ffmpeg -y -f rawvideo -pixel_format rgb24 "
        "-video_size %dx%d -framerate 60 -i pipe:0 "
        "-c:v libx264 -preset fast -crf 18 -pix_fmt yuv420p "
        "'%s' 2>/dev/null",
        WIN_W, WIN_H, fname);
    ffmpeg_pipe = popen(cmd, "w");
    if (!ffmpeg_pipe) {
        fprintf(stderr, "video_start: popen failed — is ffmpeg in PATH?\n");
        return 0;
    }
    printf("Recording to: %s\n", fname);
    return 1;
}

static void video_stop(void) {
    if (!ffmpeg_pipe) return;
    pclose(ffmpeg_pipe);
    ffmpeg_pipe = NULL;
    printf("Recording stopped.\n");
}

/* Read current renderer content and pipe RGB24 to ffmpeg.
   SDL_RenderReadPixels is slow but fine for 60 fps at 1200x780. */
static void video_frame(SDL_Renderer *ren) {
    if (!ffmpeg_pipe) return;
    /* Read as RGB24 directly — SDL packs 3 bytes per pixel */
    SDL_RenderReadPixels(ren, NULL, SDL_PIXELFORMAT_RGB24,
                         frame_buf, WIN_W * 3);
    fwrite(frame_buf, (size_t)WIN_W * WIN_H * 3, 1, ffmpeg_pipe);
}

/* ======================================================================
   Main
   ====================================================================== */
int main(void) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *win = SDL_CreateWindow(
        "Earth Satellite Orbit Simulation",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIN_W, WIN_H, SDL_WINDOW_SHOWN);
    if (!win) { fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError()); return 1; }

    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!ren) { fprintf(stderr, "SDL_CreateRenderer: %s\n", SDL_GetError()); return 1; }

    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);

    SDL_Texture *earth_tex = SDL_CreateTexture(ren,
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        WIN_W, WIN_H);
    if (!earth_tex) { fprintf(stderr, "SDL_CreateTexture: %s\n", SDL_GetError()); return 1; }

    printf("Building land mask..."); fflush(stdout);
    build_land_mask();
    printf(" done\n");
    build_stars();

    Sat sats[NUM_SATS];
    for (int i = 0; i < NUM_SATS; i++) sat_init(&sats[i], &ORBITS[i]);

    double view_yaw    = 0.2;
    double view_pitch  = 0.45;
    double zoom        = ZOOM_INIT;
    double earth_angle = 0.0;
    double sim_time    = 0.0;
    double sim_acc     = 0.0;
    double time_scale  = 800.0;

    int paused            = 0;
    int dragging          = 0;
    int mx_prev           = 0, my_prev = 0;
    int use_j2            = 0;
    int show_ground_track = 0;
    int recording         = 0;
    uint32_t frame_count  = 0;

    uint32_t t_prev = SDL_GetTicks();
    int running = 1;
    SDL_Event ev;

    while (running) {
        /* ── Events ──────────────────────────────────────── */
        while (SDL_PollEvent(&ev)) {
            switch (ev.type) {
            case SDL_QUIT: running = 0; break;
            case SDL_KEYDOWN:
                switch (ev.key.keysym.sym) {
                case SDLK_ESCAPE: case SDLK_q:
                    running = 0; break;
                case SDLK_SPACE:
                    paused = !paused; break;
                case SDLK_r:
                    view_yaw = 0.2; view_pitch = 0.45; zoom = ZOOM_INIT; break;
                case SDLK_EQUALS: case SDLK_PLUS:
                    time_scale = fmin(time_scale * 2.0, 1e6); break;
                case SDLK_MINUS:
                    time_scale = fmax(time_scale / 2.0, 1.0); break;
                case SDLK_j:
                    use_j2 = !use_j2;
                    printf("J2: %s\n", use_j2 ? "ON" : "OFF"); break;
                case SDLK_g:
                    show_ground_track = !show_ground_track; break;
                case SDLK_v:
                    if (!recording) { if (video_start()) recording = 1; }
                    else            { video_stop(); recording = 0; }
                    break;
                case SDLK_1: case SDLK_2: case SDLK_3: case SDLK_4:
                case SDLK_5: case SDLK_6: case SDLK_7: case SDLK_8: {
                    int idx = ev.key.keysym.sym - SDLK_1;
                    if (idx < NUM_SATS) sats[idx].visible ^= 1;
                    break; }
                }
                break;
            case SDL_MOUSEBUTTONDOWN:
                if (ev.button.button == SDL_BUTTON_LEFT) {
                    dragging = 1; mx_prev = ev.button.x; my_prev = ev.button.y;
                }
                break;
            case SDL_MOUSEBUTTONUP:
                if (ev.button.button == SDL_BUTTON_LEFT) dragging = 0;
                break;
            case SDL_MOUSEMOTION:
                if (dragging) {
                    view_yaw   += (ev.motion.x - mx_prev) * 0.006;
                    view_pitch += (ev.motion.y - my_prev) * 0.006;
                    view_pitch  = fmax(-PI/2.0+0.05, fmin(PI/2.0-0.05, view_pitch));
                    mx_prev = ev.motion.x; my_prev = ev.motion.y;
                }
                break;
            case SDL_MOUSEWHEEL:
                zoom *= (ev.wheel.y > 0) ? 1.12 : (1.0/1.12);
                zoom  = fmax(ZOOM_MIN, fmin(ZOOM_MAX, zoom));
                break;
            }
        }

        /* ── Physics ─────────────────────────────────────── */
        uint32_t t_now   = SDL_GetTicks();
        double   dt_real = (t_now - t_prev) * 0.001;
        if (dt_real > 0.1) dt_real = 0.1;
        t_prev = t_now;

        if (!paused) {
            sim_acc += time_scale * dt_real;
            int steps = 0;
            while (sim_acc >= DT_PHYS && steps < 600) {
                for (int i = 0; i < NUM_SATS; i++)
                    if (sats[i].visible) sat_step(&sats[i], DT_PHYS, use_j2);
                earth_angle += OMEGA_E * DT_PHYS;
                sim_time    += DT_PHYS;
                sim_acc     -= DT_PHYS;
                steps++;
            }
        }

        /* ── Render ──────────────────────────────────────── */
        SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
        SDL_RenderClear(ren);

        for (int i = 0; i < N_STARS; i++) {
            SDL_SetRenderDrawColor(ren, star_b[i], star_b[i], star_b[i], 255);
            SDL_RenderDrawPoint(ren, star_x[i], star_y[i]);
        }

        double Rpx = BASE_R * zoom;
        void *raw; int pitch;
        SDL_LockTexture(earth_tex, NULL, &raw, &pitch);
        memset(raw, 0, (size_t)pitch * WIN_H);
        render_earth((uint32_t*)raw, pitch/4,
                     Rpx, view_yaw, view_pitch, earth_angle);
        SDL_UnlockTexture(earth_tex);
        SDL_RenderCopy(ren, earth_tex, NULL, NULL);

        for (int i = 0; i < NUM_SATS; i++)
            if (sats[i].visible)
                render_orbit(ren, sats[i].el, Rpx, view_yaw, view_pitch);

        if (show_ground_track)
            render_ground_tracks(ren, sats, Rpx, view_yaw, view_pitch, earth_angle);

        for (int i = 0; i < NUM_SATS; i++)
            if (sats[i].visible)
                render_satellite(ren, &sats[i], Rpx, view_yaw, view_pitch);

        render_hud(ren, sats, time_scale, sim_time,
                   paused, use_j2, show_ground_track, recording, frame_count);

        if (recording) video_frame(ren);

        SDL_RenderPresent(ren);
        frame_count++;
    }

    if (recording) video_stop();
    SDL_DestroyTexture(earth_tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
