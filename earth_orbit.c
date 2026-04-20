// earth_orbit.c  —  Satellite Orbital Mechanics Simulation
// Build:  gcc -O2 -std=c17 -o earth_orbit earth_orbit.c $(sdl2-config --cflags --libs) -lm
//
// Controls:
//   Left-drag    rotate view (3D mode)
//   Scroll       zoom in/out (3D mode)
//   +/=          faster
//   -            slower
//   1-8          toggle satellite
//   R            reset view
//   Space        pause / resume
//   J            toggle J2 oblateness perturbation
//   G            toggle ground track display (3D mode)
//   P            toggle Mercator map / 3D sphere view
//   V            start/stop MP4 video capture  (requires ffmpeg in PATH)
//   Q / Esc      quit

#include <SDL2/SDL.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "map_data.h"   /* Natural Earth 1:50m coastlines + borders (gen_map_data.py) */

/* ======================================================================
   Constants
   ====================================================================== */
#define WIN_W    1200
#define WIN_H     780
#define CX        480
#define CY        390
#define BASE_R    270.0
#define ZOOM_INIT   0.15
#define ZOOM_MIN    0.04
#define ZOOM_MAX    8.0

#define RE        6371000.0
#define GM        3.986004418e14
#define OMEGA_E   7.2921150e-5
#define J2_COEFF  1.08263e-3
#define PI        3.14159265358979323846
#define DEG       (PI/180.0)

#define DT_PHYS   20.0
#define NUM_SATS   8
#define TRAIL_LEN  500
#define TRAIL_STEP   5
#define ORBIT_PTS  7200        /* 20 pts/deg — sub-2px segments at max zoom (8×) */

/* Ground track history — 1500 samples × 100 s/sample ≈ 41.7 h.
   Enough for ≥1 full orbit of every satellite type.
   GT_BREAK is a sentinel lat value written into the ring buffer when a
   satellite is re-enabled, so the drawing code can skip that segment.  */
#define GT_LEN    1500
#define GT_BREAK  99.0f   /* sentinel: valid lat range is ±π/2 ≈ ±1.57 */

/* Mercator map clipped to ±82° latitude, right edge leaves room for panel */
#define MAP_LAT_CLIP (82.0 * DEG)
#define MAP_W        (WIN_W - 200)   /* map viewport width, panel sits right of this */

/* ======================================================================
   5×7 bitmap font
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

static inline V3     v3add(V3 a, V3 b)    { return (V3){a.x+b.x,a.y+b.y,a.z+b.z}; }
static inline V3     v3scl(V3 a, double s){ return (V3){a.x*s,a.y*s,a.z*s}; }
static inline double v3dot(V3 a, V3 b)    { return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline double v3len(V3 a)           { return sqrt(v3dot(a,a)); }
static inline V3     v3norm(V3 a)         { double l=v3len(a); return v3scl(a,1.0/l); }
static inline V3     v3cross(V3 a, V3 b)  { return (V3){a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }

static V3 rx(V3 v, double a) {
    double c=cos(a),s=sin(a);
    return (V3){v.x, c*v.y-s*v.z, s*v.y+c*v.z};
}
static V3 ry(V3 v, double a) __attribute__((unused));
static V3 ry(V3 v, double a) {
    double c=cos(a),s=sin(a);
    return (V3){c*v.x+s*v.z, v.y, -s*v.x+c*v.z};
}
static V3 rz(V3 v, double a) {
    double c=cos(a),s=sin(a);
    return (V3){c*v.x-s*v.y, s*v.x+c*v.y, v.z};
}

static V3 eci_to_view(V3 p, double yaw, double pitch) {
    return rx(rz(p,-yaw),-pitch);
}
static V3 view_to_eci(V3 p, double yaw, double pitch) {
    return rz(rx(p,pitch),yaw);
}

static int px_x(V3 v, double Rpx) { return (int)(CX + Rpx*v.x/RE); }
static int px_y(V3 v, double Rpx) { return (int)(CY - Rpx*v.y/RE); }

static int occluded(V3 v, double Rpx) {
    (void)Rpx;
    double nx=v.x/RE, ny=v.y/RE, r2=nx*nx+ny*ny;
    if (r2>=1.0) return 0;
    return v.z < sqrt(1.0-r2)*RE;
}

/* ======================================================================
   Continent polygons  (lon, lat in degrees)
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
   Land mask (720×360, 0.5°/cell)
   ====================================================================== */
#define LM_LAT 360
#define LM_LON 720
static uint8_t land_mask[LM_LAT][LM_LON];

static int pip(double lon, double lat, const double poly[][2], int n) {
    int inside=0;
    for (int i=0,j=n-1; i<n; j=i++) {
        double xi=poly[i][0],yi=poly[i][1],xj=poly[j][0],yj=poly[j][1];
        if (((yi>lat)!=(yj>lat)) && lon<(xj-xi)*(lat-yi)/(yj-yi)+xi)
            inside=!inside;
    }
    return inside;
}

static int point_on_land(double lon, double lat) {
    if (lat<-65.0) return 1;
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
    for (int la=0; la<LM_LAT; la++) {
        double lat=(la+0.5)*0.5-90.0;
        for (int lo=0; lo<LM_LON; lo++) {
            double lon=(lo+0.5)*0.5-180.0;
            land_mask[la][lo]=(uint8_t)point_on_land(lon,lat);
        }
    }
}

static uint8_t lm_lookup(double lat_rad, double lon_rad) {
    double lat=lat_rad*(180.0/PI), lon=lon_rad*(180.0/PI);
    int la=(int)((lat+90.0)*2.0), lo=(int)((lon+180.0)*2.0);
    if (la<0)       la=0;
    if (la>=LM_LAT) la=LM_LAT-1;
    if (lo<0)       lo=0;
    if (lo>=LM_LON) lo=LM_LON-1;
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
    double E=M;
    for (int i=0; i<60; i++) {
        double dE=(M-E+e*sin(E))/(1.0-e*cos(E));
        E+=dE;
        if (fabs(dE)<1e-13) break;
    }
    return E;
}

static V3 orbit_pos_eci(const OrbEl *el, double nu) {
    double cO=cos(el->Omega),sO=sin(el->Omega);
    double co=cos(el->omega),so=sin(el->omega);
    double ci=cos(el->inc), si=sin(el->inc);
    double Px= cO*co-sO*so*ci, Py=sO*co+cO*so*ci, Pz=so*si;
    double Qx=-cO*so-sO*co*ci, Qy=-sO*so+cO*co*ci, Qz=co*si;
    double p=el->a*(1.0-el->e*el->e);
    double r=p/(1.0+el->e*cos(nu));
    double xo=r*cos(nu),yo=r*sin(nu);
    return (V3){Px*xo+Qx*yo, Py*xo+Qy*yo, Pz*xo+Qz*yo};
}

static void el_to_state(const OrbEl *el, V3 *pos, V3 *vel) {
    double E=kepler(el->M0,el->e);
    double nu=2.0*atan2(sqrt(1.0+el->e)*sin(E/2.0),sqrt(1.0-el->e)*cos(E/2.0));
    double cO=cos(el->Omega),sO=sin(el->Omega);
    double co=cos(el->omega),so=sin(el->omega);
    double ci=cos(el->inc), si=sin(el->inc);
    double Px= cO*co-sO*so*ci, Py=sO*co+cO*so*ci, Pz=so*si;
    double Qx=-cO*so-sO*co*ci, Qy=-sO*so+cO*co*ci, Qz=co*si;
    double p=el->a*(1.0-el->e*el->e), r=p/(1.0+el->e*cos(nu));
    double xo=r*cos(nu),yo=r*sin(nu);
    *pos=(V3){Px*xo+Qx*yo, Py*xo+Qy*yo, Pz*xo+Qz*yo};
    double h=sqrt(GM*p);
    double vr=(GM/h)*el->e*sin(nu), vt=(GM/h)*(1.0+el->e*cos(nu));
    double vxo=vr*cos(nu)-vt*sin(nu), vyo=vr*sin(nu)+vt*cos(nu);
    *vel=(V3){Px*vxo+Qx*vyo, Py*vxo+Qy*vyo, Pz*vxo+Qz*vyo};
}

/* ======================================================================
   Satellite state
   ====================================================================== */
typedef struct {
    V3  pos, vel;
    const OrbEl *el;
    /* 3D trail (ECI positions) */
    V3  trail[TRAIL_LEN];
    int trail_head, trail_cnt, trail_ctr;
    /* 2D ground track (Earth-fixed lat/lon in radians) */
    float gt_lat[GT_LEN];
    float gt_lon[GT_LEN];
    int   gt_head, gt_cnt;
    int   visible;
} Sat;

static void sat_init(Sat *s, const OrbEl *el) {
    s->el=el;
    el_to_state(el,&s->pos,&s->vel);
    s->trail_head=0; s->trail_cnt=0; s->trail_ctr=0;
    s->gt_head=0;    s->gt_cnt=0;
    s->visible=1;
}

static V3 gravity(V3 pos, int use_j2) {
    double r2=v3dot(pos,pos), r=sqrt(r2);
    V3 a=v3scl(pos,-GM/(r2*r));
    if (use_j2) {
        double z2r2=(pos.z*pos.z)/r2;
        double fac=1.5*J2_COEFF*GM*RE*RE/(r2*r2*r);
        a.x+=fac*pos.x*(1.0-5.0*z2r2);
        a.y+=fac*pos.y*(1.0-5.0*z2r2);
        a.z+=fac*pos.z*(3.0-5.0*z2r2);
    }
    return a;
}

static void sat_step(Sat *s, double dt, int use_j2, double earth_angle) {
    /* Adaptive sub-stepping: near perigee of highly eccentric orbits
       (e.g. Molniya, perigee ~535 km) the satellite moves ~10 km/s and the
       force changes ~5.8 % per 20 s step — too large for energy conservation.
       Sub-divide so each sub-step covers < 1 % force change.
         alt < 1000 km → 8 sub-steps of 2.5 s  (force Δ ≈ 0.7 %)
         alt < 4000 km → 4 sub-steps of 5 s     (force Δ ≈ 1.4 %)
         alt <10000 km → 2 sub-steps of 10 s
         otherwise    → 1 sub-step  of 20 s                              */
    double r   = v3len(s->pos);
    double alt = r - RE;
    int    ns  = (alt < 1000e3) ? 8 :
                 (alt < 4000e3) ? 4 :
                 (alt <10000e3) ? 2 : 1;
    double sub = dt / ns;

    for (int k = 0; k < ns; k++) {
        V3 a0  = gravity(s->pos, use_j2);
        s->vel = v3add(s->vel, v3scl(a0, 0.5*sub));
        s->pos = v3add(s->pos, v3scl(s->vel, sub));
        V3 a1  = gravity(s->pos, use_j2);
        s->vel = v3add(s->vel, v3scl(a1, 0.5*sub));
    }

    if (++s->trail_ctr >= TRAIL_STEP) {
        s->trail_ctr=0;
        /* 3D trail */
        s->trail[s->trail_head]=s->pos;
        s->trail_head=(s->trail_head+1)%TRAIL_LEN;
        if (s->trail_cnt<TRAIL_LEN) s->trail_cnt++;
        /* 2D ground track: rotate ECI → Earth-fixed */
        V3 ef=rz(v3norm(s->pos),-earth_angle);
        s->gt_lat[s->gt_head]=(float)asin(fmax(-1.0,fmin(1.0,ef.z)));
        s->gt_lon[s->gt_head]=(float)atan2(ef.y,ef.x);
        s->gt_head=(s->gt_head+1)%GT_LEN;
        if (s->gt_cnt<GT_LEN) s->gt_cnt++;
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
    for (int i=0; i<N_STARS; i++) {
        int x,y;
        do {
            x=rand()%WIN_W; y=rand()%WIN_H;
            double dx=x-CX,dy=y-CY;
            if (dx*dx+dy*dy>(BASE_R+30)*(BASE_R+30)) break;
        } while(1);
        star_x[i]=x; star_y[i]=y;
        star_b[i]=(uint8_t)(80+rand()%176);
    }
}

/* ======================================================================
   Orbit definitions
   ====================================================================== */
static const OrbEl ORBITS[NUM_SATS] = {
    { 42164000.0,0.000, 0.0*DEG,  0.0*DEG,  0.0*DEG,  0.0*DEG,"GEO",     0,  255,255},
    { 42164000.0,0.000,28.0*DEG, 45.0*DEG,  0.0*DEG,120.0*DEG,"GEOSYNC",255,255,255},
    { 26560000.0,0.740,63.4*DEG, 40.0*DEG,270.0*DEG,180.0*DEG,"MOLNIYA",255, 50, 50},
    {  6778000.0,0.000,51.6*DEG, 45.0*DEG,  0.0*DEG, 90.0*DEG,"ISS",    255,255,  0},
    {  7000000.0,0.000,90.0*DEG, 90.0*DEG,  0.0*DEG,  0.0*DEG,"POLAR",   50,255, 50},
    { 26560000.0,0.000,55.0*DEG,160.0*DEG,  0.0*DEG,240.0*DEG,"MEO/GPS",255, 50,255},
    { 42164000.0,0.270,63.4*DEG,180.0*DEG,270.0*DEG,270.0*DEG,"TUNDRA", 255,165,  0},
    {  7178000.0,0.000,98.7*DEG,130.0*DEG,  0.0*DEG,180.0*DEG,"SSO",    255,150,200},
};

/* ======================================================================
   Full GPS constellation  —  Walker 24/6/2
   Forward declarations needed by footprint/GPS map renderers below.
   ====================================================================== */
static int map_lon_to_px(double lon);
static int map_lat_to_py(double lat);
#define GPS_N 24

/* Plane colours (dim, so they don't overwhelm the main 8 sats) */
#define GPS_CR0 100  /* plane A – blue   */
#define GPS_CG0 140
#define GPS_CB0 255
#define GPS_CR1 100  /* plane B – green  */
#define GPS_CG1 220
#define GPS_CB1 140
#define GPS_CR2 220  /* plane C – pink   */
#define GPS_CG2 100
#define GPS_CB2 160
#define GPS_CR3 100  /* plane D – cyan   */
#define GPS_CG3 220
#define GPS_CB3 220
#define GPS_CR4 220  /* plane E – yellow */
#define GPS_CG4 190
#define GPS_CB4  80
#define GPS_CR5 180  /* plane F – purple */
#define GPS_CG5 100
#define GPS_CB5 220

static const OrbEl GPS_ORBITS[GPS_N] = {
/* a(m)           e     i           Omega      omega  M0          name  R          G          B         */
/* Plane A  Ω=  0°  offset M=  0° */
{26559750.0,0.0,55.0*DEG,  0.0*DEG,0.0,  0.0*DEG,"GPS",GPS_CR0,GPS_CG0,GPS_CB0},
{26559750.0,0.0,55.0*DEG,  0.0*DEG,0.0, 90.0*DEG,"GPS",GPS_CR0,GPS_CG0,GPS_CB0},
{26559750.0,0.0,55.0*DEG,  0.0*DEG,0.0,180.0*DEG,"GPS",GPS_CR0,GPS_CG0,GPS_CB0},
{26559750.0,0.0,55.0*DEG,  0.0*DEG,0.0,270.0*DEG,"GPS",GPS_CR0,GPS_CG0,GPS_CB0},
/* Plane B  Ω= 60°  offset M= 30° */
{26559750.0,0.0,55.0*DEG, 60.0*DEG,0.0, 30.0*DEG,"GPS",GPS_CR1,GPS_CG1,GPS_CB1},
{26559750.0,0.0,55.0*DEG, 60.0*DEG,0.0,120.0*DEG,"GPS",GPS_CR1,GPS_CG1,GPS_CB1},
{26559750.0,0.0,55.0*DEG, 60.0*DEG,0.0,210.0*DEG,"GPS",GPS_CR1,GPS_CG1,GPS_CB1},
{26559750.0,0.0,55.0*DEG, 60.0*DEG,0.0,300.0*DEG,"GPS",GPS_CR1,GPS_CG1,GPS_CB1},
/* Plane C  Ω=120°  offset M= 60° */
{26559750.0,0.0,55.0*DEG,120.0*DEG,0.0, 60.0*DEG,"GPS",GPS_CR2,GPS_CG2,GPS_CB2},
{26559750.0,0.0,55.0*DEG,120.0*DEG,0.0,150.0*DEG,"GPS",GPS_CR2,GPS_CG2,GPS_CB2},
{26559750.0,0.0,55.0*DEG,120.0*DEG,0.0,240.0*DEG,"GPS",GPS_CR2,GPS_CG2,GPS_CB2},
{26559750.0,0.0,55.0*DEG,120.0*DEG,0.0,330.0*DEG,"GPS",GPS_CR2,GPS_CG2,GPS_CB2},
/* Plane D  Ω=180°  offset M= 90° */
{26559750.0,0.0,55.0*DEG,180.0*DEG,0.0, 90.0*DEG,"GPS",GPS_CR3,GPS_CG3,GPS_CB3},
{26559750.0,0.0,55.0*DEG,180.0*DEG,0.0,180.0*DEG,"GPS",GPS_CR3,GPS_CG3,GPS_CB3},
{26559750.0,0.0,55.0*DEG,180.0*DEG,0.0,270.0*DEG,"GPS",GPS_CR3,GPS_CG3,GPS_CB3},
{26559750.0,0.0,55.0*DEG,180.0*DEG,0.0,  0.0*DEG,"GPS",GPS_CR3,GPS_CG3,GPS_CB3},
/* Plane E  Ω=240°  offset M=120° */
{26559750.0,0.0,55.0*DEG,240.0*DEG,0.0,120.0*DEG,"GPS",GPS_CR4,GPS_CG4,GPS_CB4},
{26559750.0,0.0,55.0*DEG,240.0*DEG,0.0,210.0*DEG,"GPS",GPS_CR4,GPS_CG4,GPS_CB4},
{26559750.0,0.0,55.0*DEG,240.0*DEG,0.0,300.0*DEG,"GPS",GPS_CR4,GPS_CG4,GPS_CB4},
{26559750.0,0.0,55.0*DEG,240.0*DEG,0.0, 30.0*DEG,"GPS",GPS_CR4,GPS_CG4,GPS_CB4},
/* Plane F  Ω=300°  offset M=150° */
{26559750.0,0.0,55.0*DEG,300.0*DEG,0.0,150.0*DEG,"GPS",GPS_CR5,GPS_CG5,GPS_CB5},
{26559750.0,0.0,55.0*DEG,300.0*DEG,0.0,240.0*DEG,"GPS",GPS_CR5,GPS_CG5,GPS_CB5},
{26559750.0,0.0,55.0*DEG,300.0*DEG,0.0,330.0*DEG,"GPS",GPS_CR5,GPS_CG5,GPS_CB5},
{26559750.0,0.0,55.0*DEG,300.0*DEG,0.0, 60.0*DEG,"GPS",GPS_CR5,GPS_CG5,GPS_CB5},
};

/* Lightweight struct for GPS — no trail/gt history needed */
typedef struct { V3 pos, vel; const OrbEl *el; } GpsSat;
static GpsSat gps_sats[GPS_N];

static void gps_init(GpsSat *s, const OrbEl *el) {
    s->el = el;
    el_to_state(el, &s->pos, &s->vel);
}

static void gps_step(GpsSat *s, double dt, int use_j2) {
    /* GPS circular orbit at 26 560 km — 1 sub-step always sufficient */
    V3 a0 = gravity(s->pos, use_j2);
    s->vel = v3add(s->vel, v3scl(a0, 0.5*dt));
    s->pos = v3add(s->pos, v3scl(s->vel, dt));
    V3 a1 = gravity(s->pos, use_j2);
    s->vel = v3add(s->vel, v3scl(a1, 0.5*dt));
}

/* ======================================================================
   Footprint circle — points on Earth's surface visible above elev_deg
   ====================================================================== */
#define FP_PTS 90   /* points on footprint circle */

static void footprint_eci(V3 sat_pos, double elev_deg, V3 out[FP_PTS]) {
    double h      = v3len(sat_pos) - RE;
    double elev   = elev_deg * DEG;
    double sin_eta = (RE / (RE + h)) * cos(elev);
    if (sin_eta > 1.0) sin_eta = 1.0;
    double eta    = asin(sin_eta);
    double lambda = PI/2.0 - elev - eta;   /* Earth central half-angle */
    if (lambda <= 0.0) { memset(out, 0, FP_PTS * sizeof(V3)); return; }

    V3 ss = v3norm(sat_pos);   /* sub-satellite unit vector */

    /* Orthonormal basis perpendicular to ss */
    V3 ref = (fabs(ss.z) < 0.9) ? (V3){0,0,1} : (V3){1,0,0};
    V3 e1  = v3norm(v3cross(ss, ref));
    V3 e2  = v3cross(ss, e1);

    for (int j = 0; j < FP_PTS; j++) {
        double theta = j * (2.0*PI / FP_PTS);
        V3 pt = v3add(v3scl(ss, cos(lambda)),
                 v3add(v3scl(e1, sin(lambda)*cos(theta)),
                       v3scl(e2, sin(lambda)*sin(theta))));
        out[j] = v3scl(pt, RE);
    }
}

/* Draw footprint circle on the 3D sphere */
static void render_footprint_3d(SDL_Renderer *ren, V3 pos,
                                  uint8_t cr, uint8_t cg, uint8_t cb,
                                  double Rpx, double yaw, double pitch)
{
    V3 circle[FP_PTS];
    footprint_eci(pos, 5.0, circle);
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    for (int j = 0; j < FP_PTS; j++) {
        V3 p0 = eci_to_view(circle[j],             yaw, pitch);
        V3 p1 = eci_to_view(circle[(j+1)%FP_PTS],  yaw, pitch);
        int occ0 = occluded(p0, Rpx);
        int occ1 = occluded(p1, Rpx);
        SDL_SetRenderDrawColor(ren, cr, cg, cb,
                               (uint8_t)((occ0 || occ1) ? 25 : 160));
        SDL_RenderDrawLine(ren, px_x(p0,Rpx), px_y(p0,Rpx),
                                px_x(p1,Rpx), px_y(p1,Rpx));
    }
}

/* Draw footprint circle on the Mercator map */
static void render_footprint_map(SDL_Renderer *ren, V3 pos,
                                   double earth_angle,
                                   uint8_t cr, uint8_t cg, uint8_t cb)
{
    V3 circle[FP_PTS];
    footprint_eci(pos, 5.0, circle);
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    int prev_px=-1, prev_py=-1;
    double prev_lon=0.0;
    for (int j = 0; j <= FP_PTS; j++) {
        V3 ef  = rz(v3norm(circle[j % FP_PTS]), -earth_angle);
        double lat = asin(fmax(-1.0, fmin(1.0, ef.z)));
        double lon = atan2(ef.y, ef.x);
        int mpx = map_lon_to_px(lon);
        int mpy = map_lat_to_py(lat);
        if (prev_px >= 0) {
            double dlon = fabs(lon - prev_lon);
            if (dlon < PI) {
                SDL_SetRenderDrawColor(ren, cr, cg, cb, 130);
                SDL_RenderDrawLine(ren, prev_px, prev_py, mpx, mpy);
            }
        }
        prev_px=mpx; prev_py=mpy; prev_lon=lon;
    }
}

/* Draw GPS constellation dots in 3D mode (small diamond markers) */
static void render_gps_3d(SDL_Renderer *ren,
                            double Rpx, double yaw, double pitch)
{
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_NONE);
    for (int i = 0; i < GPS_N; i++) {
        V3 sv = eci_to_view(gps_sats[i].pos, yaw, pitch);
        if (occluded(sv, Rpx)) continue;
        int sx = px_x(sv, Rpx), sy = px_y(sv, Rpx);
        if (sx<3||sx>=WIN_W-3||sy<3||sy>=WIN_H-3) continue;
        const OrbEl *e = gps_sats[i].el;
        SDL_SetRenderDrawColor(ren, e->cr, e->cg, e->cb, 210);
        SDL_RenderDrawLine(ren, sx-3, sy,   sx,   sy-3);
        SDL_RenderDrawLine(ren, sx,   sy-3, sx+3, sy  );
        SDL_RenderDrawLine(ren, sx+3, sy,   sx,   sy+3);
        SDL_RenderDrawLine(ren, sx,   sy+3, sx-3, sy  );
    }
}

/* Draw GPS constellation dots in Mercator map mode */
static void render_gps_map(SDL_Renderer *ren, double earth_angle)
{
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_NONE);
    for (int i = 0; i < GPS_N; i++) {
        V3 ef  = rz(v3norm(gps_sats[i].pos), -earth_angle);
        double lat = asin(fmax(-1.0, fmin(1.0, ef.z)));
        double lon = atan2(ef.y, ef.x);
        int mpx = map_lon_to_px(lon);
        int mpy = map_lat_to_py(lat);
        if (mpx<3||mpx>=MAP_W-3||mpy<3||mpy>=WIN_H-3) continue;
        const OrbEl *e = gps_sats[i].el;
        SDL_SetRenderDrawColor(ren, e->cr, e->cg, e->cb, 200);
        SDL_Rect dot = {mpx-2, mpy-2, 5, 5};
        SDL_RenderFillRect(ren, &dot);
    }
}

/* ======================================================================
   Natural Earth precise line rendering into map_tex_buf
   ====================================================================== */

/* These are needed by both the NE helpers and build_mercator_texture */
static double   g_map_y_max;                    /* log(tan(π/4 + clip/2)) */
static uint32_t map_tex_buf[WIN_H * WIN_W];     /* 3.7 MB in BSS          */

/* Degrees → pixel (uses g_map_y_max, set before build_mercator_texture) */
static int ne_lx(double lon_d) {
    return (int)((lon_d + 180.0) / 360.0 * MAP_W);
}
static int ne_ly(double lat_d) {
    if (lat_d >  82.0) lat_d =  82.0;
    if (lat_d < -82.0) lat_d = -82.0;
    double lr = lat_d * (PI/180.0);
    double y  = log(tan(PI/4.0 + lr/2.0));
    return (int)((g_map_y_max - y) / (2.0*g_map_y_max) * WIN_H);
}

/* Bresenham line into map_tex_buf (texture stride = WIN_W) */
static void buf_line(int x0, int y0, int x1, int y1, uint32_t col) {
    int dx = abs(x1-x0), dy = abs(y1-y0);
    int sx = x0<x1 ? 1 : -1, sy = y0<y1 ? 1 : -1;
    int err = dx-dy, imax = dx+dy+2;
    for (int i=0; i<imax; i++) {
        if (x0>=0 && x0<MAP_W && y0>=0 && y0<WIN_H)
            map_tex_buf[y0*WIN_W + x0] = col;
        if (x0==x1 && y0==y1) break;
        int e2=err*2;
        if (e2>-dy) { err-=dy; x0+=sx; }
        if (e2< dx) { err+=dx; y0+=sy; }
    }
}

/* Draw a Natural Earth polyline dataset into the texture buffer.
   thick=1 draws a second row offset +1 in y for 2-pixel coastlines. */
static void draw_ne_lines(const float pts[][2],
                           const int *starts, const int *lens, int npolys,
                           uint32_t col, int thick)
{
    for (int i=0; i<npolys; i++) {
        const float (*line)[2] = pts + starts[i];
        int n = lens[i];
        for (int j=1; j<n; j++) {
            double lon0=line[j-1][0], lat0=line[j-1][1];
            double lon1=line[j][0],   lat1=line[j][1];
            if (fabs(lon1-lon0) > 180.0) continue;   /* antimeridian break */
            int x0=ne_lx(lon0), y0=ne_ly(lat0);
            int x1=ne_lx(lon1), y1=ne_ly(lat1);
            buf_line(x0, y0,   x1, y1,   col);
            if (thick) buf_line(x0, y0+1, x1, y1+1, col);
        }
    }
}

/* ======================================================================
   Mercator map background texture
   ====================================================================== */

/* Map-space projections (radians input — used by render_map_view overlays) */
static int map_lon_to_px(double lon) {
    return (int)((lon + PI) / (2.0*PI) * MAP_W);
}
static int map_lat_to_py(double lat) {
    if (lat >  MAP_LAT_CLIP) lat =  MAP_LAT_CLIP;
    if (lat < -MAP_LAT_CLIP) lat = -MAP_LAT_CLIP;
    double y = log(tan(PI/4.0 + lat/2.0));
    return (int)((g_map_y_max - y) / (2.0*g_map_y_max) * WIN_H);
}

static SDL_Texture *build_mercator_texture(SDL_Renderer *ren) {
    /* Colour palette — NASA MCC dark theme */
    /* Uniform dark ocean background with lat/lon grid only.
       Actual geography is drawn by the Natural Earth line pass below.   */
    const uint32_t C_OCEAN   = 0xFF06101E;   /* deep navy                */
    const uint32_t C_GRID    = 0xFF0E1C30;   /* dim 30° grid             */
    const uint32_t C_GRID_EQ = 0xFF162840;   /* equator / prime meridian */

    for (int py = 0; py < WIN_H; py++) {
        double ym      = g_map_y_max * (1.0 - 2.0*py / (double)WIN_H);
        double lat     = 2.0*atan(exp(ym)) - PI/2.0;
        double lat_deg = lat * (180.0/PI);
        double la30    = fmod(fabs(lat_deg), 30.0);

        for (int px = 0; px < MAP_W; px++) {
            double lon   = ((double)px / (double)MAP_W) * 2.0*PI - PI;
            double lo_d  = lon * (180.0/PI);
            double lo30  = fmod(lo_d < 0 ? lo_d + 360.0 : lo_d, 30.0);

            int on_eq   = fabs(lat_deg) < 0.28;
            int on_pm   = fabs(lo_d)    < 0.20;
            int on_grid = (la30 < 0.50) || (lo30 < 0.25);

            uint32_t col = C_OCEAN;
            if      (on_eq || on_pm) col = C_GRID_EQ;
            else if (on_grid)        col = C_GRID;

            map_tex_buf[py * WIN_W + px] = col;
        }
    }

    /* ── Overlay precise Natural Earth coastlines + country borders ── */
    /* Country borders first (dimmer, underneath coastlines) */
    draw_ne_lines(border_pts, border_start, border_len, BORDER_NPOLYS,
                  0xFF2A6835, 0);   /* dim green, 1 px */
    /* Coastlines on top — 2 px wide, brighter green */
    draw_ne_lines(coast_pts,  coast_start,  coast_len,  COAST_NPOLYS,
                  0xFF55D060, 1);   /* bright green, 2 px */

    SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
                                          SDL_TEXTUREACCESS_STATIC, WIN_W, WIN_H);
    if (!tex) return NULL;
    SDL_UpdateTexture(tex, NULL, map_tex_buf, WIN_W*4);
    return tex;
}

/* ======================================================================
   Mercator map view renderer
   ====================================================================== */
static void render_map_view(SDL_Renderer *ren, SDL_Texture *map_tex,
                             const Sat sats[NUM_SATS],
                             double earth_angle,
                             double sim_time, int paused,
                             int use_j2, int show_gps, int show_fp,
                             int recording,
                             uint32_t frame_count)
{
    /* Background map — blit only into the map area, not under the panel */
    SDL_Rect map_dest = {0, 0, MAP_W, WIN_H};
    SDL_Rect map_src  = {0, 0, MAP_W, WIN_H};
    SDL_RenderCopy(ren, map_tex, &map_src, &map_dest);

    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);

    /* ── Draw accumulated ground tracks ──────────────────────────── */
    for (int i=0; i<NUM_SATS; i++) {
        if (!sats[i].visible) continue;
        const Sat *s = &sats[i];
        if (s->gt_cnt < 2) continue;

        int prev_px=-1, prev_py=-1;
        double prev_lon=0.0;

        for (int k=s->gt_cnt-1; k>=0; k--) {
            int idx = (s->gt_head - 1 - k + GT_LEN) % GT_LEN;
            double lat = s->gt_lat[idx];
            double lon = s->gt_lon[idx];

            int mpx = map_lon_to_px(lon);
            int mpy = map_lat_to_py(lat);

            /* Age factor: 0=oldest, 1=newest */
            float age = 1.0f - (float)k / (float)s->gt_cnt;
            uint8_t alpha = (uint8_t)(25 + age * age * 200);

            if (prev_px >= 0) {
                /* Skip if this point is a re-enable sentinel */
                if (s->gt_lat[idx] >= GT_BREAK - 1.0f) {
                    prev_px = -1;   /* force a line break */
                } else {
                    /* Antimeridian break: skip segment if lon jump > 180° */
                    double dlon = fabs(lon - prev_lon);
                    if (dlon < PI) {
                        SDL_SetRenderDrawColor(ren,
                            (uint8_t)(s->el->cr * (0.4f + 0.6f*age)),
                            (uint8_t)(s->el->cg * (0.4f + 0.6f*age)),
                            (uint8_t)(s->el->cb * (0.4f + 0.6f*age)),
                            alpha);
                        SDL_RenderDrawLine(ren, prev_px, prev_py, mpx, mpy);
                    }
                }
            }
            /* Don't update prev if this was a sentinel */
            if (s->gt_lat[idx] < GT_BREAK - 1.0f)
                { prev_px=mpx; prev_py=mpy; prev_lon=lon; }
        }
    }

    /* ── Draw current sub-satellite positions ─────────────────────── */
    for (int i=0; i<NUM_SATS; i++) {
        if (!sats[i].visible) continue;
        const Sat *s = &sats[i];

        /* Current Earth-fixed position */
        V3 ef  = rz(v3norm(s->pos), -earth_angle);
        double lat = asin(fmax(-1.0,fmin(1.0,ef.z)));
        double lon = atan2(ef.y,ef.x);

        int mpx = map_lon_to_px(lon);
        int mpy = map_lat_to_py(lat);

        if (mpx<5 || mpx>=MAP_W-5 || mpy<5 || mpy>=WIN_H-5) continue;

        const OrbEl *e = s->el;

        /* Glow halo */
        SDL_SetRenderDrawColor(ren, e->cr, e->cg, e->cb, 40);
        for (int dy=-6; dy<=6; dy++)
            for (int dx=-6; dx<=6; dx++)
                if (dx*dx+dy*dy<=36)
                    SDL_RenderDrawPoint(ren, mpx+dx, mpy+dy);

        /* Filled circle (radius 4) */
        SDL_SetRenderDrawColor(ren, e->cr, e->cg, e->cb, 255);
        SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_NONE);
        for (int dy=-4; dy<=4; dy++)
            for (int dx=-4; dx<=4; dx++)
                if (dx*dx+dy*dy<=16)
                    SDL_RenderDrawPoint(ren, mpx+dx, mpy+dy);

        /* Crosshair extending outward */
        SDL_RenderDrawLine(ren, mpx-9, mpy,  mpx-5, mpy);
        SDL_RenderDrawLine(ren, mpx+5, mpy,  mpx+9, mpy);
        SDL_RenderDrawLine(ren, mpx, mpy-9,  mpx, mpy-5);
        SDL_RenderDrawLine(ren, mpx, mpy+5,  mpx, mpy+9);

        /* White centre dot */
        SDL_SetRenderDrawColor(ren,255,255,255,255);
        SDL_RenderDrawPoint(ren, mpx, mpy);

        /* Name label — place right of dot, flip left if near right edge */
        int lx = (mpx + WIN_W - 196 > WIN_W - 210) ? mpx - 6*((int)strlen(e->name)+1) - 2
                                                     : mpx + 13;
        int ly = mpy - 4;
        /* shadow */
        draw_str(ren, lx+1, ly+1, e->name, 0,0,0, 1);
        draw_str(ren, lx,   ly,   e->name, e->cr, e->cg, e->cb, 1);

        SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    }

    /* ── GPS constellation dots ────────────────────────────────────── */
    if (show_gps) render_gps_map(ren, earth_angle);

    /* ── Footprint circles ──────────────────────────────────────────── */
    if (show_fp) {
        for (int i=0; i<NUM_SATS; i++)
            if (sats[i].visible)
                render_footprint_map(ren, sats[i].pos, earth_angle,
                                     sats[i].el->cr, sats[i].el->cg, sats[i].el->cb);
        if (show_gps)
            for (int i=0; i<GPS_N; i++)
                render_footprint_map(ren, gps_sats[i].pos, earth_angle,
                                     gps_sats[i].el->cr, gps_sats[i].el->cg, gps_sats[i].el->cb);
    }

    /* ── Lat/lon axis tick labels ──────────────────────────────────── */
    /* Latitude labels on left edge */
    static const int lat_ticks[] = {-60,-30,0,30,60};
    for (int t=0; t<5; t++) {
        double lat_r = lat_ticks[t]*DEG;
        int py = map_lat_to_py(lat_r);
        char buf[8];
        if (lat_ticks[t]==0) {
            snprintf(buf,sizeof(buf),"EQ");
        } else {
            snprintf(buf,sizeof(buf),"%d", lat_ticks[t]);
        }
        /* tick line */
        SDL_SetRenderDrawColor(ren, 50,80,60, 200);
        SDL_RenderDrawLine(ren, 0, py, MAP_W, py);
        draw_str(ren, 3, py-4, buf, 70,110,70, 1);
    }
    /* Longitude labels on bottom edge */
    static const int lon_ticks[] = {-150,-120,-90,-60,-30,0,30,60,90,120,150,180};
    for (int t=0; t<12; t++) {
        double lon_r = lon_ticks[t]*DEG;
        int ppx = map_lon_to_px(lon_r);
        if (ppx < 0 || ppx >= MAP_W) continue;
        char buf[8]; snprintf(buf,sizeof(buf),"%d",lon_ticks[t]);
        SDL_SetRenderDrawColor(ren, 50,80,60, 200);
        SDL_RenderDrawLine(ren, ppx, 0, ppx, WIN_H-28);
        draw_str(ren, ppx-12, WIN_H-44, buf, 70,110,70, 1);
    }

    /* ── Title bar ─────────────────────────────────────────────────── */
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(ren, 0,0,0, 180);
    SDL_Rect tbar = {0, 0, MAP_W, 16};
    SDL_RenderFillRect(ren, &tbar);
    draw_str(ren, 6, 4, "GROUND TRACK MONITOR  MERCATOR PROJECTION", 60,160,80, 1);

    /* ── Legend panel (right side) ─────────────────────────────────── */
    SDL_SetRenderDrawColor(ren, 0,0,0, 200);
    SDL_Rect panel = {WIN_W-200, 0, 200, WIN_H};
    SDL_RenderFillRect(ren, &panel);

    /* Separator line */
    SDL_SetRenderDrawColor(ren, 40,80,50, 255);
    SDL_RenderDrawLine(ren, WIN_W-200, 0, WIN_W-200, WIN_H);

    draw_str(ren, WIN_W-196, 8,  "SATELLITE POSITIONS", 60,160,80, 1);
    draw_str(ren, WIN_W-196, 20, "-------------------", 30,70,40, 1);

    int legend_y = 34;
    for (int i=0; i<NUM_SATS; i++) {
        const Sat  *s = &sats[i];
        const OrbEl*e = s->el;

        /* Colour swatch */
        SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_NONE);
        SDL_SetRenderDrawColor(ren,
            s->visible ? e->cr : 40,
            s->visible ? e->cg : 40,
            s->visible ? e->cb : 40, 255);
        SDL_Rect sw = {WIN_W-196, legend_y, 10, 8};
        SDL_RenderFillRect(ren, &sw);

        /* Name */
        char nbuf[12]; snprintf(nbuf,sizeof(nbuf),"%d %s", i+1, e->name);
        draw_str(ren, WIN_W-182, legend_y,
                 nbuf, s->visible?e->cr:50, s->visible?e->cg:50, s->visible?e->cb:50, 1);

        if (s->visible) {
            /* Current lat/lon */
            V3 ef  = rz(v3norm(s->pos),-earth_angle);
            double lat_d = asin(fmax(-1.0,fmin(1.0,ef.z))) * (180.0/PI);
            double lon_d = atan2(ef.y,ef.x) * (180.0/PI);
            char pos[24];
            snprintf(pos,sizeof(pos),"%.1f%c %.1f%c",
                     fabs(lat_d), lat_d>=0?'N':'S',
                     fabs(lon_d), lon_d>=0?'E':'W');
            draw_str(ren, WIN_W-196, legend_y+10, pos, 120,180,120, 1);
        }

        SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
        legend_y += 26;
    }

    /* Divider before status */
    SDL_SetRenderDrawColor(ren, 30,60,40, 200);
    SDL_RenderDrawLine(ren, WIN_W-196, legend_y+2, WIN_W-4, legend_y+2);
    legend_y += 8;

    /* J2 / GPS / FP status */
    draw_str(ren, WIN_W-196, legend_y,
             use_j2 ? "J2: ON " : "J2: OFF",
             use_j2?80:100, use_j2?200:100, use_j2?80:100, 1);
    legend_y += 12;
    draw_str(ren, WIN_W-196, legend_y,
             show_gps ? "GPS: ON " : "GPS: OFF",
             show_gps?80:100, show_gps?200:100, show_gps?80:100, 1);
    draw_str(ren, WIN_W-130, legend_y,
             show_fp ? "FP: ON " : "FP: OFF",
             show_fp?80:100, show_fp?200:100, show_fp?80:100, 1);

    /* ── Bottom status bar ─────────────────────────────────────────── */
    SDL_SetRenderDrawColor(ren, 0,0,0, 190);
    SDL_Rect bot = {0, WIN_H-28, MAP_W, 28};
    SDL_RenderFillRect(ren, &bot);

    double days  = sim_time/86400.0;
    double hours = fmod(sim_time,86400.0)/3600.0;
    char buf[160];
    snprintf(buf,sizeof(buf),"SIM: %4.0fD %05.2fH",days,hours);
    draw_str(ren, 10, WIN_H-20, buf, 100,200,120, 1);

    if (paused)
        draw_str(ren, 260, WIN_H-20, "PAUSED", 255,200,50, 2);

    if (recording) {
        int blink=(frame_count/30)&1;
        draw_str(ren, WIN_W-240, WIN_H-20, "REC",
                 255, blink?50:0, blink?50:0, 1);
    }

    draw_str(ren, 10, WIN_H-54,
             "+/- SPEED   SPC PAUSE   1-8 TOGGLE   J J2   C GPS   F FOOTPRINT   P 3D VIEW   Q QUIT",
             55,90,65, 1);
}

/* ======================================================================
   Earth sphere rendering (per-pixel raycast, streaming texture)
   ====================================================================== */
static void render_earth(uint32_t *px, int stride,
                          double Rpx, double view_yaw, double view_pitch,
                          double earth_angle)
{
    int iy0=(int)(CY-Rpx-2), iy1=(int)(CY+Rpx+2);
    int ix0=(int)(CX-Rpx-2), ix1=(int)(CX+Rpx+2);
    if (iy0<0)      iy0=0;
    if (iy1>=WIN_H) iy1=WIN_H-1;
    if (ix0<0)      ix0=0;
    if (ix1>=WIN_W) ix1=WIN_W-1;

    for (int py=iy0; py<=iy1; py++) {
        double ndy=(CY-py)/Rpx;
        for (int pxx=ix0; pxx<=ix1; pxx++) {
            double ndx=(pxx-CX)/Rpx, r2=ndx*ndx+ndy*ndy;
            uint32_t pixel=0xFF000000;

            if (r2<=1.0) {
                double nz=sqrt(1.0-r2);
                V3 N_view={ndx,ndy,nz};
                V3 N_eci=view_to_eci(N_view,view_yaw,view_pitch);
                V3 N_earth=rz(N_eci,-earth_angle);
                double lat=asin(fmax(-1.0,fmin(1.0,N_earth.z)));
                double lon=atan2(N_earth.y,N_earth.x);
                int is_land=lm_lookup(lat,lon);
                double diffuse=fmax(0.0,N_eci.x);
                double light=0.22+0.78*diffuse;
                double lat_deg=lat*(180.0/PI), abslat=fabs(lat_deg);
                double fr,fg,fb;
                if (is_land) {
                    if (abslat>65.0) { fr=220;fg=225;fb=240; }
                    else { double t=fmax(0.0,1.0-abslat/65.0); fr=35+20*(1-t);fg=100+30*t;fb=20; }
                } else { fr=5;fg=40;fb=130; }
                fr*=light; fg*=light; fb*=light;
                if (abslat>70.0) { double f=(abslat-70.0)/20.0; fr+=f*(230-fr);fg+=f*(235-fg);fb+=f*(245-fb); }
                double lo30=fmod(fabs(lon*(180.0/PI)),30.0), la30=fmod(abslat,30.0);
                if (la30<0.7||lo30<0.7) { fr*=0.75;fg*=0.75;fb=fb*0.75+15; }
                double glow=(1.0-nz); glow=glow*glow*glow*0.18;
                fb+=glow*180; fg+=glow*80; fr+=glow*20;
                pixel=0xFF000000|((uint32_t)(uint8_t)fmin(255,fr)<<16)
                      |((uint32_t)(uint8_t)fmin(255,fg)<<8)|(uint8_t)fmin(255,fb);
            } else if (r2<=1.055) {
                double rl=sqrt(r2), glow=((1.055-rl)/0.055); glow=glow*glow;
                pixel=0xFF000000|((uint32_t)(uint8_t)(glow*10)<<16)
                      |((uint32_t)(uint8_t)(glow*35)<<8)|(uint8_t)(glow*70);
            }
            px[py*stride+pxx]=pixel;
        }
    }
}

/* ======================================================================
   Orbit path (analytical Keplerian ellipse)
   ====================================================================== */
static void render_orbit(SDL_Renderer *ren, const OrbEl *el,
                          double Rpx, double view_yaw, double view_pitch)
{
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    int psx=0,psy=0,pocc=0;
    for (int j=0; j<=ORBIT_PTS; j++) {
        double nu=j*(2.0*PI/ORBIT_PTS);
        V3 pv=eci_to_view(orbit_pos_eci(el,nu),view_yaw,view_pitch);
        int sx=px_x(pv,Rpx), sy=px_y(pv,Rpx), occ=occluded(pv,Rpx);
        if (j>0) {
            SDL_SetRenderDrawColor(ren,el->cr,el->cg,el->cb,(uint8_t)(occ&&pocc?40:130));
            SDL_RenderDrawLine(ren,psx,psy,sx,sy);
        }
        psx=sx; psy=sy; pocc=occ;
    }
}

/* ======================================================================
   Ground tracks on 3D sphere (nadir line + dot on surface)
   ====================================================================== */
static void render_ground_tracks(SDL_Renderer *ren,
                                  const Sat sats[NUM_SATS],
                                  double Rpx, double view_yaw, double view_pitch,
                                  double earth_angle)
{
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    for (int i=0; i<NUM_SATS; i++) {
        if (!sats[i].visible) continue;
        V3 ef   = rz(v3norm(sats[i].pos),-earth_angle);
        V3 surf = v3scl(rz(ef,earth_angle),RE);
        V3 sv   = eci_to_view(surf,view_yaw,view_pitch);
        if (sv.z<=0.0) continue;
        int gx=px_x(sv,Rpx), gy=px_y(sv,Rpx);
        if (gx<3||gx>=WIN_W-3||gy<3||gy>=WIN_H-3) continue;
        const OrbEl *e=sats[i].el;
        V3 satv=eci_to_view(sats[i].pos,view_yaw,view_pitch);
        if (!occluded(satv,Rpx)) {
            SDL_SetRenderDrawColor(ren,e->cr,e->cg,e->cb,55);
            SDL_RenderDrawLine(ren,px_x(satv,Rpx),px_y(satv,Rpx),gx,gy);
        }
        SDL_SetRenderDrawColor(ren,e->cr,e->cg,e->cb,220);
        SDL_Rect outer={gx-3,gy-3,7,7}; SDL_RenderFillRect(ren,&outer);
        SDL_SetRenderDrawColor(ren,0,0,0,220);
        SDL_Rect inner={gx-1,gy-1,3,3}; SDL_RenderFillRect(ren,&inner);
    }
}

/* ======================================================================
   Satellite trail + dot (3D view)
   ====================================================================== */
static void render_satellite(SDL_Renderer *ren, const Sat *s,
                              double Rpx, double view_yaw, double view_pitch)
{
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    for (int k=0; k<s->trail_cnt; k++) {
        int idx=(s->trail_head-1-k+TRAIL_LEN)%TRAIL_LEN;
        V3 tv=eci_to_view(s->trail[idx],view_yaw,view_pitch);
        if (occluded(tv,Rpx)) continue;
        int tsx=px_x(tv,Rpx),tsy=px_y(tv,Rpx);
        if (tsx<0||tsx>=WIN_W||tsy<0||tsy>=WIN_H) continue;
        float f=1.0f-(float)k/(float)s->trail_cnt;
        SDL_SetRenderDrawColor(ren,(uint8_t)(s->el->cr*f),(uint8_t)(s->el->cg*f),
                               (uint8_t)(s->el->cb*f),(uint8_t)(f*f*180));
        SDL_RenderDrawPoint(ren,tsx,tsy);
    }
    V3 sv=eci_to_view(s->pos,view_yaw,view_pitch);
    if (occluded(sv,Rpx)) return;
    int sx=px_x(sv,Rpx),sy=px_y(sv,Rpx);
    if (sx<4||sx>=WIN_W-4||sy<4||sy>=WIN_H-4) return;
    SDL_SetRenderDrawBlendMode(ren,SDL_BLENDMODE_NONE);
    SDL_SetRenderDrawColor(ren,s->el->cr,s->el->cg,s->el->cb,255);
    SDL_RenderDrawLine(ren,sx-4,sy,sx+4,sy);
    SDL_RenderDrawLine(ren,sx,sy-4,sx,sy+4);
    SDL_RenderDrawLine(ren,sx-3,sy-3,sx+3,sy+3);
    SDL_RenderDrawLine(ren,sx+3,sy-3,sx-3,sy+3);
    SDL_SetRenderDrawColor(ren,255,255,255,255);
    SDL_Rect dot={sx-1,sy-1,3,3}; SDL_RenderFillRect(ren,&dot);
}

/* ======================================================================
   3D HUD
   ====================================================================== */
static void render_hud(SDL_Renderer *ren,
                        const Sat sats[NUM_SATS],
                        double time_scale, double sim_time,
                        int paused, int use_j2,
                        int show_gt, int show_gps, int show_fp,
                        int recording,
                        uint32_t frame_count)
{
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(ren,0,0,0,180);
    SDL_Rect bg={WIN_W-200,10,190,NUM_SATS*26+88};
    SDL_RenderFillRect(ren,&bg);

    draw_str(ren,WIN_W-196,14,"ORBITS",200,200,200,1);

    for (int i=0; i<NUM_SATS; i++) {
        int y=28+i*26;
        const OrbEl *e=sats[i].el;
        SDL_SetRenderDrawBlendMode(ren,SDL_BLENDMODE_NONE);
        SDL_SetRenderDrawColor(ren,
            sats[i].visible?e->cr:60, sats[i].visible?e->cg:60, sats[i].visible?e->cb:60,255);
        SDL_Rect sw={WIN_W-196,y+1,12,9}; SDL_RenderFillRect(ren,&sw);
        char label[32]; snprintf(label,sizeof(label),"%d %s",i+1,e->name);
        draw_str(ren,WIN_W-180,y,label,
                 sats[i].visible?e->cr:80, sats[i].visible?e->cg:80, sats[i].visible?e->cb:80,1);
    }

    int ty=28+NUM_SATS*26+6;
    draw_str(ren,WIN_W-196,ty,"J2:",150,150,150,1);
    draw_str(ren,WIN_W-172,ty, use_j2?"ON ":"OFF", use_j2?80:150, use_j2?210:150, use_j2?80:150,1);
    draw_str(ren,WIN_W-140,ty,"GT:",150,150,150,1);
    draw_str(ren,WIN_W-116,ty, show_gt?"ON ":"OFF", show_gt?80:150, show_gt?210:150, show_gt?80:150,1);
    ty+=14;
    draw_str(ren,WIN_W-196,ty,"GPS:",150,150,150,1);
    draw_str(ren,WIN_W-166,ty, show_gps?"ON ":"OFF", show_gps?80:150, show_gps?210:150, show_gps?80:150,1);
    draw_str(ren,WIN_W-134,ty,"FP:",150,150,150,1);
    draw_str(ren,WIN_W-112,ty, show_fp?"ON ":"OFF", show_fp?80:150, show_fp?210:150, show_fp?80:150,1);

    char buf[160];
    double days=sim_time/86400.0, hours=fmod(sim_time,86400.0)/3600.0;
    snprintf(buf,sizeof(buf),"SPEED: %gX",time_scale);
    draw_str(ren,10,WIN_H-22,buf,180,220,180,1);
    snprintf(buf,sizeof(buf),"SIM: %4.0fD %05.2fH",days,hours);
    draw_str(ren,160,WIN_H-22,buf,180,180,220,1);

    if (paused) draw_str(ren,420,WIN_H-22,"PAUSED",255,200,50,2);

    if (recording) {
        int blink=(frame_count/30)&1;
        draw_str(ren,WIN_W-200,WIN_H-22,"REC",255,blink?50:0,blink?50:0,1);
    }

    draw_str(ren,10,WIN_H-50,
             "+/- SPEED   R RESET   SPC PAUSE   1-8 TOGGLE   SCROLL ZOOM",85,85,85,1);
    draw_str(ren,10,WIN_H-38,
             "J J2   G GRND TRK   C GPS   F FOOTPRINT   P MAP   V VIDEO   Q QUIT",85,85,85,1);
}

/* ======================================================================
   ffmpeg video capture
   ====================================================================== */
static FILE   *ffmpeg_pipe = NULL;
static uint8_t frame_buf[WIN_W * WIN_H * 4];

static int video_start(void) {
    char fname[80], cmd[512];
    time_t t=time(NULL);
    struct tm *tm=localtime(&t);
    strftime(fname,sizeof(fname),"earth_orbit_%Y%m%d_%H%M%S.mp4",tm);
    snprintf(cmd,sizeof(cmd),
        "ffmpeg -y -f rawvideo -pixel_format rgb24 "
        "-video_size %dx%d -framerate 60 -i pipe:0 "
        "-c:v libx264 -preset fast -crf 18 -pix_fmt yuv420p "
        "'%s' 2>/dev/null",
        WIN_W, WIN_H, fname);
    ffmpeg_pipe=popen(cmd,"w");
    if (!ffmpeg_pipe) { fprintf(stderr,"video_start: popen failed\n"); return 0; }
    printf("Recording to: %s\n",fname);
    return 1;
}

static void video_stop(void) {
    if (!ffmpeg_pipe) return;
    pclose(ffmpeg_pipe); ffmpeg_pipe=NULL;
    printf("Recording stopped.\n");
}

static void video_frame(SDL_Renderer *ren) {
    if (!ffmpeg_pipe) return;
    SDL_RenderReadPixels(ren,NULL,SDL_PIXELFORMAT_RGB24,frame_buf,WIN_W*3);
    fwrite(frame_buf,(size_t)WIN_W*WIN_H*3,1,ffmpeg_pipe);
}

/* ======================================================================
   Main
   ====================================================================== */
int main(void) {
    if (SDL_Init(SDL_INIT_VIDEO)<0) {
        fprintf(stderr,"SDL_Init: %s\n",SDL_GetError()); return 1;
    }

    SDL_Window *win=SDL_CreateWindow(
        "Earth Satellite Orbit Simulation",
        SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,
        WIN_W,WIN_H,SDL_WINDOW_SHOWN);
    if (!win) { fprintf(stderr,"SDL_CreateWindow: %s\n",SDL_GetError()); return 1; }

    SDL_Renderer *ren=SDL_CreateRenderer(win,-1,
        SDL_RENDERER_ACCELERATED|SDL_RENDERER_PRESENTVSYNC);
    if (!ren) { fprintf(stderr,"SDL_CreateRenderer: %s\n",SDL_GetError()); return 1; }

    SDL_SetRenderDrawBlendMode(ren,SDL_BLENDMODE_BLEND);

    SDL_Texture *earth_tex=SDL_CreateTexture(ren,
        SDL_PIXELFORMAT_ARGB8888,SDL_TEXTUREACCESS_STREAMING,WIN_W,WIN_H);
    if (!earth_tex) { fprintf(stderr,"SDL_CreateTexture: %s\n",SDL_GetError()); return 1; }

    printf("Building land mask..."); fflush(stdout);
    build_land_mask();
    printf(" done\n");

    /* Mercator y-range */
    g_map_y_max = log(tan(PI/4.0 + MAP_LAT_CLIP/2.0));

    printf("Building Mercator map texture (Natural Earth 1:50m)..."); fflush(stdout);
    SDL_Texture *map_tex = build_mercator_texture(ren);
    if (!map_tex) { fprintf(stderr,"build_mercator_texture failed\n"); return 1; }
    printf(" done\n");

    build_stars();

    Sat sats[NUM_SATS];
    for (int i=0; i<NUM_SATS; i++) sat_init(&sats[i],&ORBITS[i]);
    for (int i=0; i<GPS_N; i++)    gps_init(&gps_sats[i],&GPS_ORBITS[i]);

    double view_yaw   = 0.2;
    double view_pitch = 0.45;
    double zoom       = ZOOM_INIT;
    double earth_angle= 0.0;
    double sim_time   = 0.0;
    double sim_acc    = 0.0;
    double time_scale = 800.0;

    int paused            = 0;
    int dragging          = 0;
    int mx_prev=0, my_prev=0;
    int use_j2            = 0;
    int show_ground_track = 0;
    int map_mode          = 0;
    int show_gps          = 0;   /* C toggle — full GPS constellation */
    int show_fp           = 0;   /* F toggle — footprint circles */
    int recording         = 0;
    uint32_t frame_count  = 0;

    uint32_t t_prev=SDL_GetTicks();
    int running=1;
    SDL_Event ev;

    while (running) {
        /* ── Events ───────────────────────────────────── */
        while (SDL_PollEvent(&ev)) {
            switch (ev.type) {
            case SDL_QUIT: running=0; break;
            case SDL_KEYDOWN:
                switch (ev.key.keysym.sym) {
                case SDLK_ESCAPE: case SDLK_q: running=0; break;
                case SDLK_SPACE:
                    paused = !paused;
                    if (!paused) {
                        /* Discard any accumulated backlog so the simulation
                           resumes smoothly rather than jumping forward.      */
                        sim_acc = 0.0;
                        t_prev  = SDL_GetTicks();
                    }
                    break;
                case SDLK_p:      map_mode=!map_mode; break;
                case SDLK_r:
                    if (!map_mode) { view_yaw=0.2;view_pitch=0.45;zoom=ZOOM_INIT; }
                    break;
                case SDLK_EQUALS: case SDLK_PLUS:
                    time_scale=fmin(time_scale*2.0,1e6); break;
                case SDLK_MINUS:
                    time_scale=fmax(time_scale/2.0,1.0); break;
                case SDLK_j:
                    use_j2=!use_j2;
                    printf("J2: %s\n",use_j2?"ON":"OFF"); break;
                case SDLK_c:
                    show_gps=!show_gps; break;
                case SDLK_f:
                    show_fp=!show_fp; break;
                case SDLK_g:
                    if (!map_mode) show_ground_track=!show_ground_track;
                    break;
                case SDLK_v:
                    if (!recording) { if (video_start()) recording=1; }
                    else            { video_stop(); recording=0; }
                    break;
                case SDLK_1: case SDLK_2: case SDLK_3: case SDLK_4:
                case SDLK_5: case SDLK_6: case SDLK_7: case SDLK_8: {
                    int idx=ev.key.keysym.sym-SDLK_1;
                    if (idx<NUM_SATS) {
                        sats[idx].visible ^= 1;
                        if (sats[idx].visible) {
                            /* Re-enabled: insert a break sentinel so the
                               ground track renderer doesn't connect the old
                               track to the new one with a straight line.   */
                            Sat *s = &sats[idx];
                            s->gt_lat[s->gt_head] = GT_BREAK;
                            s->gt_lon[s->gt_head] = 0.0f;
                            s->gt_head = (s->gt_head + 1) % GT_LEN;
                            if (s->gt_cnt < GT_LEN) s->gt_cnt++;
                            /* Also discard stale 3D trail positions */
                            s->trail_cnt  = 0;
                            s->trail_head = 0;
                        }
                    }
                    break; }
                }
                break;
            case SDL_MOUSEBUTTONDOWN:
                if (!map_mode && ev.button.button==SDL_BUTTON_LEFT) {
                    dragging=1; mx_prev=ev.button.x; my_prev=ev.button.y;
                }
                break;
            case SDL_MOUSEBUTTONUP:
                if (ev.button.button==SDL_BUTTON_LEFT) dragging=0;
                break;
            case SDL_MOUSEMOTION:
                if (!map_mode && dragging) {
                    view_yaw  +=(ev.motion.x-mx_prev)*0.006;
                    view_pitch+=(ev.motion.y-my_prev)*0.006;
                    view_pitch =fmax(-PI/2.0+0.05,fmin(PI/2.0-0.05,view_pitch));
                    mx_prev=ev.motion.x; my_prev=ev.motion.y;
                }
                break;
            case SDL_MOUSEWHEEL:
                if (!map_mode) {
                    zoom*=(ev.wheel.y>0)?1.12:(1.0/1.12);
                    zoom=fmax(ZOOM_MIN,fmin(ZOOM_MAX,zoom));
                }
                break;
            }
        }

        /* ── Physics ──────────────────────────────────── */
        uint32_t t_now=SDL_GetTicks();
        double dt_real=(t_now-t_prev)*0.001;
        if (dt_real>0.1) dt_real=0.1;
        t_prev=t_now;

        if (!paused) {
            sim_acc += time_scale * dt_real;
            int steps = 0;
            while (sim_acc >= DT_PHYS && steps < 600) {
                for (int i=0; i<NUM_SATS; i++)
                    sat_step(&sats[i], DT_PHYS, use_j2, earth_angle); /* always */
                for (int i=0; i<GPS_N; i++)
                    gps_step(&gps_sats[i], DT_PHYS, use_j2);          /* always */
                earth_angle += OMEGA_E * DT_PHYS;
                sim_time    += DT_PHYS;
                sim_acc     -= DT_PHYS;
                steps++;
            }
            /* If physics is capped (time_scale too high to keep up),
               discard the excess rather than letting the backlog grow.
               Without this, resuming from pause would cause a large jump. */
            if (sim_acc > DT_PHYS) sim_acc = 0.0;
        }

        /* ── Render ───────────────────────────────────── */
        SDL_SetRenderDrawColor(ren,0,0,0,255);
        SDL_RenderClear(ren);

        if (map_mode) {
            /* ── 2D Mercator map mode ─────────────────── */
            render_map_view(ren,map_tex,sats,
                            earth_angle,sim_time,paused,
                            use_j2,show_gps,show_fp,recording,frame_count);
        } else {
            /* ── 3D sphere mode ───────────────────────── */
            for (int i=0; i<N_STARS; i++) {
                SDL_SetRenderDrawColor(ren,star_b[i],star_b[i],star_b[i],255);
                SDL_RenderDrawPoint(ren,star_x[i],star_y[i]);
            }

            double Rpx=BASE_R*zoom;
            void *raw; int pitch;
            SDL_LockTexture(earth_tex,NULL,&raw,&pitch);
            memset(raw,0,(size_t)pitch*WIN_H);
            render_earth((uint32_t*)raw,pitch/4,Rpx,view_yaw,view_pitch,earth_angle);
            SDL_UnlockTexture(earth_tex);
            SDL_RenderCopy(ren,earth_tex,NULL,NULL);

            for (int i=0; i<NUM_SATS; i++)
                if (sats[i].visible)
                    render_orbit(ren,sats[i].el,Rpx,view_yaw,view_pitch);

            if (show_ground_track)
                render_ground_tracks(ren,sats,Rpx,view_yaw,view_pitch,earth_angle);

            /* Footprints drawn before satellite dots so dots sit on top */
            if (show_fp) {
                for (int i=0; i<NUM_SATS; i++)
                    if (sats[i].visible)
                        render_footprint_3d(ren, sats[i].pos,
                                            sats[i].el->cr, sats[i].el->cg, sats[i].el->cb,
                                            Rpx, view_yaw, view_pitch);
                if (show_gps)
                    for (int i=0; i<GPS_N; i++)
                        render_footprint_3d(ren, gps_sats[i].pos,
                                            gps_sats[i].el->cr, gps_sats[i].el->cg, gps_sats[i].el->cb,
                                            Rpx, view_yaw, view_pitch);
            }

            if (show_gps) render_gps_3d(ren, Rpx, view_yaw, view_pitch);

            for (int i=0; i<NUM_SATS; i++)
                if (sats[i].visible)
                    render_satellite(ren,&sats[i],Rpx,view_yaw,view_pitch);

            render_hud(ren,sats,time_scale,sim_time,
                       paused,use_j2,show_ground_track,show_gps,show_fp,recording,frame_count);
        }

        if (recording) video_frame(ren);
        SDL_RenderPresent(ren);
        frame_count++;
    }

    if (recording) video_stop();
    SDL_DestroyTexture(map_tex);
    SDL_DestroyTexture(earth_tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
