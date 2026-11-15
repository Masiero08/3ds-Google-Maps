#ifndef MAP_ROUTE_STEPS_H
#define MAP_ROUTE_STEPS_H
/*
 * Turn-by-turn maneuver list for an active route.
 *
 * OSRM returns one "step" per maneuver (turn, roundabout, arrival...).
 * This module keeps those maneuvers, places each one along the route
 * geometry (in metres from the start), and answers "what is the next
 * instruction and how far away is it" from RouteFollow's progress.
 * It is platform independent so it can be tested on the host.
 */
#include <stdbool.h>
#include <stddef.h>
#include "route_follow.h"

#define ROUTE_STEPS_MAX 512
#define ROUTE_STEP_NAME_MAX 48

typedef enum {
    ROUTE_STEP_OTHER = 0,
    ROUTE_STEP_DEPART,
    ROUTE_STEP_TURN,
    ROUTE_STEP_NEW_NAME,
    ROUTE_STEP_CONTINUE,
    ROUTE_STEP_MERGE,
    ROUTE_STEP_ON_RAMP,
    ROUTE_STEP_OFF_RAMP,
    ROUTE_STEP_FORK,
    ROUTE_STEP_END_OF_ROAD,
    ROUTE_STEP_ROUNDABOUT,     /* roundabout, rotary, roundabout turn */
    ROUTE_STEP_EXIT_ROUNDABOUT,
    ROUTE_STEP_ARRIVE
} RouteStepType;

typedef enum {
    ROUTE_MOD_NONE = 0,
    ROUTE_MOD_UTURN,
    ROUTE_MOD_SHARP_RIGHT,
    ROUTE_MOD_RIGHT,
    ROUTE_MOD_SLIGHT_RIGHT,
    ROUTE_MOD_STRAIGHT,
    ROUTE_MOD_SLIGHT_LEFT,
    ROUTE_MOD_LEFT,
    ROUTE_MOD_SHARP_LEFT
} RouteStepModifier;

typedef enum {
    ROUTE_LANG_EN = 0,
    ROUTE_LANG_PT_BR
} RouteLang;

typedef struct {
    double lat, lon;          /* maneuver location */
    RouteStepType type;
    RouteStepModifier modifier;
    int exit;                 /* roundabout exit number, 0 if unknown */
    char name[ROUTE_STEP_NAME_MAX]; /* road name after the maneuver */
    double along_m;           /* filled by route_steps_attach */
} RouteStep;

typedef struct {
    RouteStep steps[ROUTE_STEPS_MAX];
    size_t count;
    bool attached;
} RouteSteps;

/* Map OSRM maneuver strings ("turn", "slight left"...). NULL is accepted. */
RouteStepType route_step_type_from_osrm(const char *type);
RouteStepModifier route_step_modifier_from_osrm(const char *modifier);

void route_steps_clear(RouteSteps *s);
/* Append one maneuver. Returns false when full or the location is invalid. */
bool route_steps_add(RouteSteps *s, double lat, double lon, RouteStepType type,
                     RouteStepModifier modifier, int exit, const char *name);
/* Place every maneuver along the route geometry. Call after
 * route_follow_begin() succeeded. Maneuvers are kept in order, so each one
 * is searched only from the previous maneuver onwards (routes that pass the
 * same point twice stay correct). */
bool route_steps_attach(RouteSteps *s, const RouteFollow *r);

/* Next maneuver ahead of the current progress, or NULL. The departure step
 * is never returned. distance_m receives the distance in metres to it. */
const RouteStep *route_steps_next(const RouteSteps *s, const RouteFollow *r,
                                  double *distance_m);

/* "Vire à direita em 200 m" / "Turn right in 200 m". With distance_m < 0
 * the distance part is left out. Returns false if buf is too small. */
bool route_step_format(const RouteStep *step, double distance_m, RouteLang lang,
                       char *buf, size_t size);
/* Metric distance text: "50 m", "350 m", "1,2 km" (pt-BR) / "1.2 km". */
void route_format_distance(double metres, RouteLang lang, char *buf, size_t size);
#endif
