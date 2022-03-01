#include "cpx.h"

void cpxInitRoute(const CPXTarget_t source, const CPXTarget_t destination, const CPXFunction_t function, CPXRouting_t* route) {
    route->source = source;
    route->destination = destination;
    route->function = function;
}

void cpxRouteToPacked(const CPXRouting_t* route, CPXRoutingPacked_t* packed) {
    packed->source = route->source;
    packed->destination = route->destination;
    packed->function = route->function;
}

void cpxPackedToRoute(const CPXRoutingPacked_t* packed, CPXRouting_t* route) {
    route->source = packed->source;
    route->destination = packed->destination;
    route->function = packed->function;
}
