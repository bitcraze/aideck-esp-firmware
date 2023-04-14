#include "cpx.h"
#include "freertos/FreeRTOS.h"

void cpxInitRoute(const CPXTarget_t source, const CPXTarget_t destination, const CPXFunction_t function, CPXRouting_t* route) {
    route->source = source;
    route->destination = destination;
    route->function = function;
    route->version = CPX_VERSION;
}

void cpxRouteToPacked(const CPXRouting_t* route, CPXRoutingPacked_t* packed) {
    packed->source = route->source;
    packed->destination = route->destination;
    packed->function = route->function;
    packed->version = route->version;
    packed->lastPacket = route->lastPacket;
}

void cpxPackedToRoute(const CPXRoutingPacked_t* packed, CPXRouting_t* route) {
    if(CPX_VERSION == packed->version)
    {
        route->version = packed->version;
        route->source = packed->source;
        route->destination = packed->destination;
        route->function = packed->function;
        route->lastPacket = packed->lastPacket;
    }
}
