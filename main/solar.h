#ifndef __SOLAR_H__
#define __SOLAR_H__

#include <stddef.h>
#include <stdint.h>

typedef struct {
    const char id[32];
    const char name[32];
    const char unit[4];
    const char format[4];
    const uint16_t addr;
} homie_property_t;

typedef struct {
    const char id[32];
    const char name[32];
    const homie_property_t *properties;
    const size_t nproperties;
} homie_node_t;

typedef struct {
    const homie_node_t *node;
    const homie_property_t *prop;
    uint32_t val;
} queue_item_t;

void publish_node_attributes(void);
void task_solar_read(void *args);

#endif