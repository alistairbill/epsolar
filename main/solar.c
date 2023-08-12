#include "solar.h"
#include "common.h"
#include "homie.h"
#include "modbus.h"

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

static const homie_property_t array_props[] = {
    {.id="voltage", .name="Voltage", .unit="V", .addr=0x3100, .format="%hu"},
    {.id="current", .name="Current", .unit="A", .addr=0x3101, .format="%hu"},
    {.id="power",   .name="Power",   .unit="W", .addr=0x3102, .format="%u"},
};

static const homie_property_t battery_props[] = {
    {.id="temperature", .name="Temperature", .unit="°C", .addr=0x3110, .format="%hd"},
    {.id="level", .name="Level", .unit="%", .addr=0x311a, .format="%hu"},
    {.id="status", .name="Status", .unit="", .addr=0x3200, .format="%hu"},
    {.id="voltage", .name="Voltage", .unit="V", .addr=0x331a, .format="%hu"},
    {.id="current", .name="Current", .unit="A", .addr=0x331b, .format="%d"},
};

static const homie_property_t device_props[] = {
    {.id="temperature", .name="Temperature", .unit="°C", .addr=0x3111, .format="%hd"},
    {.id="charging-status", .name="Charging status", .unit="", .addr=0x3201, .format="%hu"},
    {.id="discharging-status", .name="Discharging status", .unit="", .addr=0x3202, .format="%hu"},
};

static const homie_property_t load_props[] = {
    {.id="voltage", .name="Voltage", .unit="V", .addr=0x310c, .format="%hu"},
    {.id="current", .name="Current", .unit="A", .addr=0x310d, .format="%hu"},
    {.id="power",   .name="Power",   .unit="W", .addr=0x310e, .format="%u"},
};

static const homie_node_t nodes[] = {
    {.id = "array", .name = "Solar array", .properties = array_props, .nproperties = ARRAY_SIZE(array_props)},
    {.id = "battery", .name = "Battery", .properties = battery_props, .nproperties = ARRAY_SIZE(battery_props)},
    {.id = "device", .name = "Device", .properties = device_props, .nproperties = ARRAY_SIZE(device_props)},
    {.id = "load", .name = "Load", .properties = load_props, .nproperties = ARRAY_SIZE(load_props)},
};

void publish_node_attributes(void)
{
    char subtopic[HOMIE_MAX_TOPIC_LEN];
    char node_properties[128];
    size_t rem;
    for (size_t i = 0; i < ARRAY_SIZE(nodes); i++) {
        size_t len = snprintf(subtopic, sizeof(subtopic), "%s", nodes[i].id);
        snprintf(subtopic + len, sizeof(subtopic) - len, "/$name");
        homie_publish(subtopic, QOS_1, RETAINED, nodes[i].name);
        snprintf(subtopic + len, sizeof(subtopic) - len, "/$type");
        homie_publish(subtopic, QOS_1, RETAINED, "sensor");
        node_properties[0] = '\0';
        rem = sizeof(node_properties);
        for (size_t j = 0; j < nodes[i].nproperties; j++) {
            if (j > 0) {
                strncat(node_properties, ",", rem--);
            }
            size_t prop_len = snprintf(subtopic + len, sizeof(subtopic) - len, "/%s", nodes[i].properties[j].id);
            strncat(node_properties, nodes[i].properties[j].id, rem);
            rem -= (prop_len - 1);
            snprintf(subtopic + len + prop_len, sizeof(subtopic) - len - prop_len, "/$name");
            homie_publish(subtopic, QOS_1, RETAINED, nodes[i].properties[j].name);
            snprintf(subtopic + len + prop_len, sizeof(subtopic) - len - prop_len, "/$unit");
            homie_publish(subtopic, QOS_1, RETAINED, nodes[i].properties[j].unit);
            snprintf(subtopic + len + prop_len, sizeof(subtopic) - len - prop_len, "/$datatype");
            homie_publish(subtopic, QOS_1, RETAINED, "integer");
        }
        snprintf(subtopic + len, sizeof(subtopic) - len, "/$properties");
        homie_publish(subtopic, QOS_1, RETAINED, node_properties);
    }
}

void task_solar_read(void *args)
{
    QueueHandle_t queue = (QueueHandle_t)args;
    modbus_init();
    uint16_t buf[2];
    for (size_t i = 0; i < ARRAY_SIZE(nodes); i++) {
        for (size_t j = 0; j < nodes[i].nproperties; j++) {
            const homie_property_t *prop = &nodes[i].properties[j];
            memset(buf, 0, sizeof(buf));
            uint16_t num_registers = prop->format[1] == 'h' ? 1 : 2;
            int res = modbus_read_input_registers(1, prop->addr, num_registers, buf);
            queue_item_t reg_val = {
                .node = &nodes[i],
                .prop = prop,
                .val = buf[1] << 16 | buf[0],
            };
            if (res > 0) {
                xQueueSend(queue, &reg_val, 0);
            }
        }
    }
    xEventGroupSetBits(services_event_group, READ_COMPLETED_BIT);
    vTaskDelete(NULL);
}