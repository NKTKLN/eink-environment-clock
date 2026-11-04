/* MQTT client with Home Assistant discovery. Driven from the main loop. */
#pragma once

#include <stdbool.h>

/* Call about once a second. Starts/stops/reconfigures the client when the
 * settings or the Wi-Fi state change and publishes state every
 * SENSOR_UPDATE_INTERVAL_S or right away when sensors_updated is set. */
void mqtt_ha_tick(bool sensors_updated);

/* Ask for a state publish on the next tick (notification changed, ...). */
void mqtt_ha_notify_changed(void);

bool mqtt_ha_connected(void);
