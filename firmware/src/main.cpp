#include <Arduino.h>

#include "configuration.h"
#include "display_task.h"
#include "interface_task.h"
#include "motor_task.h"
#include "mqtt_task.h"
#include "hass_client.h"

Configuration config;

#if SK_DISPLAY
static DisplayTask display_task(0);
static DisplayTask *display_task_p = &display_task;
#else
static DisplayTask *display_task_p = nullptr;
#endif
static MotorTask motor_task(1, config);

static HassClient hass_client(1);

InterfaceTask interface_task(0, motor_task, display_task_p, hass_client);

#if SK_MQTT
static MQTTTask mqtt_task(0, motor_task, interface_task);
#endif

void setup()
{
#if SK_DISPLAY
  display_task.setLogger(&interface_task);
  display_task.begin();

  // Connect display to motor_task's knob state feed
  motor_task.addListener(display_task.getKnobStateQueue());
#endif

#if SK_MQTT
  // Connect mqtt to motor_task's knob state feed
  motor_task.addListener(mqtt_task.getKnobStateQueue());
  motor_task.addListener(hass_client.getKnobStateQueue());
#endif
  interface_task.begin();

  hass_client.setInterfaceTask(&interface_task); // Interface nach Erstellung setzen

  config.setLogger(&interface_task);
  config.loadFromDisk();

  interface_task.setConfiguration(&config);

  motor_task.setLogger(&interface_task);
  motor_task.begin();
  mqtt_task.begin();

  // Initialize HassClient after MQTT/WiFi is ready
  hass_client.begin();

  // Free up the Arduino loop task
  vTaskDelete(NULL);
}

void loop()
{
  char buf[50];
  interface_task.log("Stack high water:");
  snprintf(buf, sizeof(buf), "  main: %d", uxTaskGetStackHighWaterMark(NULL));
  interface_task.log(buf);
  #if SK_DISPLAY
    snprintf(buf, sizeof(buf), "  display: %d", uxTaskGetStackHighWaterMark(display_task.getHandle()));
    interface_task.log(buf);
  #endif
  snprintf(buf, sizeof(buf), "  motor: %d", uxTaskGetStackHighWaterMark(motor_task.getHandle()));
  interface_task.log(buf);
  snprintf(buf, sizeof(buf), "  interface: %d", uxTaskGetStackHighWaterMark(interface_task.getHandle()));
  interface_task.log(buf);
  #if SK_MQTT
    snprintf(buf, sizeof(buf), "  mqtt: %d", uxTaskGetStackHighWaterMark(mqtt_task.getHandle()));
    interface_task.log(buf);
    snprintf(buf, sizeof(buf), "  hass_client: %d", uxTaskGetStackHighWaterMark(hass_client.getHandle()));
    interface_task.log(buf);
  #endif
  snprintf(buf, sizeof(buf), "Heap -- free: %d, largest: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  interface_task.log(buf);
  vTaskDelay(pdMS_TO_TICKS(1000));
}