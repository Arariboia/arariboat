#include <Arduino.h>
#include "mqtt_client.h"
#include "WiFi.h"

esp_mqtt_client_handle_t client;

static void log_error_if_nonzero(const char *message, int error_code)
{
     if (error_code != 0) {
          printf("Last error %s: 0x%x\n", message, error_code);
     }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
     printf("Event dispatched from event loop base=%s, event_id=%" PRIi32 "\n", base, event_id);
     esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
     esp_mqtt_client_handle_t client = event->client;
     int msg_id;
     switch ((esp_mqtt_event_id_t)event_id) {
     case MQTT_EVENT_CONNECTED:
          printf("MQTT_EVENT_CONNECTED\n");
          msg_id = esp_mqtt_client_publish(client, "arariboia", "Connected", 0, 1, 0);
          printf("sent publish successful, msg_id=%d\n", msg_id);

          msg_id = esp_mqtt_client_subscribe(client, "arariboia", 0);
          printf("sent subscribe successful, msg_id=%d\n", msg_id);

     case MQTT_EVENT_DISCONNECTED:
          printf("MQTT_EVENT_DISCONNECTED\n");
          break;

     case MQTT_EVENT_SUBSCRIBED:
          printf("MQTT_EVENT_SUBSCRIBED, msg_id=%d\n", event->msg_id);
          msg_id = esp_mqtt_client_publish(client, "arariboia", "data", 0, 0, 0);
          printf("sent publish successful, msg_id=%d\n", msg_id);
          break;
     case MQTT_EVENT_UNSUBSCRIBED:
          printf("MQTT_EVENT_UNSUBSCRIBED, msg_id=%d\n", event->msg_id);
          break;
     case MQTT_EVENT_PUBLISHED:
          printf("MQTT_EVENT_PUBLISHED, msg_id=%d\n", event->msg_id);
          break;
     case MQTT_EVENT_DATA:
          printf("MQTT_EVENT_DATA\n");
          printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
          printf("DATA=%.*s\r\n", event->data_len, event->data);
          break;
     case MQTT_EVENT_ERROR:
          printf("MQTT_EVENT_ERROR\n");
          if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                printf("Last errno string (%s)\n", strerror(event->error_handle->esp_transport_sock_errno));

          }
          break;
     default:
          printf("Other event id:%d\n", event->event_id);
          break;
     }
}

static void mqtt_app_start(void) {
    #define CONFIG_BROKER_URL "mqtt://64.181.181.169:1883"
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.uri = CONFIG_BROKER_URL;
	mqtt_cfg.username = "arariboia";
	mqtt_cfg.password = "arariboia123";

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

bool send_data_mqtt(const char* topic, const char* data) {
     if (client == NULL) {
          printf("MQTT client not initialized\n");
          return false;
     }

     int msg_id = esp_mqtt_client_publish(client, topic, data, 0, 0, 0);
     if (msg_id == -1) {
          printf("Failed to publish message\n");
          return false;
     }
     return true;
}

bool send_test_data() {
     const char* topic = "arariboia";
     static int count = 0;
     char data[50];
     snprintf(data, sizeof(data), "%d", count++);
     return send_data_mqtt(topic, data);
}

bool send_test_periodically(uint32_t interval_ms) {
     assert(interval_ms > 0);
     static unsigned long last_time = 0;
     if (millis() - last_time < interval_ms) {
          return false;
     }
     last_time = millis();
     return send_test_data();
}

void mqtt_task(void* parameter) {
	mqtt_app_start();
	while (true) {
		if (WiFi.status() != WL_CONNECTED) {
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}
		send_test_periodically(1000);
	}
}