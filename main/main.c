#include <string.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_ieee802154.h>
#include <esp_log.h>
#include <esp_phy_init.h>
#include <esp_mac.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sdkconfig.h"

#include "802154_proto.h"

#define TAG "main"
#define RADIO_TAG "ieee802154"

#define PANID 0x4242
#define CHANNEL 19

#define SHORT_NOT_CONFIGURED 0xFFFE
#define SHORT_TEST_RECEIVER 0x2222

static QueueHandle_t packet_rx_queue = NULL;

static void debug_print_packet(uint8_t* packet, uint8_t packet_length);
static void debug_handler_task(void *pvParameters);

void esp_ieee802154_receive_done(uint8_t* frame, esp_ieee802154_frame_info_t* frame_info) {
    ESP_EARLY_LOGI(RADIO_TAG, "rx OK, received %d bytes", frame[0]);
    BaseType_t task;
    xQueueSendToBackFromISR(packet_rx_queue, frame, &task);
}

void esp_ieee802154_receive_failed(uint16_t error) {
    ESP_EARLY_LOGI(RADIO_TAG, "rx failed, error %d", error);
}

void esp_ieee802154_receive_sfd_done(void) {
    ESP_EARLY_LOGI(RADIO_TAG, "rx sfd done, Radio state: %d", esp_ieee802154_get_state());
}

void esp_ieee802154_transmit_done(const uint8_t *frame, const uint8_t *ack, esp_ieee802154_frame_info_t *ack_frame_info) {
    ESP_EARLY_LOGI(RADIO_TAG, "tx OK, sent %d bytes, ack %d", frame[0], ack != NULL);
}

void esp_ieee802154_transmit_failed(const uint8_t *frame, esp_ieee802154_tx_error_t error) {
    ESP_EARLY_LOGI(RADIO_TAG, "tx failed, error %d", error);
}

void esp_ieee802154_transmit_sfd_done(uint8_t *frame) {
    ESP_EARLY_LOGI(RADIO_TAG, "tx sfd done");
}

void app_main() {
    ESP_LOGI(TAG, "Initializing NVS from flash...");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    packet_rx_queue = xQueueCreate(8, 257);
    xTaskCreate(debug_handler_task, "debug_handler_task", 8192, NULL, 20, NULL);

    ESP_ERROR_CHECK(esp_ieee802154_enable());
    ESP_ERROR_CHECK(esp_ieee802154_set_coordinator(false));
    ESP_ERROR_CHECK(esp_ieee802154_set_promiscuous(false));
    ESP_ERROR_CHECK(esp_ieee802154_set_rx_when_idle(true));

    // esp_ieee802154_set_extended_address needs the MAC in reversed byte order
    uint8_t eui64[8] = {0};
    esp_read_mac(eui64, ESP_MAC_IEEE802154);
    uint8_t eui64_rev[8] = {0};
    for (int i=0; i<8; i++) {
        eui64_rev[7-i] = eui64[i];
    }
    esp_ieee802154_set_extended_address(eui64_rev);
    esp_ieee802154_set_short_address(SHORT_TEST_RECEIVER);
    ESP_ERROR_CHECK(esp_ieee802154_set_panid(PANID));
    ESP_ERROR_CHECK(esp_ieee802154_set_channel(CHANNEL));

    ESP_ERROR_CHECK(esp_ieee802154_receive());

#define MAKE_IT_WORK
#ifdef MAKE_IT_WORK
    // basically bogus data
    uint8_t bogus_message[8] = { 0x8, 0,0,0,0,0,0, 0};
    esp_ieee802154_transmit(bogus_message, false);
#endif

    uint8_t extended_address[8];
    esp_ieee802154_get_extended_address(extended_address);
    ESP_LOGI(TAG, "Receiver ready, panId=0x%04x, channel=%d, long=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, short=%04x",
             esp_ieee802154_get_panid(), esp_ieee802154_get_channel(),
             extended_address[0], extended_address[1], extended_address[2], extended_address[3],
             extended_address[4], extended_address[5], extended_address[6], extended_address[7],
             esp_ieee802154_get_short_address());

    // All done, the rest is up to handlers
    while (true) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void debug_handler_task(void *pvParameters) {
    uint8_t packet[257];
    while (xQueueReceive(packet_rx_queue, &packet, portMAX_DELAY) != pdFALSE) {
        debug_print_packet(&packet[1], packet[0]);
    }

    ESP_LOGE("debug_handler_task", "Terminated");
    vTaskDelete(NULL);
}


static void debug_print_packet(uint8_t* packet, uint8_t packet_length) {
    if (packet_length < sizeof(mac_fcs_t)) return;  // Can't be a packet if it's shorter than the frame control field

    uint8_t position = 0;

    mac_fcs_t* fcs = (mac_fcs_t*) &packet[position];
    position += sizeof(uint16_t);

    ESP_LOGI(RADIO_TAG, "Frame type:                   %x", fcs->frameType);
    ESP_LOGI(RADIO_TAG, "Security Enabled:             %s", fcs->secure ? "True" : "False");
    ESP_LOGI(RADIO_TAG, "Frame pending:                %s", fcs->framePending ? "True" : "False");
    ESP_LOGI(RADIO_TAG, "Acknowledge request:          %s", fcs->ackReqd ? "True" : "False");
    ESP_LOGI(RADIO_TAG, "PAN ID Compression:           %s", fcs->panIdCompressed ? "True" : "False");
    ESP_LOGI(RADIO_TAG, "Reserved:                     %s", fcs->rfu1 ? "True" : "False");
    ESP_LOGI(RADIO_TAG, "Sequence Number Suppression:  %s", fcs->sequenceNumberSuppression ? "True" : "False");
    ESP_LOGI(RADIO_TAG, "Information Elements Present: %s", fcs->informationElementsPresent ? "True" : "False");
    ESP_LOGI(RADIO_TAG, "Destination addressing mode:  %x", fcs->destAddrType);
    ESP_LOGI(RADIO_TAG, "Frame version:                %x", fcs->frameVer);
    ESP_LOGI(RADIO_TAG, "Source addressing mode:       %x", fcs->srcAddrType);

    if (fcs->rfu1) {
        ESP_LOGE(RADIO_TAG, "Reserved field 1 is set, ignoring packet");
        return;
    }

    switch (fcs->frameType) {
        case FRAME_TYPE_BEACON:
        {
            ESP_LOGI(RADIO_TAG, "Beacon");
            break;
        }
        case FRAME_TYPE_DATA:
        {
            uint8_t sequence_number = packet[position];
            position += sizeof(uint8_t);
            ESP_LOGI(RADIO_TAG, "Data (%u)", sequence_number);

            uint16_t pan_id         = 0;
            uint8_t  dst_addr[8]    = {0};
            uint8_t  src_addr[8]    = {0};
            uint16_t short_dst_addr = 0;
            uint16_t short_src_addr = 0;
            bool     broadcast      = false;

            switch (fcs->destAddrType) {
                case ADDR_MODE_NONE:
                {
                    ESP_LOGI(RADIO_TAG, "Without PAN ID or address field");
                    break;
                }
                case ADDR_MODE_SHORT:
                {
                    pan_id = *((uint16_t*) &packet[position]);
                    position += sizeof(uint16_t);
                    short_dst_addr = *((uint16_t*) &packet[position]);
                    position += sizeof(uint16_t);
                    if (pan_id == 0xFFFF && short_dst_addr == 0xFFFF) {
                        broadcast = true;
                        pan_id    = *((uint16_t*) &packet[position]);  // srcPan
                        position += sizeof(uint16_t);
                        ESP_LOGI(RADIO_TAG, "Broadcast on PAN %04x", pan_id);
                    } else {
                        ESP_LOGI(RADIO_TAG, "On PAN %04x to short address %04x", pan_id, short_dst_addr);
                    }
                    break;
                }
                case ADDR_MODE_LONG:
                {
                    pan_id = *((uint16_t*) &packet[position]);
                    position += sizeof(uint16_t);
                    for (uint8_t idx = 0; idx < sizeof(dst_addr); idx++) {
                        dst_addr[idx] = packet[position + sizeof(dst_addr) - 1 - idx];
                    }
                    position += sizeof(dst_addr);
                    ESP_LOGI(RADIO_TAG, "On PAN %04x to long address %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", pan_id, dst_addr[0],
                             dst_addr[1], dst_addr[2], dst_addr[3], dst_addr[4], dst_addr[5], dst_addr[6], dst_addr[7]);
                    break;
                }
                default:
                {
                    ESP_LOGE(RADIO_TAG, "With reserved destination address type, ignoring packet");
                    return;
                }
            }

            switch (fcs->srcAddrType) {
                case ADDR_MODE_NONE:
                {
                    ESP_LOGI(RADIO_TAG, "Originating from the PAN coordinator");
                    break;
                }
                case ADDR_MODE_SHORT:
                {
                    short_src_addr = *((uint16_t*) &packet[position]);
                    position += sizeof(uint16_t);
                    ESP_LOGI(RADIO_TAG, "Originating from short address %04x", short_src_addr);
                    break;
                }
                case ADDR_MODE_LONG:
                {
                    for (uint8_t idx = 0; idx < sizeof(src_addr); idx++) {
                        src_addr[idx] = packet[position + sizeof(src_addr) - 1 - idx];
                    }
                    position += sizeof(src_addr);
                    ESP_LOGI(RADIO_TAG, "Originating from long address %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", src_addr[0], src_addr[1],
                             src_addr[2], src_addr[3], src_addr[4], src_addr[5], src_addr[6], src_addr[7]);
                    break;
                }
                default:
                {
                    ESP_LOGE(RADIO_TAG, "With reserved source address type, ignoring packet");
                    return;
                }
            }

            uint8_t* header        = &packet[0];
            uint8_t  header_length = position;
            uint8_t* data          = &packet[position];
            uint8_t  data_length   = packet_length - position - sizeof(uint16_t);
            position += data_length;

            ESP_LOGI(RADIO_TAG, "Data length: %u", data_length);

            uint16_t checksum = *((uint16_t*) &packet[position]);

            ESP_LOGI(RADIO_TAG, "Checksum: %04x", checksum);

            ESP_LOGI(RADIO_TAG, "PAN %04x S %04x %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X to %04x %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X %s", pan_id,
                     short_src_addr, src_addr[0], src_addr[1], src_addr[2], src_addr[3], src_addr[4], src_addr[5], src_addr[6], src_addr[7],
                     short_dst_addr, dst_addr[0], dst_addr[1], dst_addr[2], dst_addr[3], dst_addr[4], dst_addr[5], dst_addr[6], dst_addr[7],
                     broadcast ? "BROADCAST" : "");

            if (broadcast)
                for (uint8_t idx = 0; idx < 8; idx++) dst_addr[idx] = 0xFF;

            break;
        }
        case FRAME_TYPE_ACK:
        {
            uint8_t sequence_number = packet[position++];
            ESP_LOGI(RADIO_TAG, "Ack (%u)", sequence_number);
            break;
        }
        default:
        {
            ESP_LOGE(RADIO_TAG, "Packet ignored because of frame type (%u)", fcs->frameType);
            break;
        }
    }
    ESP_LOGI(RADIO_TAG, "-----------------------");
}
