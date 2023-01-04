#include <stdio.h>
#include "fmt.h"
#include "od.h"

#include "net/netdev.h"
#include "mutex.h"

#include "protocol.h"
#include "lora.h"

static consume_data_cb_t *protocol_packet_consumer;

static mutex_t lora_lock = MUTEX_INIT;

#define ENABLE_DEBUG 0
#include "debug.h"
#define HEXDUMP(msg, buffer, len) if (ENABLE_DEBUG) { puts(msg); od_hex_dump((char *)buffer, len, 0); }

void protocol_init(consume_data_cb_t *packet_consumer)
{
    protocol_packet_consumer = packet_consumer;
}

void protocol_in(const char *buffer, size_t len, int16_t *rssi, int8_t *snr)
{
    embit_header_t *header;
    char *payload;
    size_t n;
    mutex_lock(&lora_lock);
    header = (embit_header_t *)(void const *)buffer;
    HEXDUMP("RECEIVED PACKET:", buffer, len);
    if ((header->signature == EMB_SIGNATURE) && (len > EMB_HEADER_LEN)) {
        payload = (char *)buffer + EMB_HEADER_LEN;
        n = len - EMB_HEADER_LEN;
        embit_packet_t packet = { .header = *header, .payload = payload, .payload_len = n, .rssi = *rssi, .snr = *snr };
        protocol_packet_consumer(&packet);
    }
    mutex_unlock(&lora_lock);
}

void protocol_out(const embit_header_t *header, const char *buffer, const size_t len)
{
    iolist_t packet, payload;
    mutex_lock(&lora_lock);
    packet.iol_base = (void *)header;
    packet.iol_len = EMB_HEADER_LEN;
    packet.iol_next = &payload;
    payload.iol_base = (void *)buffer;
    payload.iol_len = len;
    payload.iol_next = NULL;
    lora_write(&packet);
    mutex_unlock(&lora_lock);
}
