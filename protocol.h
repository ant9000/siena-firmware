#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <inttypes.h>
#include "net/netdev.h"

#define EMB_BROADCAST 0xff
#define EMB_SIGNATURE 0x00e0

#define DEFAULT_EMB_NETWORK 1
#define DEFAULT_EMB_ADDRESS 1

#ifndef EMB_ADDRESS
#define EMB_ADDRESS DEFAULT_EMB_ADDRESS
#endif

#ifndef EMB_NETWORK
#define EMB_NETWORK DEFAULT_EMB_NETWORK
#endif

typedef struct {
    uint16_t signature;
    uint16_t counter;
    uint16_t network;
    uint8_t dst;
    uint8_t src;
} embit_header_t;

#define EMB_HEADER_LEN  sizeof(embit_header_t)

typedef struct {
    embit_header_t header;
    char *payload;
    size_t payload_len;
    int16_t rssi;
    int8_t snr;
} embit_packet_t;

typedef ssize_t (consume_data_cb_t)(const embit_packet_t *packet);

void protocol_init(consume_data_cb_t *packet_consumer);
void protocol_in(const char *buffer, size_t len, int16_t *rssi, int8_t *snr);
void protocol_out(const embit_header_t *header, const char *buffer, const size_t len);

#endif /* PROTOCOL_H */
