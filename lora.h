#ifndef LORA_H
#define LORA_H

#include <inttypes.h>
#include "net/lora.h"
#include "net/netdev.h"
#include "sx127x.h"

#define MAX_PACKET_LEN  128

#define DEFAULT_LORA_BANDWIDTH        LORA_BW_500_KHZ
#define DEFAULT_LORA_SPREADING_FACTOR LORA_SF7
#define DEFAULT_LORA_CODERATE         LORA_CR_4_5
#define DEFAULT_LORA_CHANNEL          (867000000UL)
#define DEFAULT_LORA_POWER            SX127X_RADIO_TX_POWER

typedef void (lora_data_cb_t)(const char *buffer, size_t len, int16_t *rssi, int8_t *snr);

typedef struct {
    uint8_t bandwidth;
    uint8_t spreading_factor;
    uint8_t coderate;
    uint32_t channel;
    int16_t power;
    uint8_t boost;
    lora_data_cb_t *data_cb;
} lora_state_t;

int lora_init(const lora_state_t *state);
void lora_off(void);
uint8_t lora_get_power(void);
void lora_set_power(uint8_t power);
int lora_write(const iolist_t *packet);
void lora_listen(void);

#endif /* LORA_H */
