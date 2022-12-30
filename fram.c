#include <stdio.h>

#include "at24cxxx.h"
#include "at24cxxx_params.h"

static at24cxxx_t at24cxxx_dev;

int fram_init(void)
{
    printf("EEPROM size: %u byte\n", AT24CXXX_EEPROM_SIZE);
    printf("Page size  : %u byte\n", AT24CXXX_PAGE_SIZE);
    int status = at24cxxx_init(&at24cxxx_dev, &at24cxxx_params[0]);
    if (status != AT24CXXX_OK) {
        printf("ERROR: FRAM initialization failed (%d)\n", status);
        return 1;
    }
    puts("FRAM initialized successfully");
    return 0;
}

int fram_read(uint32_t pos, void *data, size_t len)
{
    if (pos + len > AT24CXXX_EEPROM_SIZE) {
        puts("Failed: cannot read out of FRAM bounds");
        return 1;
    }
    int status = at24cxxx_read(&at24cxxx_dev, pos, data, len);
    if (status != AT24CXXX_OK) {
        printf("ERROR: read from FRAM failed (%d)\n", status);
        return 1;
    }
    return 0;
}

int fram_write(uint32_t pos, uint8_t *data, size_t len)
{
    if (pos + len > AT24CXXX_EEPROM_SIZE) {
        puts("Failed: cannot write out of FRAM bounds");
        return 1;
    }
    int status = at24cxxx_write(&at24cxxx_dev, pos, data, len);
    if (status != AT24CXXX_OK) {
        printf("ERROR: write to FRAM failed (%d)\n", status);
        return 1;
    }
    return 0;
}

int fram_erase(void)
{
    int status = at24cxxx_erase(&at24cxxx_dev);
    if (status != AT24CXXX_OK) {
        printf("ERROR: FRAM erase failed (%d)\n", status);
        return 1;
    }
    puts("FRAM erased");
    return 0;
}
