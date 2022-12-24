#include <stdio.h>
#include "soniclib.h"
#include "chirp_bsp.h"
#include "periph/gpio.h"
#include "periph/i2c.h"

#include "saml21_cpu_debug.h"
#include "saml21_backup_mode.h"
#include "ultrasound_display_config_info.h"

#define WAKEUP_PIN  GPIO_PIN(PA, 6)

#define CHIRP_SENSOR_FW_INIT_FUNC	    ch201_gprstr_init   /* standard STR firmware */
#define CHIRP_SENSOR_TARGET_INT_HIST	1		// num of previous results kept in history
#define CHIRP_SENSOR_TARGET_INT_THRESH  1		// num of target detections req'd to interrupt
#define CHIRP_SENSOR_TARGET_INT_RESET   0		// if non-zero, target filter resets after interrupt
#define	CHIRP_SENSOR_MAX_RANGE_MM		4000	/* maximum range, in mm */
#define	CHIRP_SENSOR_THRESHOLD_0		0	/* close range threshold (0 = use default) */
#define	CHIRP_SENSOR_THRESHOLD_1		0	/* standard threshold (0 = use default) */
#define	CHIRP_SENSOR_RX_HOLDOFF			0	/* # of samples to ignore at start of meas */
#define	CHIRP_SENSOR_RX_LOW_GAIN		0	/* # of samples (0 = use default) */
#define	CHIRP_SENSOR_TX_LENGTH			0	/* Tx pulse length, in cycles (0 = use default) */
#define	MEASUREMENT_INTERVAL_MS		    1000	// 1000ms interval = 1Hz sampling

/* Bit flags used in main loop to check for completion of I/O or timer operations.  */
#define DATA_READY_FLAG     (1 << 0)        // data ready from sensor

typedef struct {
    uint32_t        range;       // from ch_get_range()
    uint16_t        amplitude;   // from ch_get_amplitude()
    uint16_t        num_samples; // from ch_get_num_samples()
} chirp_data_t;

chirp_data_t chirp_data[CHIRP_MAX_NUM_SENSORS];
ch_dev_t     chirp_devices[CHIRP_MAX_NUM_SENSORS];
ch_group_t   chirp_group;

static uint32_t active_devices;
static uint8_t num_connected_sensors = 0;

static void handle_data_ready(void) {
    puts("DATA READY!");
/*
    uint8_t dev_num;
    uint8_t ret_val = 0;
    for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            chirp_data[dev_num].range = ch_get_range(dev_ptr, CH_RANGE_ECHO_ONE_WAY);
            if (chirp_data[dev_num].range != CH_NO_TARGET) {
                chirp_data[dev_num].amplitude = ch_get_amplitude(dev_ptr);
                uint32_t range =chirp_data[dev_num].range;
                printf("Port %d: Range: %0.1f mm   Amplitude: %u  ",
                        dev_num,
                        (float) range/32.0f,
                        chirp_data[dev_num].amplitude);
                printf("\n");
            }
        }
    }
*/
}

static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num,
                                ch_interrupt_type_t __attribute__((unused)) int_type)
{
    (void)dev_num;
    chdrv_int_group_interrupt_enable(grp_ptr);
}

void sensors_init(void)
{
    ch_group_t *grp_ptr = &chirp_group;
    uint8_t    chirp_error = 0;
    uint8_t    num_ports;
    uint8_t    dev_num;

	chbsp_board_init(grp_ptr);

	printf("\nTDK InvenSense CH-201 STR Example\n");
	printf("    Compile time:  %s %s\n", __DATE__, __TIME__);
	printf("    SonicLib version: %u.%u.%u\n", SONICLIB_VER_MAJOR, SONICLIB_VER_MINOR, SONICLIB_VER_REV);
	printf("\n");

    printf("Initializing sensor(s)...\n");
    num_ports = ch_get_num_ports(grp_ptr);
    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = &(chirp_devices[dev_num]);
        chirp_error |= ch_init(dev_ptr, grp_ptr, dev_num, CHIRP_SENSOR_FW_INIT_FUNC);
    }

    if (chirp_error == 0) {
        printf("starting group...\n");
        chirp_error = ch_group_start(grp_ptr);
    }

    if (chirp_error == 0) {
        printf("OK\n");
    } else {
        printf("FAILED: %d\n", chirp_error);
    }
    printf("\n");

    printf("Sensor\tType \t  Freq   \t  B/W  \t RTC Cal\tFirmware\n");
    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            printf("%d\tCH%d\t%u Hz\t%u Hz\t%u@%ums\t%s\n", dev_num,
                                            ch_get_part_number(dev_ptr),
                                            (unsigned int) ch_get_frequency(dev_ptr),
                                            (unsigned int) ch_get_bandwidth(dev_ptr),
                                            ch_get_rtc_cal_result(dev_ptr),
                                            ch_get_rtc_cal_pulselength(dev_ptr),
                                            ch_get_fw_version_string(dev_ptr));
        }
    }
    printf("\n");

    ch_io_int_callback_set(grp_ptr, sensor_int_callback);

    printf ("Configuring sensor(s)...\n");
    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_config_t dev_config;
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            num_connected_sensors++;
            active_devices |= (1 << dev_num);
            dev_config.mode = CH_MODE_FREERUN;
            dev_config.tgt_int_filter = CH_TGT_INT_FILTER_ANY;
            ch_set_target_int_counter(dev_ptr, CHIRP_SENSOR_TARGET_INT_HIST,
                                      CHIRP_SENSOR_TARGET_INT_THRESH, CHIRP_SENSOR_TARGET_INT_RESET);
            dev_config.max_range = CHIRP_SENSOR_MAX_RANGE_MM;
            dev_config.static_range = ch_mm_to_samples(dev_ptr, CHIRP_SENSOR_MAX_RANGE_MM);
            dev_config.sample_interval = MEASUREMENT_INTERVAL_MS;
            dev_config.thresh_ptr = NULL;
            /* Apply sensor configuration */
            chirp_error = ch_set_config(dev_ptr, &dev_config);
            /* Apply detection threshold settings, if specified */
            if (!chirp_error && (CHIRP_SENSOR_THRESHOLD_0 != 0)) {
                chirp_error = ch_set_threshold(dev_ptr, 0, CHIRP_SENSOR_THRESHOLD_0);
            }
            if (!chirp_error && (CHIRP_SENSOR_THRESHOLD_1 != 0)) {
                chirp_error = ch_set_threshold(dev_ptr, 1, CHIRP_SENSOR_THRESHOLD_1);
            }
            /* Apply other sensor settings, if not using defaults */
            if (!chirp_error && (CHIRP_SENSOR_RX_HOLDOFF != 0)) {
                chirp_error = ch_set_rx_holdoff(dev_ptr, CHIRP_SENSOR_RX_HOLDOFF);
            }
            if (!chirp_error && (CHIRP_SENSOR_RX_LOW_GAIN != 0)) {
                chirp_error = ch_set_rx_low_gain(dev_ptr, CHIRP_SENSOR_RX_LOW_GAIN);
            }
            if (!chirp_error && (CHIRP_SENSOR_TX_LENGTH != 0)) {
                chirp_error = ch_set_tx_length(dev_ptr, CHIRP_SENSOR_TX_LENGTH);
            }
            /* Enable sensor interrupt if using free-running mode
             *   Note that interrupt is automatically enabled if using
             *   triggered modes.
             */
            if ((!chirp_error) && (dev_config.mode == CH_MODE_FREERUN)) {
                chdrv_int_set_dir_in(dev_ptr);
                chdrv_int_interrupt_enable(dev_ptr);
            }
            /* Read back and display config settings */
            if (!chirp_error) {
                ultrasound_display_config_info(dev_ptr);
            } else {
                printf("Device %d: Error during ch_set_config()\n", dev_num);
            }
            /* Display detection thresholds */
            printf("\t\tthreshold_0 = %d\tthreshold_1 = %d\n", ch_get_threshold(dev_ptr, 0),
                                                 ch_get_threshold(dev_ptr, 1));
            /* Display other sensor settings */
            printf("\t\trx_holdoff = %d\t\trx_low_gain = %d\ttx_length = %d\n",
                                                ch_get_rx_holdoff(dev_ptr),
                                                ch_get_rx_low_gain(dev_ptr),
                                                ch_get_tx_length(dev_ptr));
            /* Display target interrupt filter setting */
            ch_tgt_int_filter_t filter = ch_get_target_interrupt(dev_ptr);
            printf("\t\tTarget interrupt filter: ");
            if (filter == CH_TGT_INT_FILTER_OFF) {
                printf("OFF");
            } else if (filter == CH_TGT_INT_FILTER_ANY) {
                printf("ANY");
            } else if (filter == CH_TGT_INT_FILTER_COUNTER) {
                printf("COUNTER");
            }
            if (filter == CH_TGT_INT_FILTER_COUNTER) {
                uint8_t meas_hist;
                uint8_t count_thresh;
                uint8_t reset;

                ch_get_target_int_counter(dev_ptr, &meas_hist, &count_thresh, &reset);
                printf("\n\t\t   meas_hist=%d  count_thresh=%d", meas_hist, count_thresh);
                printf("  reset=%s", reset ? "yes":"no");
            }
            printf("\n");
            /* Get number of active samples per measurement */
            chirp_data[dev_num].num_samples = ch_get_num_samples(dev_ptr);
            /* Turn on an LED to indicate device connected */
            if (!chirp_error) {
                chbsp_led_on(dev_num);
            }
        }
    }
    printf("\n");
}

int main(void)
{
    switch(saml21_wakeup_cause()) {
        case BACKUP_EXTWAKE:
            handle_data_ready();
            break;
        default:
            sensors_init();
            break;
    }
    puts("Entering backup mode.");
    saml21_backup_mode_enter(WAKEUP_PIN, -1);
    // never reached
    return 0;
}
