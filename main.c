#include <stdio.h>
#include "soniclib.h"
#include "chirp_bsp.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "periph/adc.h"

#include "lora.h"
#include "protocol.h"
#include "fram.h"
#include "saml21_cpu_debug.h"
#include "saml21_backup_mode.h"
#include "ultrasound_display_config_info.h"
#include "hdc3020.h"
#include "hdc3020_params.h"
#include "ztimer.h"
#include "periph/rtc_mem.h"


#if defined(BOARD_LORA3A_H10)
#define ADC_VCC    (0)
#define ADC_VPANEL (1)
#define VPANEL_ENABLE  GPIO_PIN(PA, 27)
#endif



#define EXTWAKE { .pin=EXTWAKE_PIN6, .polarity=EXTWAKE_LOW, .flags=EXTWAKE_IN }  // put EXTWAKE_LOW fo TDK present; EXTWAKE_HIGH for no TDK present

#define CHIRP_SENSOR_FW_INIT_FUNC	    ch201_gprstr_init   /* standard STR firmware */
#define CHIRP_SENSOR_TARGET_INT_HIST	1		// num of previous results kept in history
#define CHIRP_SENSOR_TARGET_INT_THRESH  1		// num of target detections req'd to interrupt
#define CHIRP_SENSOR_TARGET_INT_RESET   0		// if non-zero, target filter resets after interrupt
#define	CHIRP_SENSOR_MAX_RANGE_MM		1000	/* maximum range, in mm */
#define	CHIRP_SENSOR_THRESHOLD_0		0	/* close range threshold (0 = use default) */
#define	CHIRP_SENSOR_THRESHOLD_1		0	/* standard threshold (0 = use default) */
#define	CHIRP_SENSOR_RX_HOLDOFF			0	/* # of samples to ignore at start of meas */
#define	CHIRP_SENSOR_RX_LOW_GAIN		0	/* # of samples (0 = use default) */
#define	CHIRP_SENSOR_TX_LENGTH			0	/* Tx pulse length, in cycles (0 = use default) */
#define	MEASUREMENT_INTERVAL_MS		    1000	// 1000ms interval = 1Hz sampling

#ifndef EMB_ADDRESS
	#define EMB_ADDRESS 1
#endif

#ifndef EMB_NETWORK
#define EMB_NETWORK 1
#endif

#ifndef SLEEP_TIME_SEC
	#define SLEEP_TIME_SEC 5
#endif
#ifndef LISTEN_TIME_MSEC
	// use 450 if BW 125kHz; 150 if BW 500kHz
	#define LISTEN_TIME_MSEC 150
#endif

static embit_packet_t q_packet;
static int gateway_received_rssi;
static char q_payload[MAX_PACKET_LEN];
static kernel_pid_t main_pid;

/* Bit flags used in main loop to check for completion of I/O or timer operations.  */
#define DATA_READY_FLAG     (1 << 0)        // data ready from sensor

static saml21_extwake_t extwake = EXTWAKE;

ch_dev_t     chirp_devices[CHIRP_MAX_NUM_SENSORS];
ch_group_t   chirp_group;

static uint32_t active_devices;
static uint8_t num_connected_sensors = 0;
volatile uint32_t taskflags = 0;

static hdc3020_t hdc3020;

static lora_state_t lora;

char message[MAX_PACKET_LEN];

static struct {
    uint16_t sleep_seconds;
    uint16_t message_counter;
    int16_t last_rssi;
    int8_t last_snr;
    uint8_t tx_power;
    uint8_t boost;
    uint8_t retries;
    uint8_t tdkon;   // 0=off; 1=on
    uint16_t lastRange;  // maintain the range measured in the last wakeup to be able to check for changes
    uint8_t sensorStatus;
} persist;

static struct {
    uint8_t cpuid[CPUID_LEN];
    int32_t vcc;
    int32_t vpanel;
    double temp;
    double hum;
    uint32_t range;
    uint16_t amplitude;
} measures;

void waitCurrentMeasure(uint32_t milliseconds, char* step) {
	printf("waitCurrentMeasure %s\n", step);
	ztimer_sleep(ZTIMER_MSEC, milliseconds);
}

void parse_command(char *ptr, size_t len) {
	char *token;
	int8_t txpow=0;
	puts("parse_command\n");
    if((len > 2) && (strlen(ptr) == (size_t)(len-1)) && (ptr[0] == '@') && (ptr[len-2] == '$')) {
		token = strtok(ptr+1, ",");
        uint32_t seconds = strtoul(token, NULL, 0);
//seconds = 5;
        printf("Instructed to sleep for %lu seconds\n", seconds);

        persist.sleep_seconds = (seconds > 0 ) && (seconds < 36000) ? (uint16_t)seconds : SLEEP_TIME_SEC;
        if ((uint32_t)persist.sleep_seconds != seconds) {
            printf("Corrected sleep value: %u seconds\n", persist.sleep_seconds);
        }
        token = strtok(NULL, ",");

//token[0] = 'B';

        if (token[0]=='B') {
			printf("Boost out selected!\n");
			persist.boost = 1;
		} else {
			printf("RFO out selected!\n");
			persist.boost = 0;
		}
        token = strtok(NULL, "$");
        txpow = atoi(token);
//txpow = 14;
        if (txpow!=0) {
			persist.tx_power = txpow;
			printf("Instructed to tx at level %d\n",txpow);
		}
        // to be added a complete error recovery/received commands validation for transmission errors
    }
}

ssize_t packet_received(const embit_packet_t *packet)
{
    // discard packets if not for us and not broadcast
    printf("packet->header.dst = %d\n", packet->header.dst);
    if ((packet->header.dst != EMB_ADDRESS) && (packet->header.dst != EMB_BROADCAST)) { return 0; }


    // dump message to stdout
    printf(
        "{\"CNT\":%u,\"NET\":%u,\"DST\":%u,\"SRC\":%u,\"RSSI\":%d,\"SNR\":%d,\"LEN\"=%d,\"DATA\"=\"%s\"}\n",
        packet->header.counter, packet->header.network, packet->header.dst, packet->header.src,
        packet->rssi, packet->snr, packet->payload_len, packet->payload
    );
    msg_t msg;
    memcpy(q_payload, packet->payload, packet->payload_len);
    memcpy(&q_packet, packet, sizeof(embit_packet_t));
    q_packet.payload = q_payload;
    gateway_received_rssi = packet->rssi;
    msg.content.ptr = &q_packet;
    msg_send(&msg, main_pid);
    return 0;
}

void print_persist(char * step) {
	printf("%s Persist: seconds:%d counter:%d rssi:%d snr:%d power:%d boost:%d retries:%d tdkon:%d range:%d status:%d\n",
	step, persist.sleep_seconds, persist.message_counter, persist.last_rssi, persist.last_snr, persist.tx_power,
	persist.boost, persist.retries, persist.tdkon, persist.lastRange, persist.sensorStatus);
}

void listen_to(void) {
    msg_t msg;
    embit_packet_t *packet;
	// wait for a command
	lora_listen();
	if (ztimer_msg_receive_timeout(ZTIMER_MSEC, &msg, LISTEN_TIME_MSEC) != -ETIME) {
		// parse command
		packet = (embit_packet_t *)msg.content.ptr;
		persist.last_rssi = packet->rssi;
		persist.last_snr = packet->snr;
		persist.retries = 2;
		printf("rx rssi %d, rx snr %d, retries=%d\n",packet->rssi, packet->snr, persist.retries);
		char *ptr = packet->payload;
		size_t len = packet->payload_len;
		parse_command(ptr, len);
	} else {
		puts("No command received.");
		if (persist.retries > 0) {
			persist.retries--;
		} else {
			// elapsed max number of retries
			// set power to maximum and after 10 seconds send again.
			persist.boost = 1;
			persist.tx_power = 14;
		}
		persist.sleep_seconds = 25 + EMB_ADDRESS % 10;
		printf("retries = %d, new seconds for retry = %d\n", persist.retries, persist.sleep_seconds);
	}
	lora_off();
}

void send_to(uint8_t dst, char *buffer, size_t len)
{
    embit_header_t header;
    header.signature = EMB_SIGNATURE;
    header.counter = ++persist.message_counter;
    header.network = EMB_NETWORK;
    header.dst = dst;
    header.src = EMB_ADDRESS;
    printf("Sending %d+%d bytes packet #%u to 0x%02X signature:%x network:%x source:%d :\n", EMB_HEADER_LEN, len, persist.message_counter, dst, header.signature, header.network, header.src);
    printf("%s\n", buffer);
    if (lora_init(&(lora)) != 0) {
        puts("ERROR: cannot initialize radio.");
        puts("STOP.");
        return;
    }

    protocol_out(&header, buffer, len);
//  lora_off();
    puts("Sent.");
}

int read_word(uint8_t dev_num, uint16_t mem_addr, uint16_t * data_ptr)
{
    int error;
    // TODO
    uint8_t bus_num  = chirp_devices[dev_num].bus_index;
    uint8_t i2c_addr = chirp_devices[dev_num].i2c_address;
    i2c_acquire(bus_num);
    error = i2c_read_regs(bus_num, i2c_addr, mem_addr, (uint8_t *)data_ptr, 2, 0);
    i2c_release(bus_num);
    return error;
}

uint32_t get_range(uint8_t dev_num, ch_range_t range_type)
{
    uint8_t     tof_reg;
    uint8_t     tof_sf_reg;
    uint32_t    range = CH_NO_TARGET;
    uint16_t    time_of_flight;
    uint16_t    scale_factor;
    int         err;

    tof_reg = CH201_GPRSTR_REG_TOF;
    err = read_word(dev_num, tof_reg, &time_of_flight);
    if (!err && (time_of_flight != UINT16_MAX)) { // If object detected
        tof_sf_reg = CH201_GPRSTR_REG_TOF_SF;
        err = read_word(dev_num, tof_sf_reg, &scale_factor);
        if (scale_factor != 0) {
            uint32_t num = (CH_SPEEDOFSOUND_MPS * (uint32_t) chirp_group.rtc_cal_pulse_ms * (uint32_t) time_of_flight);
            uint32_t den = ((uint32_t) chirp_devices[dev_num].rtc_cal_result * (uint32_t) scale_factor) >> 11;
            range = (num / den);
            range *= 2;
            if (range_type == CH_RANGE_ECHO_ONE_WAY) {
                range /= 2;
            }
        }
    }
    return range;
}

uint16_t get_amplitude(uint8_t dev_num)
{
    uint8_t  amplitude_reg;
    uint16_t amplitude;
    amplitude_reg = CH201_GPRSTR_REG_AMPLITUDE;
    read_word(dev_num, amplitude_reg, &amplitude);
    return amplitude;
}

static void handle_data_ready(void)
{
    char message[MAX_PACKET_LEN];

    puts("DATA READY!");
    for (size_t i=0; i < CHIRP_MAX_NUM_SENSORS; i++) {
        if (chirp_devices[i].sensor_connected) {
            measures.range = get_range(i, CH_RANGE_ECHO_ONE_WAY);
            if (measures.range != CH_NO_TARGET) {
				persist.lastRange = (int)(measures.range/32.0f);
				measures.amplitude = get_amplitude(i);
				printf("Port %u   Range: %0.1f mm   Amplitude: %u\n", i, (float) measures.range/32.0f, measures.amplitude);
				snprintf(message, sizeof(message),
				"vcc:%ld,vpan:%ld,temp:%.2f,hum:%.2f,txp:%c:%d,rxdb:%d,rxsnr:%d,sleep:%d,Range(mm):%d,Ampl:%u",
				measures.vcc, measures.vpanel, measures.temp,
				measures.hum, persist.boost?'B':'R', persist.tx_power,
				persist.last_rssi, persist.last_snr, persist.sleep_seconds, (int)(measures.range/32.0f), measures.amplitude);
				send_to(EMB_BROADCAST, message, strlen(message)+1);
				listen_to();
            } else {
				printf("\nNO_TARGET !!! %f\n\n", (float)measures.range);
			}
        }
    }
}

static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num,
                                ch_interrupt_type_t __attribute__((unused)) int_type)
{
    (void)dev_num;
    taskflags = 1;
    chdrv_int_group_interrupt_enable(grp_ptr);
}

void internal_sensors_init(void)
{
    puts("Internal Sensors init.");
    if (hdc3020_init(&hdc3020, hdc3020_params) == HDC3020_OK) {
        puts("HDC3020 init.");
    }
}

void internal_sensors_read(void)
{
//    double temp = 999, hum = 999;
//    int32_t vcc;
//    int32_t vpanel;
//    char message[MAX_PACKET_LEN];

    puts("Internal Sensors read.");
    if (hdc3020_init(&hdc3020, hdc3020_params) == HDC3020_OK) {
        if (hdc3020_read(&hdc3020, &measures.temp, &measures.hum) == HDC3020_OK) {
            printf("Temp: %.1f °C, RH: %.1f %%\n", measures.temp, measures.hum);
        }
    }
    // read vcc
    measures.vcc = adc_sample(ADC_VCC, ADC_RES_12BIT)*4000/4095;  // corrected value (1V = 4095 counts)
    // read vpanel
    ztimer_sleep(ZTIMER_MSEC, 30);
    measures.vpanel = adc_sample(ADC_VPANEL, ADC_RES_12BIT)*3933/4095; // adapted to real resistor partition value (75k over 220k)
    printf("Vsupercap (mV): %ld; Vpanel(mV): %ld\n", measures.vcc, measures.vpanel);

    hdc3020_deinit(&hdc3020);

//    snprintf(message, sizeof(message), "Temp: %.1f °C, RH: %.1f %%, Vsupercap (mV): %ld, Vpanel(mV): %ld\n", temp, hum, vcc, vpanel);
//    send_to(EMB_BROADCAST, message, strlen(message)+1);
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
        }
    }
    printf("\n");

}

void freeze_data(void)
{
    size_t offset = 0;
    // WARNING: any pointer inside the structures will be dangling after a reset!!!
    fram_erase();
    fram_write(offset, (void *)&chirp_devices, sizeof(chirp_devices));
    offset += sizeof(chirp_devices);
    fram_write(offset, (void *)&chirp_group, sizeof(chirp_group));
    offset += sizeof(chirp_group);
    puts("Chirp sensor data freezed.");
}

void thaw_data(void)
{
    size_t offset = 0;
    // WARNING: any pointer inside the structures will be dangling after a reset!!!
    fram_read(offset, (void *)&chirp_devices, sizeof(chirp_devices));
    offset += sizeof(chirp_devices);
    fram_read(offset, (void *)&chirp_group, sizeof(chirp_group));
    offset += sizeof(chirp_group);
    puts("Chirp sensor data thawed.");
}

void board_startup(void)
{
    memset(&lora, 0, sizeof(lora));
    lora.bandwidth        = DEFAULT_LORA_BANDWIDTH;
    lora.spreading_factor = DEFAULT_LORA_SPREADING_FACTOR;
    lora.coderate         = DEFAULT_LORA_CODERATE;
    lora.channel          = DEFAULT_LORA_CHANNEL;
    lora.power            = DEFAULT_LORA_POWER;
    lora.data_cb          = *protocol_in;
    lora_off();

    fram_init();
}

void board_sleep(void)
{
    puts("Entering backup mode.");

    // turn radio off
//    lora_off();

    // turn off FRAM
    fram_off();

    // turn I2C devices off (leave internal bus I2C_DEV(0) alone)
    for(size_t i = 1; i < I2C_NUMOF; i++) {
        i2c_release(I2C_DEV(i));
        i2c_deinit_pins(I2C_DEV(i));
        gpio_init(i2c_config[i].scl_pin, GPIO_IN_PU);
        gpio_init(i2c_config[i].sda_pin, GPIO_IN_PU);
    }
//    persist.lastRange = measures.range;
    print_persist("GO TO SLEEP");
    rtc_mem_write(0, (char *)&persist, sizeof(persist));
printf("persist.sleep_seconds = %d but 60\n", persist.sleep_seconds);
    saml21_backup_mode_enter(RADIO_OFF_NOT_REQUESTED, extwake, 60);
}

void board_loop(void)
{
    puts("Starting measurements.");
    while (1) {
        if (taskflags==0) {
            chbsp_proc_sleep();
            /* We only continue here after an interrupt wakes the processor */
        } else {
            /* Sensor has interrupted - handle sensor data */
            taskflags = 0;
            handle_data_ready();
        }
    }
}

void persist_init(void) {
	// init persist values
	persist.message_counter = 0;
	persist.last_rssi = -1;
	persist.last_snr = -1;
	persist.tx_power = 14;
	persist.boost = 1;
	persist.retries = 2;
	persist.tdkon = 1;
	persist.lastRange = 9999;
	persist.sensorStatus = -1;
    persist.sleep_seconds = SLEEP_TIME_SEC;
}

int main(void)
{
	puts("\n");
	printf("SIENA-FIRMWARE Compiled: %s,%s\n", __DATE__, __TIME__);
    main_pid = thread_getpid();
    protocol_init(*packet_received);

    board_startup();
	printf("Sensor set: Address: %d Bandwidth: %d, Frequency: %ld\n            Spreading Factor: %d, Coderate: %d, Listen Time ms: %d\n",
			EMB_ADDRESS, lora.bandwidth, lora.channel, lora.spreading_factor, lora.coderate, LISTEN_TIME_MSEC);
	printf("Wakeup cause = %d\n\n",saml21_wakeup_cause());
    switch(saml21_wakeup_cause()) {
        case BACKUP_EXTWAKE:
	        rtc_mem_read(0, (char *)&persist, sizeof(persist));
	        lora.boost=persist.boost;
	        lora.power=persist.tx_power;
			print_persist("EXTWAKE");
			internal_sensors_read();
            thaw_data();
            handle_data_ready();
            break;
        case BACKUP_RTC:
	        rtc_mem_read(0, (char *)&persist, sizeof(persist));
	        lora.boost=persist.boost;
	        lora.power=persist.tx_power;
			print_persist("BACKUP_RTC");
			printf(" Periodic Wakeup !\n");
			internal_sensors_read();
			if (persist.tdkon == 0) {  // if tdk is off
				if (measures.vpanel > 1000) {
//				if (0) {
					sensors_init();
					freeze_data();
					persist.tdkon = 1;
				}
			} else {  // tdk is on
				if (measures.vpanel < 1000) {
//				if (0) {
					// switch off TDK to save power
//					gpio_clear(GPIO_PIN(PA, 31));
//			waitCurrentMeasure(10000, "waiting switch off TDK 10s");
					persist.tdkon = 0;
				}
			}
			persist.lastRange=9999;
			snprintf(message, sizeof(message),
			"vcc:%ld,vpan:%ld,temp:%.2f,hum:%.2f,txp:%c:%d,rxdb:%d,rxsnr:%d,sleep:%d,Range(mm):%d,Ampl:%u",
			measures.vcc, measures.vpanel, measures.temp,
			measures.hum, persist.boost?'B':'R', persist.tx_power,
			persist.last_rssi, persist.last_snr, persist.sleep_seconds, persist.lastRange, measures.amplitude);
			send_to(EMB_BROADCAST, message, strlen(message)+1);
			listen_to();
            break;
        default:
			snprintf(message, sizeof(message), "Start Demo Siena\n");
			send_to(EMB_BROADCAST, message, strlen(message)+1);
			puts("\nDefault ======================================================\n\n");
			persist_init();
            fram_off();
			gpio_clear(GPIO_PIN(PA, 31));
            internal_sensors_read();
//			if (measures.vpanel > 1000) {
			if (1) {
				sensors_init();
				freeze_data();
			}
            break;
    }
#ifdef BACKUP_MODE
    board_sleep();
#else
    board_loop();
#endif
    // never reached
    return 0;
}
