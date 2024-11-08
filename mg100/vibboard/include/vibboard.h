#include "sensor_adv_format.h"

#define VIBBOARD_NR 6
#define TD_NR_FEATURES 5
#define FD_NR_PEAKS 4

struct vibboard_device{
    int8_t device_id;
    char device_name[30];

    // Device state 0 means the device is transmitting the machine state, device state 1 means the device is transmitting the SP values, init -1
    int8_t vibboard_mode;
    int8_t device_state;
    uint16_t device_battery_voltage;

    // Machine state depends on the application, init in -1
    int8_t TD_machine_state;

    uint64_t last_relative_update_time;

    /*-----------TD----------*/
    uint8_t TD_acc_sp_data_scalars[TD_NR_FEATURES]; // vibration data is sent in signaled integer with 2 bytes that comes from a float multipled by TD_acc_mean_scalar

    float TD_acc_sp_data_x_axis[TD_NR_FEATURES];

    float TD_acc_sp_data_y_axis[TD_NR_FEATURES];

    float TD_acc_sp_data_z_axis[TD_NR_FEATURES];

    /*-----------FD----------*/
    uint8_t FD_acc_data_peaks_freq_scalar;
    uint8_t FD_acc_data_peaks_amp_scalar;

    int32_t FD_acc_data_peaks_freq_z_axis[FD_NR_PEAKS];
    int32_t FD_acc_data_peaks_amps_z_axis[FD_NR_PEAKS];
    int32_t FD_acc_data_peaks_freq_y_axis[FD_NR_PEAKS];
    int32_t FD_acc_data_peaks_amps_y_axis[FD_NR_PEAKS];
    int32_t FD_acc_data_peaks_freq_x_axis[FD_NR_PEAKS];
    int32_t FD_acc_data_peaks_amps_x_axis[FD_NR_PEAKS];
};

void vibboard_init_table(struct vibboard_device *vibboard_devices);
void vibboard_update_table(struct vibboard_device vibboard_aux_aux);
void print_vibboard_table(struct vibboard_device *vibboard_devices);
void start_scan_vibboard(struct vibboard_device* vibboard_devices);
bool MatchVibboardManufacturerID(struct net_buf_simple *ad);
bool MatchVibboardBT_ID(struct net_buf_simple *ad, struct vibboard_device *vibboard_aux_aux);
bool GetVibboardBT_Data(struct net_buf_simple *ad, struct vibboard_device *user_data);


void fill_vibboard_SP_data(struct vibboard_device **vibboard_device, struct bt_data *data);