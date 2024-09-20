#include "sensor_adv_format.h"

#define VIBBOARD_NR 15
#define FD_NR_PEAKS 4

struct vibboard_device{
    int8_t device_id;
    char device_name[30];

    uint16_t last_relative_update_time;

    // Device state 0 means the device is transmitting the machine state, device state 1 means the device is transmitting the SP values, init -1
    int8_t TD_device_state;

    // Machine state depends on the application, init in -1
    int8_t TD_machine_state;

    /*-----------TD----------*/
    uint8_t TD_acc_mean_scalar; // vibration data is sent in signaled integer with 2 bytes that comes from a float multipled by TD_acc_mean_scalar
    uint8_t TD_acc_std_dev_scalar;
    uint8_t TD_acc_kurtosis_scalar;
    uint8_t TD_acc_skewness_scalar;
    uint8_t TD_acc_rms_scalar;

    float TD_acc_mean_x_axis;
    float TD_acc_std_dev_x_axis;
    float TD_acc_kurtosis_x_axis;
    float TD_acc_skewness_x_axis;
    float TD_acc_rms_x_axis;

    float TD_acc_mean_y_axis;
    float TD_acc_std_dev_y_axis;
    float TD_acc_kurtosis_y_axis;
    float TD_acc_skewness_y_axis;
    float TD_acc_rms_y_axis;

    float TD_acc_mean_z_axis;
    float TD_acc_std_dev_z_axis;
    float TD_acc_kurtosis_z_axis;
    float TD_acc_skewness_z_axis;
    float TD_acc_rms_z_axis;

    /*-----------FD----------*/
    uint8_t FD_acc_data_peaks_freq_scalar;
    uint8_t FD_acc_data_peaks_amp_scalar;

    uint16_t FD_acc_data_peaks_freq_z_axis[FD_NR_PEAKS];
    uint16_t FD_acc_data_peaks_amps_z_axis[FD_NR_PEAKS];
    uint16_t FD_acc_data_peaks_freq_y_axis[FD_NR_PEAKS];
    uint16_t FD_acc_data_peaks_amps_y_axis[FD_NR_PEAKS];
    uint16_t FD_acc_data_peaks_freq_x_axis[FD_NR_PEAKS];
    uint16_t FD_acc_data_peaks_amps_x_axis[FD_NR_PEAKS];
};

void vibboard_init_table(struct vibboard_device *vibboard_devices);
void vibboard_update_table(struct vibboard_device vibboard_aux_aux);
void print_vibboard_table(struct vibboard_device *vibboard_devices);
void start_scan_vibboard(struct vibboard_device* vibboard_devices);
bool MatchVibboard(struct net_buf_simple *ad);

void fill_vibboard_SP_data(struct vibboard_device **vibboard_device, struct bt_data *data);