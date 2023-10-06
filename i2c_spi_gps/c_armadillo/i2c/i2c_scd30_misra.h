// This version has been modified to more suit MISRA complience from the originals
//
#ifndef __i2c_scd_misra_
#define __i2c_scd_misra_
uint16_t crc_calc_misra(uint8_t *in);
int32_t crc_check_misra(uint8_t *in);
int scd30_access_nogoto(char *bus, uint16_t cmd, int write_data_count, uint16_t write_data, int read_data_count, uint16_t *read_data);
int scd30_access_misra(char *bus, uint16_t cmd, int write_data_count, uint16_t write_data, int read_data_count, uint16_t *read_data);
int scd30_access_misra2(char *bus, uint16_t cmd, int write_data_count, uint16_t write_data, int read_data_count, uint16_t *read_data);
int scd30_read_measurement_misra(char *bus, float *out);
int scd30_get_data_ready_status_misra(char *bus);
int scd30_set_value_misra(char *bus, uint16_t cmd, uint16_t write_data);
int scd30_reset_misra(char *bus);
#endif
