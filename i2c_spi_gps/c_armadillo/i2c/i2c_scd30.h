int scd30_read_measurement(char *bus, float *out);
int scd30_get_data_ready_status(char *bus);
int scd30_set_value(char *bus, uint16_t cmd, uint16_t write_data);
int scd30_reset(char *bus);
