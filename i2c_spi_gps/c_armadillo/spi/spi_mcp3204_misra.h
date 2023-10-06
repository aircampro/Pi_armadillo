typedef enum
{
        P0_N1,
        P1_N0,
        P2_N3,
        P3_N2
} ch_diff;

int mcp3204_access_misra(char *bus, uint8_t cmd, uint16_t *read_data);
int mcp3204_read_misra(char *bus, int ch, uint16_t *out);
int mcp3204_read_diff_misra(char *bus, ch_diff ch_conf, uint16_t *out);
