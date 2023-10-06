typedef enum
{
        P0_N1,
        P1_N0,
        P2_N3,
        P3_N2
} ch_diff;

int mcp3204_read(char *bus, int ch, uint16_t *out);
int mcp3204_read_diff(char *bus, ch_diff ch_conf, uint16_t *out);
