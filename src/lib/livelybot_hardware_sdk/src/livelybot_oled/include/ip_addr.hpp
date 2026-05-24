#include <cstdint>

#define LO_NET      "lo"
#define ETHERNET    "none"
#define WLAN        "wlan0"

void update_ip_addr(void);
uint32_t get_ip_data_u32(uint8_t i);
uint32_t* get_ip_data_u32_all(void);

