#include "utils.h"

void scan_i2c_bus(i2c_port_t i2c_port) {
    printf("\nScanning I2C bus on port %d...\n", i2c_port);
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    
    for (int addr = 0; addr < 128; addr++) {
        if (addr % 16 == 0) {
            printf("%02x: ", addr);
        }

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            printf("%02x ", addr);
        } else {
            printf("-- ");
        }
        
        if (addr % 16 == 15) {
            printf("\n");
        }
    }
    printf("\nScan complete.\n");
}