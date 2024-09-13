#ifndef TEMP_DRV_H
#define TEMP_DRV_H

#include <esp_err.h>

extern esp_err_t spi_init(void);
extern float read_temp(void);

#endif
