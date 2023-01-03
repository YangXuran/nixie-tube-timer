#ifndef	TUBE_DRV_H
#define	TUBE_DRV_H

#include <esp_err.h>

extern esp_err_t tube_init(void);
extern uint8_t get_tube_init_status(int32_t ch);
extern void tube_show_num_with_trans(int32_t ch, uint8_t num);
extern void tube_show_num(int32_t ch, uint8_t num);
extern void tube_test_num(void);
extern void tube_shutdown(int32_t ch);

#endif
