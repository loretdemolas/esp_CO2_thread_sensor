#pragma once
#include "esp_err.h"

esp_err_t ot_manager_init(void);
void ot_manager_start_joiner(void);

void start_openThread(void);
void openthread_mainloop_task(void *arg);