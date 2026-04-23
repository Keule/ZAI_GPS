#pragma once

#ifndef ESP_LOGE
#define ESP_LOGE(tag, fmt, ...) ((void)0)
#endif
#ifndef ESP_LOGW
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#endif
#ifndef ESP_LOGI
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#endif
#ifndef ESP_LOGD
#define ESP_LOGD(tag, fmt, ...) ((void)0)
#endif
#ifndef ESP_LOGV
#define ESP_LOGV(tag, fmt, ...) ((void)0)
#endif

typedef int esp_log_level_t;
#ifndef ESP_LOG_NONE
#define ESP_LOG_NONE 0
#endif
