#pragma once

#include "esp_check.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_ERROR_CHECK_RETURN_ERR(x) ESP_RETURN_ON_ERROR(x, TAG, "")
#define BSP_ERROR_CHECK_RETURN_NULL(x) ESP_RETURN_ON_FALSE(x == ESP_OK, NULL, TAG, "")
#define BSP_NULL_CHECK(x, ret) ESP_RETURN_ON_FALSE(x, ret, TAG, "")
#define BSP_NULL_CHECK_GOTO(x, goto_tag) ESP_GOTO_ON_FALSE(x, ESP_ERR_NO_MEM, goto_tag, TAG, "")

#ifdef __cplusplus
}
#endif
