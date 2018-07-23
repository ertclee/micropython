/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ayke van Laethem
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#if MICROPY_PY_BLUETOOTH

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "py/mperrno.h"
#include "extmod/modbluetooth.h"


// Semaphore to serialze asynchronous calls.
STATIC SemaphoreHandle_t mp_bt_call_complete;
esp_bt_status_t mp_bt_call_status;

STATIC void mp_bt_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);


// Convert an esp_err_t into an errno number.
STATIC int mp_bt_esp_errno(esp_err_t err) {
    if (err != 0) {
        return MP_EPERM;
    }
    return 0;
}


// Convert the result of an asynchronous call to an errno value.
STATIC int mp_bt_status_errno() {
    if (mp_bt_call_status != ESP_BT_STATUS_SUCCESS) {
        return MP_EPERM;
    }
    return 0;
}


// Initialize at early boot.
void mp_bt_init(void) {
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    mp_bt_call_complete = xSemaphoreCreateBinary();
}


int mp_bt_enable(void) {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_bt_controller_init(&bt_cfg);
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    err = esp_bluedroid_init();
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    err = esp_bluedroid_enable();
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    err = esp_ble_gap_register_callback(mp_bt_gap_callback);
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    return 0;
}


void mp_bt_disable(void) {
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
}


bool mp_bt_is_enabled(void) {
    return esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED;
}


int mp_bt_advertise_start(mp_bt_adv_type_t type, uint16_t interval, uint8_t *adv_data, size_t adv_data_len, uint8_t *sr_data, size_t sr_data_len) {
    if (adv_data != NULL) {
        esp_err_t err = esp_ble_gap_config_adv_data_raw(adv_data, adv_data_len);
        if (err != 0) {
            return mp_bt_esp_errno(err);
        }
        xSemaphoreTake(mp_bt_call_complete, portMAX_DELAY);
    }

    if (sr_data != NULL) {
        esp_err_t err = esp_ble_gap_config_scan_rsp_data_raw(sr_data, sr_data_len);
        if (err != 0) {
            return mp_bt_esp_errno(err);
        }
        xSemaphoreTake(mp_bt_call_complete, portMAX_DELAY);
    }

    esp_ble_adv_params_t ble_adv_params = {0,
        .adv_int_min       = interval,
        .adv_int_max       = interval,
        .adv_type          = type,
        .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
        .channel_map       = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };
    esp_err_t err = esp_ble_gap_start_advertising(&ble_adv_params);
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    xSemaphoreTake(mp_bt_call_complete, portMAX_DELAY);
    return mp_bt_status_errno();
}


void mp_bt_advertise_stop(void) {
    esp_err_t err = esp_ble_gap_stop_advertising();
    if (err != 0) {
        return;
    }
    xSemaphoreTake(mp_bt_call_complete, portMAX_DELAY);
}


// Event callbacks. Most API calls generate an event here to report the
// result.
STATIC void mp_bt_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            xSemaphoreGive(mp_bt_call_complete);
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            xSemaphoreGive(mp_bt_call_complete);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            mp_bt_call_status = param->adv_start_cmpl.status;
            xSemaphoreGive(mp_bt_call_complete);
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            mp_bt_call_status = param->adv_stop_cmpl.status;
            xSemaphoreGive(mp_bt_call_complete);
            break;
        default:
            ESP_LOGI("bluetooth", "unknown GAP event: %d", event);
            break;
    }
}

#endif // MICROPY_PY_BLUETOOTH
