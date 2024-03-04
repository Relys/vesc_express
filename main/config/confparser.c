// This file is autogenerated by VESC Tool

#include <string.h>
#include "buffer.h"
#include "conf_general.h"
#include "confparser.h"

int32_t confparser_serialize_main_config_t(uint8_t *buffer, const main_config_t *conf) {
	int32_t ind = 0;

	buffer_append_uint32(buffer, MAIN_CONFIG_T_SIGNATURE, &ind);

	buffer_append_int16(buffer, conf->controller_id, &ind);
	buffer[ind++] = conf->can_baud_rate;
	buffer_append_int16(buffer, conf->can_status_rate_hz, &ind);
	buffer[ind++] = conf->wifi_mode;
	strcpy((char*)buffer + ind, conf->wifi_sta_ssid);
	ind += strlen(conf->wifi_sta_ssid) + 1;
	strcpy((char*)buffer + ind, conf->wifi_sta_key);
	ind += strlen(conf->wifi_sta_key) + 1;
	strcpy((char*)buffer + ind, conf->wifi_ap_ssid);
	ind += strlen(conf->wifi_ap_ssid) + 1;
	strcpy((char*)buffer + ind, conf->wifi_ap_key);
	ind += strlen(conf->wifi_ap_key) + 1;
	buffer[ind++] = conf->use_tcp_local;
	buffer[ind++] = conf->use_tcp_hub;
	strcpy((char*)buffer + ind, conf->tcp_hub_url);
	ind += strlen(conf->tcp_hub_url) + 1;
	buffer_append_uint16(buffer, conf->tcp_hub_port, &ind);
	strcpy((char*)buffer + ind, conf->tcp_hub_id);
	ind += strlen(conf->tcp_hub_id) + 1;
	strcpy((char*)buffer + ind, conf->tcp_hub_pass);
	ind += strlen(conf->tcp_hub_pass) + 1;
	buffer[ind++] = conf->ble_mode;
	strcpy((char*)buffer + ind, conf->ble_name);
	ind += strlen(conf->ble_name) + 1;
	buffer_append_uint32(buffer, conf->ble_pin, &ind);
	buffer_append_uint32(buffer, conf->ble_service_capacity, &ind);
	buffer_append_uint32(buffer, conf->ble_chr_descr_capacity, &ind);
	buffer[ind++] = conf->pubremote_mode;

	return ind;
}

bool confparser_deserialize_main_config_t(const uint8_t *buffer, main_config_t *conf) {
	int32_t ind = 0;

	uint32_t signature = buffer_get_uint32(buffer, &ind);
	if (signature != MAIN_CONFIG_T_SIGNATURE) {
		return false;
	}

	conf->controller_id = buffer_get_int16(buffer, &ind);
	conf->can_baud_rate = buffer[ind++];
	conf->can_status_rate_hz = buffer_get_int16(buffer, &ind);
	conf->wifi_mode = buffer[ind++];
	strcpy(conf->wifi_sta_ssid, (char*)buffer + ind);
	ind += strlen(conf->wifi_sta_ssid) + 1;
	strcpy(conf->wifi_sta_key, (char*)buffer + ind);
	ind += strlen(conf->wifi_sta_key) + 1;
	strcpy(conf->wifi_ap_ssid, (char*)buffer + ind);
	ind += strlen(conf->wifi_ap_ssid) + 1;
	strcpy(conf->wifi_ap_key, (char*)buffer + ind);
	ind += strlen(conf->wifi_ap_key) + 1;
	conf->use_tcp_local = buffer[ind++];
	conf->use_tcp_hub = buffer[ind++];
	strcpy(conf->tcp_hub_url, (char*)buffer + ind);
	ind += strlen(conf->tcp_hub_url) + 1;
	conf->tcp_hub_port = buffer_get_uint16(buffer, &ind);
	strcpy(conf->tcp_hub_id, (char*)buffer + ind);
	ind += strlen(conf->tcp_hub_id) + 1;
	strcpy(conf->tcp_hub_pass, (char*)buffer + ind);
	ind += strlen(conf->tcp_hub_pass) + 1;
	conf->ble_mode = buffer[ind++];
	strcpy(conf->ble_name, (char*)buffer + ind);
	ind += strlen(conf->ble_name) + 1;
	conf->ble_pin = buffer_get_uint32(buffer, &ind);
	conf->ble_service_capacity = buffer_get_uint32(buffer, &ind);
	conf->ble_chr_descr_capacity = buffer_get_uint32(buffer, &ind);
	conf->pubremote_mode = buffer_get_uint32(buffer, &ind);
	return true;
}

void confparser_set_defaults_main_config_t(main_config_t *conf) {
	conf->controller_id = HW_DEFAULT_ID;
	conf->can_baud_rate = CONF_CAN_BAUD_RATE;
	conf->can_status_rate_hz = CONF_CAN_STATUS_RATE_HZ;
	conf->wifi_mode = CONF_WIFI_MODE;
	strcpy(conf->wifi_sta_ssid, CONF_WIFI_STA_SSID);
	strcpy(conf->wifi_sta_key, CONF_WIFI_STA_KEY);
	strcpy(conf->wifi_ap_ssid, CONF_WIFI_AP_SSID);
	strcpy(conf->wifi_ap_key, CONF_WIFI_AP_KEY);
	conf->use_tcp_local = CONF_USE_TCP_LOCAL;
	conf->use_tcp_hub = CONF_USE_TCP_HUB;
	strcpy(conf->tcp_hub_url, CONF_TCP_HUB_URL);
	conf->tcp_hub_port = CONF_TCP_HUB_PORT;
	strcpy(conf->tcp_hub_id, CONF_TCP_HUB_ID);
	strcpy(conf->tcp_hub_pass, CONF_TCP_HUB_PASS);
	conf->ble_mode = CONF_BLE_MODE;
	strcpy(conf->ble_name, CONF_BLE_NAME);
	conf->ble_pin = CONF_BLE_PIN;
	conf->ble_service_capacity = CONF_BLE_SERVICE_CAPACITY;
	conf->ble_chr_descr_capacity = CONF_BLE_CHR_DESCR_CAPACITY;
	conf->pubremote_mode = CONF_PUBREMOTE_MODE;
}

