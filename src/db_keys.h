#pragma once
#include <GyverDB.h>

// ============================================================
//  Ключи базы данных — макрос DB_KEYS создаёт namespace kk
//  с хэш-константами для каждого поля.
//
//  ВАЖНО: никогда не переставляй и не удаляй существующие
//  ключи из середины — безопасно только добавлять в конец.
// ============================================================

DB_KEYS(kk,
    // WiFi
    wifi_ssid,
    wifi_pass,
    wifi_apply,

    // MQTT
    mqtt_broker,
    mqtt_port,
    mqtt_user,
    mqtt_pass,
    mqtt_apply,

    // Устройство
    device_id,

    // Параметры сенсора
    pub_interval,
    sensor_maxdist,

    // Параметры VL53L1X
    vl53_threshold,

    // Логика двери
    door_approach_delta,  // на сколько см должно уменьшиться расстояние за 1 сек (приближение)
    door_open_dist,       // макс. дистанция при которой открывать (см)
    door_close_delay,     // задержка закрытия после освобождения проёма (мс)

    // Лейблы статуса (только для уникальных ID виджетов, значения не хранятся)
    lbl_wifi,
    lbl_mqtt,
    lbl_presence,
    lbl_moving,
    lbl_static,
    lbl_mov_dist,
    lbl_stat_dist,
    lbl_energy,
    lbl_vl53,
    lbl_door,
    lbl_ld2410
);
