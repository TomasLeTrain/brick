#include "storage.h"

Storage* active;

Storage* get_active_store() {
    return active;
}

void Storage::init() {
    preferences.begin("potatomelt", false);
    active = this;
}

int Storage::get_target_rpm() {
    return preferences.getInt("target_rpm_index", 3);
}


void Storage::set_target_rpm(int rpm) {
    preferences.putInt("target_rpm_index", rpm);
}

void Storage::set_accel_correction(int rpm, float corr) {
    std::string key = "a_cor_" + std::to_string(rpm);
    preferences.putFloat(key.c_str(), corr);
    // Serial.printf("Setting accel factor. Key: %s val: %f \n", key.c_str(), corr);
}

float Storage::get_accel_correction(int rpm) {
    std::string key = "a_cor_" + std::to_string(rpm);
    float f = preferences.getFloat(key.c_str(), 0.0f);
    // Serial.printf("Getting accel factor. Key: %s val: %f \n", key.c_str(), f);
    return f;
}

int Storage::get_trans_trim() {
    return preferences.getInt("target_trans_trim_idx", 4);
}

void Storage::set_trans_trim(int idx) {
    preferences.putInt("target_trans_trim_idx", idx);
}
