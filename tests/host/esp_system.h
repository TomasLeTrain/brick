#pragma once
inline uint32_t esp_random() { static uint32_t session = 42; return ++session; }
