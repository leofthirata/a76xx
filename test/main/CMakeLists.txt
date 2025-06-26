idf_component_register(
SRCS "test.cpp"
# SRCS "bridge.cpp"
INCLUDE_DIRS "."
PRIV_REQUIRES "a76xx" "esp_timer" "nvs_flash" "LedBicolor"
)
