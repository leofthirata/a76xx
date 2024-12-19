idf_component_register(
SRCS "test.cpp"
# SRCS "bridge.cpp"
INCLUDE_DIRS "."
PRIV_REQUIRES "A76XX" "esp_timer" "nvs_flash" "arduino" "LedBicolor"
)
