include_directories(wifi-scan)

add_library(wifi-scan SHARED
        wifi-scan/wifi_scan.c
)
target_link_libraries(wifi-scan
        mnl
)

target_compile_options(wifi-scan PRIVATE -w)
