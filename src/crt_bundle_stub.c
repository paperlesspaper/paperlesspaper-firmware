#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>

// Stubs to decouple the 69 KB Mozilla root certificate bundle.
// AWS IoT uses explicit device and CA certificates via net.setCACert().
// If no bundle is attached, these stubs avoid pulling in x509_crt_bundle.S.obj.

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t esp_crt_bundle_attach(void* conf) {
   return ESP_FAIL;
}

void esp_crt_bundle_detach(void* conf) {
}

esp_err_t esp_crt_bundle_set(const unsigned char* x509_bundle, size_t bundle_size) {
   return ESP_FAIL;
}

bool esp_crt_bundle_in_use(const void* ca_chain) {
   return false;
}

#ifdef __cplusplus
}
#endif
