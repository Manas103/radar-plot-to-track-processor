#pragma once

#include <string>

namespace radar {

// Self-rolled SHA-256 (no external crypto dependency), used only to prove
// bit-for-bit determinism of a canonical track-history log, not for any
// security purpose. Same category of dependency-avoidance precedent as
// cross-platform-client-update-agent's self-rolled SHA-256/HMAC-SHA256.
std::string sha256_hex(const std::string& data);

} // namespace radar
