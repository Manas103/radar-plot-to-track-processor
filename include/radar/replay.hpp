#pragma once

#include <istream>
#include <ostream>
#include <string>
#include <vector>

#include "radar/tracker.hpp"
#include "radar/types.hpp"

namespace radar {

// Plain-text session log format, used both as the "recorded scans" claim's
// record/replay medium and as the scenario input format the independent
// Python reference tracker reads. It is intentionally whitespace-delimited
// text (not a binary struct dump) so both languages can parse and write it
// with their own trivial, independent parsers instead of sharing a binary
// layout.
//
//   RADAR_SCAN_LOG v1
//   SCAN <scan_id> <timestamp_s> <count>
//   <x> <y>            (repeated count times)
//   SCAN <scan_id> ...
//   ...

void write_scan_log_header(std::ostream& os);
void write_scan(std::ostream& os, const ScanMessage& scan);

// Reads one scan from the stream. Returns false at clean EOF (no more
// "SCAN" records).
bool read_next_scan(std::istream& is, ScanMessage& out);

// One line of the canonical, hashable track-history log: the scan id, the
// confirmed/tentative counts, and the sorted list of confirmed track ids.
// Two runs over the same recorded input produce byte-identical
// concatenations of this line if and only if the pipeline is deterministic,
// which is exactly what the bit-for-bit replay claim measures.
std::string canonical_scan_line(const ScanResult& result, const std::vector<Track>& tracks);

} // namespace radar
