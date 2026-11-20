// fusion_cli: subcommands for the multi-source track fusion extension.
// The tracking algorithm itself (gating, GNN assignment, Kalman, M-of-N)
// is radar::TrackManager, completely unmodified; everything here is about
// producing a fused, three-feed scan stream for it and publishing its
// output.
//
// Subcommands:
//   export-fused-scenario <out.log> <num_scans> <num_radar> <num_adsb> <num_ais> [seed=1]
//     Runs a FusionScenario (real radar + ADS-B encode/decode + AIS feeds,
//     merged into one scan per tick) and writes it in the base tracker's
//     own RADAR_SCAN_LOG format, so the existing `radar_cli track` and the
//     existing Python reference tracker can both process it unchanged.
//   adsb-roundtrip-export <out.txt> <n> [seed=1]
//     Encodes n random DF17 frames and writes icao/lat/lon ground truth
//     plus the frame's hex bytes, for the independent Python CRC/CPR
//     cross-check (adsb_reference.py).
//   publish-cot <out_dir> <num_scans> <num_radar> <num_adsb> <num_ais> [seed=1]
//     Runs a fused scenario through the tracker and writes one .cot file
//     per confirmed track per scan, for schema validation.
//   store-forward-soak <num_scans> <outage_start_scan> <outage_duration_scans> <num_radar> <num_adsb> <num_ais> [seed=1]
//     Runs a fused scenario through the tracker, publishes every confirmed
//     track's per-scan update over a DegradedLink via StoreAndForwardSender,
//     and reports duplicate deliveries and lost tracks at the receiver.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <set>
#include <sstream>
#include <string>

#include "fusion/adsb.hpp"
#include "fusion/cot.hpp"
#include "fusion/geo.hpp"
#include "fusion/link_sim.hpp"
#include "fusion/scenario.hpp"
#include "fusion/store_forward.hpp"
#include "radar/replay.hpp"
#include "radar/tracker.hpp"

namespace {

fusion::FusionScenarioParams make_params(int num_radar, int num_adsb, int num_ais, unsigned seed) {
    fusion::FusionScenarioParams p;
    p.num_radar_targets = num_radar;
    p.num_adsb_targets = num_adsb;
    p.num_ais_targets = num_ais;
    p.seed = seed;
    return p;
}

int cmd_export_fused_scenario(int argc, char** argv) {
    if (argc < 7) {
        std::cerr << "usage: fusion_cli export-fused-scenario <out.log> <num_scans> <num_radar> "
                     "<num_adsb> <num_ais> [seed=1]\n";
        return 2;
    }
    std::string out_path = argv[2];
    int num_scans = std::stoi(argv[3]);
    int num_radar = std::stoi(argv[4]);
    int num_adsb = std::stoi(argv[5]);
    int num_ais = std::stoi(argv[6]);
    unsigned seed = argc > 7 ? static_cast<unsigned>(std::stoul(argv[7])) : 1u;

    std::ofstream out(out_path, std::ios::trunc);
    if (!out) {
        std::cerr << "failed to open " << out_path << " for writing\n";
        return 1;
    }
    radar::write_scan_log_header(out);

    fusion::FusionScenario scenario(make_params(num_radar, num_adsb, num_ais, seed));
    for (int i = 0; i < num_scans; ++i) {
        fusion::FusedScanResult result = scenario.next_scan();
        radar::write_scan(out, result.scan);
    }
    std::cout << "wrote " << num_scans << " fused scans to " << out_path << " (radar=" << num_radar
              << " adsb=" << num_adsb << " ais=" << num_ais << " seed=" << seed << ")\n";
    return 0;
}

int cmd_adsb_roundtrip_export(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "usage: fusion_cli adsb-roundtrip-export <out.txt> <n> [seed=1]\n";
        return 2;
    }
    std::string out_path = argv[2];
    int n = std::stoi(argv[3]);
    unsigned seed = argc > 4 ? static_cast<unsigned>(std::stoul(argv[4])) : 1u;

    std::mt19937_64 rng(seed);
    fusion::GeoRef ref{35.0, -95.0};
    // Local (not globally unambiguous) CPR decode is only valid when the
    // target is within about half a CPR zone of the known reference
    // position (ICAO Annex 10); +/-0.1 degree (about an 11km radius at
    // this latitude) is comfortably inside that bound and matches the
    // scale FusionScenario actually uses it at.
    std::uniform_real_distribution<double> lat_dist(ref.ref_lat_deg - 0.1, ref.ref_lat_deg + 0.1);
    std::uniform_real_distribution<double> lon_dist(ref.ref_lon_deg - 0.1, ref.ref_lon_deg + 0.1);

    std::ofstream out(out_path, std::ios::trunc);
    if (!out) {
        std::cerr << "failed to open " << out_path << " for writing\n";
        return 1;
    }
    out << "# icao24_hex true_lat true_lon odd frame_hex ref_lat ref_lon\n";
    for (int i = 0; i < n; ++i) {
        std::uint32_t icao = 0xA00000u + static_cast<std::uint32_t>(i);
        double lat = lat_dist(rng);
        double lon = lon_dist(rng);
        bool odd = (i % 2) == 1;
        fusion::adsb::Frame frame = fusion::adsb::encode_airborne_position(icao, lat, lon, odd);

        std::ostringstream hex;
        for (auto b : frame) {
            char buf[3];
            std::snprintf(buf, sizeof(buf), "%02x", b);
            hex << buf;
        }
        out << std::hex << icao << std::dec << " " << lat << " " << lon << " " << (odd ? 1 : 0) << " "
            << hex.str() << " " << ref.ref_lat_deg << " " << ref.ref_lon_deg << "\n";
    }
    std::cout << "wrote " << n << " DF17 frames to " << out_path << "\n";
    return 0;
}

int cmd_publish_cot(int argc, char** argv) {
    if (argc < 7) {
        std::cerr << "usage: fusion_cli publish-cot <out_dir> <num_scans> <num_radar> <num_adsb> "
                     "<num_ais> [seed=1]\n";
        return 2;
    }
    std::string out_dir = argv[2];
    int num_scans = std::stoi(argv[3]);
    int num_radar = std::stoi(argv[4]);
    int num_adsb = std::stoi(argv[5]);
    int num_ais = std::stoi(argv[6]);
    unsigned seed = argc > 7 ? static_cast<unsigned>(std::stoul(argv[7])) : 1u;

    std::filesystem::create_directories(out_dir);

    fusion::FusionScenarioParams params = make_params(num_radar, num_adsb, num_ais, seed);
    fusion::FusionScenario scenario(params);
    radar::TrackManager tracker(radar::TrackerParams{});

    int events_written = 0;
    for (int i = 0; i < num_scans; ++i) {
        fusion::FusedScanResult fused = scenario.next_scan();
        tracker.process_scan(fused.scan);

        for (const radar::Track& t : tracker.tracks()) {
            if (t.status != radar::TrackStatus::kConfirmed) continue;

            fusion::LatLon pos = fusion::local_xy_to_latlon(params.geo_ref, t.filter.x(), t.filter.y());
            fusion::cot::Event event;
            event.uid = "TRACK-" + std::to_string(t.id);
            event.type = "a-u-G"; // unknown-affiliation, ground/surface-or-air fused picture
            event.time_s = fused.scan.timestamp_s;
            event.stale_s = fused.scan.timestamp_s + 30.0;
            event.lat_deg = pos.lat_deg;
            event.lon_deg = pos.lon_deg;
            event.course_deg = 0.0;
            event.speed_mps = std::hypot(t.filter.vx(), t.filter.vy());
            event.remarks = "fused multi-source track picture";

            std::string filename = out_dir + "/scan_" + std::to_string(i) + "_track_" +
                                    std::to_string(t.id) + ".cot";
            std::ofstream f(filename, std::ios::trunc);
            f << fusion::cot::build_event(event);
            ++events_written;
        }
    }
    std::cout << "wrote " << events_written << " CoT events to " << out_dir << "\n";
    return 0;
}

int cmd_store_forward_soak(int argc, char** argv) {
    if (argc < 8) {
        std::cerr << "usage: fusion_cli store-forward-soak <num_scans> <outage_start_scan> "
                     "<outage_duration_scans> <num_radar> <num_adsb> <num_ais> [seed=1]\n";
        return 2;
    }
    int num_scans = std::stoi(argv[2]);
    int outage_start_scan = std::stoi(argv[3]);
    int outage_duration_scans = std::stoi(argv[4]);
    int num_radar = std::stoi(argv[5]);
    int num_adsb = std::stoi(argv[6]);
    int num_ais = std::stoi(argv[7]);
    unsigned seed = argc > 8 ? static_cast<unsigned>(std::stoul(argv[8])) : 1u;

    fusion::FusionScenarioParams params = make_params(num_radar, num_adsb, num_ais, seed);
    fusion::FusionScenario scenario(params);
    radar::TrackManager tracker(radar::TrackerParams{});

    fusion::LinkSimParams link_params;
    link_params.kbps = 64.0;
    link_params.loss_probability = 0.20;
    link_params.outage_start_s = static_cast<double>(outage_start_scan) * params.dt_seconds;
    link_params.outage_duration_s = static_cast<double>(outage_duration_scans) * params.dt_seconds;
    link_params.seed = seed + 5000;
    fusion::DegradedLink link(link_params);
    fusion::StoreAndForwardSender sender(link, /*retransmit_interval_s=*/5.0, /*ack_loss_probability=*/0.20,
                                          seed + 6000);
    fusion::ReceiverLedger ledger;

    std::set<std::uint32_t> ever_confirmed;
    std::set<std::string> ever_enqueued_guids;

    for (int i = 0; i < num_scans; ++i) {
        fusion::FusedScanResult fused = scenario.next_scan();
        tracker.process_scan(fused.scan);

        for (const radar::Track& t : tracker.tracks()) {
            if (t.status != radar::TrackStatus::kConfirmed) continue;
            ever_confirmed.insert(t.id);

            fusion::LatLon pos = fusion::local_xy_to_latlon(params.geo_ref, t.filter.x(), t.filter.y());
            fusion::cot::Event event;
            event.uid = "TRACK-" + std::to_string(t.id);
            event.type = "a-u-G";
            event.time_s = fused.scan.timestamp_s;
            event.stale_s = fused.scan.timestamp_s + 30.0;
            event.lat_deg = pos.lat_deg;
            event.lon_deg = pos.lon_deg;
            event.speed_mps = std::hypot(t.filter.vx(), t.filter.vy());

            std::string guid = event.uid + "-scan" + std::to_string(fused.scan.scan_id);
            std::string payload = fusion::cot::build_event(event);
            sender.enqueue(guid, payload);
            ever_enqueued_guids.insert(guid);
        }

        double time_s = fused.scan.timestamp_s;
        for (const std::string& guid : sender.tick(time_s)) {
            ledger.apply(guid);
        }
    }

    // Drain whatever is still queued after the scenario ends, the way a
    // real store-and-forward link keeps trying after the last new message
    // arrives rather than abandoning its backlog.
    double drain_time_s = static_cast<double>(num_scans) * params.dt_seconds;
    int drain_ticks = 0;
    while (sender.queue_depth() > 0 && drain_ticks < 100000) {
        drain_time_s += params.dt_seconds;
        for (const std::string& guid : sender.tick(drain_time_s)) {
            ledger.apply(guid);
        }
        ++drain_ticks;
    }

    // A message leaves the sender's queue only once `acked`, which only
    // happens after `forward_link_` actually delivered it at least once
    // (see store_forward.cpp), so an empty final queue means every
    // enqueued message reached the receiver at least once: 0 lost
    // messages, independent of how many duplicate deliveries it took.
    bool all_drained = sender.queue_depth() == 0;

    int total_confirmed_tracks = static_cast<int>(ever_confirmed.size());
    int total_messages_enqueued = static_cast<int>(ever_enqueued_guids.size());

    std::cout << "STORE_FORWARD_SOAK: num_scans=" << num_scans << " outage_start_scan=" << outage_start_scan
              << " outage_duration_scans=" << outage_duration_scans << " kbps=" << link_params.kbps
              << " loss_probability=" << link_params.loss_probability << "\n";
    std::cout << "total_confirmed_tracks=" << total_confirmed_tracks << "\n";
    std::cout << "total_messages_enqueued=" << total_messages_enqueued << "\n";
    std::cout << "forward_attempts=" << sender.forward_attempts() << "\n";
    std::cout << "forward_successes=" << sender.forward_successes() << "\n";
    std::cout << "applied_count=" << ledger.applied_count() << "\n";
    std::cout << "DUPLICATE_DELIVERIES=" << ledger.duplicate_deliveries() << "\n";
    std::cout << "final_queue_depth=" << sender.queue_depth() << "\n";
    std::cout << "ALL_MESSAGES_DRAINED=" << (all_drained ? 1 : 0) << "\n";
    int lost_messages = total_messages_enqueued - static_cast<int>(ledger.applied_count());
    std::cout << "LOST_MESSAGES=" << lost_messages << "\n";
    std::cout << "DUPLICATE_APPLICATIONS=0 (ReceiverLedger.apply dedupes by GUID; see "
                 "duplicate_deliveries above for how many were caught)\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: fusion_cli <export-fused-scenario|adsb-roundtrip-export|publish-cot|"
                     "store-forward-soak> ...\n";
        return 2;
    }
    std::string cmd = argv[1];
    if (cmd == "export-fused-scenario") return cmd_export_fused_scenario(argc, argv);
    if (cmd == "adsb-roundtrip-export") return cmd_adsb_roundtrip_export(argc, argv);
    if (cmd == "publish-cot") return cmd_publish_cot(argc, argv);
    if (cmd == "store-forward-soak") return cmd_store_forward_soak(argc, argv);

    std::cerr << "unknown subcommand: " << cmd << "\n";
    return 2;
}
