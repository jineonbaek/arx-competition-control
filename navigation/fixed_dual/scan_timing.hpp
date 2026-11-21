#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

struct ScanTiming {
    static constexpr std::uint32_t max_offset_ns = 150'000'000;
    static constexpr std::uint64_t max_timebase_difference_ns = 1'000'000;
    enum class Reason { none, point_count_mismatch, timebase_mismatch, offset_outside_scan };

    static const char* description(Reason reason) {
        switch (reason) {
            case Reason::point_count_mismatch: return "point count mismatch";
            case Reason::timebase_mismatch: return "header stamp and timebase mismatch";
            case Reason::offset_outside_scan: return "point offset outside scan";
            case Reason::none: return "";
        }
        return "unknown scan error";
    }

    struct Rejections {
        std::size_t total = 0;
        std::size_t consecutive = 0;
        std::uint32_t last_offset_ns = 0;
        std::int64_t last_stamp_ns = 0;
        Reason last_reason = Reason::none;
    };

    std::array<Rejections, 2> rejected{};

    template <typename Points>
    bool accept(int source, std::int64_t stamp_ns, std::uint64_t timebase_ns,
                std::size_t declared_count, const Points& points) {
        std::uint32_t largest = 0;
        for (const auto& point : points) {
            largest = std::max(largest, point.offset_time);
        }

        const auto stamp = static_cast<std::uint64_t>(stamp_ns);
        const auto difference = std::max(stamp, timebase_ns) - std::min(stamp, timebase_ns);
        Reason reason = Reason::none;
        if (declared_count != points.size()) reason = Reason::point_count_mismatch;
        else if (stamp_ns < 0 || difference > max_timebase_difference_ns) reason = Reason::timebase_mismatch;
        else if (largest > max_offset_ns) reason = Reason::offset_outside_scan;

        auto& stats = rejected.at(source);
        if (reason == Reason::none) {
            stats.consecutive = 0;
            return true;
        }

        ++stats.total;
        ++stats.consecutive;
        stats.last_offset_ns = largest;
        stats.last_stamp_ns = stamp_ns;
        stats.last_reason = reason;
        return false;
    }
};
