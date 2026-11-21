#include "../scan_timing.hpp"
#include "../windows.hpp"

#include <cassert>
#include <vector>

namespace {
struct Point {
    uint32_t offset_time;
};
}

int main() {
    constexpr int64_t start = 1'000'000'000;
    constexpr int64_t step = 100'000'000;
    constexpr int front = 0;
    constexpr int rear = 1;
    ScanTiming timing;
    Windows windows;

    auto receive = [&](int source, int64_t stamp, const std::vector<Point>& points) {
        if (!timing.accept(source, stamp, stamp, points.size(), points)) return;
        std::vector<Sample> samples;
        for (const auto& point : points) {
            samples.push_back({stamp + point.offset_time, source});
        }
        windows.add(source, stamp, std::move(samples));
    };

    const std::vector<Point> normal(32, {20'000'000});
    auto corrupt = normal;
    corrupt.back().offset_time = 150'000'001;
    receive(front, start, normal);
    receive(rear, start, normal);
    receive(front, start + step, normal);
    receive(rear, start + step, corrupt);

    // Do not advance the source clock or insert the valid prefix of a bad frame.
    assert(windows.latest[rear] == start);
    assert(windows.pending.size() == 3 * normal.size());
    assert(!windows.pop());
    assert(timing.rejected[front].total == 0);
    assert(timing.rejected[rear].total == 1);
    assert(timing.rejected[rear].consecutive == 1);
    assert(timing.rejected[rear].last_offset_ns == 150'000'001);
    assert(timing.rejected[rear].last_stamp_ns == start + step);
    assert(timing.rejected[rear].last_reason == ScanTiming::Reason::offset_outside_scan);

    receive(rear, start + 2 * step, corrupt);
    assert(timing.rejected[rear].consecutive == 2);
    receive(front, start + 2 * step, normal);
    receive(rear, start + 3 * step, normal);
    receive(front, start + 3 * step, normal);
    receive(rear, start + 4 * step, normal);
    receive(front, start + 4 * step, normal);

    int complete = 0;
    while (auto window = windows.pop()) {
        if (!sufficient_sources(*window, windows.enabled)) continue;
        assert(window->start == start || window->start == start + 3 * step);
        ++complete;
    }
    assert(complete == 2);
    assert(timing.rejected[rear].total == 2);
    assert(timing.rejected[rear].consecutive == 0);
    const auto stamp = start + 5 * step;
    assert(!timing.accept(front, stamp, stamp, normal.size() + 1, normal));
    assert(timing.rejected[front].last_reason == ScanTiming::Reason::point_count_mismatch);
    assert(!timing.accept(front, stamp, stamp - 1'000'001, normal.size(), normal));
    assert(timing.rejected[front].last_reason == ScanTiming::Reason::timebase_mismatch);
    assert(timing.rejected[front].consecutive == 2);
    assert(timing.accept(front, stamp, stamp - 1'000'000, 1, std::vector<Point>{{150'000'000}}));
    assert(timing.rejected[front].total == 2);
    assert(timing.rejected[front].consecutive == 0);
}
