#include "autolabel/grid_prefilter.hpp"

#include <cmath>
#include <cstdlib>

namespace autolabel {

CellCoord cell_of(const Pose& pose, const PrefilterParams& params) {
    CellCoord c;
    c.cx = static_cast<std::int64_t>(std::floor(pose.x / params.cell_size_m));
    c.cy = static_cast<std::int64_t>(std::floor(pose.y / params.cell_size_m));
    return c;
}

bool cells_adjacent(const CellCoord& a, const CellCoord& b) {
    return std::abs(a.cx - b.cx) <= 1 && std::abs(a.cy - b.cy) <= 1;
}

} // namespace autolabel
