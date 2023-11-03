#ifndef RADRR_TRACKER_H
#define RADRR_TRACKER_H

// system
#include <memory>
#include <string>
#include <vector>
#include <math.h>

#include "../src/tracker/commonlib/DBSCAN.h"
#include "type.h"

int radar_track_main(std::vector<RadarDemo::radar_point_t> &new_meas);
#endif