#pragma once

#include <string>
#include <vector>

#include "iou.h"
#include "kalman.h"
#include "lshape.h"
#include "hungarian_optimizer.h"

#include "../common.h"

class lidar_tracker
{

public:
    lidar_tracker(/* args */) {
        optimizer_ = new assign::HungarianOptimizer<double>();
        optimizer_->costs()->Reserve(100, 100);
    };
    ~lidar_tracker() {
        track_list.clear();
        track_list.shrink_to_fit();
        delete optimizer_;
    };

    void tracking_proc(std::vector<Bndbox>& det_boxes);


public:
    std::vector<simple_tracker> track_list;

private:
    const bool USING_MODEL_DETECTION = true;
    bool DEBUG_SWITCH = false;

    assign::HungarianOptimizer<double>* optimizer_ = nullptr;
    std::vector<box_t> trace_box_list;
    std::vector<box_t> det_box_list;

private:
    void lidar_trace_predict(double dt);

    void creat_trace_box(std::vector<box_t>& trace_box_list);

    void creat_det_box(std::vector<Bndbox>& det_boxes,
        std::vector<box_t>& det_box_list);

    void re_compute_shape(box_t& sub_det_box);

    void match_trace_wiht_det(std::vector<box_t>& trace_box_list,
        std::vector<box_t>& det_box_list);

    std::vector<greedy_match_info_t> build_cost_matrix(
        std::vector<box_t>& trace_box_list, std::vector<box_t>& det_box_list);

    void assignment_used_for_model(std::vector<greedy_match_info_t>& cost_matrix,
        std::vector<box_t>& trace_box_list, std::vector<box_t>& det_box_list);

    static bool cost_compare(const greedy_match_info_t& c1,
        const greedy_match_info_t& c2);

    void track_update_all(void);

    void track_managment(std::vector<box_t>& det_box_list);

    void track_simple_classifity(void);

    void compute_shape4out(void);

    void trace_ComputeAbsVel(simple_tracker& trace, const double yawRate,
        const double speed);

    void trace_UpdateMotionStatus(simple_tracker& trace);

    void trace_UpdateHeading(simple_tracker& sub_trace);
};


