#include "commonfunctions.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <string.h>
#include <algorithm>
#include <fstream> 

#include "../../include/common/iou.h"
/**
 * @name:
 * @description:
 * @param {float} a
 * @param {float} b
 * @param {float} c
 * @return {*}
 */
bool BetweenOrNot(float a, float b, float c) {
  if ((c >= a) && (c <= b)) {
    return true;
  } else {
    return false;
  }
}

/**
 * @name:
 * @description:
 * @param {float} x1
 * @param {float} y1
 * @param {float} x2
 * @param {float} y2
 * @return {*}
 */
float ComputeTwoPointDistance(float x1, float y1, float x2, float y2) {
  return (sqrt(pow((x1 - x2), 2.0) + pow((y1 - y2), 2.0)));
}

/**
 * @name:
 * @description:
 * @param {float} p1_X
 * @param {float} p1_Y
 * @param {float} p2_X
 * @param {float} p2_Y
 * @param {float} *line_k
 * @param {float} *line_b
 * @return {*}
 */
void ComputeLineInfoByTwoPoints(float p1_X, float p1_Y, float p2_X, float p2_Y,
                                float *line_k, float *line_b) {
  float Diff_X, Diff_Y;
  float temp_k, temp_b;

  Diff_X = p2_X - p1_X + 0.000001F;
  Diff_Y = p2_Y - p1_Y;

  temp_k = (Diff_Y / Diff_X);
  temp_b = p1_Y - temp_k * p1_X;

  *line_k = temp_k;
  *line_b = temp_b;
}

/**
 * @name:
 * @descripttion: return xy info if two line intersects
 * @param {float} line1_k
 * @param {float} line1_b
 * @param {float} line2_k
 * @param {float} line2_b
 * @param {float} *xpos
 * @param {float} *ypos
 * @return {*}
 */
bool ComputeTwoLineIntersectPos(float line1_k, float line1_b, float line2_k,
                                float line2_b, float *xpos, float *ypos) {
  float tempX, tempY;

  if (line1_k == line2_k) {
    return false;  // no intersect
  } else {
    tempX = (line1_b - line2_b) / (line2_k - line1_k);
    tempY = line1_k * tempX + line1_b;

    *xpos = tempX;
    *ypos = tempY;

    return true;
  }
}

/**
 * @name:
 * @descripttion:
 * @param {float} line_k
 * @param {float} line_b
 * @param {float} Xpos
 * @param {float} Ypos
 * @return {*}
 */
float ComputeDistance_point2Line(float line_k, float line_b, float Xpos,
                                 float Ypos) {
  return (fabs(line_k * Xpos - Ypos + line_b) / (sqrt(line_k * line_k + 1.0)));
}

/**
 * @name:
 * @descripttion:
 * @param {float} minVal
 * @param {float} maxVal
 * @param {float} val_input
 * @return {*}
 */
float Valuelimit(float minVal, float maxVal, float val_input) {
  if (val_input > maxVal) {
    val_input = maxVal;
  }

  if (val_input < minVal) {
    val_input = minVal;
  }

  return val_input;
}

/**
 * @name:
 * @descripttion:
 * @param {vector<float>} &dataArray
 * @param {float} *meanVal
 * @param {float} *variance
 * @return {*}
 */
void ComputeMeanAndVari(const std::deque<double> dataArray, double *mean_val, double *vari_val) {
  double tempMean = 0.0F;
  double tempVari = 0.0F;

  for (const auto sub_data:dataArray) {
    tempMean += sub_data;
  }

  tempMean = tempMean / dataArray.size();

  for (const auto sub_data:dataArray) {
    tempVari += (pow((sub_data - tempMean), 2.0) / dataArray.size());
  }

  *mean_val = tempMean;
  *vari_val = tempVari;
}

/**
 * @name:
 * @description:
 * @param {float} LatPos
 * @param {float} LongPos
 * @return {*}
 */
float ComputeAngle(float LatPos, float LongPos) {
  return atan2(LatPos, LongPos);
}

/**
 * @name:
 * @descripttion:
 * @param {float} Xcenter
 * @param {float} Ycenter
 * @param {float} headingAng
 * @param {float} half_wid
 * @param {float} half_len
 * @param {float} *cornerPos
 * @return {*}
 */
void ComputeCornerPos(float Xcenter, float Ycenter, float headingAng,
                      float half_wid, float half_len, float *cornerPos) {
  /*
0*****1*****2
*     *     *
*     *     *7
7*****X*****3
*     *     *
*     *     *
6*****5*****4
*/

  float plusOrminus[8][2] = {
      {1.0, 1.0},

      {0.0, 1.0},

      {-1.0, 1.0},

      {-1.0, 0.0},

      {-1.0, -1.0},

      {0.0, -1.0},

      {+1.0, -1.0},

      {+1.0, 0.0},
  };

  float new_heading = headingAng;

  if(fabs(new_heading) > M_PI_2)
  {
    new_heading = (new_heading > 0.0) ? (new_heading - M_PI):(new_heading + M_PI);

    if(new_heading < 0.0)
    {
        double tempdata = half_len;
        half_len = half_wid;
        half_wid = tempdata;
        new_heading = new_heading + M_PI_2;
    }
  }

  float RotMat[2][2] = {{cos(new_heading), sin(new_heading)},
                        {-1.0*sin(new_heading), cos(new_heading)}};
  float tempX, tempY;

  for (uint8_t idx = 0; idx < 8; idx++) {
    tempX = Xcenter + (plusOrminus[idx][0] * half_wid) * RotMat[0][0] +
            (plusOrminus[idx][1] * half_len) * RotMat[0][1];
    tempY = Ycenter + (plusOrminus[idx][0] * half_wid) * RotMat[1][0] +
            (plusOrminus[idx][1] * half_len) * RotMat[1][1];

    cornerPos[2 * idx + 0] = tempX;
    cornerPos[2 * idx + 1] = tempY;
  }

  cornerPos[2 * 8 + 0] = Xcenter;
  cornerPos[2 * 8 + 1] = Ycenter;
}

/**
 * @name:
 * @description:
 * @param {float} x
 * @param {float} m
 * @param {float} u
 * @return {*}
 */
float ComputeGaussianVal(float x, float m, float u) {
  float val = 0.0F;
  float maxProb = 1.0 / (SQRT_2PI * u) * expf(-0.5 / (pow(u, 2.0)));
  val = 1.0 / (SQRT_2PI * u) * expf(-0.5 * pow((x - m), 2.0) / (pow(u, 2.0))) / maxProb;

  return val;
}

/**
 * @name:
 * @description:
 * @param {float} *corpos
 * @param {float} detX
 * @param {float} detY
 * @return {*}
 */
uint8_t FindCloseCorner(float *corpos, float detX, float detY) {
  uint8_t idx = 0;
  float dis = 10e4;
  float tempDis;

  for (uint8_t i = 0; i < 4; i++) {
    tempDis =
        ComputeTwoPointDistance(detX, detY, corpos[i * 4], corpos[i * 4 + 1]);

    if (tempDis < dis) {
      dis = tempDis;
      idx = i;
    }
  }

  return idx;
}

std::vector<runTime_t> RuntimeTable;
/**
 * @name:
 * @description:
 * @param {string} moduleName
 * @return {*}
 */
void RuntimeAnalyze(std::string moduleName) {

  return ;

  struct timeval timestamp;
  runTime_t tempTime;
  std::vector<runTime_t>::iterator lastTime;

  //获取时间戳
  gettimeofday(&timestamp, NULL);

  tempTime.moduleName = moduleName;
  tempTime.curTime_s = timestamp.tv_sec;
  tempTime.curTime_us = timestamp.tv_usec;

  if (moduleName == "Init") {
    RuntimeTable.clear();
    tempTime.passedTime_us = 0;
  } else {
    lastTime = RuntimeTable.end() - 1;

    tempTime.passedTime_us =
        (tempTime.curTime_s - lastTime[0].curTime_s) * 1e6 +
        (tempTime.curTime_us - lastTime[0].curTime_us);
  }

  RuntimeTable.push_back(tempTime);

  static bool init_ = false;
  static std::string log_file_path = "/apollo/data/log/radar_tracker_loginfo_";

  if (!init_) {
    log_file_path.append(
        std::to_string((uint32_t)(tempTime.curTime_s  * 1e6 + tempTime.curTime_us)).append(".csv"));  // log文件名
    init_ = true;
  }

  std::ofstream local_File(log_file_path.c_str(),
                           std::ios::out | std::ios::app);

  if(moduleName == "Init")
  {
    local_File << std::to_string(tempTime.curTime_s + tempTime.curTime_us * 1e-6) << " , ";
  }

  local_File << moduleName << " , " << std::to_string(tempTime.passedTime_us) << " , ";

  if(moduleName == "END")
  {
    local_File << " \n";
  }

  local_File.close();
}

/**
 * @name:
 * @description:
 * @param {float} val1
 * @param {float} val2
 * @return {*}
 */
float returnBiggerValue(float val1, float val2) {
  return ((val1 >= val2) ? val1 : val2);
}

/**
 * @name:
 * @description:
 * @param {float} *elli_center
 * @param {float} l1
 * @param {float} l2
 * @param {float} theta
 * @param {float} *point
 * @return {*}
 */
float computePoint2Ellipse(float *elli_center, float l1, float l2, float theta,
                           float *point) {
  float scale_;
  float transedP[2], transedP2[2];

  transedP[0] = point[0] - elli_center[0];
  transedP[1] = point[1] - elli_center[1];

  transedP2[0] = transedP[0] * cos(theta) + transedP[1] * sin(theta);
  transedP2[1] = transedP[0] * -1.0F * sin(theta) + transedP[1] * cos(theta);

  scale_ = powf((transedP2[0] / l1), 2.0F) + powf((transedP2[1] / l2), 2.0F);

  return scale_;
}


/**
 * @name: point_in_rect_or_not
 * @description: 判定任意点是否落入某矩阵
 * @param {double} *point
 * @param {double} len
 * @param {double} wid
 * @param {double} heading
 * @return {*}
 */
bool point_in_rect_or_not(double *point, double len, double wid, double heading)
{
    bool in_rect = false;
    double rota_ang;
    double latpos, longpos;

    rota_ang = heading;
    if(heading > (0.5 * M_PI))
    {
        rota_ang = -1.0*M_PI + heading;
    }

    if(heading < (-0.5 * M_PI))
    {
      rota_ang = M_PI + heading;
    }

    rota_ang = rota_ang * -1.0;

    latpos = point[0] * cos(rota_ang) - point[1] * sin(rota_ang);
    longpos = point[0] * sin(rota_ang) + point[1] * cos(rota_ang);

    if((fabs(latpos) < (0.5*wid)) && (fabs(longpos) < (0.5*len)))
    {
        in_rect = true;
    }

    return in_rect;
}


/**
 * @name: compute_box_iou
 * @description: 计算各类IOU
 * @param {box_t} &box_1
 * @param {box_t} &box_2
 * @param {uint8_t} output_idx
 * @return {*}
 */
double compute_box_iou(const box_t &box_1, const box_t &box_2, const uint8_t output_idx)
{
    double bbox1_x_mim = box_1.x - 0.5 * box_1.wid;
    double bbox1_x_max = box_1.x + 0.5 * box_1.wid;
    double bbox1_y_mim = box_1.y - 0.5 * box_1.len;
    double bbox1_y_max = box_1.y + 0.5 * box_1.len;

    double bbox2_x_mim = box_2.x - 0.5 * box_2.wid;
    double bbox2_x_max = box_2.x + 0.5 * box_2.wid;
    double bbox2_y_mim = box_2.y - 0.5 * box_2.len;
    double bbox2_y_max = box_2.y + 0.5 * box_2.len;

    double bbox1_size = box_1.wid * box_1.len;
    double bbox2_size = box_2.wid * box_2.len;

    double w = std::max(std::min(bbox1_x_max, bbox2_x_max) - std::max(bbox1_x_mim, bbox2_x_mim), 0.0);
    double h = std::max(std::min(bbox1_y_max, bbox2_y_max) - std::max(bbox1_y_mim, bbox2_y_mim), 0.0);

    double return_val = 0.0;
    double area_i  = w*h;

    switch(output_idx)
    {
        case 0:
                return_val = (area_i/(bbox1_size + bbox2_size - area_i)); //IOU
                break;
        case 1:
//                return_val = ((area_i/(bbox1_size)+area_i/(bbox2_size))) * 0.5;
                return_val = (bbox1_size < bbox2_size) ? (area_i / bbox1_size) : (area_i / bbox2_size);

                break;
        default:
                return_val = (area_i/(bbox1_size + bbox2_size - area_i)); //IOU
                break;
    }

    return return_val;
}


/**
 * @name: compute_box_ratio
 * @description: 计算两个box的长宽比
 * @param {box_t} &box_1
 * @param {box_t} &box_2
 * @return {*}
 */
double compute_box_ratio(const box_t &box_1, const box_t &box_2)
{
    double wid_r = (box_1.wid < box_2.wid) ? (box_1.wid / box_2.wid) : (box_2.wid / box_1.wid);
    double len_r = (box_1.len < box_2.len) ? (box_1.len / box_2.len) : (box_2.len / box_1.len);

    return (wid_r * len_r);
}

/**
 * @name: compute_box_distance
 * @description: 计算box中心点的欧氏距离
 * @param {box_t} &box_1
 * @param {box_t} &box_2
 * @return {*}
 */
double compute_box_distance(const box_t &box_1, const box_t &box_2)
{
    double lat_dis = (box_1.x - box_2.x);
    double long_dis = (box_1.y - box_2.y);
    return (exp(-0.5 * sqrt(pow(lat_dis, 2.0) + pow(long_dis, 2.0))));
}

/**
 * @name: compute_box_Giou
 * @description: 计算GIOU
 * @param {box_t} &box_1
 * @param {box_t} &box_2
 * @return {*}
 */
double compute_box_Giou(const box_t &box_1, const box_t &box_2)
{
    double bbox1_x_mim = box_1.x - 0.5 * box_1.wid;
    double bbox1_x_max = box_1.x + 0.5 * box_1.wid;
    double bbox1_y_mim = box_1.y - 0.5 * box_1.len;
    double bbox1_y_max = box_1.y + 0.5 * box_1.len;

    double bbox2_x_mim = box_2.x - 0.5 * box_2.wid;
    double bbox2_x_max = box_2.x + 0.5 * box_2.wid;
    double bbox2_y_mim = box_2.y - 0.5 * box_2.len;
    double bbox2_y_max = box_2.y + 0.5 * box_2.len;

    double bbox1_size = box_1.wid * box_1.len;
    double bbox2_size = box_2.wid * box_2.len;

    double w = std::max(std::min(bbox1_x_max, bbox2_x_max) - std::max(bbox1_x_mim, bbox2_x_mim), 0.0);
    double h = std::max(std::min(bbox1_y_max, bbox2_y_max) - std::max(bbox1_y_mim, bbox2_y_mim), 0.0);

    double unio_size = bbox1_size + bbox2_size - (w*h);
    double iou_ = (w*h)/unio_size;

    double w2 = std::max(bbox1_x_max, bbox2_x_max) - std::min(bbox1_x_mim, bbox2_x_mim);
    double h2 = std::max(bbox1_y_max, bbox2_y_max) - std::min(bbox1_y_mim, bbox2_y_mim);
    double convex_size = w2*h2;

    return (iou_ - (convex_size - unio_size) / convex_size);
}

/**
 * @name: box_fusion
 * @description: 两个box融合
 * @param {box_t} &box_1
 * @param {box_t} &box_2
 * @return {*}
 */
box_t box_fusion(const box_t &box_1, const box_t &box_2)
{
    box_t new_box;

    double bbox1_x_mim = box_1.x - 0.5 * box_1.wid;
    double bbox1_x_max = box_1.x + 0.5 * box_1.wid;
    double bbox1_y_mim = box_1.y - 0.5 * box_1.len;
    double bbox1_y_max = box_1.y + 0.5 * box_1.len;

    double bbox2_x_mim = box_2.x - 0.5 * box_2.wid;
    double bbox2_x_max = box_2.x + 0.5 * box_2.wid;
    double bbox2_y_mim = box_2.y - 0.5 * box_2.len;
    double bbox2_y_max = box_2.y + 0.5 * box_2.len;

    double new_x_min = std::min(bbox1_x_mim, bbox2_x_mim);
    double new_y_min = std::min(bbox1_y_mim, bbox2_y_mim);
    double new_x_max = std::max(bbox1_x_max, bbox2_x_max);
    double new_y_max = std::max(bbox1_y_max, bbox2_y_max);

    new_box.x = (new_x_min + new_x_max) * 0.5;
    new_box.y = (new_y_min + new_y_max) * 0.5;
    new_box.len = new_y_max - new_y_min;
    new_box.wid = new_x_max - new_x_min;

    new_box.v = (box_1.v + box_2.v) * 0.5;

    return new_box;
}


//support roat IOU
const double eps = 1e-8;
/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void creat_rect_box_point(double x_pos, double y_pos, double len, double wid, double ori,
                          rect_point_struct &r1)
{
    if(fabs(ori) > (0.5*3.1415926) )
    {
        ori = (ori > 0.0)?(ori - 3.1415926):(ori+3.1415926);
    }

    double rot[4] = {cos(ori), -1.0*sin(ori), sin(ori), cos(ori)};

    double p1[2] = {x_pos, y_pos};
    double temp_x, temp_y;
    double half_wid = 0.5 * wid;
    double half_len = 0.5 * len;

    temp_x = -1.0 * half_wid;
    temp_y =  half_len;
    r1.x1 = rot[0] * temp_x + rot[1] * temp_y + p1[0];
    r1.y1 = rot[2] * temp_x + rot[3] * temp_y + p1[1];

    temp_x = half_wid;
    temp_y = half_len;
    r1.x2 = rot[0] * temp_x + rot[1] * temp_y + p1[0];
    r1.y2 = rot[2] * temp_x + rot[3] * temp_y + p1[1];

    temp_x = half_wid;
    temp_y = -1.0 * half_len;
    r1.x3 = rot[0] * temp_x + rot[1] * temp_y + p1[0];
    r1.y3 = rot[2] * temp_x + rot[3] * temp_y + p1[1];

    temp_x = -1.0 * half_wid;
    temp_y = -1.0 * half_len;
    r1.x4 = rot[0] * temp_x + rot[1] * temp_y + p1[0];
    r1.y4 = rot[2] * temp_x + rot[3] * temp_y + p1[1];
}


/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {rect_point_&} r1
 * @param {rect_point_&} r2
 * @return {*}
 */
double IOU_With_Rot(const rect_point_struct & r1, const rect_point_struct & r2)
{
    double inter = intersection_area(r1,r2);

    double o = inter / (calcularea(r1) + calcularea(r2) - inter);

    return (o >= 0) ? o : 0;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {rect_point_&} r1
 * @param {rect_point_&} r2
 * @return {*}
 */
double intersection_area(const rect_point_struct & r1, const rect_point_struct & r2){
    myPoint p1[10],p2[10];

    p1[0].x=r1.x2;
    p1[0].y=r1.y2;
    p1[1].x=r1.x3;
    p1[1].y=r1.y3;
    p1[2].x=r1.x4;
    p1[2].y=r1.y4;
    p1[3].x=r1.x1;
    p1[3].y=r1.y1;

    p2[0].x=r2.x2;
    p2[0].y=r2.y2;
    p2[1].x=r2.x3;
    p2[1].y=r2.y3;
    p2[2].x=r2.x4;
    p2[2].y=r2.y4;
    p2[3].x=r2.x1;
    p2[3].y=r2.y1;
    double area = SPIA(p1, p2, 4, 4);
    return area;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {rect_point_&} r
 * @return {*}
 */
double calcularea(const rect_point_struct & r){
    float d12=sqrt(pow(r.x2-r.x1,2)+pow(r.y2-r.y1,2));
    float d14=sqrt(pow(r.x4-r.x1,2)+pow(r.y4-r.y1,2));
    float d24=sqrt(pow(r.x2-r.x4,2)+pow(r.y2-r.y4,2));
    float d32=sqrt(pow(r.x2-r.x3,2)+pow(r.y2-r.y3,2));
    float d34=sqrt(pow(r.x3-r.x4,2)+pow(r.y3-r.y4,2));
    float p1=(d12+d14+d24)/2;
    float p2=(d24+d32+d34)/2;
    float s1=sqrt(p1*(p1-d12)*(p1-d14)*(p1-d24));
    float s2=sqrt(p2*(p2-d32)*(p2-d34)*(p2-d24));
    return s1+s2;
}