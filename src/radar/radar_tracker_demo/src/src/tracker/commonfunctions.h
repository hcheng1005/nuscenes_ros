/*
 * @Descripttion:
 * @version:
 * @Author: ChengHAO
 * @Date: 2022-08-26 11:53:56
 * @LastEditors: ChengHAO
 * @LastEditTime: 2022-09-15 13:47:09
 */
#ifndef COMMONFUNCTIONS_H
#define COMMONFUNCTIONS_H

#include "inttypes.h"
#include "math.h"
#include <vector>
#include <iostream>
#include <sys/time.h>
#include <deque>

#define SQRT_2PI    (sqrt(2.0*3.1415926))

struct rect_point_struct
{
    double x1;
    double y1;

    double x2;
    double y2;

    double x3;
    double y3;

    double x4;
    double y4;
};

struct MyPoint
{
    double x, y;
};

//#define OPEN_DEBUG_INFO (1)

//用于分析各个模块运行时间
typedef struct
{
    std::string moduleName;
    uint32_t curTime_s;
    uint32_t curTime_us;   //us
    uint32_t passedTime_us; //us
}runTime_t;

typedef struct box_
{
    double x;
    double y;
    double v;
    double wid;
    double len;
}box_t;


bool BetweenOrNot(float a, float b, float c);
float ComputeTwoPointDistance(float x1, float y1, float x2, float y2);
void ComputeLineInfoByTwoPoints(float p1_X, float p1_Y, float p2_X, float p2_Y, float *line_k, float *line_b);
bool ComputeTwoLineIntersectPos(float line1_k, float line1_b, float line2_k, float line2_b, float *xpos, float *ypos);
float ComputeDistance_point2Line(float line_k, float line_b, float Xpos, float Ypos);
float Valuelimit(float minVal, float maxVal, float val_input);
void ComputeMeanAndVari(const std::deque<double> dataArray, double *mean_val, double *vari_val);
float ComputeAngle(float LatPos, float LongPos);
void ComputeCornerPos(float Xcenter, float Ycenter, float headingAng, float half_wid, float half_len, float *cornerPos);
float ComputeGaussianVal(float x, float m, float u);
uint8_t FindCloseCorner(float *corpos, float detX, float detY);
void RuntimeAnalyze(std::string moduleName);
bool point_in_rect_or_not(double *point, double len, double wid, double heading);
float returnBiggerValue(float val1, float val2);
float computePoint2Ellipse(float *elli_center, float l1, float l2, float theta, float *point);
double compute_box_iou(const box_t &box_1, const box_t &box_2, const uint8_t output_idx);
double compute_box_Giou(const box_t &box_1, const box_t &box_2);
double compute_box_distance(const box_t &box_1, const box_t &box_2);
double compute_box_ratio(const box_t &box_1, const box_t &box_2);
box_t box_fusion(const box_t &box_1, const box_t &box_2);

double IOU_With_Rot(const rect_point_struct &r1, const rect_point_struct &r2);
void creat_rect_box_point(double x_pos, double y_pos, double len, double wid, double ori,
                          rect_point_struct &r1);
double intersection_area(const rect_point_struct &r1, const rect_point_struct &r2);
MyPoint intersection(MyPoint a,MyPoint b,MyPoint c,MyPoint d);
double calcularea(const rect_point_struct & r);
double PolygonArea(MyPoint p[], int n);
double cross(MyPoint a,MyPoint b,MyPoint c);
int dcmp(double x);
double SPIA(MyPoint a[], MyPoint b[], int na, int nb);///SimplePolygonIntersectArea 调用此函数
double CPIA(MyPoint a[], MyPoint b[], int na, int nb);//ConvexPolygonIntersectArea
bool cmpr(const rect_point_struct &a,const rect_point_struct &b);

#endif
