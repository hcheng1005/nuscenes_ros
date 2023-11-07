
#ifndef IOU_H
#define IOU_H

#include <vector>
#include <stack>

struct rect_basic_struct
{
    double center_pos[3]; // x,y,z (横/纵/高)
    double box_len;
    double box_wid;
    double box_height;
    double heading;
    double score;
};

struct rect_box_struct
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

struct myPoint
{
    double x, y;
};

struct rect_corners_struct
{
    myPoint corners[4];
};


void creat_rect_box_point(const rect_basic_struct & rect_1, rect_corners_struct &box_corners);
double intersection_area(const rect_corners_struct & box_1, const rect_corners_struct & box_2);
myPoint intersection(myPoint a,myPoint b,myPoint c,myPoint d);
double calcul_rect_area(const rect_basic_struct &r);
double PolygonArea(myPoint p[], int n);
double cross(myPoint a,myPoint b,myPoint c);
int dcmp(double x);
double SPIA(myPoint a[], myPoint b[], int na, int nb);  
double CPIA(myPoint a[], myPoint b[], int na, int nb);  

void find_p0(myPoint &p0, std::vector<myPoint> &points);
bool cmp_(myPoint &p1, myPoint &p2);
void find_convex_hull(std::vector<myPoint> points, myPoint p0);
double compute_convexHullArea(const rect_corners_struct &r1, const rect_corners_struct &r2);

// 计算各类IOU
double IOU_2D(const rect_basic_struct &rect_1, const rect_basic_struct &rect_2);
double IOU_3D(const rect_basic_struct &rect_1, const rect_basic_struct &rect_2);
double GIOU_2D(const rect_basic_struct &rect_1, const rect_basic_struct &rect_2);
double GIOU_3D(const rect_basic_struct &rect_1, const rect_basic_struct &rect_2);

#endif
