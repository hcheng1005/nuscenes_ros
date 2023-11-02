
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

struct MyPoint
{
    double x, y;
};

struct rect_corners_struct
{
    MyPoint corners[4];
};


void creat_rect_box_point(const rect_basic_struct & rect_1, rect_corners_struct &box_corners);
double intersection_area(const rect_corners_struct & box_1, const rect_corners_struct & box_2);
MyPoint intersection(MyPoint a,MyPoint b,MyPoint c,MyPoint d);
double calcul_rect_area(const rect_basic_struct &r);
double PolygonArea(MyPoint p[], int n);
double cross(MyPoint a,MyPoint b,MyPoint c);
int dcmp(double x);
double SPIA(MyPoint a[], MyPoint b[], int na, int nb);///SimplePolygonIntersectArea 调用此函数
double CPIA(MyPoint a[], MyPoint b[], int na, int nb);//ConvexPolygonIntersectArea

void find_p0(MyPoint &p0, std::vector<MyPoint> &points);
bool cmp_(MyPoint &p1, MyPoint &p2);
void find_convex_hull(std::vector<MyPoint> points, MyPoint p0);
double compute_convexHullArea(const rect_corners_struct &r1, const rect_corners_struct &r2);

double IOU_2D(const rect_basic_struct &rect_1, const rect_basic_struct &rect_2);


#endif
