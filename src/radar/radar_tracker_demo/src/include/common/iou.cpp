#include "iou.h"

#include <math.h>
#include <string.h>

#include <algorithm>
#include <iostream>
#include <stack>
#include <vector>

const double eps = 1e-8;
const double MAXNUM = 1e10;

// 通过长宽信息计算2D矩形角点
void creat_rect_box_point(const rect_basic_struct &rect_1,
                          rect_corners_struct &box_corners)
{
    double rot[4] = {cos(rect_1.heading), -1.0 * sin(rect_1.heading),
                     sin(rect_1.heading), cos(rect_1.heading)};
    double p1[2] = {rect_1.center_pos[1], rect_1.center_pos[0]};
    double half_wid = 0.5 * rect_1.box_wid;
    double half_len = 0.5 * rect_1.box_len;

    double pos_symbol[4][2] = {1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0, -1.0};

    for (uint8_t idx = 0; idx < 4; idx++)
    {
        // 纵向距离
        box_corners.corners[idx].x = rot[0] * pos_symbol[idx][0] * half_len +
                                     rot[1] * pos_symbol[idx][1] * half_wid + p1[1];

        // 横向距离
        box_corners.corners[idx].y = rot[2] * pos_symbol[idx][0] * half_len +
                                     rot[3] * pos_symbol[idx][1] * half_wid + p1[0];
    }
}

double intersection_area(const rect_corners_struct &box_1,
                         const rect_corners_struct &box_2)
{
    myPoint p1[5], p2[5];
    for (uint8_t idx = 0; idx < 4; idx++)
    {
        p1[idx] = box_1.corners[idx];
        p2[idx] = box_2.corners[idx];
    }

    double area = SPIA(p1, p2, 4, 4);
    return area;
}

double SPIA(myPoint a[], myPoint b[], int na, int nb)
{
    int i, j;
    myPoint t1[4], t2[4];
    double res = 0, num1, num2;
    a[na] = t1[0] = a[0], b[nb] = t2[0] = b[0];

    for (i = 2; i < na; i++)
    {
        t1[1] = a[i - 1], t1[2] = a[i];
        num1 = dcmp(cross(t1[1], t1[2], t1[0]));
        if (num1 < 0)
        {
            std::swap(t1[1], t1[2]);
        }

        for (j = 2; j < nb; j++)
        {
            t2[1] = b[j - 1], t2[2] = b[j];
            num2 = dcmp(cross(t2[1], t2[2], t2[0]));
            if (num2 < 0)
            {
                std::swap(t2[1], t2[2]);
            }
            res += CPIA(t1, t2, 3, 3) * num1 * num2;
        }
    }
    return res;
}

// 极角排序
int dcmp(double x)
{
    if (x > eps)
        return 1;
    return x < -eps ? -1 : 0;
}

double cross(myPoint a, myPoint b, myPoint c) /// 叉积
{
    return (a.x - c.x) * (b.y - c.y) - (b.x - c.x) * (a.y - c.y);
}

double CPIA(myPoint a[], myPoint b[], int na, int nb)
{
    myPoint p[20], tmp[20];
    int tn, sflag, eflag;
    a[na] = a[0], b[nb] = b[0];
    memcpy(p, b, sizeof(myPoint) * (nb + 1));
    for (int i = 0; i < na && nb > 2; i++)
    {
        sflag = dcmp(cross(a[i + 1], p[0], a[i]));
        for (int j = tn = 0; j < nb; j++, sflag = eflag)
        {
            if (sflag >= 0)
                tmp[tn++] = p[j];
            eflag = dcmp(cross(a[i + 1], p[j + 1], a[i]));
            if ((sflag ^ eflag) == -2)
                tmp[tn++] = intersection(a[i], a[i + 1], p[j], p[j + 1]); /// 求交点
        }
        memcpy(p, tmp, sizeof(myPoint) * tn);
        nb = tn, p[nb] = p[0];
    }
    if (nb < 3)
        return 0.0;
    return PolygonArea(p, nb);
}

double calcul_rect_area(const rect_basic_struct &r)
{
#if 0
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
#endif

    return (r.box_len * r.box_wid);
}

myPoint intersection(myPoint a, myPoint b, myPoint c, myPoint d)
{
    myPoint p = a;
    double t = ((a.x - c.x) * (c.y - d.y) - (a.y - c.y) * (c.x - d.x)) /
               ((a.x - b.x) * (c.y - d.y) - (a.y - b.y) * (c.x - d.x));
    p.x += (b.x - a.x) * t;
    p.y += (b.y - a.y) * t;
    return p;
}

// 计算多边形面积
double PolygonArea(myPoint p[], int n)
{
    if (n < 3)
    {
        return 0.0;
    }
    double s = p[0].y * (p[n - 1].x - p[1].x);
    p[n] = p[0];
    for (int i = 1; i < n; ++i)
    {
        s += p[i].y * (p[i - 1].x - p[i + 1].x);
    }

    return fabs(s * 0.5);
}

// variables for calcul convexHull
myPoint p0;
std::stack<myPoint> convex_hull;

void find_convex_hull(std::vector<myPoint> points, myPoint p0)
{
    // p0和p1是凸包中的点
    convex_hull.push(points[0]);
    convex_hull.push(points[1]);

    uint i = 2;
    // p1,p2为栈顶两个节点
    myPoint p1 = points.at(0);
    myPoint p2 = points.at(1);
    while (i < points.size())
    {
        // 如果points[i]和points[i-1]在同一个角度，则不再对points[i]进行计算
        if ((points.at(i - 1).y - p0.y) * (points.at(i).x - p0.x) ==
            (points.at(i - 1).x - p0.x) * (points.at(i).y - p0.y))
        {
            i++;
            continue;
        }

        // 如果叉积大于0，将当前点压入栈
        if (cross(p1, p2, points.at(i)) >= 0)
        {
            // 假设现在栈中为a,b,c,d,cross(c,d,e)大于等于0
            convex_hull.push(points.at(i)); // a,b,c,d,e,p1=c,p2=d
            p1 = p2;                        // p1=d
            p2 = convex_hull.top();         // p2=e
            i++;
        }

        // 如果叉积小于0，对栈中节点进行处理
        else
        {
            while (1)
            {
                if (convex_hull.size() == 2)
                {
                    std::cout << "-------------------convex_hull.size() == "
                                 "2-------------------";
                    return;
                }
                // 假设现在栈中为a,b,c,d,cross(c,d,e)小于0
                convex_hull.pop();      // a,b,c
                convex_hull.pop();      // a,b
                p2 = p1;                // p2=c;
                p1 = convex_hull.top(); // p1=b
                convex_hull.push(p2);   // a,b,c
                // cross(b,c,e)
                if (cross(p1, p2, points.at(i)) >= 0)
                {
                    convex_hull.push(points.at(i)); // a,b,c,e
                    p1 = p2;                        // p1=c
                    p2 = convex_hull.top();         // p2=e
                    i++;
                    break;
                }
            }
        }
    }
}

// 寻找p0
void find_p0(myPoint &p0, std::vector<myPoint> &points)
{
    p0 = points[0];
    for (uint i = 1; i < points.size(); i++)
    {
        if (points.at(i).y < p0.y)
        {
            p0 = points.at(i);
        }
        else if (points.at(i).y == p0.y)
        {
            if (points.at(i).x < p0.x)
            {
                p0 = points.at(i);
            }
        }
    }
}

// 极角排序
bool cmp_(myPoint &p1, myPoint &p2)
{
    // p0排首位
    if (p1.x == p0.x && p1.y == p0.y)
        return true;
    if (p2.x == p0.x && p2.y == p0.y)
        return false;

    // 计算极角（等于0则赋予一个极大值）
    double angle1 = p1.x == p0.x ? MAXNUM : (p1.y - p0.y) / (p1.x - p0.x);
    double angle2 = p2.x == p0.x ? MAXNUM : (p2.y - p0.y) / (p2.x - p0.x);
    // 小于0则赋予一个更大的值
    if (angle1 < 0)
        angle1 += 2 * MAXNUM;
    if (angle2 < 0)
        angle2 += 2 * MAXNUM;

    // 极角排序
    if (angle1 < angle2)
        return true;
    else if (angle1 == angle2)
    {
        if (p1.y > p2.y)
            return true;
        else
            return false;
    }
    else
        return false;
}

double compute_convexHullArea(const rect_corners_struct &r1,
                              const rect_corners_struct &r2)
{
    std::vector<myPoint> point_list;

    for (uint8_t idx = 0; idx < 4; idx++)
    {
        point_list.push_back(r1.corners[idx]);
    }

    for (uint8_t idx = 0; idx < 4; idx++)
    {
        point_list.push_back(r2.corners[idx]);
    }

    while (!convex_hull.empty())
    {
        convex_hull.pop();
    }
    find_p0(p0, point_list);
    std::sort(point_list.begin(), point_list.end(), cmp_); // 按极角排序

    // 搜索并构造凸包点
    find_convex_hull(point_list, p0); // 搜索凸包

    // 计算凸包面积
    myPoint ppp[10];
    uint8_t num_ = 0;
    while (!convex_hull.empty())
    {
        ppp[num_].x = convex_hull.top().x;
        ppp[num_].y = convex_hull.top().y;
        convex_hull.pop();
        num_++;
    }
    double ares_ = PolygonArea(ppp, num_);

    return ares_;
}

int orientation(myPoint p, myPoint q, myPoint r)
{
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0)
        return 0;             // Collinear
    return (val > 0) ? 1 : 2; // Clockwise or Counterclockwise
}

bool comparePoints(myPoint p1, myPoint p2)
{
    return (p1.x < p2.x) || (p1.x == p2.x && p1.y < p2.y);
}

std::vector<myPoint> convexHull(std::vector<myPoint> points)
{
    int n = points.size();
    if (n < 3)
        return points;

    std::vector<myPoint> hull;

    int l = 0;
    for (int i = 1; i < n; ++i)
    {
        if (points[i].x < points[l].x)
        {
            l = i;
        }
    }

    int p = l, q;
    do
    {
        hull.push_back(points[p]);
        q = (p + 1) % n;
        for (int i = 0; i < n; ++i)
        {
            if (orientation(points[p], points[i], points[q]) == 2)
            {
                q = i;
            }
        }
        p = q;
    } while (p != l);

    return hull;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
double IOU_2D(const rect_basic_struct &rect_1,
              const rect_basic_struct &rect_2)
{
    rect_corners_struct box_1, box_2;

    creat_rect_box_point(rect_1, box_1);
    creat_rect_box_point(rect_2, box_2);

    // compute_convexHullArea(box_1, box_2);

    // a new way to compute convexHullArea
    // std::vector<myPoint> points = {
    //     {box_1.corners[0].x, box_1.corners[0].y},
    //     {box_1.corners[1].x, box_1.corners[1].y},
    //     {box_1.corners[2].x, box_1.corners[2].y},
    //     {box_1.corners[3].x, box_1.corners[3].y},
    //     {box_2.corners[0].x, box_2.corners[0].y},
    //     {box_2.corners[1].x, box_2.corners[1].y},
    //     {box_2.corners[2].x, box_2.corners[2].y},
    //     {box_2.corners[3].x, box_2.corners[3].y},
    // };

    // std::sort(points.begin(), points.end(), comparePoints);
    // std::vector<myPoint> hull = convexHull(points);

    // // 计算凸包的面积（假设凸包至少有三个点）
    // double area = 0;
    // int n = hull.size();
    // for (int i = 0; i < n - 1; ++i)
    // {
    //     area += (hull[i].x * hull[i + 1].y - hull[i + 1].x * hull[i].y);
    // }
    // area += (hull[n - 1].x * hull[0].y - hull[0].x * hull[n - 1].y);
    // area = std::abs(area) / 2;

    // std::cout << "凸包的面积为：" << area << std::endl;

    double I_ = intersection_area(box_1, box_2);
    double U_ = (calcul_rect_area(rect_1) + calcul_rect_area(rect_2) - I_);
    double iou_value = I_ / U_;
    return (iou_value >= 0) ? iou_value : 0;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
double IOU_3D(const rect_basic_struct &rect_1, const rect_basic_struct &rect_2)
{
    // 首先计算2dIOU
    rect_corners_struct box_1, box_2;

    creat_rect_box_point(rect_1, box_1);
    creat_rect_box_point(rect_2, box_2);

    // compute_convexHullArea(box_1, box_2);

    double I_ = intersection_area(box_1, box_2);

    double z1 = rect_1.center_pos[2];
    double h1 = rect_1.box_height;

    double z2 = rect_2.center_pos[2];
    double h2 = rect_2.box_height;

    double overlap_height = std::max(0.0, std::min((z1 + h1 / 2) - (z2 - h2 / 2), (z2 + h2 / 2) - (z1 - h1 / 2)));
    I_ = I_ * overlap_height;

    double U_ = rect_1.box_len * rect_1.box_wid * rect_1.box_height +
                rect_2.box_len * rect_2.box_wid * rect_2.box_height - I_;
    double iou_value = I_ / U_;
    return (iou_value >= 0) ? iou_value : 0;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
double GIOU_2D(const rect_basic_struct &rect_1, const rect_basic_struct &rect_2)
{
    rect_corners_struct box_1, box_2;

    creat_rect_box_point(rect_1, box_1);
    creat_rect_box_point(rect_2, box_2);

    // compute intersection and union
    double I = intersection_area(box_1, box_2);
    double U = rect_1.box_wid * rect_1.box_len + rect_2.box_wid * rect_1.box_len - I;

    // compute the convex area
    double C = compute_convexHullArea(box_1, box_2);

    // compute giou
    return (I / U - (C - U) / C);
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
double GIOU_3D(const rect_basic_struct &rect_1, const rect_basic_struct &rect_2)
{
    rect_corners_struct box_1, box_2;

    creat_rect_box_point(rect_1, box_1);
    creat_rect_box_point(rect_2, box_2);

    // compute intersection and union
    double I = intersection_area(box_1, box_2);

    double z1 = rect_1.center_pos[2];
    double h1 = rect_1.box_height;

    double z2 = rect_2.center_pos[2];
    double h2 = rect_2.box_height;

    // 计算重叠高度
    double overlap_height = std::max(0.0, std::min((z1 + h1 / 2) - (z2 - h2 / 2), (z2 + h2 / 2) - (z1 - h1 / 2)));

    // 计算重叠体积
    I = I * overlap_height;

    // 计算联合体积
    double U = rect_1.box_len * rect_1.box_wid * rect_1.box_height +
               rect_2.box_len * rect_2.box_wid * rect_2.box_height - I;

    // compute the convex area
    double C = compute_convexHullArea(box_1, box_2);
    double union_height = std::max((z1 + h1 / 2) - (z2 - h2 / 2), (z2 + h2 / 2) - (z1 - h1 / 2));
    C = C * union_height;

    // compute giou
    double giou = I / U - (C - U) / C;
    return giou;
}