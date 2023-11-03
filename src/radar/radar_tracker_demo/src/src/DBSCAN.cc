
#include "DBSCAN.h"

#include <fstream>
#include <iostream>

using namespace DBSCAN;

bool showInfo = false;
std::vector<size_t> pointMap[GriDSize_Lat][GriDSize_Long];

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void DBSCAN::KNN_DBSCAN(std::vector<Point4DBSCAN> &pointSet,
                        std::vector<std::vector<size_t>> &clusterSet)
{
  uint8_t minNum = 0;
  std::vector<size_t> clusterMember;

  clusterSet.clear();

  // 将点云进行栅格索引映射
  GridMappingPoint(pointSet);

  for (size_t n = 0; n < pointSet.size(); n++)
  {
    // 聚类入口
    ScanPoints(n, pointSet, &clusterMember, &minNum);

    if (clusterMember.size() >= minNum)
    {
      clusterSet.push_back(clusterMember);
    }
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<Point4DBSCAN>} &pointSet
 * @return {*}
 */
void DBSCAN::GridMappingPoint(std::vector<Point4DBSCAN> &pointSet)
{
  size_t curLat, curLong;
  size_t tempIDX;

  for (size_t i1 = 0; i1 < GriDSize_Lat; i1++)
  {
    for (size_t i2 = 0; i2 < GriDSize_Long; i2++)
    {
      pointMap[i1][i2].clear();
    }
  }

  for (size_t i = 0; i < pointSet.size(); i++)
  {
    size_t latIdx = (pointSet[i].PointInfo.DistLat + MAX_Lat) / Grid_Reso;
    size_t longIdx = pointSet[i].PointInfo.DistLong / Grid_Reso;

    size_t scanLen = 0.0;

    // 点云横纵向初始扫描位置
    curLat = (pointSet[i].PointInfo.DistLat + MAX_Lat) / Grid_Reso;
    curLong = pointSet[i].PointInfo.DistLong / Grid_Reso;

    scanLen = pointSet[i].DBSCAN_para.Search_R / Grid_Reso + 1U;

    tempIDX = curLat + scanLen;
    if (tempIDX >= GriDSize_Lat)
    {
      tempIDX = (GriDSize_Lat - 1);
    }
    pointSet[i].PointInfo.scanLat[1] = tempIDX;

    tempIDX = curLat - scanLen;
    if (tempIDX <= 0)
    {
      tempIDX = 0;
    }
    pointSet[i].PointInfo.scanLat[0] = tempIDX;

    tempIDX = curLong + scanLen;
    if (tempIDX >= GriDSize_Long)
    {
      tempIDX = (GriDSize_Long - 1);
    }
    pointSet[i].PointInfo.scanLong[1] = tempIDX;

    tempIDX = curLong - scanLen;
    if (tempIDX <= 0)
    {
      tempIDX = 0;
    }
    pointSet[i].PointInfo.scanLong[0] = tempIDX;

    pointMap[latIdx][longIdx].push_back(i);
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void DBSCAN::ScanPoints(size_t start_Idx, std::vector<Point4DBSCAN> &pointSet,
                        std::vector<size_t> *scanResult, uint8_t *minNumer)
{
  std::vector<size_t> memberIdx;

  double diff_range, diff_lat, diff_long;
  double diff_V = 0;
  bool matchFlag = false;

  scanResult->clear();
  scanResult->push_back(start_Idx);

  if (pointSet[start_Idx].PointInfo.valid == false)
  {
    *minNumer = 255;
    return;
  }

  pointSet[start_Idx].PointInfo.valid = false;

  uint8_t tempminNumer = 255;

  bool is_static_ = false;

  for (size_t idx = 0; idx < scanResult->size(); idx++)
  {
    size_t ptsNum = 0;
    Point4DBSCAN *point2 = &pointSet[scanResult->at(idx)];

    if (point2->DBSCAN_para.static_or_dyna == 1)
    {
      is_static_ = true;
    }

    if (point2->DBSCAN_para.minPts < tempminNumer)
    {
      tempminNumer = point2->DBSCAN_para.minPts;
    }

    // 遍历该点云左右和上下扫描范围
    for (size_t n1 = point2->PointInfo.scanLat[0];
         n1 <= point2->PointInfo.scanLat[1]; n1++)
    {
      for (size_t n2 = point2->PointInfo.scanLong[0];
           n2 <= point2->PointInfo.scanLong[1]; n2++)
      {
        // 对应扫描位置是否存在点云
        for (auto &idx2 : pointMap[n1][n2])
        {
          if (idx2 == scanResult->at(idx))
          {
            continue;
          }

          matchFlag = false;

          Point4DBSCAN *point3 = &pointSet[idx2];

          if (point3->PointInfo.valid == true)
          {
            // 计算两点之间的距离
            diff_lat = fabs(point2->PointInfo.DistLat - point3->PointInfo.DistLat);
            diff_long = fabs(point2->PointInfo.DistLong - point3->PointInfo.DistLong);
            diff_range = sqrtf(pow(diff_lat, 2.0) + pow(diff_long, 2.0));

            // 计算多普勒速度差
            diff_V = fabs(point2->PointInfo.V - point3->PointInfo.V);

            double diff_Azi = fabs(point2->PointInfo.Azi - point3->PointInfo.Azi);

            double range_gate = (point2->DBSCAN_para.Search_R >= point3->DBSCAN_para.Search_R) ?\
                                 point2->DBSCAN_para.Search_R : point3->DBSCAN_para.Search_R;

            // 两点距离差
            if (diff_range < range_gate)
            {
              if (!is_static_) // 0: 动态点
              {
                if (fabs(point2->PointInfo.DistLat - point3->PointInfo.DistLat) < 1.5)
                {
                  if (diff_V < 1.0F)
                  {
                    matchFlag = true; // TBD：行人和障碍物容易聚在一起
                  }
                  else
                  {
                    if ((diff_range < (range_gate * 0.5F)) &&
                        (diff_V < 2.0F))
                    {
                      matchFlag = true;
                    }
                  }
                }
              }
              else // 1：静止点
              {
                if (fabs(point2->PointInfo.DistLat - point3->PointInfo.DistLat) < 1.0)
                {
                  matchFlag = true;
                }
              }
            }
            else
            {
              // 动态点云补充聚类逻辑
              if ((!is_static_) && ((diff_range < (range_gate * 1.2F)) &&
                                    (diff_Azi < 1.5F) && (fabs(diff_V) < 0.51F)))
              {
                matchFlag = true;
              }
            }

            if ((matchFlag) && (!is_static_) && (diff_lat > 1.0))
            {
              matchFlag = false;
            }

            if (matchFlag == true)
            {
              ptsNum++;
              point3->PointInfo.valid = false;
              scanResult->push_back(idx2);
            }
          }
        }
      }
    }
  }

  *minNumer = tempminNumer;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<Point4DBSCAN>} &pointSet
 * @return {*}
 */
void DBSCAN::ComputePara(std::vector<Point4DBSCAN> &pointSet)
{
  // 根据不同的条件，决定该点的SCAN参数
  for (auto &point : pointSet)
  {
    point.DBSCAN_para.Search_R = 1.5F;

    if (fabs(point.PointInfo.V) > 5.0F)
    {
      point.DBSCAN_para.Search_R = 2.5F;
    }

    point.DBSCAN_para.minPts = 2;

    point.DBSCAN_para.pointType = 255;
  }
}
