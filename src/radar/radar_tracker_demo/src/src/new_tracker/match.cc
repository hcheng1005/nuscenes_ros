#include "match.h"
#include <algorithm>

void matchAlgGreedy(std::vector<std::vector<float>> &costMatrix,
                    std::vector<std::vector<int>> &traceMatchedMeas,
                    std::vector<int> &measMatchedResult)
{
    traceMatchedMeas.resize(costMatrix.at(0).size());
    measMatchedResult.resize(costMatrix.size());

    std::vector<greedyData_t> newCostMatrix;
    int traceIdx = 0, measIdx = 0;
    for (const auto &subCost : costMatrix)
    {
        measMatchedResult.at(measIdx) = 0;

        traceIdx = 0;
        for (const auto &subSubCost : subCost)
        {
            greedyData_t subGreedyData{traceIdx, measIdx, subSubCost};
            newCostMatrix.push_back(subGreedyData);

            traceIdx++;
        }
        measIdx++;
    }

    // 从大到小排序
    std::sort(newCostMatrix.begin(), newCostMatrix.end(), greedAlgCompareFunction);

    // 执行greedy分配
    for (const auto &subCost : newCostMatrix)
    {
        if (subCost.costVal > 0.01) // 最低分配门限
        {
            if (measMatchedResult.at(subCost.measIdx) == 0)
            {
                traceMatchedMeas.at(subCost.traceIdx).push_back(subCost.measIdx);
                measMatchedResult.at(subCost.measIdx) = 1;
            }
        }
        else
        {
            break; // 分配结束
        }
    }
}

bool greedAlgCompareFunction(const greedyData_t &A, const greedyData_t &B)
{
    return (A.costVal > B.costVal);
}