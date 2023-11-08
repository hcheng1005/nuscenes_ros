#pragma once

#include <cmath>
#include <vector>

typedef struct greedyData_t {
  int traceIdx;
  int measIdx;
  float costVal;
};

void matchAlgGreedy(std::vector<std::vector<float>> &costMatrix,
                    std::vector<std::vector<int>> &traceMatchedMeas,
                    std::vector<int> &measMatchedResult);

bool greedAlgCompareFunction(const greedyData_t &A, const greedyData_t &B);
