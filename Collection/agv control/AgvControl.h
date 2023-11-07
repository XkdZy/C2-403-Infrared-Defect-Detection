#pragma once

#include <iostream>
using namespace std;

#include "Eigen/Dense"

#pragma region Ours Function

bool AdjustAGV2Goal(const Eigen::Vector3d& goalPos);
bool MoveAGV2OneGoal(const Eigen::Vector3d& goal, bool towardGoal = false, const Eigen::Vector3d towardPos = Eigen::Vector3d(0,0,0));

#pragma endregion

