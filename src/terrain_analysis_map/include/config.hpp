#pragma once

#include <string>
struct TerrainAnalysisConfig {
    double scanVoxelSize = 0.05;
    double decayTime = 2.0;
    double noDecayDis = 4.0;
    double clearingDis = 8.0;
    bool clearingCloud = false;
    bool useSorting = true;
    double quantileZ = 0.25;
    bool considerDrop = false;
    bool limitGroundLift = false;
    double maxGroundLift = 0.15;
    bool clearDyObs = false;
    double minDyObsDis = 0.3;
    double minDyObsAngle = 0;
    double minDyObsRelZ = -0.5;
    double absDyObsRelZThre = 0.2;
    double minDyObsVFOV = -16.0;
    double maxDyObsVFOV = 16.0;
    int minDyObsPointNum = 1;
    bool noDataObstacle = false;
    int noDataBlockSkipNum = 0;
    int minBlockPointNum = 10;
    double vehicleHeight = 1.5;
    int voxelPointUpdateThre = 100;
    double voxelTimeUpdateThre = 2.0;
    double minRelZ = -1.5;
    double maxRelZ = 0.2;
    double disRatioZ = 0.2;
    std::string cloud_name;
    std::string odom_name;
    double batchMaxPoints=20000;
    double batchMaxInterval=0.5;
};