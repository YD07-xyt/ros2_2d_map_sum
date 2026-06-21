#pragma once

#include "config.hpp"
#include <Eigen/Core>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <queue>
#include <spdlog/spdlog.h>
#include <vector>
namespace TerrainAnalysis {

class TerrainAnalysisMap {
public:
  TerrainAnalysisMap(TerrainAnalysisConfig config) : config_(config) {
    rawBatchCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    //裁剪后的点云
    laserCloudCrop = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    laserCloudDwz = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    terrainCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    terrainCloudElev = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    
    terrainVoxelCloud.resize(kTerrainVoxelNum);
    for (int i = 0; i < kTerrainVoxelNum; ++i) {
        terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }


    //降采样体素地图设置大小
    downSizeFilter.setLeafSize(config_.scanVoxelSize, config_.scanVoxelSize,
                               config_.scanVoxelSize);

    terrainVoxelUpdateNum.assign(kTerrainVoxelNum, 0);
    terrainVoxelUpdateTime.assign(kTerrainVoxelNum, 0.0f);
    planarVoxelElev.assign(kPlanarVoxelNum, 0.0f);
    planarVoxelEdge.assign(kPlanarVoxelNum, 0);
    planarVoxelDyObs.assign(kPlanarVoxelNum, 0);
    planarPointElev.resize(kPlanarVoxelNum);

    planarVoxelConn.assign(kPlanarVoxelNum, 0);
  };
  void updateOdom(std::vector<double> &odom_update) {
    vehicleX = odom_update[0];
    vehicleY = odom_update[1];
    vehicleZ = odom_update[2];
    vehicleRoll = odom_update[3];
    vehiclePitch = odom_update[4];
    vehicleYaw = odom_update[5];

    sinVehicleRoll = sin(vehicleRoll);
    cosVehicleRoll = cos(vehicleRoll);
    sinVehiclePitch = sin(vehiclePitch);
    cosVehiclePitch = cos(vehiclePitch);
    sinVehicleYaw = sin(vehicleYaw);
    cosVehicleYaw = cos(vehicleYaw);

    if (noDataInited == 0) {
      vehicleXRec = vehicleX;
      vehicleYRec = vehicleY;
      noDataInited = 1;
    }
    if (noDataInited == 1) {
      float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                       (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
      if (dis >= config_.noDecayDis)
        noDataInited = 2;
    }
  }
  void updateTime(double CloudTime, double system_Init_Time) {
    laserCloudTime = CloudTime;
    systemInitTime = system_Init_Time;
  };
  void processBatch() {
    if (rawBatchCloud->empty())
      return;

    // 裁剪并填充 laserCloudCrop
    laserCloudCrop->clear();
    for (const auto &pt : rawBatchCloud->points) {
      float dis = sqrt((pt.x - vehicleX) * (pt.x - vehicleX) +
                       (pt.y - vehicleY) * (pt.y - vehicleY));
      if (pt.z - vehicleZ > config_.minRelZ - config_.disRatioZ * dis &&
          pt.z - vehicleZ < config_.maxRelZ + config_.disRatioZ * dis &&
          dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
        laserCloudCrop->push_back(pt);
      }
    }

    rawBatchCloud->clear(); // 清空缓冲区
    updateMap();            // 原有建图流程
  }
  void updatelaserCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr PointCLoud) {
    // 1. 把新点存入缓冲区（不裁剪）
    for (const auto &pt : PointCLoud->points) {
      pcl::PointXYZI point = pt;
      point.intensity = laserCloudTime - systemInitTime;
      rawBatchCloud->push_back(point);
    }

    // 2. 双条件触发处理
    bool timeTrigger =
        (laserCloudTime - lastBatchTime) >= config_.batchMaxInterval;
    bool countTrigger = rawBatchCloud->size() >= config_.batchMaxPoints;

    if (timeTrigger || countTrigger) {
      processBatch();
      lastBatchTime = laserCloudTime; // 更新上次处理时间
    }
    // pcl::PointXYZI point;

    // laserCloudCrop->clear();

    // int laserCloudSize = PointCLoud->points.size();
    // for (int i = 0; i < laserCloudSize; i++) {
    //   point = PointCLoud->points[i];

    //   float pointX = point.x;
    //   float pointY = point.y;
    //   float pointZ = point.z;

    //   float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
    //                    (pointY - vehicleY) * (pointY - vehicleY));
    //   if (pointZ - vehicleZ > config_.minRelZ - config_.disRatioZ * dis &&
    //       pointZ - vehicleZ < config_.maxRelZ + config_.disRatioZ * dis &&
    //       dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
    //     point.x = pointX;
    //     point.y = pointY;
    //     point.z = pointZ;
    //     point.intensity = laserCloudTime - systemInitTime;
    //     laserCloudCrop->push_back(point);
    //   }
    // }
  }
  void updateMap() {

    // terrain voxel roll over
    //滚动更新地图
    float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
    float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
    // odom左移 -->平移体素向左一格
    while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
      for (int indY = 0; indY < terrainVoxelWidth; indY++) {
        // 保存最右侧一列（即将被覆盖的列）的指针
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                              indY];
        // 所有列向右移动一格
        for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
        }
        //将保存的最右侧列指针放到最左侧
        terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
        // 清空最左侧列（原最右侧列）的点云数据
        terrainVoxelCloud[indY]->clear();
      }
      terrainVoxelShiftX--; // 网格中心偏移计数减1
      terrainVoxelCenX =
          terrainVoxelSize * terrainVoxelShiftX; // 更新网格中心X坐标
    }

    while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
      for (int indY = 0; indY < terrainVoxelWidth; indY++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[indY];
        for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
        }
        terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] =
            terrainVoxelCloudPtr;
        terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
            ->clear();
      }
      terrainVoxelShiftX++;
      terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
    }

    while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
      for (int indX = 0; indX < terrainVoxelWidth; indX++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[terrainVoxelWidth * indX +
                              (terrainVoxelWidth - 1)];
        for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
        }
        terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
        terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
      }
      terrainVoxelShiftY--;
      terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
    }

    while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
      for (int indX = 0; indX < terrainVoxelWidth; indX++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[terrainVoxelWidth * indX];
        for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
        }
        terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] =
            terrainVoxelCloudPtr;
        terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
            ->clear();
      }
      terrainVoxelShiftY++;
      terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
    }
    //=========================================================//
    //====================点云分配到地形体素======================//
    //=========================================================//
    // stack registered laser scans
    pcl::PointXYZI point;
    //
    int laserCloudCropSize = laserCloudCrop->points.size();
    for (int i = 0; i < laserCloudCropSize; i++) {
      point = laserCloudCrop->points[i];
      // + terrainVoxelSize / 2 加上半个体素大小，使取整时自动四舍五入
      int indX = static_cast<int>((point.x - vehicleX + terrainVoxelSize / 2)
                                  //归一化
                                  / terrainVoxelSize)
                 //转换到网格索引
                 + terrainVoxelHalfWidth;
      int indY = static_cast<int>((point.y - vehicleY + terrainVoxelSize / 2) /
                                  terrainVoxelSize) +
                 terrainVoxelHalfWidth;
      //修正负数边界处理
      if (point.x - vehicleX + terrainVoxelSize / 2 < 0) {
        indX--;
      }
      if (point.y - vehicleY + terrainVoxelSize / 2 < 0) {
        indY--;
      }
      /*边界检查：确保索引在 [0, terrainVoxelWidth-1] 范围内
      存储点：将点添加到对应体素的点云中
      统计更新：累加该体素中点的数量（用于后续处理判断）*/
      if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
          indY < terrainVoxelWidth) {
        terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
        terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
      }
    }
    //================================================================== //
    //=======================地形体素降采样与衰减==========================  //
    //================================================================== //
    //遍历所有地形体素，触发降采样与点过滤
    // 点数足够：累计点数 ≥ voxelPointUpdateThre（默认100），保证统计意义
    // 超时：距上次处理的时间 ≥
    // voxelTimeUpdateThre（默认2秒），防止某个体素长期不更新
    // 清除指令：clearingCloud 为 true 时强制处理所有体素

    for (int ind = 0; ind < kTerrainVoxelNum; ind++) {
      if (terrainVoxelUpdateNum[ind] >= config_.voxelPointUpdateThre ||
          laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=
              config_.voxelTimeUpdateThre ||
          config_.clearingCloud) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[ind];

        //====
        //降采样
        laserCloudDwz->clear();
        downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
        downSizeFilter.filter(*laserCloudDwz);
        //====

        terrainVoxelCloudPtr->clear();

        int laserCloudDwzSize = laserCloudDwz->points.size();

        for (int i = 0; i < laserCloudDwzSize; i++) {
          point = laserCloudDwz->points[i];
          float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                           (point.y - vehicleY) * (point.y - vehicleY));
          //  相对高度范围：minRelZ - disRatioZ*dis 到 maxRelZ +
          //  disRatioZ*dis，范围随距离扩大，适应远处地面起伏更大或传感器噪声；

          // 时间衰减：点的“年龄” = 当前时间 − 点的时间戳（intensity
          // 存储），年龄超过 decayTime 且距离 ≥ noDecayDis 时丢弃。近处点（dis
          // < noDecayDis）永不过期，保证局部地图稳定；

          // 手动清除：若点距离 < clearingDis 且 clearingCloud
          // 激活，则丢弃（允许清除固定半径内的旧障碍）。
          if (point.z - vehicleZ > config_.minRelZ - config_.disRatioZ * dis &&
              point.z - vehicleZ < config_.maxRelZ + config_.disRatioZ * dis &&
              (laserCloudTime - systemInitTime - point.intensity <
                   config_.decayTime ||
               dis < config_.noDecayDis) &&
              !(dis < config_.clearingDis && config_.clearingCloud)) {
            terrainVoxelCloudPtr->push_back(point);
          }
        }

        terrainVoxelUpdateNum[ind] = 0;
        terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
      }
    }
    //提取机器人周围的局部点云
    terrainCloud->clear();
    for (int indX = terrainVoxelHalfWidth - 5;
         indX <= terrainVoxelHalfWidth + 5; indX++) {
      for (int indY = terrainVoxelHalfWidth - 5;
           indY <= terrainVoxelHalfWidth + 5; indY++) {
        // 将每个体素内的点云合并到一个新的点云中
        *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
      }
    }

    // estimate ground and compute elevation for each point
    for (int i = 0; i < kPlanarVoxelNum; i++) {
      planarVoxelElev[i] = 0;
      planarVoxelEdge[i] = 0;
      planarVoxelDyObs[i] = 0;
      planarPointElev[i].clear();
    }
    //点云分配到平面体素并收集高度，同时进行动态障碍检测
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];

      int indX = static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                                  planarVoxelSize) +
                 planarVoxelHalfWidth;
      int indY = static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                                  planarVoxelSize) +
                 planarVoxelHalfWidth;

      if (point.x - vehicleX + planarVoxelSize / 2 < 0)
        indX--;
      if (point.y - vehicleY + planarVoxelSize / 2 < 0)
        indY--;

      if (point.z - vehicleZ > config_.minRelZ &&
          point.z - vehicleZ < config_.maxRelZ) {
        for (int dX = -1; dX <= 1; dX++) {
          for (int dY = -1; dY <= 1; dY++) {
            if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                indY + dY >= 0 && indY + dY < planarVoxelWidth) {
              planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY]
                  .push_back(point.z);
            }
          }
        }
      }

      //动态障碍检测（坐标系变换到车辆自身系）
      if (config_.clearDyObs) {
        if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
            indY < planarVoxelWidth) {
          float pointX1 = point.x - vehicleX;
          float pointY1 = point.y - vehicleY;
          float pointZ1 = point.z - vehicleZ;

          float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
          if (dis1 > config_.minDyObsDis) {
            float angle1 =
                atan2(pointZ1 - config_.minDyObsRelZ, dis1) * 180.0 / M_PI;
            if (angle1 > config_.minDyObsAngle) {
              float pointX2 = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
              float pointY2 =
                  -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
              float pointZ2 = pointZ1;

              float pointX3 =
                  pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;
              float pointY3 = pointY2;
              float pointZ3 =
                  pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;

              float pointX4 = pointX3;
              float pointY4 =
                  pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
              float pointZ4 =
                  -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;

              float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
              float angle4 = atan2(pointZ4, dis4) * 180.0 / M_PI;
              if ((angle4 > config_.minDyObsVFOV &&
                   angle4 < config_.maxDyObsVFOV) ||
                  fabs(pointZ4) < config_.absDyObsRelZThre) {
                planarVoxelDyObs[planarVoxelWidth * indX + indY]++;
              }
            }
          } else {
            planarVoxelDyObs[planarVoxelWidth * indX + indY] +=
                config_.minDyObsPointNum;
          }
        }
      }
    }
    //利用当前帧点云反向清除动态标记
    if (config_.clearDyObs) {
      for (int i = 0; i < laserCloudCropSize; i++) {
        point = laserCloudCrop->points[i];

        int indX = static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                                    planarVoxelSize) +
                   planarVoxelHalfWidth;
        int indY = static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                                    planarVoxelSize) +
                   planarVoxelHalfWidth;

        if (point.x - vehicleX + planarVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + planarVoxelSize / 2 < 0)
          indY--;

        if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
            indY < planarVoxelWidth) {
          float pointX1 = point.x - vehicleX;
          float pointY1 = point.y - vehicleY;
          float pointZ1 = point.z - vehicleZ;

          float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
          float angle1 =
              atan2(pointZ1 - config_.minDyObsRelZ, dis1) * 180.0 / M_PI;
          if (angle1 > config_.minDyObsAngle) {
            planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;
          }
        }
      }
    }
    //地面高度估计（分位数或最小值法）
    if (config_.useSorting) {
      // 对每个体素内的点按 Z 排序，取 quantileZ 分位数（默认 0.25，即 25%
      // 分位数，接近最低点但排除少数离群低点）。

      // 若 limitGroundLift 开启且分位数高度超过最小值 + maxGroundLift（默认
      // 0.15 m），则将地面高度钳制为最小值 +
      // 最大升量。这用于防止台阶、小障碍等突然抬高地面估计，保持地面连续性。
      for (int i = 0; i < kPlanarVoxelNum; i++) {
        int planarPointElevSize = planarPointElev[i].size();
        if (planarPointElevSize > 0) {
          sort(planarPointElev[i].begin(), planarPointElev[i].end());

          int quantileID =
              static_cast<int>(config_.quantileZ * planarPointElevSize);
          if (quantileID < 0)
            quantileID = 0;
          else if (quantileID >= planarPointElevSize)
            quantileID = planarPointElevSize - 1;

          if (planarPointElev[i][quantileID] >
                  planarPointElev[i][0] + config_.maxGroundLift &&
              config_.limitGroundLift) {
            planarVoxelElev[i] = planarPointElev[i][0] + config_.maxGroundLift;
          } else {
            planarVoxelElev[i] = planarPointElev[i][quantileID];
          }
        }
      }
    } else {
      //直接取体素内最小 Z 作为地面高度（更保守，但易受噪点影响）。
      for (int i = 0; i < kPlanarVoxelNum; i++) {
        int planarPointElevSize = planarPointElev[i].size();
        if (planarPointElevSize > 0) {
          float minZ = 1000.0;
          int minID = -1;
          for (int j = 0; j < planarPointElevSize; j++) {
            if (planarPointElev[i][j] < minZ) {
              minZ = planarPointElev[i][j];
              minID = j;
            }
          }

          if (minID != -1) {
            planarVoxelElev[i] = planarPointElev[i][minID];
          }
        }
      }
    }

    // terrain_analysis_ext
    // check terrain connectivity to remove ceiling
    // 0：未处理/未访问的体素。

    // 1：已入队等待处理的体素（BFS中间状态）。

    // 2：已处理且与车辆所在区域连通的地面体素。

    //-1：与车辆所在区域高度差过大，标记为天花板/悬空结构。
    if (checkTerrainConn) {
      int seedInd = -1;
      int centerX = planarVoxelHalfWidth;
      int centerY = planarVoxelHalfWidth;

      // 搜索范围从小到大，例如先找 3x3，再 5x5，直到找到有点的体素
      for (int radius = 0; radius < planarVoxelHalfWidth && seedInd == -1;
           radius++) {
        for (int dx = -radius; dx <= radius && seedInd == -1; dx++) {
          for (int dy = -radius; dy <= radius && seedInd == -1; dy++) {
            if (abs(dx) != radius && abs(dy) != radius)
              continue; // 只检查最外圈
            int nx = centerX + dx;
            int ny = centerY + dy;
            if (nx >= 0 && nx < planarVoxelWidth && ny >= 0 &&
                ny < planarVoxelWidth) {
              int idx = planarVoxelWidth * nx + ny;
              // 要求该体素内的点数 >= minBlockPointNum（或其他合理阈值）
              if (planarPointElev[idx].size() >= config_.minBlockPointNum) {
                seedInd = idx;
                break;
              }
            }
          }
        }
      }

      // 如果依然找不到，回退到中心体素（沿用旧逻辑）
      if (seedInd == -1) {
        seedInd =
            planarVoxelWidth * planarVoxelHalfWidth + planarVoxelHalfWidth;
        if (planarPointElev[seedInd].size() == 0)
          planarVoxelElev[seedInd] = vehicleZ + terrainUnderVehicle;
      }

      planarVoxelQueue.push(seedInd);
      planarVoxelConn[seedInd] = 1;
      //============
      int ind = planarVoxelWidth * planarVoxelHalfWidth + planarVoxelHalfWidth;
      // spdlog::info("center voxel index: {}, point count: {}", ind,
      // planarPointElev[ind].size());
      if (planarPointElev[ind].size() == 0)
        planarVoxelElev[ind] = vehicleZ + terrainUnderVehicle;

      planarVoxelQueue.push(ind);
      planarVoxelConn[ind] = 1;
      while (!planarVoxelQueue.empty()) {
        int front = planarVoxelQueue.front();
        planarVoxelConn[front] = 2;
        planarVoxelQueue.pop();

        int indX = static_cast<int>(front / planarVoxelWidth);
        int indY = front % planarVoxelWidth;
        for (int dX = -10; dX <= 10; dX++) {
          for (int dY = -10; dY <= 10; dY++) {
            if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                indY + dY >= 0 && indY + dY < planarVoxelWidth) {
              ind = planarVoxelWidth * (indX + dX) + indY + dY;
              if (planarVoxelConn[ind] == 0 &&
                  planarPointElev[ind].size() > 0) {
                if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) <
                    terrainConnThre) {
                  planarVoxelQueue.push(ind);
                  planarVoxelConn[ind] = 1;
                } else if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) >
                           ceilingFilteringThre) {
                  planarVoxelConn[ind] = -1;
                }
              }
            }
          }
        }
      }
    }

    //生成带相对高程的点云 terrainCloudElev
    terrainCloudElev->clear();
    int terrainCloudElevSize = 0;
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];
      if (point.z - vehicleZ > config_.minRelZ &&
          point.z - vehicleZ < config_.maxRelZ) {
        int indX = static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                                    planarVoxelSize) +
                   planarVoxelHalfWidth;
        int indY = static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                                    planarVoxelSize) +
                   planarVoxelHalfWidth;

        if (point.x - vehicleX + planarVoxelSize / 2 < 0) {
          indX--;
        }
        if (point.y - vehicleY + planarVoxelSize / 2 < 0) {
          indY--;
        }
        if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
            indY < planarVoxelWidth) {
          int ind = planarVoxelWidth * indX +
                    indY; // 体素索引
                          // spdlog::info("planarVoxelConn[ind]:{}",
                          // planarVoxelConn[ind]);
          // ========== 新增：地形连通性过滤 ==========
          if (checkTerrainConn && planarVoxelConn[ind] != 2) {
            continue; // 仅保留连通地面（标记为2）的体素
          }
          // ========================================
          if (planarVoxelDyObs[ind] < config_.minDyObsPointNum ||
              !config_.clearDyObs) {
            float disZ = point.z - planarVoxelElev[ind];
            if (config_.considerDrop)
              disZ = fabs(disZ);
            int planarPointElevSize = planarPointElev[ind].size();
            if (disZ >= 0 && disZ < config_.vehicleHeight &&
                planarPointElevSize >= config_.minBlockPointNum) {
              terrainCloudElev->push_back(point);
              terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;

              terrainCloudElevSize++;
            }
          }
        }
      }
    }

    //当 noDataObstacle 开启且机器人已移动足够距离（noDataInited == 2）时
    if (config_.noDataObstacle && noDataInited == 2) {
      //标记数据不足体素
      for (int i = 0; i < kPlanarVoxelNum; i++) {
        int planarPointElevSize = planarPointElev[i].size();
        if (planarPointElevSize < config_.minBlockPointNum) {
          planarVoxelEdge[i] = 1;
        }
      }
      //形态学扩散 只保留内部连片的无数据区域，避免在少量零散空洞中生成障碍物。
      for (int noDataBlockSkipCount = 0;
           noDataBlockSkipCount < config_.noDataBlockSkipNum;
           noDataBlockSkipCount++) {
        for (int i = 0; i < kPlanarVoxelNum; i++) {
          if (planarVoxelEdge[i] >= 1) {
            int indX = static_cast<int>(i / planarVoxelWidth);
            int indY = i % planarVoxelWidth;
            bool edgeVoxel = false;
            for (int dX = -1; dX <= 1; dX++) {
              for (int dY = -1; dY <= 1; dY++) {
                if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                    indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                  if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY +
                                      dY] < planarVoxelEdge[i]) {
                    edgeVoxel = true;
                  }
                }
              }
            }

            if (!edgeVoxel)
              planarVoxelEdge[i]++;
          }
        }
      }
      //在保留体素中插入障碍方块
      for (int i = 0; i < kPlanarVoxelNum; i++) {
        if (planarVoxelEdge[i] > config_.noDataBlockSkipNum) {
          int indX = static_cast<int>(i / planarVoxelWidth);
          int indY = i % planarVoxelWidth;

          point.x = planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
          point.y = planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
          point.z = vehicleZ;
          point.intensity = config_.vehicleHeight;

          point.x -= planarVoxelSize / 4.0;
          point.y -= planarVoxelSize / 4.0;
          terrainCloudElev->push_back(point);

          point.x += planarVoxelSize / 2.0;
          terrainCloudElev->push_back(point);

          point.y += planarVoxelSize / 2.0;
          terrainCloudElev->push_back(point);

          point.x -= planarVoxelSize / 2.0;
          terrainCloudElev->push_back(point);
        }
      }
    }
    ///=====
    // ========== 历史点云：离开局部地图的区域，保留最近 5 秒内的点 ==========
    const double historicalDecayTime = 5.0; // 历史点云保留时间（秒）
    const int localHalfRange =
        5; // 局部地图半范围（与前面提取 terrainCloud 一致）

    pcl::PointCloud<pcl::PointXYZI>::Ptr historicalCloud(
        new pcl::PointCloud<pcl::PointXYZI>());

    for (int indX = 0; indX < terrainVoxelWidth; indX++) {
      // 跳过局部地图范围内的体素
      if (indX >= terrainVoxelHalfWidth - localHalfRange &&
          indX <= terrainVoxelHalfWidth + localHalfRange) {
        continue; // X 在局部范围，等待 Y 的判断
      }
      // X 不在局部范围，所有 Y 都可以加入
      for (int indY = 0; indY < terrainVoxelWidth; indY++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr voxel =
            terrainVoxelCloud[terrainVoxelWidth * indX + indY];
        for (const auto &pt : voxel->points) {
          double age = laserCloudTime - systemInitTime - pt.intensity;
          if (age < historicalDecayTime) {
            // 简单高度过滤（可选，保留车辆附近常见高度范围）
            if (pt.z - vehicleZ > config_.minRelZ &&
                pt.z - vehicleZ < config_.maxRelZ) {
              pcl::PointXYZI hist_pt = pt;
              hist_pt.intensity = -1.0; // 标记为历史点（与局部地图点区分）
              historicalCloud->push_back(hist_pt);
            }
          }
        }
      }
    }

    // 处理 Y 方向：当 X 在局部范围内时，Y 可能部分在局部范围外
    for (int indX = terrainVoxelHalfWidth - localHalfRange;
         indX <= terrainVoxelHalfWidth + localHalfRange; indX++) {
      for (int indY = 0; indY < terrainVoxelWidth; indY++) {
        // 跳过局部地图范围内的体素
        if (indY >= terrainVoxelHalfWidth - localHalfRange &&
            indY <= terrainVoxelHalfWidth + localHalfRange) {
          continue;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr voxel =
            terrainVoxelCloud[terrainVoxelWidth * indX + indY];
        for (const auto &pt : voxel->points) {
          double age = laserCloudTime - systemInitTime - pt.intensity;
          if (age < historicalDecayTime) {
            if (pt.z - vehicleZ > config_.minRelZ &&
                pt.z - vehicleZ < config_.maxRelZ) {
              pcl::PointXYZI hist_pt = pt;
              hist_pt.intensity = -1.0;
              historicalCloud->push_back(hist_pt);
            }
          }
        }
      }
    }

    // 对历史点云降采样（避免过多点）
    pcl::PointCloud<pcl::PointXYZI>::Ptr historicalCloudDwz(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> histDownSizeFilter;
    histDownSizeFilter.setLeafSize(planarVoxelSize, planarVoxelSize,
                                   planarVoxelSize); // 或独立参数
    histDownSizeFilter.setInputCloud(historicalCloud);
    histDownSizeFilter.filter(*historicalCloudDwz);

    // 合并到最终输出点云
    *terrainCloudElev += *historicalCloudDwz;
    //===
    config_.clearingCloud = false;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr get_pointcloud() {
    return terrainCloudElev;
  }

private:
  double laserCloudTime, systemInitTime;
  int noDataInited = 0;
  // terrain voxel parameters
  //单个体素的边长（米）
  float terrainVoxelSize = 2.0;
  //map size 网格宽度（体素个数）
  const int terrainVoxelWidth = 43;

  int terrainVoxelShiftX = 0;
  int terrainVoxelShiftY = 0;

  int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
  //体素的总数
  int kTerrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;

  // planar voxel parameters
  //单个体素边长（米）
  float planarVoxelSize = 0.2;
  //网格宽度
  const int planarVoxelWidth = 51;

  int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
  int kPlanarVoxelNum = planarVoxelWidth * planarVoxelWidth;
  TerrainAnalysisConfig config_;

  float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
  float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
  float vehicleXRec = 0, vehicleYRec = 0;

  float sinVehicleRoll = 0, cosVehicleRoll = 0;
  float sinVehiclePitch = 0, cosVehiclePitch = 0;
  float sinVehicleYaw = 0, cosVehicleYaw = 0;

private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr rawBatchCloud;
  double lastBatchTime = 0; // 上次处理时的 laserCloudTime
private:
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> terrainVoxelCloud;

  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev;
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
  std::vector<int> terrainVoxelUpdateNum;
  std::vector<float> terrainVoxelUpdateTime;
  std::vector<float> planarVoxelElev;
  std::vector<int> planarVoxelEdge;
  std::vector<int> planarVoxelDyObs;
  std::vector<std::vector<float>> planarPointElev;

private:
  bool checkTerrainConn = true;
  double terrainUnderVehicle = -0.75;

  //
  double terrainConnThre = 2.0;
  //
  
  double ceilingFilteringThre = 2.0;
  std::queue<int> planarVoxelQueue;
  std::vector<int> planarVoxelConn;
  // int planarVoxelConn[kPlanarVoxelNum] = {0};
};

} // namespace TerrainAnalysis