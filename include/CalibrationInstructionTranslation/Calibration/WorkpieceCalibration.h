#pragma once

#include <RotationBodyTrajectoryPlanning/Core/PlanningTypes.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <array>
#include <string>
#include <vector>

namespace smrobot::spray::rotationbody
{
    enum class CalibrationMode
    {
        Cylinder3d,
        Circle2d
    };

    using CalibrationPointList = std::vector<
        Eigen::Vector3d,
        Eigen::aligned_allocator<Eigen::Vector3d>>;

    struct CalibrationAxisFit
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CalibrationMode mode{ CalibrationMode::Cylinder3d };
        Eigen::Vector3d axisPointBaseMeters = Eigen::Vector3d::Zero();
        Eigen::Vector3d axisDirectionBase = Eigen::Vector3d::UnitZ();
        double radiusMeters{ 0.0 };
        double rmsResidualMeters{ 0.0 };
        double maximumResidualMeters{ 0.0 };
        int iterations{ 0 };
        bool lowAxialSpan{ false };
        std::vector<double> residualsMeters;
        CalibrationPointList sourcePointsBaseMeters;
    };

    struct WorkpieceFrameCalibration
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Isometry3d baseFromPlanning = Eigen::Isometry3d::Identity();
        TransformComponents baseFromPlanningComponents;
        Eigen::Vector3d topReferenceBaseMeters = Eigen::Vector3d::Zero();
        Eigen::Vector3d topCenterBaseMeters = Eigen::Vector3d::Zero();
        Eigen::Vector3d baseOriginBaseMeters = Eigen::Vector3d::Zero();
        Eigen::Vector3d yDirectionStartBaseMeters = Eigen::Vector3d::Zero();
        Eigen::Vector3d yDirectionEndBaseMeters = Eigen::Vector3d::Zero();
        Eigen::Vector3d xAxisBase = Eigen::Vector3d::UnitX();
        Eigen::Vector3d yAxisBase = Eigen::Vector3d::UnitY();
        Eigen::Vector3d zAxisBase = Eigen::Vector3d::UnitZ();
        Eigen::Vector4d abbQuaternionWxyz{ 1.0, 0.0, 0.0, 0.0 };
        double workpieceHeightMeters{ 0.0 };
        double fittedRadiusMeters{ 0.0 };
        double yDirectionDistanceMeters{ 0.0 };
        bool shortYDirectionBaseline{ false };
    };

    struct ModeTwoCalibrationInput
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::array<Eigen::Vector3d, 6> circlePointsBaseMeters{};
        Eigen::Vector3d topReferenceBaseMeters = Eigen::Vector3d::Zero();
        Eigen::Vector3d yDirectionStartBaseMeters = Eigen::Vector3d::Zero();
        Eigen::Vector3d yDirectionEndBaseMeters = Eigen::Vector3d::Zero();
        Eigen::Vector3d safetyPositionBaseMeters = Eigen::Vector3d::Zero();
    };

    class CalibrationTextParser final
    {
    public:
        static PlanningResult<ModeTwoCalibrationInput> parseModeTwo(
            const std::string& text);
    };

    class WorkpieceCalibrationSolver final
    {
    public:
        static PlanningResult<CalibrationAxisFit> fitCylinder3d(
            const CalibrationPointList& pointsBaseMeters);

        static PlanningResult<CalibrationAxisFit> fitCircle2d(
            const CalibrationPointList& pointsBaseMeters,
            double axialPlaneToleranceMeters = 0.0001);

        static PlanningResult<WorkpieceFrameCalibration> computeWorkpieceFrame(
            const CalibrationAxisFit& axisFit,
            const Eigen::Vector3d& topReferenceBaseMeters,
            double workpieceHeightMeters,
            const Eigen::Vector3d& yDirectionStartBaseMeters,
            const Eigen::Vector3d& yDirectionEndBaseMeters);
    };
}
