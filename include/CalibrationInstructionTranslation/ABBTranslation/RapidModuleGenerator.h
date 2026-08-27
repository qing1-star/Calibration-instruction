#pragma once

#include <RotationBodyTrajectoryPlanning/TrajectoryPlanning/TrajectoryTypes.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <cstddef>
#include <string>
#include <vector>

namespace smrobot::spray::rotationbody
{
    struct RapidExportSettings
    {
        Eigen::Vector3d safetyPositionBaseMeters = Eigen::Vector3d::Zero();
        double safetySpeedMetersPerSecond{ 0.2 };
        std::string moduleName{ "SprayRotation" };
        std::string fileName{ "Spraybichi" };
        std::string toolDataName{ "penqiang" };
        std::string outputDirectory;
    };

    struct RapidPreviewStep
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        RapidSequenceEntryKind sourceKind{ RapidSequenceEntryKind::SafetyPoint };
        std::string instruction;
        std::string targetName;
        std::string trajectoryPassId;
        std::size_t sequenceIndex{ 0 };
        Eigen::Isometry3d baseFromTool = Eigen::Isometry3d::Identity();
    };

    using RapidPreviewSteps = std::vector<
        RapidPreviewStep,
        Eigen::aligned_allocator<RapidPreviewStep>>;

    struct RapidModule
    {
        std::string code;
        std::vector<std::string> warnings;
        RapidPreviewSteps previewSteps;
    };

    class RapidModuleGenerator
    {
    public:
        static PlanningResult<RapidModule> generate(
            const PublishedTrajectoryPlan& plan,
            const RapidExportSettings& settings,
            const std::vector<RapidSequenceEntry>& sequence);
        static bool isValidRapidIdentifier(const std::string& value) noexcept;
    };
}
