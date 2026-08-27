#include <CalibrationInstructionTranslation/ABBTranslation/RapidModuleGenerator.h>

#include <RotationBodyTrajectoryPlanning/TrajectoryPlanning/TrajectoryGroupEditor.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <locale>
#include <optional>
#include <sstream>

namespace smrobot::spray::rotationbody
{
    namespace
    {
        std::string number(double value)
        {
            if(std::abs(value) < 5.0e-10) {
                value = 0.0;
            }
            std::ostringstream stream;
            stream.imbue(std::locale::classic());
            stream << std::fixed << std::setprecision(6) << value;
            std::string result = stream.str();
            while(result.size() > 1 && result.back() == '0') {
                result.pop_back();
            }
            if(!result.empty() && result.back() == '.') {
                result.pop_back();
            }
            return result;
        }

        std::string padded2(int value)
        {
            std::ostringstream stream;
            stream << std::setw(2) << std::setfill('0') << value;
            return stream.str();
        }

        std::string fixed3(double value)
        {
            if(std::abs(value) < 5.0e-10) {
                value = 0.0;
            }
            std::ostringstream stream;
            stream.imbue(std::locale::classic());
            stream << std::fixed << std::setprecision(3) << value;
            return stream.str();
        }

        Eigen::Isometry3d basePose(
            const PublishedTrajectoryPlan& plan,
            const TrajectoryPosePoint& point)
        {
            return plan.baseFromPlanning * point.planningFromTool;
        }

        std::string robtarget(
            const std::string& name,
            const Eigen::Isometry3d& pose,
            const char* externalAxes)
        {
            Eigen::Quaterniond quaternion(pose.linear());
            quaternion.normalize();
            if(quaternion.w() < 0.0) {
                quaternion.coeffs() *= -1.0;
            }
            const Eigen::Vector3d millimeters = pose.translation() * 1000.0;
            std::ostringstream stream;
            stream << "    CONST robtarget " << name << ":=[["
                << number(millimeters.x()) << ','
                << number(millimeters.y()) << ','
                << number(millimeters.z()) << "],["
                << number(quaternion.w()) << ','
                << number(quaternion.x()) << ','
                << number(quaternion.y()) << ','
                << number(quaternion.z())
                << "],[0,0,0,0]," << externalAxes << "];\n";
            return stream.str();
        }

        const TrajectoryPass* sequencePass(
            const PublishedTrajectoryPlan& plan,
            const RapidSequenceEntry& entry)
        {
            return entry.kind == RapidSequenceEntryKind::Trajectory
                ? TrajectoryGroupEditor::find(plan.group, entry.trajectoryPassId)
                : nullptr;
        }

        PlanningResult<Eigen::Matrix3d> safetyOrientation(
            const PublishedTrajectoryPlan& plan,
            const std::vector<RapidSequenceEntry>& sequence,
            std::size_t safetyIndex)
        {
            for(std::size_t index = safetyIndex + 1; index < sequence.size(); ++index) {
                if(const TrajectoryPass* pass = sequencePass(plan, sequence[index])) {
                    return PlanningResult<Eigen::Matrix3d>::success(
                        basePose(plan, pass->trajectory.linearPoints.front()).linear());
                }
            }
            for(std::size_t index = safetyIndex; index-- > 0;) {
                if(const TrajectoryPass* pass = sequencePass(plan, sequence[index])) {
                    return PlanningResult<Eigen::Matrix3d>::success(
                        basePose(plan, pass->trajectory.linearPoints.back()).linear());
                }
            }
            return PlanningResult<Eigen::Matrix3d>::failure(
                PlanningErrorCode::InvalidArgument,
                "A safety point requires an adjacent trajectory from which to inherit orientation.");
        }

        bool coincident(
            const Eigen::Vector3d& lhs,
            const Eigen::Vector3d& rhs) noexcept
        {
            return (lhs - rhs).norm() <= 1.0e-7;
        }
    }

    PlanningResult<RapidModule> RapidModuleGenerator::generate(
        const PublishedTrajectoryPlan& plan,
        const RapidExportSettings& settings,
        const std::vector<RapidSequenceEntry>& sequence)
    {
        if(plan.objectId.empty() || !plan.baseFromPlanning.matrix().allFinite() ||
            !settings.safetyPositionBaseMeters.allFinite() ||
            !std::isfinite(settings.safetySpeedMetersPerSecond) ||
            settings.safetySpeedMetersPerSecond <= 0.0 ||
            !isValidRapidIdentifier(settings.moduleName) ||
            !isValidRapidIdentifier(settings.toolDataName) || sequence.empty()) {
            return PlanningResult<RapidModule>::failure(
                PlanningErrorCode::InvalidArgument,
                "ABB RAPID settings, workpiece base pose, or instruction sequence is invalid.");
        }
        PlanningResult<void> groupValidation = TrajectoryGroupEditor::validate(plan.group);
        if(!groupValidation) {
            return PlanningResult<RapidModule>::failure(
                groupValidation.error.code,
                groupValidation.error.message);
        }

        std::ostringstream speedDeclarations;
        std::ostringstream controlDeclarations;
        std::ostringstream targetDeclarations;
        std::ostringstream mainProcedure;
        std::ostringstream sprayProcedure;
        RapidPreviewSteps previewSteps;
        speedDeclarations.imbue(std::locale::classic());
        controlDeclarations.imbue(std::locale::classic());
        targetDeclarations.imbue(std::locale::classic());
        mainProcedure.imbue(std::locale::classic());
        sprayProcedure.imbue(std::locale::classic());

        const TrajectoryPass* firstPass = nullptr;
        for(const RapidSequenceEntry& entry : sequence) {
            if(entry.kind != RapidSequenceEntryKind::Trajectory) continue;
            firstPass = sequencePass(plan, entry);
            if(firstPass != nullptr) break;
        }
        if(firstPass == nullptr || !firstPass->trajectory.hasValidPoints() ||
            !std::isfinite(firstPass->trajectory.parameters.positionerRpm)) {
            return PlanningResult<RapidModule>::failure(
                PlanningErrorCode::InvalidArgument,
                "The ABB instruction sequence must contain a trajectory with a finite positioner RPM.");
        }

        speedDeclarations << "    VAR speeddata vSafeCustom:=["
            << number(settings.safetySpeedMetersPerSecond * 1000.0)
            << ",100,5000,1000];\n";
        controlDeclarations << "    PERS num nTableRPM:="
            << number(firstPass->trajectory.parameters.positionerRpm)
            << ";\n"
            << "    PERS num nSprayTimes:=15;\n"
            << "    VAR num i;\n";
        mainProcedure << "    PROC main()\n"
            << "        ConfJ \\Off;\n"
            << "        ConfL \\Off;\n"
            << "\n        StartTable;\n"
            << "\n        FOR i FROM 1 TO nSprayTimes DO\n"
            << "            SprayOnce;\n"
            << "        ENDFOR\n"
            << "\n        StopTable;\n"
            << "    ENDPROC\n\n"
            << "    PROC StartTable()\n"
            << "        ActUnit STN1;\n"
            << "        IndReset STN1,1\\RefNum:=0\\Short;\n"
            << "        IndCMove STN1,1,nTableRPM * 6;\n"
            << "        WaitTime 3;\n"
            << "    ENDPROC\n\n";

        constexpr const char* safetyExternalAxes =
            "[9E+09,90.6655,-0.000945636,9E+09,9E+09,9E+09]";
        constexpr const char* passExternalAxes =
            "[9E+09,90.6666,-0.000918239,9E+09,9E+09,9E+09]";

        int safetyNumber = 0;
        int trajectoryNumber = 0;
        std::optional<Eigen::Isometry3d> previousTrajectoryEnd;
        for(std::size_t sequenceIndex = 0;
            sequenceIndex < sequence.size();
            ++sequenceIndex) {
            const RapidSequenceEntry& entry = sequence[sequenceIndex];
            if(entry.kind == RapidSequenceEntryKind::SafetyPoint) {
                PlanningResult<Eigen::Matrix3d> orientation =
                    safetyOrientation(plan, sequence, sequenceIndex);
                // Once the sequence has started, keep safety moves continuous
                // with the final pose already sent to the robot.  The first
                // safety point still inherits from its following trajectory.
                if(previousTrajectoryEnd) {
                    orientation = PlanningResult<Eigen::Matrix3d>::success(
                        previousTrajectoryEnd->linear());
                }
                if(!orientation) {
                    return PlanningResult<RapidModule>::failure(
                        orientation.error.code,
                        orientation.error.message);
                }
                ++safetyNumber;
                const std::string name = "pSafe" + padded2(safetyNumber);
                Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
                pose.translation() = settings.safetyPositionBaseMeters;
                pose.linear() = orientation.value;
                targetDeclarations << robtarget(name, pose, safetyExternalAxes);
                sprayProcedure << "        MoveJ " << name
                    << ",vSafeCustom,fine," << settings.toolDataName
                    << ";\n";
                RapidPreviewStep preview;
                preview.sourceKind = RapidSequenceEntryKind::SafetyPoint;
                preview.instruction = "MoveJ";
                preview.targetName = name;
                preview.sequenceIndex = sequenceIndex;
                preview.baseFromTool = pose;
                previewSteps.push_back(std::move(preview));
                previousTrajectoryEnd.reset();
                continue;
            }

            const TrajectoryPass* pass = sequencePass(plan, entry);
            if(pass == nullptr || !pass->trajectory.hasValidPoints()) {
                return PlanningResult<RapidModule>::failure(
                    PlanningErrorCode::InvalidArgument,
                    "The ABB instruction sequence references a missing trajectory.");
            }
            if(!std::isfinite(pass->trajectory.parameters.positionerRpm) ||
                std::abs(pass->trajectory.parameters.positionerRpm -
                    firstPass->trajectory.parameters.positionerRpm) > 1.0e-9) {
                return PlanningResult<RapidModule>::failure(
                    PlanningErrorCode::InvalidArgument,
                    "All trajectories in one ABB export must use the same finite positioner RPM.");
            }
            ++trajectoryNumber;
            const std::string suffix = padded2(trajectoryNumber);
            const std::string startName = "pPass" + suffix + "Start";
            const std::string endName = "pPass" + suffix + "End";
            const std::string speedName = "vSpray" + suffix;
            Eigen::Isometry3d start =
                basePose(plan, pass->trajectory.linearPoints.front());
            Eigen::Isometry3d end =
                basePose(plan, pass->trajectory.linearPoints.back());

            // A reversed pass is commonly used to return over the same spray
            // line.  When its start position is the previous pass' end,
            // preserve the complete tool orientation for the whole pass.
            // This avoids an unnecessary 180-degree wrist rotation while the
            // robot travels back along the already coincident line.
            if(previousTrajectoryEnd && coincident(
                previousTrajectoryEnd->translation(), start.translation())) {
                start.linear() = previousTrajectoryEnd->linear();
                end.linear() = previousTrajectoryEnd->linear();
            }
            speedDeclarations << "    VAR speeddata " << speedName << ":=["
                << number(pass->trajectory.parameters.speedMetersPerSecond * 1000.0)
                << ",100,5000,1000];\n";
            targetDeclarations << robtarget(startName, start, passExternalAxes);
            targetDeclarations << robtarget(endName, end, passExternalAxes);
            sprayProcedure << "\n        ! Pass" << trajectoryNumber
                << ", tilt="
                << fixed3(pass->trajectory.parameters.tiltRadians *
                    180.0 / std::acos(-1.0))
                << " deg, D="
                << fixed3(pass->trajectory.parameters.sprayDistanceMeters * 1000.0)
                << " mm\n";
            sprayProcedure << "        MoveJ " << startName
                << ",vSafeCustom,fine," << settings.toolDataName
                << ";\n";
            sprayProcedure << "        MoveL " << endName << ',' << speedName
                << ",fine," << settings.toolDataName
                << ";\n";

            RapidPreviewStep startPreview;
            startPreview.sourceKind = RapidSequenceEntryKind::Trajectory;
            startPreview.instruction = "MoveJ";
            startPreview.targetName = startName;
            startPreview.trajectoryPassId = entry.trajectoryPassId;
            startPreview.sequenceIndex = sequenceIndex;
            startPreview.baseFromTool = start;
            previewSteps.push_back(std::move(startPreview));

            RapidPreviewStep endPreview;
            endPreview.sourceKind = RapidSequenceEntryKind::Trajectory;
            endPreview.instruction = "MoveL";
            endPreview.targetName = endName;
            endPreview.trajectoryPassId = entry.trajectoryPassId;
            endPreview.sequenceIndex = sequenceIndex;
            endPreview.baseFromTool = end;
            previewSteps.push_back(std::move(endPreview));
            previousTrajectoryEnd = end;
        }
        mainProcedure << "    PROC SprayOnce()\n"
            << sprayProcedure.str()
            << "    ENDPROC\n\n"
            << "    PROC StopTable()\n"
            << "        IndCMove STN1,1,0;\n"
            << "        WaitTime 3;\n"
            << "    ENDPROC\n";

        RapidModule module;
        std::ostringstream output;
        output << "MODULE " << settings.moduleName << "\n"
            << "    ! Generated by RS2026 rotation-body trajectory planning.\n"
            << "    ! Workpiece-local poses are converted with T_base_planning.\n"
            << "    ! nTableRPM is copied from trajectory planning and remains operator-editable.\n"
            << "    ! nSprayTimes defaults to 15 and remains operator-editable.\n"
            << "    ! Verify reachability, collisions, TCP and robot configuration in RobotStudio.\n\n"
            << speedDeclarations.str() << '\n'
            << controlDeclarations.str() << '\n'
            << targetDeclarations.str() << '\n'
            << mainProcedure.str()
            << "ENDMODULE\n";
        module.code = output.str();
        module.previewSteps = std::move(previewSteps);
        return PlanningResult<RapidModule>::success(std::move(module));
    }

    bool RapidModuleGenerator::isValidRapidIdentifier(const std::string& value) noexcept
    {
        if(value.empty() || value.size() > 32 ||
            !std::isalpha(static_cast<unsigned char>(value.front()))) {
            return false;
        }
        return std::all_of(
            value.begin() + 1,
            value.end(),
            [](char character) {
                const unsigned char value = static_cast<unsigned char>(character);
                return std::isalnum(value) || character == '_';
            });
    }
}
