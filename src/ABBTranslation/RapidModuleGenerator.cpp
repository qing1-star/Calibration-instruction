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

        std::string padded3(int value)
        {
            std::ostringstream stream;
            stream << std::setw(3) << std::setfill('0') << value;
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

        std::ostringstream declarations;
        std::ostringstream procedure;
        declarations.imbue(std::locale::classic());
        procedure.imbue(std::locale::classic());
        RapidPreviewSteps previewSteps;
        declarations << "    VAR speeddata vSafeCustom:=["
            << number(settings.safetySpeedMetersPerSecond * 1000.0)
            << ",100,5000,1000];\n";
        procedure << "    PROC main()\n"
            << "        ConfJ \\Off;\n"
            << "        ConfL \\Off;\n";

        constexpr const char* externalAxes =
            "[9E9,9E9,9E9,9E9,9E9,9E9]";
        int safetyNumber = 0;
        int trajectoryNumber = 0;
        std::optional<Eigen::Isometry3d> previousTrajectoryEnd;
        for(std::size_t sequenceIndex = 0;
            sequenceIndex < sequence.size(); ++sequenceIndex) {
            const RapidSequenceEntry& entry = sequence[sequenceIndex];
            if(entry.kind == RapidSequenceEntryKind::SafetyPoint) {
                PlanningResult<Eigen::Matrix3d> orientation =
                    safetyOrientation(plan, sequence, sequenceIndex);
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
                const std::string name = "pSafe" + padded3(safetyNumber);
                Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
                pose.translation() = settings.safetyPositionBaseMeters;
                pose.linear() = orientation.value;
                declarations << robtarget(name, pose, externalAxes);
                procedure << "        MoveJ " << name
                    << ",vSafeCustom,fine," << settings.toolDataName
                    << "\\WObj:=wobj0;\n";
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
            ++trajectoryNumber;
            const std::string suffix = padded3(trajectoryNumber);
            const std::string startName = "pTraj" + suffix + "Start";
            const std::string endName = "pTraj" + suffix + "End";
            const std::string speedName = "vSpray" + suffix;
            Eigen::Isometry3d start =
                basePose(plan, pass->trajectory.linearPoints.front());
            Eigen::Isometry3d end =
                basePose(plan, pass->trajectory.linearPoints.back());
            if(previousTrajectoryEnd && coincident(
                previousTrajectoryEnd->translation(), start.translation())) {
                start.linear() = previousTrajectoryEnd->linear();
                end.linear() = previousTrajectoryEnd->linear();
            }
            declarations << "    VAR speeddata " << speedName << ":=["
                << number(pass->trajectory.parameters.speedMetersPerSecond * 1000.0)
                << ",100,5000,1000];\n"
                << robtarget(startName, start, externalAxes)
                << robtarget(endName, end, externalAxes);
            procedure << "        MoveJ " << startName
                << ",vSafeCustom,fine," << settings.toolDataName
                << "\\WObj:=wobj0;\n"
                << "        MoveL " << endName << ',' << speedName
                << ",fine," << settings.toolDataName
                << "\\WObj:=wobj0;\n";

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
        procedure << "    ENDPROC\n";

        RapidModule module;
        std::ostringstream output;
        output << "MODULE " << settings.moduleName << "\n"
            << "    ! Generated by RS2026 rotation-body trajectory planning.\n"
            << "    ! Workpiece-local poses are converted with T_base_planning.\n"
            << "    ! Verify reachability, collisions, TCP and robot configuration in RobotStudio.\n\n"
            << declarations.str() << '\n'
            << procedure.str()
            << "ENDMODULE\n";
        module.code = output.str();
        module.previewSteps = std::move(previewSteps);
        return PlanningResult<RapidModule>::success(std::move(module));
    }

    PlanningResult<RapidModule> RapidModuleGenerator::generateScheme(
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

        struct IndexedPass
        {
            const TrajectoryPass* pass{ nullptr };
        };
        std::vector<IndexedPass> passes;
        for(std::size_t index = 0; index < sequence.size(); ++index) {
            if(sequence[index].kind != RapidSequenceEntryKind::Trajectory) continue;
            const TrajectoryPass* pass = sequencePass(plan, sequence[index]);
            if(pass == nullptr || !pass->trajectory.hasValidPoints()) {
                return PlanningResult<RapidModule>::failure(
                    PlanningErrorCode::InvalidArgument,
                    "The ABB instruction sequence references a missing trajectory.");
            }
            passes.push_back({ pass });
        }
        if(passes.size() < 2 || passes.size() % 2 != 0) {
            return PlanningResult<RapidModule>::failure(
                PlanningErrorCode::InvalidArgument,
                "SprayScheme requires an even number of trajectories arranged as forward/return pairs.");
        }
        const double positionerRpm =
            passes.front().pass->trajectory.parameters.positionerRpm;
        if(!std::isfinite(positionerRpm)) {
            return PlanningResult<RapidModule>::failure(
                PlanningErrorCode::InvalidArgument,
                "SprayScheme requires a finite positioner RPM.");
        }

        std::ostringstream speedDeclarations;
        std::ostringstream controlDeclarations;
        std::ostringstream targetDeclarations;
        std::ostringstream sprayProcedure;
        speedDeclarations.imbue(std::locale::classic());
        controlDeclarations.imbue(std::locale::classic());
        targetDeclarations.imbue(std::locale::classic());
        sprayProcedure.imbue(std::locale::classic());
        speedDeclarations << "    VAR speeddata vSafeCustom:=["
            << number(settings.safetySpeedMetersPerSecond * 1000.0)
            << ",100,5000,1000];\n";
        controlDeclarations << "    PERS num nTableRPM:="
            << number(positionerRpm) << ";\n"
            << "    PERS num nSprayTimes:=15;\n"
            << "    VAR num i;\n";

        constexpr const char* safetyExternalAxes =
            "[9E+09,90.6655,-0.000945636,9E+09,9E+09,9E+09]";
        constexpr const char* passExternalAxes =
            "[9E+09,90.6666,-0.000918239,9E+09,9E+09,9E+09]";
        for(std::size_t pairIndex = 0; pairIndex < passes.size() / 2; ++pairIndex) {
            const IndexedPass& forward = passes[pairIndex * 2];
            const IndexedPass& returning = passes[pairIndex * 2 + 1];
            if(!std::isfinite(forward.pass->trajectory.parameters.positionerRpm) ||
                !std::isfinite(returning.pass->trajectory.parameters.positionerRpm) ||
                std::abs(forward.pass->trajectory.parameters.positionerRpm - positionerRpm) > 1.0e-9 ||
                std::abs(returning.pass->trajectory.parameters.positionerRpm - positionerRpm) > 1.0e-9) {
                return PlanningResult<RapidModule>::failure(
                    PlanningErrorCode::InvalidArgument,
                    "All trajectories in one SprayScheme export must use the same positioner RPM.");
            }
            if(std::abs(forward.pass->trajectory.parameters.speedMetersPerSecond -
                returning.pass->trajectory.parameters.speedMetersPerSecond) > 1.0e-9) {
                return PlanningResult<RapidModule>::failure(
                    PlanningErrorCode::InvalidArgument,
                    "Each SprayScheme forward/return pair must use the same spray speed.");
            }

            Eigen::Isometry3d start = basePose(
                plan, forward.pass->trajectory.linearPoints.front());
            Eigen::Isometry3d end = basePose(
                plan, forward.pass->trajectory.linearPoints.back());
            Eigen::Isometry3d returnStart = basePose(
                plan, returning.pass->trajectory.linearPoints.front());
            Eigen::Isometry3d returnEnd = basePose(
                plan, returning.pass->trajectory.linearPoints.back());
            if(!coincident(end.translation(), returnStart.translation()) ||
                !coincident(start.translation(), returnEnd.translation())) {
                return PlanningResult<RapidModule>::failure(
                    PlanningErrorCode::InvalidArgument,
                    "Each SprayScheme trajectory pair must return over the same line in reverse.");
            }
            returnEnd.linear() = end.linear();

            Eigen::Isometry3d safeIn = Eigen::Isometry3d::Identity();
            safeIn.translation() = settings.safetyPositionBaseMeters;
            safeIn.linear() = start.linear();
            Eigen::Isometry3d safeOut = Eigen::Isometry3d::Identity();
            safeOut.translation() = settings.safetyPositionBaseMeters;
            safeOut.linear() = returnEnd.linear();

            const std::string suffix = padded2(static_cast<int>(pairIndex + 1));
            const std::string safeInName = "pSafe" + suffix + "In";
            const std::string startName = "pPass" + suffix + "Start";
            const std::string endName = "pPass" + suffix + "End";
            const std::string returnName = "pPass" + suffix + "Return";
            const std::string safeOutName = "pSafe" + suffix + "Out";
            const std::string speedName = "vSpray" + suffix;
            speedDeclarations << "    VAR speeddata " << speedName << ":=["
                << number(forward.pass->trajectory.parameters.speedMetersPerSecond * 1000.0)
                << ",100,5000,1000];\n";
            targetDeclarations << robtarget(safeInName, safeIn, safetyExternalAxes)
                << robtarget(startName, start, passExternalAxes)
                << robtarget(endName, end, passExternalAxes)
                << robtarget(returnName, returnEnd, passExternalAxes)
                << robtarget(safeOutName, safeOut, safetyExternalAxes)
                << '\n';

            if(pairIndex > 0) {
                sprayProcedure << "        MoveJ " << safeInName
                    << ",vSafeCustom,fine," << settings.toolDataName << ";\n";
            }
            sprayProcedure << "        MoveJ " << startName
                << ",vSafeCustom,fine," << settings.toolDataName << ";\n"
                << "        MoveL " << endName << ',' << speedName
                << ",fine," << settings.toolDataName << ";\n"
                << "        MoveL " << returnName << ',' << speedName
                << ",fine," << settings.toolDataName << ";\n"
                << "        MoveJ " << safeOutName
                << ",vSafeCustom,fine," << settings.toolDataName << ";\n\n";
        }

        std::ostringstream mainProcedure;
        mainProcedure << "    PROC main()\n"
            << "        ConfJ \\Off;\n"
            << "        ConfL \\Off;\n\n"
            << "        MoveJ pSafe01In,vSafeCustom,fine,"
            << settings.toolDataName << ";\n\n"
            << "        StartTable;\n\n"
            << "        FOR i FROM 1 TO nSprayTimes DO\n"
            << "            SprayOnce;\n"
            << "        ENDFOR\n\n"
            << "        StopTable;\n"
            << "    ENDPROC\n\n"
            << "    PROC StartTable()\n"
            << "        ActUnit STN1;\n"
            << "        IndReset STN1,1\\RefNum:=0\\Short;\n"
            << "        IndCMove STN1,1,nTableRPM * 6;\n"
            << "        WaitTime 3;\n"
            << "    ENDPROC\n\n"
            << "    PROC SprayOnce()\n\n"
            << sprayProcedure.str()
            << "    ENDPROC\n\n"
            << "    PROC StopTable()\n"
            << "        IndCMove STN1,1,0;\n"
            << "        WaitTime 3;\n"
            << "    ENDPROC\n";

        RapidModule module;
        std::ostringstream output;
        output << "MODULE SprayScheme\n"
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
