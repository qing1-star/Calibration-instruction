#include <CalibrationInstructionTranslation/Calibration/WorkpieceCalibration.h>

#include <RotationBodyTrajectoryPlanning/Core/TransformUtils.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

namespace smrobot::spray::rotationbody
{
    namespace
    {
        constexpr double epsilon = 1.0e-12;
        constexpr int maximumCalibrationPointCount = 12;

        struct ProjectedCircleFit
        {
            bool valid{ false };
            Eigen::Vector3d axisPoint = Eigen::Vector3d::Zero();
            Eigen::Vector3d axisDirection = Eigen::Vector3d::UnitZ();
            double radius{ 0.0 };
            double squaredError{ std::numeric_limits<double>::infinity() };
            double rmsResidual{ std::numeric_limits<double>::infinity() };
            double maximumResidual{ std::numeric_limits<double>::infinity() };
            std::vector<double> residuals;
        };

        bool finitePointList(const CalibrationPointList& points)
        {
            return std::all_of(
                points.begin(),
                points.end(),
                [](const Eigen::Vector3d& point) { return point.allFinite(); });
        }

        Eigen::Vector3d centroidOf(const CalibrationPointList& points)
        {
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for(const Eigen::Vector3d& point : points) {
                center += point;
            }
            return center / static_cast<double>(points.size());
        }

        void makePerpendicularBasis(
            const Eigen::Vector3d& normal,
            Eigen::Vector3d& first,
            Eigen::Vector3d& second)
        {
            const Eigen::Vector3d normalized = normal.normalized();
            const Eigen::Vector3d helper = std::abs(normalized.z()) < 0.85
                ? Eigen::Vector3d::UnitZ()
                : Eigen::Vector3d::UnitX();
            first = helper.cross(normalized).normalized();
            second = normalized.cross(first).normalized();
        }

        ProjectedCircleFit evaluateCylinderDirection(
            const CalibrationPointList& points,
            const Eigen::Vector3d& direction)
        {
            ProjectedCircleFit fit;
            if(points.size() < 3 || !direction.allFinite() || direction.norm() < epsilon) {
                return fit;
            }

            fit.axisDirection = direction.normalized();
            Eigen::Vector3d first;
            Eigen::Vector3d second;
            makePerpendicularBasis(fit.axisDirection, first, second);

            const Eigen::Vector3d centroid = centroidOf(points);
            Eigen::Matrix<double, maximumCalibrationPointCount, 3> design =
                Eigen::Matrix<double, maximumCalibrationPointCount, 3>::Zero();
            Eigen::Matrix<double, maximumCalibrationPointCount, 1> rightHandSide =
                Eigen::Matrix<double, maximumCalibrationPointCount, 1>::Zero();
            for(std::size_t row = 0; row < points.size(); ++row) {
                const Eigen::Vector3d relative =
                    points[row] - centroid;
                const double x = relative.dot(first);
                const double y = relative.dot(second);
                design.row(static_cast<Eigen::Index>(row)) <<
                    2.0 * x, 2.0 * y, 1.0;
                rightHandSide(static_cast<Eigen::Index>(row)) = x * x + y * y;
            }

            Eigen::ColPivHouseholderQR<decltype(design)> decomposition(design);
            decomposition.setThreshold(1.0e-12);
            if(decomposition.rank() < 3) {
                return fit;
            }

            const Eigen::Vector3d solution = decomposition.solve(rightHandSide);
            const double centerFirst = solution.x();
            const double centerSecond = solution.y();
            const double radiusSquared = solution.z() +
                centerFirst * centerFirst + centerSecond * centerSecond;
            if(!std::isfinite(radiusSquared) || radiusSquared <= 0.0) {
                return fit;
            }

            fit.axisPoint = centroid + centerFirst * first + centerSecond * second;
            fit.radius = std::sqrt(radiusSquared);
            fit.residuals.resize(points.size());
            fit.squaredError = 0.0;
            fit.maximumResidual = 0.0;
            for(std::size_t index = 0; index < points.size(); ++index) {
                const Eigen::Vector3d fromAxis = points[index] - fit.axisPoint;
                const Eigen::Vector3d radial = fromAxis -
                    fromAxis.dot(fit.axisDirection) * fit.axisDirection;
                const double residual = radial.norm() - fit.radius;
                fit.residuals[index] = residual;
                fit.squaredError += residual * residual;
                fit.maximumResidual = std::max(
                    fit.maximumResidual,
                    std::abs(residual));
            }
            fit.rmsResidual = std::sqrt(
                fit.squaredError / static_cast<double>(points.size()));
            fit.valid = fit.axisPoint.allFinite() &&
                std::isfinite(fit.rmsResidual);
            return fit;
        }

        std::vector<Eigen::Vector3d> initialCylinderDirections(
            const CalibrationPointList& points)
        {
            std::vector<Eigen::Vector3d> candidates;
            candidates.reserve(900);
            candidates.push_back(Eigen::Vector3d::UnitX());
            candidates.push_back(Eigen::Vector3d::UnitY());
            candidates.push_back(Eigen::Vector3d::UnitZ());

            const Eigen::Vector3d center = centroidOf(points);
            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for(const Eigen::Vector3d& point : points) {
                const Eigen::Vector3d relative = point - center;
                covariance += relative * relative.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
            if(eigen.info() == Eigen::Success) {
                for(int column = 0; column < 3; ++column) {
                    Eigen::Vector3d direction = eigen.eigenvectors().col(column).normalized();
                    if(direction.z() < 0.0) direction = -direction;
                    candidates.push_back(direction);
                }
            }

            constexpr int fibonacciCount = 720;
            constexpr double goldenAngle = 2.39996322972865332223;
            for(int index = 0; index < fibonacciCount; ++index) {
                const double z = 1.0 - 2.0 *
                    (static_cast<double>(index) + 0.5) /
                    static_cast<double>(fibonacciCount);
                const double radial = std::sqrt(std::max(0.0, 1.0 - z * z));
                const double angle = goldenAngle * static_cast<double>(index);
                Eigen::Vector3d direction(
                    radial * std::cos(angle),
                    radial * std::sin(angle),
                    z);
                if(direction.z() < 0.0) direction = -direction;
                candidates.push_back(direction.normalized());
            }
            return candidates;
        }

        ProjectedCircleFit refineCylinderDirection(
            const CalibrationPointList& points,
            const ProjectedCircleFit& initial,
            int& iterations)
        {
            ProjectedCircleFit current = initial;
            double damping = 1.0e-3;
            iterations = 0;
            for(int iteration = 0; iteration < 80; ++iteration) {
                ++iterations;
                if(!current.valid || current.residuals.empty()) break;

                Eigen::Vector3d tangent0;
                Eigen::Vector3d tangent1;
                makePerpendicularBasis(current.axisDirection, tangent0, tangent1);
                const int rowCount = static_cast<int>(current.residuals.size());
                Eigen::VectorXd residual(rowCount);
                for(int row = 0; row < rowCount; ++row) {
                    residual(row) = current.residuals[static_cast<std::size_t>(row)];
                }

                Eigen::MatrixXd jacobian(rowCount, 2);
                constexpr double finiteDifferenceStep = 1.0e-5;
                for(int column = 0; column < 2; ++column) {
                    const Eigen::Vector3d tangent = column == 0 ? tangent0 : tangent1;
                    const Eigen::Vector3d probeDirection =
                        (current.axisDirection + finiteDifferenceStep * tangent).normalized();
                    const ProjectedCircleFit probe =
                        evaluateCylinderDirection(points, probeDirection);
                    if(!probe.valid) {
                        jacobian.col(column).setZero();
                        continue;
                    }
                    for(int row = 0; row < rowCount; ++row) {
                        jacobian(row, column) =
                            (probe.residuals[static_cast<std::size_t>(row)] - residual(row)) /
                            finiteDifferenceStep;
                    }
                }

                const Eigen::Matrix2d normal = jacobian.transpose() * jacobian +
                    damping * Eigen::Matrix2d::Identity();
                const Eigen::Vector2d gradient = jacobian.transpose() * residual;
                Eigen::Vector2d step = -normal.ldlt().solve(gradient);
                if(!step.allFinite()) break;
                if(step.norm() > 0.35) step *= 0.35 / step.norm();

                const Eigen::Vector3d candidateDirection =
                    (current.axisDirection + step.x() * tangent0 +
                        step.y() * tangent1).normalized();
                ProjectedCircleFit candidate =
                    evaluateCylinderDirection(points, candidateDirection);
                if(candidate.valid && candidate.squaredError < current.squaredError) {
                    current = std::move(candidate);
                    damping = std::max(1.0e-9, damping * 0.35);
                    if(step.norm() < 1.0e-8) break;
                } else {
                    damping = std::min(1.0e9, damping * 5.0);
                }
            }
            return current;
        }
    }

    PlanningResult<CalibrationAxisFit> WorkpieceCalibrationSolver::fitCylinder3d(
        const CalibrationPointList& pointsBaseMeters)
    {
        if(pointsBaseMeters.size() < 8 || pointsBaseMeters.size() > 12) {
            return PlanningResult<CalibrationAxisFit>::failure(
                PlanningErrorCode::InvalidArgument,
                "Cylinder calibration requires 8 to 12 touch points.");
        }
        if(!finitePointList(pointsBaseMeters)) {
            return PlanningResult<CalibrationAxisFit>::failure(
                PlanningErrorCode::NonFiniteGeometry,
                "Cylinder calibration points must be finite.");
        }

        ProjectedCircleFit best;
        for(const Eigen::Vector3d& direction :
            initialCylinderDirections(pointsBaseMeters)) {
            ProjectedCircleFit candidate =
                evaluateCylinderDirection(pointsBaseMeters, direction);
            if(candidate.valid && candidate.squaredError < best.squaredError) {
                best = std::move(candidate);
            }
        }
        if(!best.valid) {
            return PlanningResult<CalibrationAxisFit>::failure(
                PlanningErrorCode::SingularFit,
                "Cylinder calibration points are degenerate.");
        }

        int iterations = 0;
        best = refineCylinderDirection(pointsBaseMeters, best, iterations);
        if(!best.valid) {
            return PlanningResult<CalibrationAxisFit>::failure(
                PlanningErrorCode::SingularFit,
                "Cylinder calibration did not converge.");
        }
        if(best.axisDirection.z() < 0.0) best.axisDirection = -best.axisDirection;

        CalibrationAxisFit result;
        result.mode = CalibrationMode::Cylinder3d;
        result.axisPointBaseMeters = best.axisPoint;
        result.axisDirectionBase = best.axisDirection.normalized();
        result.radiusMeters = best.radius;
        result.rmsResidualMeters = best.rmsResidual;
        result.maximumResidualMeters = best.maximumResidual;
        result.iterations = iterations;
        result.residualsMeters = std::move(best.residuals);
        result.sourcePointsBaseMeters = pointsBaseMeters;

        double minimumAxisPosition = std::numeric_limits<double>::infinity();
        double maximumAxisPosition = -std::numeric_limits<double>::infinity();
        for(const Eigen::Vector3d& point : pointsBaseMeters) {
            const double position =
                (point - result.axisPointBaseMeters).dot(result.axisDirectionBase);
            minimumAxisPosition = std::min(minimumAxisPosition, position);
            maximumAxisPosition = std::max(maximumAxisPosition, position);
        }
        result.lowAxialSpan = maximumAxisPosition - minimumAxisPosition <
            std::max(0.001, 0.05 * result.radiusMeters);
        PlanningResult<CalibrationAxisFit> output =
            PlanningResult<CalibrationAxisFit>::success(std::move(result));
        if(output.value.lowAxialSpan) {
            output.diagnostics.push_back(
                "Touch points have a short axial span; use multiple heights for stability.");
        }
        return output;
    }

    PlanningResult<CalibrationAxisFit> WorkpieceCalibrationSolver::fitCircle2d(
        const CalibrationPointList& pointsBaseMeters,
        double axialPlaneToleranceMeters)
    {
        if(pointsBaseMeters.size() != 6) {
            return PlanningResult<CalibrationAxisFit>::failure(
                PlanningErrorCode::InvalidArgument,
                "Circle calibration requires exactly 6 touch points.");
        }
        if(!finitePointList(pointsBaseMeters) ||
            !std::isfinite(axialPlaneToleranceMeters) ||
            axialPlaneToleranceMeters < 0.0) {
            return PlanningResult<CalibrationAxisFit>::failure(
                PlanningErrorCode::InvalidArgument,
                "Circle calibration inputs are invalid.");
        }

        double minimumZ = std::numeric_limits<double>::infinity();
        double maximumZ = -std::numeric_limits<double>::infinity();
        double zSum = 0.0;
        for(const Eigen::Vector3d& point : pointsBaseMeters) {
            minimumZ = std::min(minimumZ, point.z());
            maximumZ = std::max(maximumZ, point.z());
            zSum += point.z();
        }
        if(maximumZ - minimumZ > axialPlaneToleranceMeters) {
            return PlanningResult<CalibrationAxisFit>::failure(
                PlanningErrorCode::InvalidArgument,
                "Circle calibration points exceed the axial plane tolerance.");
        }

        const int rowCount = static_cast<int>(pointsBaseMeters.size());
        Eigen::MatrixXd matrix(rowCount, 3);
        Eigen::VectorXd rightHandSide(rowCount);
        for(int row = 0; row < rowCount; ++row) {
            const Eigen::Vector3d& point =
                pointsBaseMeters[static_cast<std::size_t>(row)];
            matrix(row, 0) = 2.0 * point.x();
            matrix(row, 1) = 2.0 * point.y();
            matrix(row, 2) = 1.0;
            rightHandSide(row) = point.x() * point.x() + point.y() * point.y();
        }
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> decomposition(matrix);
        decomposition.setThreshold(1.0e-9);
        if(decomposition.rank() < 3) {
            return PlanningResult<CalibrationAxisFit>::failure(
                PlanningErrorCode::SingularFit,
                "Circle calibration points are nearly collinear.");
        }

        const Eigen::Vector3d solution = decomposition.solve(rightHandSide);
        const double radiusSquared = solution.z() +
            solution.x() * solution.x() + solution.y() * solution.y();
        if(!std::isfinite(radiusSquared) || radiusSquared <= 0.0) {
            return PlanningResult<CalibrationAxisFit>::failure(
                PlanningErrorCode::SingularFit,
                "Circle calibration radius is invalid.");
        }

        CalibrationAxisFit result;
        result.mode = CalibrationMode::Circle2d;
        result.axisPointBaseMeters = Eigen::Vector3d(
            solution.x(),
            solution.y(),
            zSum / static_cast<double>(pointsBaseMeters.size()));
        result.axisDirectionBase = Eigen::Vector3d::UnitZ();
        result.radiusMeters = std::sqrt(radiusSquared);
        result.sourcePointsBaseMeters = pointsBaseMeters;
        result.residualsMeters.resize(pointsBaseMeters.size());
        double squaredError = 0.0;
        for(std::size_t index = 0; index < pointsBaseMeters.size(); ++index) {
            const double deltaX = pointsBaseMeters[index].x() - solution.x();
            const double deltaY = pointsBaseMeters[index].y() - solution.y();
            const double residual = std::hypot(deltaX, deltaY) - result.radiusMeters;
            result.residualsMeters[index] = residual;
            squaredError += residual * residual;
            result.maximumResidualMeters = std::max(
                result.maximumResidualMeters,
                std::abs(residual));
        }
        result.rmsResidualMeters = std::sqrt(
            squaredError / static_cast<double>(pointsBaseMeters.size()));
        return PlanningResult<CalibrationAxisFit>::success(std::move(result));
    }

    PlanningResult<WorkpieceFrameCalibration>
    WorkpieceCalibrationSolver::computeWorkpieceFrame(
        const CalibrationAxisFit& axisFit,
        const Eigen::Vector3d& topReferenceBaseMeters,
        double workpieceHeightMeters,
        const Eigen::Vector3d& yDirectionStartBaseMeters,
        const Eigen::Vector3d& yDirectionEndBaseMeters)
    {
        if(!axisFit.axisPointBaseMeters.allFinite() ||
            !axisFit.axisDirectionBase.allFinite() ||
            !topReferenceBaseMeters.allFinite() ||
            !yDirectionStartBaseMeters.allFinite() ||
            !yDirectionEndBaseMeters.allFinite() ||
            !std::isfinite(workpieceHeightMeters) ||
            workpieceHeightMeters <= 0.0 ||
            axisFit.axisDirectionBase.norm() < epsilon) {
            return PlanningResult<WorkpieceFrameCalibration>::failure(
                PlanningErrorCode::InvalidArgument,
                "Workpiece frame calibration inputs are invalid.");
        }

        Eigen::Vector3d zAxis = axisFit.axisDirectionBase.normalized();
        if((topReferenceBaseMeters - axisFit.axisPointBaseMeters).dot(zAxis) < 0.0) {
            zAxis = -zAxis;
        }
        const Eigen::Vector3d topCenter = axisFit.axisPointBaseMeters +
            (topReferenceBaseMeters - axisFit.axisPointBaseMeters).dot(zAxis) * zAxis;
        const Eigen::Vector3d baseOrigin = topCenter - workpieceHeightMeters * zAxis;

        const Eigen::Vector3d measuredY =
            yDirectionEndBaseMeters - yDirectionStartBaseMeters;
        const double measuredYDistance = measuredY.norm();
        if(measuredYDistance < 0.001) {
            return PlanningResult<WorkpieceFrameCalibration>::failure(
                PlanningErrorCode::InvalidArgument,
                "The measured Y-direction points must be at least 1 mm apart.");
        }
        Eigen::Vector3d yAxis = measuredY - measuredY.dot(zAxis) * zAxis;
        if(yAxis.norm() < 0.001) {
            return PlanningResult<WorkpieceFrameCalibration>::failure(
                PlanningErrorCode::InvalidAxisSelection,
                "The measured Y direction is too close to the rotary axis.");
        }
        yAxis.normalize();
        Eigen::Vector3d xAxis = yAxis.cross(zAxis);
        if(xAxis.norm() < epsilon) {
            return PlanningResult<WorkpieceFrameCalibration>::failure(
                PlanningErrorCode::InvalidAxisSelection,
                "The workpiece X direction could not be constructed.");
        }
        xAxis.normalize();
        yAxis = zAxis.cross(xAxis).normalized();

        WorkpieceFrameCalibration result;
        result.baseFromPlanning.linear().col(0) = xAxis;
        result.baseFromPlanning.linear().col(1) = yAxis;
        result.baseFromPlanning.linear().col(2) = zAxis;
        result.baseFromPlanning.translation() = baseOrigin;
        const PlanningResult<TransformComponents> components =
            transformComponents(result.baseFromPlanning);
        if(!components) {
            return PlanningResult<WorkpieceFrameCalibration>::failure(
                components.error.code,
                components.error.message);
        }

        Eigen::Quaterniond quaternion(result.baseFromPlanning.linear());
        quaternion.normalize();
        if(quaternion.w() < 0.0) quaternion.coeffs() *= -1.0;
        result.baseFromPlanningComponents = components.value;
        result.topReferenceBaseMeters = topReferenceBaseMeters;
        result.topCenterBaseMeters = topCenter;
        result.baseOriginBaseMeters = baseOrigin;
        result.yDirectionStartBaseMeters = yDirectionStartBaseMeters;
        result.yDirectionEndBaseMeters = yDirectionEndBaseMeters;
        result.xAxisBase = xAxis;
        result.yAxisBase = yAxis;
        result.zAxisBase = zAxis;
        result.abbQuaternionWxyz = Eigen::Vector4d(
            quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
        result.workpieceHeightMeters = workpieceHeightMeters;
        result.fittedRadiusMeters = axisFit.radiusMeters;
        result.yDirectionDistanceMeters = measuredYDistance;
        result.shortYDirectionBaseline = measuredYDistance < 0.05;

        PlanningResult<WorkpieceFrameCalibration> output =
            PlanningResult<WorkpieceFrameCalibration>::success(std::move(result));
        if(output.value.shortYDirectionBaseline) {
            output.diagnostics.push_back(
                "The measured Y-direction baseline is shorter than 50 mm.");
        }
        return output;
    }
}
