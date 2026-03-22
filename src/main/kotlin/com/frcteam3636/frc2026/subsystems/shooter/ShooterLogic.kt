package com.frcteam3636.frc2026.subsystems.shooter

import com.frcteam3636.frc2026.robot.Robot
import com.frcteam3636.frc2026.robot.Robot.Model
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.shooter.flywheel.Constants.FLYWHEEL_RADIUS
import com.frcteam3636.frc2026.subsystems.shooter.flywheel.Constants.FLYWHEEL_TO_FUEL_RATIO
import com.frcteam3636.frc2026.subsystems.shooter.flywheel.Flywheel
import com.frcteam3636.frc2026.subsystems.shooter.hood.Hood
import com.frcteam3636.frc2026.subsystems.shooter.turret.Constants.SHOOTER_OFFSET
import com.frcteam3636.frc2026.subsystems.shooter.turret.Turret
import com.frcteam3636.frc2026.utils.math.*
import com.frcteam3636.frc2026.utils.swerve.translation2dPerSecond
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import kotlin.math.*


object ShooterCalculator {

    private val stationaryLaunchVector: Vector3d
        get() {
            val distance: Distance = shooterToHub.norm.meters
            val baseFlywheelRpm: Double = Flywheel.calculateFlywheelVelocity(distance).inRPM()
            val baseHoodAngle: Double = Hood.calculateHoodAngle(distance).inRadians()

            val launchSpeedRobotRelative = baseFlywheelRpm * (2.0 * PI * FLYWHEEL_RADIUS.inMeters()) / 60.0 * FLYWHEEL_TO_FUEL_RATIO

            val horizontalSpeedComponent = launchSpeedRobotRelative * cos(baseHoodAngle)
            val verticalSpeedComponent = launchSpeedRobotRelative * sin(baseHoodAngle)

            return Vector3d(
                horizontalSpeedComponent * cos(directionToHub.inRadians()),
                horizontalSpeedComponent * sin(directionToHub.inRadians()),
                verticalSpeedComponent
            )
        }

    private val fieldRelativeLaunchVector: Vector3d
        get() {
            val stationaryVector = stationaryLaunchVector
            val robotVelocity = Drivetrain.measuredChassisSpeedsRelativeToField
            return Vector3d(
                stationaryVector.x - (robotVelocity.vxMetersPerSecond),
                stationaryVector.y - (robotVelocity.vyMetersPerSecond),
                stationaryVector.z    // robot vertical velocity should always be zero
            )
        }

    fun aimAtHub(): ShooterProfile {
        val distance: Distance = shooterToHub.norm.meters
        val baseFlywheelRpm = Flywheel.calculateFlywheelVelocity(distance).inRPM()
        val baseHoodAngle = Hood.calculateHoodAngle(distance).inRadians()

        val launchSpeed = baseFlywheelRpm * (2.0 * PI * FLYWHEEL_RADIUS.inMeters()) / 60.0 * FLYWHEEL_TO_FUEL_RATIO
        val horizontalSpeed = launchSpeed * cos(baseHoodAngle)
        val verticalSpeed = launchSpeed * sin(baseHoodAngle)

        val fieldDirection = atan2(shooterToHub.y, shooterToHub.x).radians
        val turretAngle = (fieldDirection.inRadians() - Drivetrain.estimatedPose.rotation.radians)
            .IEEErem(2 * PI).radians

        return ShooterProfile(turretAngle, baseHoodAngle.radians, baseFlywheelRpm.rpm)
    }

    fun aimAtHubShootOnMove(): ShooterProfile {
        val hubPos = hubTranslation
        val shooterPos = Translation3d (
            shooterFieldPose.translation.x.meters,
            shooterFieldPose.translation.y.meters,
            SHOOTER_HEIGHT
        )

        // Robot velocity in field coordinates (m/s)
        val robotVel = Drivetrain.measuredChassisSpeedsRelativeToField
        val robotVelVector = Vector3d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond, 0.0)

        // Horizontal distance from shooter to hub
        val dx = hubPos.x - shooterPos.x
        val dy = hubPos.y - shooterPos.y
        val dz = hubPos.z - shooterPos.z
        val shooterToHubDistance = hypot(dx, dy).meters

        // Time of flight is an estimate
        val stationaryDistance = shooterToHubDistance
        val stationaryFlywheelRpm = Flywheel.calculateFlywheelVelocity(stationaryDistance).inRPM()
        val stationaryHoodAngle = Hood.calculateHoodAngle(stationaryDistance).inRadians()
        val stationaryLaunchSpeed = stationaryFlywheelRpm * (2.0 * PI * FLYWHEEL_RADIUS.inMeters()) / 60.0 * FLYWHEEL_TO_FUEL_RATIO
        val stationaryHorizontalBallSpeed = stationaryLaunchSpeed * cos(stationaryHoodAngle)
        val timeOfFlight = if (stationaryHorizontalBallSpeed > 0) shooterToHubDistance.inMeters() / stationaryHorizontalBallSpeed else 1.0

        val ballVelocityFieldRelative = Vector3d(dx / timeOfFlight, dy / timeOfFlight, dz / timeOfFlight)

        val launchVelocityRobotRelative = ballVelocityFieldRelative.minus(robotVelVector)

        // Compute turret angle (relative to robot forward)
        val turretAngle = atan2(launchVelocityRobotRelative.y, launchVelocityRobotRelative.x).radians

        // Compute required launch speed and hood angle
        val launchSpeed = hypot(launchVelocityRobotRelative.x, launchVelocityRobotRelative.y, launchVelocityRobotRelative.z)
        val horizontalSpeed = hypot(launchVelocityRobotRelative.x, launchVelocityRobotRelative.y)
        val hoodAngle = atan2(launchVelocityRobotRelative.z, horizontalSpeed).radians

        // Convert launch speed to flywheel RPM
        val requiredRPM = launchSpeed / (2.0 * PI * FLYWHEEL_RADIUS.inMeters() * FLYWHEEL_TO_FUEL_RATIO / 60.0)

        return ShooterProfile(turretAngle, hoodAngle, requiredRPM.rpm)
    }

}

var shooterTarget: Target = Target.STATIONARY_TURRET
var shooterProfile: ShooterProfile = shooterTarget.profile()

fun setShooterTarget(target: Target): Command =
    Commands.runOnce ({
        shooterTarget = target
        shooterProfile = target.profile()
        Logger.recordOutput("Shooter/Target", target.toString())
        Logger.recordOutput("Shooter/Profile/Turret (deg)", shooterProfile.turretAngle.inDegrees())
        Logger.recordOutput("Shooter/Profile/Hood (deg)", shooterProfile.hoodAngle.inDegrees())
        Logger.recordOutput("Shooter/Profile/Flywheel (RPM)", shooterProfile.angularVelocity.inRPM())
    })

data class ShooterProfile(
    val turretAngle: Angle,
    val hoodAngle: Angle,
    val angularVelocity: AngularVelocity,
)

val hubTranslation
    get() = when (Robot.model) {
        // simulation defaults to red alliance
        Model.SIMULATION -> Translation3d(
            4.62534.meters,
            (8.07 / 2).meters,
            1.83.meters,
        )
        Model.COMPETITION -> when (DriverStation.getAlliance().orElse(Alliance.Blue)) {
            Alliance.Blue -> Translation3d(
                4.62534.meters,
                (8.07 / 2).meters,
                1.83.meters,
            )
            Alliance.Red -> Translation3d(
                (16.54 - 4.62534).meters,
                (8.07 / 2).meters,
                1.83.meters,
            )
        }
    }

private val SHOOTER_HEIGHT = 0.4318.meters
private val GRAVITY = 9.81.metersPerSecondPerSecond

val shooterFieldPose: Pose2d
    get() = Pose2d(
        Drivetrain.estimatedPose.translation + SHOOTER_OFFSET.rotateBy(Drivetrain.estimatedPose.rotation),
        Drivetrain.estimatedPose.rotation + Turret.turretAngle
    )

val shooterToHub: Vector2d
    get() = toVector2d(hubTranslation.toTranslation2d()) - toVector2d(shooterFieldPose.translation)

val directionToHub: Angle
    get() = atan2(shooterToHub.y, shooterToHub.x).radians

// used for populating interpolation tables
val hoodTunable = LoggedNetworkNumber("/Tuning/HoodTestAngle", 40.0)
val flywheelTunable = LoggedNetworkNumber("/Tuning/FlywheelSpeed", 3000.0)
val turretTunable = LoggedNetworkNumber("/Tuning/TurretAngle", 0.0)

enum class Target(val profile: () -> ShooterProfile) {
    AIM_AT_HUB (
        { ShooterCalculator.aimAtHub() }
    ),
    AIM_AT_HUB_SHOOT_ON_MOVE (
        { ShooterCalculator.aimAtHubShootOnMove() }
    ),
    STATIONARY_TURRET (
        { ShooterProfile(0.0.degrees, Hood.calculateHoodAngle(shooterToHub.norm.meters), Flywheel.calculateFlywheelVelocity(shooterToHub.norm.meters)) }
    ),
    TUNING (
        { ShooterProfile(turretTunable.get().degrees, hoodTunable.get().degrees, flywheelTunable.get().rpm) }
    )
}

enum class FeedTranslation(val target : Translation2d) {
    LeftSideNeutralZone(Translation2d(7.0.meters,7.5.meters)),
    RightSideNeutralZone(Translation2d(7.0.meters,1.50.meters)),
    LeftSideAllianceZone(Translation2d(15.0.meters,3.0.meters)),
    RightSideAllianceZone(Translation2d(15.0.meters,7.5.meters)),
}

enum class Zones(val startY : Distance, val endY : Distance){
    BlueAllianceZone(0.meters, 4.03.meters),
    RedAllianceZone(4.03.meters, 11.22.meters),
    NeutralZone(11.22.meters, 15.23.meters),
}
