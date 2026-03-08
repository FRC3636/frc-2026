package com.frcteam3636.frc2026.shooter

import com.frcteam3636.frc2026.robot.Robot
import com.frcteam3636.frc2026.robot.Robot.Model
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.flywheel.Constants.FLYWHEEL_RADIUS
import com.frcteam3636.frc2026.subsystems.flywheel.Constants.FLYWHEEL_TO_FUEL_RATIO
import com.frcteam3636.frc2026.subsystems.flywheel.Flywheel
import com.frcteam3636.frc2026.subsystems.hood.Hood
import com.frcteam3636.frc2026.subsystems.turret.Constants.SHOOTER_OFFSET
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

    private fun stationaryLaunchVector(): Vector3d {
        val distance: Distance = shooterToHub.norm.meters
        val baseFlywheelRpm: Double = Flywheel.calculateFlywheelVelocity(distance).inRPM()
        val baseHoodAngle: Double = Hood.calculateHoodAngle(distance).inDegrees()

        val launchSpeedRobotRelative = (
            baseFlywheelRpm * (2.0 * PI / 60.0) * FLYWHEEL_RADIUS.inMeters() * FLYWHEEL_TO_FUEL_RATIO
        )

        val directionToHub = shooterToHub.angle

        val horizontalSpeedComponent = launchSpeedRobotRelative * cos(baseHoodAngle)
        val verticalSpeedComponent = launchSpeedRobotRelative * sin(baseHoodAngle)

        return Vector3d(
            horizontalSpeedComponent * cos(directionToHub.radians),
            horizontalSpeedComponent * sin(directionToHub.radians),
            verticalSpeedComponent
        )
    }

    private fun robotRelativeLaunchVector(): Vector3d {
        val stationaryVector = stationaryLaunchVector()
        val robotVelocity = Drivetrain.measuredChassisSpeedsRelativeToField.translation2dPerSecond
        return Vector3d(
            stationaryVector.x - robotVelocity.x,
            stationaryVector.y - robotVelocity.y,
            stationaryVector.z    // robot vertical velocity should always be zero
        )
    }

    fun aimAtHub(compensateForMotion: Boolean): ShooterProfile {
        val vector = if (compensateForMotion) robotRelativeLaunchVector() else stationaryLaunchVector()

        val turretAngleRobotRelative = atan2(vector.y, vector.x).radians

        val horizontalMagnitude = hypot(vector.x, vector.y)
        val hoodAngle = atan2(vector.z, horizontalMagnitude).radians

        val requiredSpeed = vector.norm

        val requiredSpeedRPM = (
            requiredSpeed / (2.0 * PI / 60.0 * FLYWHEEL_RADIUS.inMeters() * FLYWHEEL_TO_FUEL_RATIO)
        ).rpm

        return ShooterProfile(turretAngleRobotRelative, hoodAngle, requiredSpeedRPM)
    }

}

var shooterTarget: Target = Target.STOWED
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

val shooterFieldPose: Pose2d
    get() = Pose2d(
        Drivetrain.estimatedPose.translation + SHOOTER_OFFSET.rotateBy(Drivetrain.estimatedPose.rotation),
        Drivetrain.estimatedPose.rotation
    )

val shooterToHub: Translation2d
    get() = hubTranslation.toTranslation2d() - shooterFieldPose.translation

// used for populating interpolation tables
val hoodTunable = LoggedNetworkNumber("/Tuning/HoodTestAngle", 20.0)
val flywheelTunable = LoggedNetworkNumber("/Tuning/FlywheelSpeed", 1000.0)

enum class Target(val profile: () -> ShooterProfile) {
    AIM_AT_HUB (
        { ShooterCalculator.aimAtHub(compensateForMotion = false) }
    ),
    AIM_AT_HUB_SHOOT_ON_MOVE (
        { ShooterCalculator.aimAtHub(compensateForMotion = true) }
    ),
    STOWED (
        { ShooterProfile(0.0.radians, 0.0.degrees, 0.0.rpm) }
    ),
    TUNING (
        { ShooterProfile(0.0.radians, hoodTunable.get().degrees, flywheelTunable.get().rpm) }
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
