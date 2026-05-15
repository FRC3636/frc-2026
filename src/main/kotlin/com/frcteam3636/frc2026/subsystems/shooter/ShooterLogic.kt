package com.frcteam3636.frc2026.subsystems.shooter

import com.frcteam3636.frc2026.robot.Robot
import com.frcteam3636.frc2026.robot.Robot.Model
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.feeder.Feeder
import com.frcteam3636.frc2026.subsystems.indexer.Indexer
import com.frcteam3636.frc2026.subsystems.shooter.flywheel.Flywheel
import com.frcteam3636.frc2026.subsystems.shooter.hood.Hood
import com.frcteam3636.frc2026.subsystems.shooter.turret.Constants.SHOOTER_OFFSET
import com.frcteam3636.frc2026.subsystems.shooter.turret.Turret
import com.frcteam3636.frc2026.utils.autos.FIELD_WIDTH_METERS
import com.frcteam3636.frc2026.utils.math.*
import com.frcteam3636.frc2026.utils.shooting.hubTranslation
import com.frcteam3636.frc2026.utils.shooting.targetPassTranslation
import com.frcteam3636.frc2026.utils.shooting.translatePose
import edu.wpi.first.math.geometry.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import kotlin.jvm.optionals.getOrNull
import kotlin.math.IEEErem
import kotlin.math.PI

// Heavy inspiration taken from https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java

object ShooterCalculator {
    fun getProfile(target: Translation2d, robotPose: Pose2d = Drivetrain.estimatedPose): ShooterProfile {
        val turretPose = robotPose.transformBy(robotToTurret)
        val distance = target.getDistance(turretPose.translation)

        val hoodAngle = Hood.calculateHoodAngle(distance.meters)
        val flywheelVelocity = Flywheel.calculateFlywheelVelocity(distance.meters)

        val turretAngleFieldRelative = (target - turretPose.translation).angle   // angle from pose to target
        val turretAngleRobotRelative = turretAngleFieldRelative.minus(turretPose.rotation) // convert to zero-forward
        val normalizedTurretAngleRobotRelative = turretAngleRobotRelative.getRadians().IEEErem(TAU).radians

        return ShooterProfile(
            normalizedTurretAngleRobotRelative,
            hoodAngle,
            flywheelVelocity
        )
    }

    fun getProfileSOTM(target: Translation2d): ShooterProfile {

        var lookaheadPose = Drivetrain.estimatedPose
        var lookaheadTurretPosition = lookaheadPose.transformBy(robotToTurret)
        var lookaheadDistance = target.getDistance(lookaheadTurretPosition.translation)
        var timeOfFlight = timeOfFlight(lookaheadDistance)

        for (i in 0..14) {
            // Assuming constant velocity and constant acceleration

            lookaheadPose = translatePose(
                Drivetrain.estimatedPose,
                Drivetrain.measuredChassisSpeeds.vxMetersPerSecond,
                Drivetrain.measuredChassisSpeeds.vyMetersPerSecond,
                Drivetrain.measuredChassisSpeeds.omegaRadiansPerSecond * (PI / 180), // Made this the degrees to radians factor temporarily to test if the * 0.1 was not random
                0.0, // Ansel, please commit your changes so I can test this
                0.0,
                0.0,
                timeOfFlight
            )

            lookaheadTurretPosition = lookaheadPose.transformBy(robotToTurret)
            lookaheadDistance = target.getDistance(lookaheadTurretPosition.translation)
            timeOfFlight = timeOfFlight(lookaheadDistance)
        }

        Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose)
        Logger.recordOutput("LaunchCalculator/LookaheadTurretPosition", lookaheadTurretPosition)
        Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadDistance)
        Logger.recordOutput("LaunchCalculator/TimeOfFlight", timeOfFlight)

        return getProfile(target, lookaheadPose)
    }

    fun getProfileWithPassing(): ShooterProfile {
        return getProfile(targetPassTranslation)
    }

    fun getProfileSOTMWithPassing(): ShooterProfile {
        return getProfileSOTM(targetPassTranslation)
    }

}

// TODO() Tune ts
private fun timeOfFlight(distance: Double): Double = 0.043856 * distance + 0.930047

fun shoot() : Command =
    Commands.sequence(
        Flywheel.runAtTarget().until(Flywheel.atDesiredFlywheelVelocity),
        Commands.parallel(
            Flywheel.runAtTarget(),
                Feeder.feed(),
                Indexer.index()
        )
    )

var shooterTarget: Target = Target.STATIONARY_TURRET
var shooterProfile: ShooterProfile = shooterTarget.profile()

val robotToTurret = Transform2d(SHOOTER_OFFSET, Rotation2d.kZero)

val shooterFieldPose: Pose2d
    get() = Pose2d(
        Drivetrain.estimatedPose.translation + SHOOTER_OFFSET.rotateBy(Drivetrain.estimatedPose.rotation),
        Drivetrain.estimatedPose.rotation + Turret.turretAngle
    )

val shooterToHub: Vector2d
    get() = toVector2d(hubTranslation.toTranslation2d()) - toVector2d(shooterFieldPose.translation)

fun setShooterTarget(target: Target): Command =
    Commands.runOnce ({
        shooterTarget = target
        shooterProfile = target.profile()
        Logger.recordOutput("Shooter/Target", target.toString())
        Logger.recordOutput("Shooter/Profile/Turret (deg)", shooterProfile.turretAngle.inDegrees())
        Logger.recordOutput("Shooter/Profile/Hood (deg)", shooterProfile.hoodAngle.inDegrees())
        Logger.recordOutput("Shooter/Profile/Flywheel (RPM)", shooterProfile.angularVelocity.inRPM())
    })

// used for tuning regressions
val hoodTunable = LoggedNetworkNumber("/Tuning/HoodTestAngle", 5.0)
val flywheelTunable = LoggedNetworkNumber("/Tuning/FlywheelSpeed", 1500.0)
val turretTunable = LoggedNetworkNumber("/Tuning/TurretAngle", 0.0)

data class ShooterProfile(
    val turretAngle: Angle,
    val hoodAngle: Angle,
    val angularVelocity: AngularVelocity,
)

enum class Target(val profile: () -> ShooterProfile) {
    AIM_AT_HUB_NO_SOTM (
        { ShooterCalculator.getProfile(hubTranslation.toTranslation2d()) }
    ),
    PASS_NO_SOTM (
        { ShooterCalculator.getProfileWithPassing() }
    ),
    AIM_AT_HUB (
        { ShooterCalculator.getProfileSOTM(hubTranslation.toTranslation2d()) }
    ),
    PASS (
        { ShooterCalculator.getProfileSOTMWithPassing() }
    ),
    STATIONARY_TURRET (
        { ShooterProfile(0.0.degrees, 5.degrees, 2000.rpm) },
    ),
    TUNING (
        { ShooterProfile(turretTunable.get().degrees, hoodTunable.get().degrees, flywheelTunable.get().rpm) }
    ),
}
