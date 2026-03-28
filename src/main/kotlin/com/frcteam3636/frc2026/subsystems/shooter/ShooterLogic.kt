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
import com.frcteam3636.frc2026.utils.autos.FIELD_HEIGHT_METERS
import com.frcteam3636.frc2026.utils.autos.FIELD_WIDTH_METERS
import com.frcteam3636.frc2026.utils.math.*
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
import kotlin.math.cos
import kotlin.math.sin

// Heavy inspiration taken from https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java

object ShooterCalculator {
    fun getProfileNoShootOnMove(target: Translation2d): ShooterProfile {
        val estimatedPose = Drivetrain.estimatedPose

        val distance = target.getDistance(estimatedPose.translation)
        val hoodAngle = Hood.calculateHoodAngle(distance.meters)
        val flywheelVelocity = Flywheel.calculateFlywheelVelocity(distance.meters)

        val turretAngleFieldRelative = (target - estimatedPose.translation).angle   // angle from lookahead pose to target
        val turretAngleRobotRelative = turretAngleFieldRelative.minus(estimatedPose.rotation) // convert to zero-forward
        val normalizedTurretAngleRobotRelative = turretAngleRobotRelative.radians.IEEErem(TAU).radians // Rotation2d.radians is cooked

        return ShooterProfile(
            normalizedTurretAngleRobotRelative,
            hoodAngle,
            flywheelVelocity
        )
    }

    fun getProfile(target: Translation2d): ShooterProfile {

        val estimatedPose = Drivetrain.estimatedPose
        val robotRelativeVelocity = Drivetrain.measuredChassisSpeeds
        val fieldRelativeVelocity = Drivetrain.measuredChassisSpeedsRelativeToField

        val delayedPose = estimatedPose.exp(
            Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * PHASE_DELAY.inSeconds(),
                robotRelativeVelocity.vyMetersPerSecond * PHASE_DELAY.inSeconds(),
                robotRelativeVelocity.omegaRadiansPerSecond * PHASE_DELAY.inSeconds()
            )
        )

        val turretPosition = delayedPose.transformBy(robotToTurret)

        var lookaheadDistance = target.getDistance(turretPosition.translation)
        var lookaheadPose = turretPosition

        val robotAngle = estimatedPose.rotation.radians
        val turretOffset = SHOOTER_OFFSET

        // I'm too tired to understand the math rn, the commented code is possibly correct
        val turretVelX = fieldRelativeVelocity.vxMetersPerSecond +
                fieldRelativeVelocity.omegaRadiansPerSecond *
                (turretOffset.y * cos(robotAngle) - turretOffset.x * sin(robotAngle))

        val turretVelY = fieldRelativeVelocity.vyMetersPerSecond +
                fieldRelativeVelocity.omegaRadiansPerSecond *
                (turretOffset.x * cos(robotAngle) - turretOffset.y * sin(robotAngle))

//        val turretVelX = fieldRelativeVelocity.vxMetersPerSecond -
//                fieldRelativeVelocity.omegaRadiansPerSecond *
//                (turretOffset.x * sin(robotAngle) + turretOffset.y * cos(robotAngle))
//
//        val turretVelY = fieldRelativeVelocity.vyMetersPerSecond +
//                fieldRelativeVelocity.omegaRadiansPerSecond *
//                (turretOffset.x * cos(robotAngle) - turretOffset.y * sin(robotAngle))

        // Iteratively account for imparted velocity by robot (turret) to offset
        for (i in 0..20) {
            val tof = timeOfFlight(lookaheadDistance)
            val offsetX = turretVelX * tof
            val offsetY = turretVelY * tof
            lookaheadPose = Pose2d(
                turretPosition.translation + Translation2d(offsetX, offsetY),
                turretPosition.rotation
            )
            lookaheadDistance = target.getDistance(lookaheadPose.translation)
        }

        val hoodAngle = Hood.calculateHoodAngle(lookaheadDistance.meters)
        val flywheelVelocity = Flywheel.calculateFlywheelVelocity(lookaheadDistance.meters)

        val turretAngleFieldRelative = (target - lookaheadPose.translation).angle   // angle from lookahead pose to target
        val turretAngleRobotRelative = turretAngleFieldRelative.minus(delayedPose.rotation) // convert to zero-forward
        val normalizedTurretAngleRobotRelative = turretAngleRobotRelative.radians.IEEErem(TAU).radians // Rotation2d.radians is cooked

        Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose)
        Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadDistance)

        return ShooterProfile(
            normalizedTurretAngleRobotRelative,
            hoodAngle,
            flywheelVelocity
        )
    }

    fun getProfileWithPassing(): ShooterProfile {
        return getProfile(targetPassTranslation)
    }

    fun getProfileNoSotmWithPassing(): ShooterProfile {
        return getProfile(targetPassTranslation)
    }

}
private val PHASE_DELAY = 0.03.seconds
private val SHOOTER_HEIGHT = 0.4318.meters
private val GRAVITY = 9.81.metersPerSecondPerSecond

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

val robotToTurret = Transform2d(SHOOTER_OFFSET, Rotation2d.kZero)

var shooterTarget: Target = Target.STATIONARY_TURRET
var shooterProfile: ShooterProfile = shooterTarget.profile()

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

// used for populating interpolation tables
val hoodTunable = LoggedNetworkNumber("/Tuning/HoodTestAngle", 40.0)
val flywheelTunable = LoggedNetworkNumber("/Tuning/FlywheelSpeed", 3000.0)
val turretTunable = LoggedNetworkNumber("/Tuning/TurretAngle", 0.0)

data class ShooterProfile(
    val turretAngle: Angle,
    val hoodAngle: Angle,
    val angularVelocity: AngularVelocity,
)

enum class Target(val profile: () -> ShooterProfile) {
    AIM_AT_HUB_NO_SOTM (
        { ShooterCalculator.getProfileNoShootOnMove(hubTranslation.toTranslation2d()) }
    ),
    AIM_AT_HUB_PASS_NO_SOTM (
        { ShooterCalculator.getProfileNoSotmWithPassing() }
    ),
    AIM_AT_HUB_NO_PASS (
        { ShooterCalculator.getProfile(hubTranslation.toTranslation2d()) }
    ),
    AIM_AT_HUB_PASS (
        { ShooterCalculator.getProfileWithPassing() }
    ),
    STATIONARY_TURRET (
        { ShooterProfile(0.0.degrees, 5.degrees, 2600.rpm) },
    ),
    TUNING (
        { ShooterProfile(turretTunable.get().degrees, hoodTunable.get().degrees, flywheelTunable.get().rpm) }
    ),
}

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

val targetPassTranslation: Translation2d
    get() {
        val alliance = DriverStation.getAlliance().getOrNull()
        val pose = Drivetrain.estimatedPose.translation

        if (alliance == Alliance.Blue){
            if (pose.inZone(Zones.TopNeutralZone) || pose.inZone(Zones.TopRedAllianceZone)) {
                return Translation2d(4.meters, (FIELD_WIDTH_METERS / 4 ).meters)
            }
            else if (pose.inZone(Zones.BottomNeutralZone) || pose.inZone(Zones.BottomRedAllianceZone)){
                return Translation2d(4.meters, (FIELD_WIDTH_METERS * 3/4).meters)
            }
            else {
                return hubTranslation.toTranslation2d()
            }
        }
        else {
            if (pose.inZone(Zones.TopNeutralZone) || pose.inZone(Zones.TopBlueAllianceZone)) {
                return Translation2d(12.6.meters, (FIELD_WIDTH_METERS / 4 ).meters)
            }
            else if (pose.inZone(Zones.BottomNeutralZone) || pose.inZone(Zones.BottomBlueAllianceZone)){
                return Translation2d(12.6.meters, (FIELD_WIDTH_METERS * 3/4).meters)
            }
            else {
                return hubTranslation.toTranslation2d()
            }
        }
    }

enum class Zones(val startX : Distance, val endX : Distance, val startY: Distance, val endY : Distance) {
    TopBlueAllianceZone(0.meters, 4.03.meters, 0.meters, 4.meters),
    BottomBlueAllianceZone(0.meters, 4.03.meters, 4.meters, 8.meters),
    TopRedAllianceZone(11.22.meters, 16.20.meters, 0.meters, 4.meters),
    BottomRedAllianceZone(11.22.meters, 16.20.meters, 4.meters, 8.meters),
    TopNeutralZone(4.03.meters, 12.22.meters, 0.meters, 4.meters),
    BottomNeutralZone(4.03.meters, 12.22.meters, 4.meters, 8.meters),
}

fun Translation2d.inZone(target: Zones): Boolean {
    return this.x.meters in target.startX..<target.endX && this.y.meters in target.startY..<target.endY
}
