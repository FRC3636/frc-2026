package com.frcteam3636.frc2026.subsystems.shooter

import com.frcteam3636.frc2026.robot.Robot
import com.frcteam3636.frc2026.robot.Robot.Model
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.feeder.Feeder
import com.frcteam3636.frc2026.subsystems.indexer.Indexer
import com.frcteam3636.frc2026.subsystems.shooter.flywheel.Constants.FLYWHEEL_RADIUS
import com.frcteam3636.frc2026.subsystems.shooter.flywheel.Constants.FLYWHEEL_TO_FUEL_RATIO
import com.frcteam3636.frc2026.subsystems.shooter.flywheel.Flywheel
import com.frcteam3636.frc2026.subsystems.shooter.hood.Hood
import com.frcteam3636.frc2026.subsystems.shooter.turret.Constants.SHOOTER_OFFSET
import com.frcteam3636.frc2026.subsystems.shooter.turret.Turret
import com.frcteam3636.frc2026.utils.math.*
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
import kotlin.jvm.optionals.getOrNull
import kotlin.math.*


object ShooterCalculator {

    // stationary fuel velocity vector for shooting to hub
    val stationaryHubLaunchVector: Vector3d
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

    // stationary fuel velocity vector for passing to field translation
    val stationaryFieldTargetVector: Vector3d
        get() {
            val target = targetPassTranslation
            val translationToTarget = target - shooterFieldPose.translation
            val distance = translationToTarget.norm
            val hubHeight = hubTranslation.z
            /*
            create trajectory with vertex halfway between robot and target translation and height 1 meter above hub
            h = -a(x)(x - d)
            a = -h / ((x)(x - d))
            */
            val trajectoryConstant = ((hubHeight + 1.0) / (distance / 2).pow(2)).unaryMinus()
            // plot hub on trajectory and solve for distance
            val a = trajectoryConstant
            val b = -distance * trajectoryConstant
            val hubDistance = ((-b - sqrt(b.pow(2) - 4 * a * hubHeight)) / (2 * a)).meters

            val baseFlywheelRpm: Double = Flywheel.calculateFlywheelVelocity(hubDistance).inRPM()
            val baseHoodAngle: Double = Hood.calculateHoodAngle(hubDistance).inRadians()

            val launchSpeedRobotRelative =
                baseFlywheelRpm * (2.0 * PI * FLYWHEEL_RADIUS.inMeters()) / 60.0 * FLYWHEEL_TO_FUEL_RATIO

            val horizontalSpeedComponent = launchSpeedRobotRelative * cos(baseHoodAngle)
            val verticalSpeedComponent = launchSpeedRobotRelative * sin(baseHoodAngle)

            return Vector3d(
                horizontalSpeedComponent * cos(directionToHub.inRadians()),
                horizontalSpeedComponent * sin(directionToHub.inRadians()),
                verticalSpeedComponent
            )
        }

    // check for alliance color and which of six zones robot is in
    val targetPassTranslation: Translation2d
        get() {
            val alliance = DriverStation.getAlliance().getOrNull()
            return if (Drivetrain.estimatedPose.translation.inZone(Zones.TopBlueAllianceZone) || Drivetrain.estimatedPose.translation.inZone(
                    Zones.TopRedAllianceZone
                )
            ) {
                FeedTranslation.TopNeutralZone.target
            } else if (Drivetrain.estimatedPose.translation.inZone(Zones.BottomBlueAllianceZone) || Drivetrain.estimatedPose.translation.inZone(
                    Zones.BottomRedAllianceZone
                )
            ) {
                FeedTranslation.BottomNeutralZone.target
            } else if (Drivetrain.estimatedPose.translation.inZone(Zones.TopNeutralZone)) {
                if (alliance == Alliance.Blue) {
                    FeedTranslation.TopBlueAllianceZone.target
                } else {
                    FeedTranslation.TopRedAllianceZone.target
                }
            } else {
                if (alliance == Alliance.Blue) {
                    FeedTranslation.BottomBlueAllianceZone.target
                } else {
                    FeedTranslation.BottomRedAllianceZone.target
                }
            }
        }

    // create fuel velocity vector that accounts for field relative robot velocity
    fun movingLaunchVector(vector: Vector3d): Vector3d {
        val robotVelocity = Drivetrain.measuredChassisSpeedsRelativeToField
        return Vector3d(
            vector.x - (robotVelocity.vxMetersPerSecond),
            vector.y - (robotVelocity.vyMetersPerSecond),
            vector.z    // robot vertical velocity should always be zero (climbing?)
        )
    }

    // creates shooter profile from fuel velocity vector to hub
    fun vectorToShooterProfile(compensateForMotion: Boolean, stationaryVector: Vector3d): ShooterProfile {
        val vector = if (compensateForMotion) movingLaunchVector(stationaryVector) else stationaryVector

        val fieldDirection = atan2(vector.y, vector.x).radians
        val turretAngleRobotRelative =  (fieldDirection.inRadians() - Drivetrain.estimatedPose.rotation.radians).IEEErem(2 * PI).radians

        val horizontalMagnitude = hypot(vector.x, vector.y)
        val hoodAngle = atan2(vector.z, horizontalMagnitude).radians

        val requiredFlywheelSpeedRPM = vector.norm / ((2.0 * PI * FLYWHEEL_RADIUS.inMeters()) / 60.0 * FLYWHEEL_TO_FUEL_RATIO)

        return ShooterProfile(turretAngleRobotRelative, hoodAngle, requiredFlywheelSpeedRPM.rpm)
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

fun shoot() : Command =
    Commands.sequence(
        Commands.parallel(
            Flywheel.runAtTarget(),
        ).until(Flywheel.atDesiredFlywheelVelocity),
        Commands.parallel(
            Flywheel.runAtTarget(),
            Commands.parallel(
                Feeder.feed(),
                Indexer.index()
            )
        )
    )
// used for populating interpolation tables
val hoodTunable = LoggedNetworkNumber("/Tuning/HoodTestAngle", 40.0)
val flywheelTunable = LoggedNetworkNumber("/Tuning/FlywheelSpeed", 3000.0)
val turretTunable = LoggedNetworkNumber("/Tuning/TurretAngle", 0.0)

enum class Target(val profile: () -> ShooterProfile) {
    AIM_AT_HUB (
        { ShooterCalculator.vectorToShooterProfile(false, ShooterCalculator.stationaryHubLaunchVector) }
    ),
    AIM_AT_HUB_SHOOT_ON_MOVE (
//        { ShooterCalculator.aimAtHubShootOnMove() }
        { ShooterCalculator.vectorToShooterProfile(true, ShooterCalculator.stationaryHubLaunchVector) }
    ),
    STATIONARY_TURRET (
        { ShooterProfile(0.0.degrees, Hood.calculateHoodAngle(shooterToHub.norm.meters), Flywheel.calculateFlywheelVelocity(shooterToHub.norm.meters)) }
    ),
    STATIONARY_PASS (
        { ShooterCalculator.vectorToShooterProfile(false, ShooterCalculator.stationaryFieldTargetVector) }
    ),
    MOVING_PASS (
        { ShooterCalculator.vectorToShooterProfile(true, ShooterCalculator.stationaryFieldTargetVector) }
    ),
    TUNING (
        { ShooterProfile(turretTunable.get().degrees, hoodTunable.get().degrees, flywheelTunable.get().rpm) }
    ),
    HUNDRED_RPM (
        {ShooterProfile(0.degrees,40.degrees,100.rpm)}
    ),
}

enum class FeedTranslation(val target : Translation2d) {
    BottomBlueAllianceZone(Translation2d(3.25.meters,7.25.meters)),
    TopBlueAllianceZone(Translation2d(3.25.meters,0.82.meters)),
    BottomRedAllianceZone(Translation2d(13.29.meters,7.25.meters)),
    TopRedAllianceZone(Translation2d(13.29.meters,0.82.meters)),
    BottomNeutralZone(Translation2d(8.25.meters,7.25.meters)),
    TopNeutralZone(Translation2d(8.25.meters,0.82.meters)),
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
