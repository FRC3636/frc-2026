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
import edu.wpi.first.hal.DriverStationJNI
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

    // stationary fuel velocity vector for passing to field translation
    fun stationaryFieldTargetVector(target: Translation2d): Vector3d {
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
        val hubDistance =((-b - sqrt(b.pow(2) - 4 * a * hubHeight)) / (2 * a)).meters

        val baseFlywheelRpm: Double = Flywheel.calculateFlywheelVelocity(hubDistance).inRPM()
        val baseHoodAngle: Double = Hood.calculateHoodAngle(hubDistance).inRadians()

        val launchSpeedRobotRelative = baseFlywheelRpm * (2.0 * PI * FLYWHEEL_RADIUS.inMeters()) / 60.0 * FLYWHEEL_TO_FUEL_RATIO

        val horizontalSpeedComponent = launchSpeedRobotRelative * cos(baseHoodAngle)
        val verticalSpeedComponent = launchSpeedRobotRelative * sin(baseHoodAngle)

        return Vector3d(
            horizontalSpeedComponent * cos(directionToHub.inRadians()),
            horizontalSpeedComponent * sin(directionToHub.inRadians()),
            verticalSpeedComponent
        )
    }

    // create fuel velocity vector that accounts for field relative robot velocity
    fun getFieldRelativeLaunchVector(vector: Vector3d): Vector3d {
        val robotVelocity = Drivetrain.measuredChassisSpeedsRelativeToField
        return Vector3d(
            vector.x - (robotVelocity.vxMetersPerSecond),
            vector.y - (robotVelocity.vyMetersPerSecond),
            vector.z    // robot vertical velocity should always be zero
        )
    }

    // creates shooter profile from fuel velocity vector to hub
    fun aimAtHub(compensateForMotion: Boolean): ShooterProfile {
        val vector = if (compensateForMotion) getFieldRelativeLaunchVector(stationaryLaunchVector) else stationaryLaunchVector

        val fieldDirection = atan2(vector.y, vector.x).radians
        val turretAngleRobotRelative =  (fieldDirection.inRadians() - Drivetrain.estimatedPose.rotation.radians).IEEErem(2 * PI).radians

        val horizontalMagnitude = hypot(vector.x, vector.y)
        val hoodAngle = atan2(vector.z, horizontalMagnitude).radians

        val requiredFlywheelSpeedRPM = vector.norm.metersPerSecond.toAngular(FLYWHEEL_RADIUS * FLYWHEEL_TO_FUEL_RATIO).inRPM()

        return ShooterProfile(turretAngleRobotRelative, hoodAngle, requiredFlywheelSpeedRPM.rpm)
    }

    // check for alliance color and which of six zones robot is in
    val targetPassTranslation: Translation2d
        get() {
            val alliance = DriverStation.getAlliance().getOrNull()
            return if (Drivetrain.estimatedPose.translation.inZone(Zones.TopBlueAllianceZone) || Drivetrain.estimatedPose.translation.inZone(Zones.TopRedAllianceZone)) {
                FeedTranslation.TopNeutralZone.target
            } else if (Drivetrain.estimatedPose.translation.inZone(Zones.BottomBlueAllianceZone) || Drivetrain.estimatedPose.translation.inZone(Zones.BottomRedAllianceZone)) {
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

    // creates shooter profile from fuel velocity vector to field translation
    fun pass(compensateForMotion: Boolean): ShooterProfile {
        val vector =  if (compensateForMotion) getFieldRelativeLaunchVector(stationaryFieldTargetVector(targetPassTranslation)) else stationaryFieldTargetVector(targetPassTranslation)
        val fieldDirection = atan2(vector.y, vector.x).radians
        val turretAngleRobotRelative =  (fieldDirection.inRadians() - Drivetrain.estimatedPose.rotation.radians).IEEErem(2 * PI).radians

        val horizontalMagnitude = hypot(vector.x, vector.y)
        val hoodAngle = atan2(vector.z, horizontalMagnitude).radians

        val requiredFlywheelSpeedRPM = vector.norm.metersPerSecond.toAngular(FLYWHEEL_RADIUS * FLYWHEEL_TO_FUEL_RATIO).inRPM()

        return ShooterProfile(turretAngleRobotRelative, hoodAngle, requiredFlywheelSpeedRPM.rpm)
    }

    // TO BE TESTED AFTER AIM AT HUB
    fun aimAtTarget(targetPosition: Translation3d, compensateForMotion: Boolean = false): ShooterProfile {

        val shooterPosition3d = Translation3d (
            shooterFieldPose.translation.x.meters,
            shooterFieldPose.translation.y.meters,
            SHOOTER_HEIGHT
        )

        val relativeTargetVector = targetPosition.minus(shooterPosition3d)
        val deltaX = relativeTargetVector.x
        val deltaY = relativeTargetVector.y
        val deltaZ = relativeTargetVector.z
        val horizontalDistance = hypot(deltaX, deltaY)

        val minimumLaunchSpeedSquared = GRAVITY.inMetersPerSecondPerSecond() * (deltaZ + sqrt(horizontalDistance.pow(2) + deltaZ.pow(2)))
        val minimumLaunchSpeed = sqrt(minimumLaunchSpeedSquared).metersPerSecond

        // The formula for theta at minimum speed is tan(theta) = (deltaZ + sqrt(distance^2 + deltaZ^2)) / distance
        val tanTheta = (deltaZ + sqrt(horizontalDistance.pow(2) + deltaZ.pow(2))) / horizontalDistance
        val stationaryLaunchHoodAngle = atan(tanTheta).radians

        val horizontalLaunchSpeed = minimumLaunchSpeed.inMetersPerSecond() * cos(stationaryLaunchHoodAngle.inRadians())
        val verticalLaunchSpeed = minimumLaunchSpeed.inMetersPerSecond() * sin(stationaryLaunchHoodAngle.inRadians())

        val directionToTarget = atan2(deltaY, deltaX).radians

        val stationaryLaunchVectorFieldRelative = Vector3d(
            horizontalLaunchSpeed * cos(directionToTarget.inRadians()),
            horizontalLaunchSpeed * sin(directionToTarget.inRadians()),
            verticalLaunchSpeed
        )

        val fieldRelativeLaunchVelocity = if (compensateForMotion) {
            val robotVelocity = Drivetrain.measuredChassisSpeedsRelativeToField.translation2dPerSecond
            Vector3d(
                stationaryLaunchVectorFieldRelative.x - robotVelocity.x,
                stationaryLaunchVectorFieldRelative.y - robotVelocity.y,
                stationaryLaunchVectorFieldRelative.z   // robot vertical velocity should always be zero
            )
        } else {
            stationaryLaunchVectorFieldRelative
        }

        val fieldDirection = atan2(fieldRelativeLaunchVelocity.y, fieldRelativeLaunchVelocity.x).radians
        val turretAngleRobotRelative = (fieldDirection.inRadians() - Drivetrain.estimatedPose.rotation.radians).IEEErem(2 * PI).radians

        val horizontalSpeedMagnitude = hypot(fieldRelativeLaunchVelocity.x, fieldRelativeLaunchVelocity.y)

        val requiredHoodAngle = atan2(fieldRelativeLaunchVelocity.z, horizontalSpeedMagnitude).radians
        val requiredSpeed = fieldRelativeLaunchVelocity.norm

        val requiredSpeedRPM = requiredSpeed / (2.0 * PI / 60.0 * FLYWHEEL_RADIUS.inMeters() * FLYWHEEL_TO_FUEL_RATIO)

        return ShooterProfile(turretAngleRobotRelative, requiredHoodAngle, requiredSpeedRPM.rpm)
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
val flywheelTunable = LoggedNetworkNumber("/Tuning/FlywheelSpeed", 1000.0)
val turretTunable = LoggedNetworkNumber("/Tuning/TurretAngle", 0.0)

enum class Target(val profile: () -> ShooterProfile) {
    AIM_AT_HUB (
        { ShooterCalculator.aimAtHub(false) }
    ),
    AIM_AT_HUB_SHOOT_ON_MOVE (
        { ShooterCalculator.aimAtHub(true) }
    ),
    STATIONARY_TURRET (
        { ShooterProfile(0.0.degrees, Hood.calculateHoodAngle(shooterToHub.norm.meters), Flywheel.calculateFlywheelVelocity(shooterToHub.norm.meters)) }
    ),
    STATIONARY_PASS (
        { ShooterCalculator.pass(false) }
    ),
    MOVING_PASS (
        { ShooterCalculator.pass(true) }
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
