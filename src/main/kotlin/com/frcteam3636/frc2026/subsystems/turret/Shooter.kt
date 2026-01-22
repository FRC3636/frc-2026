package com.frcteam3636.frc2026.subsystems.turret

import com.frcteam3636.frc2026.Robot
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.flywheel.FlywheelIO
import com.frcteam3636.frc2026.subsystems.flywheel.FlywheelIOReal
import com.frcteam3636.frc2026.subsystems.flywheel.FlywheelInputs
import com.frcteam3636.frc2026.subsystems.turret.Shooter.Flywheel.velocityInterpolationTable
import com.frcteam3636.frc2026.subsystems.turret.Shooter.Turret.hubTranslation
import com.frcteam3636.frc2026.utils.math.*
import com.frcteam3636.frc2026.utils.swerve.translation2dPerSecond
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import kotlin.math.*

object Shooter {
    object Turret : Subsystem{

        private var io = when (Robot.model) {
            Robot.Model.SIMULATION -> TurretIOSim()
            Robot.Model.COMPETITION -> TurretIOReal()
        }

        private val inputs = LoggedTurretInputs()

        private val turretLimelight = NetworkTableInstance.getDefault().getTable("turret-limelight")
        private val seeTagsDebouncer = Debouncer(0.5)
        private var seeTagsRaw = false
            set(value) {
                inputs.seeTags = seeTagsDebouncer.calculate(value)
                field = value
            }

        override fun periodic() {
            io.updateInputs(inputs)
            seeTagsRaw = turretLimelight.getEntry("tv").equals(1)
            Logger.processInputs("Turret", inputs)
        }

        fun alignToHub() {
            val camError = (turretLimelight.getEntry("tx").getDouble(0.0).degrees.inRadians())
            // align with limelight
            if (inputs.seeTags) {
                val kP = -0.1
                io.turnToAngle(inputs.turretAngle + (camError * kP).radians)
            } else {
                // if no tags are seen then align with estimated pose
                val hubTranslation = DriverStation.getAlliance()
                    .orElse(DriverStation.Alliance.Blue)
                    .hubTranslation
                val turretAngle = atan((hubTranslation.y - Drivetrain.estimatedPose.y) / (hubTranslation.x - Drivetrain.estimatedPose.x)) - Drivetrain.estimatedPose.rotation.radians
            }
        }

        fun turretBrakeMode() {
            io.setBrakeMode(true)
        }

        fun turretCoastMode() {
            io.setBrakeMode(false)
        }

        val DriverStation.Alliance.hubTranslation
            get() = when (this) {
                DriverStation.Alliance.Blue -> Translation3d(
                    4.62534.meters,
                    (8.07 / 2).meters,
                    1.83.meters,
                )

                else -> Translation3d(
                    (16.54 - 4.62534).meters,
                    (8.07 / 2).meters,
                    1.83.meters,
                )
            }
    }

    object Hood: Subsystem {
        private var io = when (Robot.model) {
            Robot.Model.SIMULATION -> HoodIOSim()
            Robot.Model.COMPETITION -> HoodIOReal()
        }

        val inputs = LoggedHoodInputs()


        // distance -> sin(angle)
        // TODO: find values
        private val angleInterpolationTable = InterpolatingTreeMap(
            InverseInterpolator.forDouble(),
            Interpolator.forDouble()
        ).apply {
            put(1.0, 45.0)
            put(1.5, 40.0)
        }

        override fun periodic() {
            io.updateInputs(inputs)
            Logger.processInputs("Hood", inputs)
        }

        fun calculateFlightTime(launchAngle: Angle, launchVelocity: LinearVelocity): Time {
            val translationalVelocity = Drivetrain.measuredChassisSpeeds.translation2dPerSecond.norm.metersPerSecond
            val verticalHubTranslation = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .hubTranslation.z

            // quadratic formula
            val firstArcTime = (((launchVelocity.getVerticalComponent(launchAngle).inMetersPerSecond().unaryMinus() +
                    sqrt(launchVelocity.getVerticalComponent(launchAngle).inMetersPerSecond().pow(2.0) -
                            4.0 * (GRAVITY.unaryMinus() / 2.0) * verticalHubTranslation.unaryMinus()))) /
                    GRAVITY.unaryMinus()).seconds
            val secondVerticalVelocity = launchVelocity.getVerticalComponent(launchAngle) - firstArcTime * GRAVITY.metersPerSecondPerSecond
            val secondArcTime = (((secondVerticalVelocity.inMetersPerSecond() * 2.0 / GRAVITY))).seconds
            return firstArcTime + secondArcTime
        }

        fun getHoodAngle(distance: Distance): Angle {
            return asin(angleInterpolationTable.get(distance.inMeters())).radians
        }



        fun hoodBrakeMode(): Command =
            run {
                io.setBrakeMode(true)
            }

        fun hoodCoastMode(): Command =
            run {
                io.setBrakeMode(false)
            }

    }

    object Flywheel: Subsystem {

        private val io: FlywheelIO = when (Robot.model) {
            Robot.Model.SIMULATION -> TODO()
            Robot.Model.COMPETITION -> FlywheelIOReal()
        }

        var inputs: FlywheelInputs = FlywheelInputs()

        override fun periodic() {
            io.updateInputs(inputs)
        }

        // distance -> velocity
        // TODO: find values
        val velocityInterpolationTable = InterpolatingTreeMap(
            InverseInterpolator.forDouble(),
            Interpolator.forDouble()
        ).apply {
            put(1.0, 3000.0)
            put(1.5, 4500.0)
        }

        fun setVoltage(volts: Voltage): Command = startEnd(
            {

                io.setMotorVoltage(volts)
            },
            {
                io.setMotorVoltage(0.volts)
            }
        )

        fun setSpeed(distance: Double): Command = startEnd(
            {
                val percentage = velocityInterpolationTable.get(distance)
                io.setSpeed(percentage)
            },
            {
                io.setSpeed(0.0)
            }
        )
    }

    val offset: Translation2d
        get() = {
            Drivetrain.measuredChassisSpeeds.translation2dPerSecond * Hood.calculateFlightTime(
                Hood.inputs.hoodAngle,
                Flywheel.inputs.linearVelocity
            )
        }


    fun distanceToHub(offset: Translation2d): Distance {
        val hubTranslation = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue)
            .hubTranslation.toTranslation2d() + offset
            return Drivetrain.estimatedPose.translation.getDistance(hubTranslation).meters
    }

    data class ShooterProfile(
        val hoodAngle: () -> Angle,
        val flywheelVelocity: () -> AngularVelocity,
    )

    // [x, y, z]
    fun vectorToShooterProfile(velocityVector: Matrix<N3, N1>): ShooterProfile {

    }

    enum class Target(val shooterProfile: ShooterProfile) {
        AIM(
            ShooterProfile({
                val distance = distanceToHub(offset)
                val angle = Hood.getHoodAngle(distance).inRadians()
                clamp(angle, 40.0.degrees.inRadians(), 60.0.degrees.inRadians()).radians
            },
            {
                val distance = distanceToHub(offset)
                velocityInterpolationTable.get(distance.inMeters()).rpm
            },)
        ),
        TUNING(
            ShooterProfile(
                {
                    hoodTunable.get().degrees
                },
                {
                    flywheelTunable.get().rpm
                }
            )

        )
    }

    val hoodTunable = LoggedNetworkNumber("/Tuning/HoodTestAngle", 40.0)
    val flywheelTunable = LoggedNetworkNumber("/Tuning/FlywheelSpeed", 1000.0)
}
