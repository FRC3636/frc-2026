package com.frcteam3636.frc2026.subsystems.turret

import com.frcteam3636.frc2026.Robot
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.drivetrain.LimelightPoseProvider
import com.frcteam3636.frc2026.subsystems.flywheel.FlywheelIO
import com.frcteam3636.frc2026.subsystems.flywheel.FlywheelIOReal
import com.frcteam3636.frc2026.subsystems.flywheel.FlywheelInputs
import com.frcteam3636.frc2026.subsystems.turret.Turret.hubTranslation
import com.frcteam3636.frc2026.utils.math.*
import com.frcteam3636.frc2026.utils.swerve.angularVelocity
import com.frcteam3636.frc2026.utils.swerve.translation2dPerSecond
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import java.lang.Math.pow
import java.sql.Driver
import kotlin.jvm.optionals.getOrDefault
import kotlin.jvm.optionals.getOrElse
import kotlin.math.asin
import kotlin.math.atan
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

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

    fun alignAtHub() {
        val camError = Math.toRadians(turretLimelight.getEntry("tx").getDouble(0.0))
        // align with limelight
        if (camError != null && inputs.seeTags) {
            val kP = -0.1
            io.turnToAngle(inputs.turretAngle + (camError * kP).radians)
        } else {
            // if no tags are seen then align with estimated pose
            val hubTranslation = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .hubTranslation
            val turretAngle = atan((hubTranslation.y - Drivetrain.estimatedPose.y) / (hubTranslation.x - Drivetrain.estimatedPose.x)) - Drivetrain.estimatedPose.rotation.radians
            io.turnToAngle(turretAngle.radians)
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

    private val inputs = LoggedHoodInputs()

    // distance -> sin(angle)
    // TODO: find values
    private val angleInterpolationTable = InterpolatingTreeMap<Double, Double>(
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

    private fun distanceToHub(offset: Translation2d): Distance {
        val hubTranslation = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue)
            .hubTranslation.toTranslation2d() + offset
        return Drivetrain.estimatedPose.translation.getDistance(hubTranslation).meters
    }

    private fun flightTime(launchAngle: Angle, launchVelocity: LinearVelocity): Time {
        val translationalVelocity = Drivetrain.measuredChassisSpeeds.translation2dPerSecond.norm.metersPerSecond
        val verticalHubTranslation = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue)
            .hubTranslation.z

        val shootSpeed = (60 + sqrt( 60.0.pow(2.0) - (4.0 * GRAVITY.unaryMinus().inMetersPerSecondPerSecond()*verticalHubTranslation))).seconds

        val firstArcTime = (sqrt(2*verticalHubTranslation/GRAVITY.inMetersPerSecondPerSecond())).seconds

        val secondArcTime = ((((((launchVelocity * sin(launchAngle.inRadians())) - (firstArcTime * GRAVITY))).inMetersPerSecond() * 2.0 / GRAVITY.inMetersPerSecondPerSecond()))).seconds
        return firstArcTime + secondArcTime
    }

    fun aimAtHub() {
        var offset = Drivetrain.measuredChassisSpeeds.translation2dPerSecond.norm.metersPerSecond * flightTime())
        var distance = distanceToHub(offset)
        var angle = asin(angleInterpolationTable.get(distance.inMeters()))
        io.turnToAngle(angle.radians)
    }

    fun hoodBrakeMode() {
        io.setBrakeMode(true)
    }

    fun hoodCoastMode() {
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

    fun setVoltage(volts: Voltage): Command = startEnd(
        {
            io.setMotorVoltage(volts)
        },
        {
            io.setMotorVoltage(0.volts)
        }
    )
}