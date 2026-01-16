package com.frcteam3636.frc2026.subsystems.turret

import com.frcteam3636.frc2026.Robot
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.drivetrain.LimelightPoseProvider
import com.frcteam3636.frc2026.utils.math.*
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import java.sql.Driver
import kotlin.jvm.optionals.getOrDefault
import kotlin.jvm.optionals.getOrElse
import kotlin.math.atan

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

    fun distanceToHub(): Distance {
        val hubTranslation = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue)
            .hubTranslation
        return Drivetrain.estimatedPose.translation.getDistance(hubTranslation).meters
    }

    fun aimAtHub() {
        val camError = turretLimelight.getEntry("tx").getDouble(0.0)
        val kP = -0.1
        // align with limelight
        if (camError != null && inputs.seeTags) {
            io.turnToAngle(inputs.turretAngle + (camError * kP).degrees)
        } else {
            // if no tags are seen then align with estimated pose
            val hubTranslation = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .hubTranslation
            val turretAngle = atan((hubTranslation.y - Drivetrain.estimatedPose.y) / (hubTranslation.x - Drivetrain.estimatedPose.x))
            io.turnToAngle(Math.toDegrees(turretAngle).degrees)
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
            DriverStation.Alliance.Blue -> Translation2d(
                4.62534.meters,
                (8.07 / 2).meters
            )

            else -> Translation2d(
                (16.54 - 4.62534).meters,
                (8.07 / 2).meters
            )
        }
}