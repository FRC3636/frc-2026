package com.frcteam3636.frc2026.subsystems.turret

import com.frcteam3636.frc2026.Robot
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.drivetrain.LimelightPoseProvider
import com.frcteam3636.frc2026.utils.math.inRadians
import com.frcteam3636.frc2026.utils.math.toRotation2d
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

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

    fun aimAtHub() {
        //TODO
        val camError = turretLimelight.getEntry("tx")
        var ty = 0.0
        if (camError != null && inputs.seeTags) {
            val tagDistance = turretLimelight.getEntry()
        }
    }

    fun turretBrakeMode() {
        io.setBrakeMode(true)
    }

    fun turretCoastMode() {
        io.setBrakeMode(false)
    }

}