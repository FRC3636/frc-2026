package com.frcteam3636.frc2026.subsystems.turret

import com.frcteam3636.frc2026.Robot
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.drivetrain.LimelightPoseProvider
import com.frcteam3636.frc2026.utils.math.inRadians
import com.frcteam3636.frc2026.utils.math.toRotation2d
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Turret : Subsystem{

    private var io = when (Robot.model) {
        Robot.Model.SIMULATION -> TurretIOSim()
        Robot.Model.COMPETITION -> TurretIOReal()
    }

    private val inputs = LoggedTurretInputs()

    val turretLimelight = LimelightPoseProvider(
        "turret-limelight",
        {
            inputs.turretAngle.toRotation2d()
        },
        {
            inputs.turretVelocity
        },
        {
            Drivetrain.inputs.gyroConnected
        },
        false
    )

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Turret", inputs)
    }

    fun aimAtHub() {
        //TODO
    }

    fun turretBrakeMode() {
        io.setBrakeMode(true)
    }

    fun turretCoastMode() {
        io.setBrakeMode(false)
    }

}