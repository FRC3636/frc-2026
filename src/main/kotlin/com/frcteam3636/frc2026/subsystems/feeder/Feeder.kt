package com.frcteam3636.frc2026.subsystems.feeder

import com.frcteam3636.frc2026.robot.Robot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Feeder : Subsystem {
    private val io: FeederIO = when (Robot.model) {
        Robot.Model.SIMULATION -> TODO("Add sim")
        Robot.Model.COMPETITION -> FeederIOReal()
    }
    val inputs = LoggedFeederInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Feeder", inputs)
    }

    fun feed(): Command = startEnd(
        {
            io.setSpeed(1.0)
        },
        {
            io.setSpeed(0.0)
        }
    )

    fun outtake(): Command = startEnd(
        {
            io.setSpeed(-0.25)
        },
        {
            io.setSpeed(0.0)
        }
    )

}