package com.frcteam3636.frc2026.subsystems.feeder

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Feeder : Subsystem {
    private val io = FeederIOReal()
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

    fun reverse(): Command = startEnd(
        {
            io.setSpeed(-1.0)
        },
        {
            io.setSpeed(0.0)
        }

    )

}