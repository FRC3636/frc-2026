package com.frcteam3636.frc2026.subsystems.climb

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

class Climb : Subsystem {
    private val io  = ClimbIOReal()

    val inputs = LoggedClimbInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Climb", inputs)
    }

    fun climbL1() : Command = Commands.sequence()
}