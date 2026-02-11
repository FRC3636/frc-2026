package com.frcteam3636.frc2026.subsystems.indexer

import com.frcteam3636.frc2026.Robot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Indexer : Subsystem {
    private val io: IndexerIO = when (Robot.model) {
        Robot.Model.SIMULATION -> TODO("Add sim io")
        Robot.Model.COMPETITION -> IndexerIOReal()
    }

    var inputs = LoggedIndexerInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)
    }

    fun index(): Command = Commands.startEnd(
        {
            io.setSpeed(0.75)
        },
        {
            io.setSpeed(0.0)
        }
    )

    fun outdex(): Command = Commands.startEnd(
        {
            io.setSpeed(-0.75)
        },
        {
            io.setSpeed(0.0)
        }
    )
}
