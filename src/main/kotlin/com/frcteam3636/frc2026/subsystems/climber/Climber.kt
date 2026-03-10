package com.frcteam3636.frc2026.subsystems.climber

import com.frcteam3636.frc2026.robot.Robot
import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.inMeters
import com.frcteam3636.frc2026.utils.math.inMetersPerSecond
import com.frcteam3636.frc2026.utils.math.inches
import com.frcteam3636.frc2026.utils.math.meters
import com.frcteam3636.frc2026.utils.math.volts
import com.frcteam3636.frc2026.utils.math.amps
import edu.wpi.first.networktables.DoubleTopic
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.Logger
import kotlin.math.abs
import java.util.Optional
import kotlin.jvm.optionals.getOrNull

object Climber : Subsystem {
    private object Constants {
        val CLIMBER_TOLERANCE = 0.2.inches
    }

    enum class Position(val height: Optional<Distance>) {
        UNHOMED(Optional.empty()),
        STOWED(Optional.of(0.meters)),
        GROUND_L1(Optional.of(0.5.meters)),
        // ... Might be more complicated than just set heights, we'll see.
    }

    private val io: ClimberIO = when (Robot.model) {
        Robot.Model.COMPETITION -> ClimberIOReal()
        Robot.Model.SIMULATION -> ClimberIOSim()
    }

    val inputs = LoggedClimberInputs()

    var targetPosition = Position.STOWED

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Climb", inputs)
    }

    fun goToTargetHeight(): Command = Commands.run({
        val targetPosition = targetPosition.height.getOrNull()
        if (targetPosition != null) {
            io.goToHeight(targetPosition)
        }
    })

    fun homeRoutine(): Command = Commands.sequence(
        Commands.runEnd(
            {
                io.setVoltage(1.0.volts)
            },
            {
                io.setVoltage(0.0.volts)
            }
        ).until({ inputs.current > 3.5.amps}),
        Commands.runOnce({
            io.setEncoderPosition(0.0.meters)
            targetPosition = Position.STOWED
        })
    )

    // Pulls down and hooks onto the L1 bar.
    fun climb(): Command = Commands.runEnd({
        io.setVoltage(12.0.volts)
    }, {
        io.setVoltage(0.0.volts)
    })
}
