package com.frcteam3636.frc2026.subsystems.climb

import com.frcteam3636.frc2026.utils.math.inMeters
import com.frcteam3636.frc2026.utils.math.inches
import com.frcteam3636.frc2026.utils.math.meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class Climb : Subsystem {
    internal companion object Constants {
        private val CLIMBER_TOLERANCE = 0.2.inches
    }

    private val io: ClimbIO = ClimbIOReal()

    val inputs = LoggedClimbInputs()

    var position = Position.STOWED
    var targetHeight = 0.meters

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Climb", inputs)
    }

    fun setTargetHeight(setPosition: Position): Command = startEnd(
        {
            position = setPosition
            targetHeight = position.height
            io.goToHeight(position.height)
        },
        {

        }
    ).until { isAtTarget() }

    fun isAtTarget(): Boolean = abs((inputs.height - targetHeight).inMeters()) < CLIMBER_TOLERANCE.inMeters()

    enum class Position(val height: Distance) {
        STOWED(0.meters),
        // ... Might be more complicated than just set heights, we'll see.
    }
}