package com.frcteam3636.frc2026.subsystems.climb

import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.inDegrees
import com.frcteam3636.frc2026.utils.math.inMeters
import com.frcteam3636.frc2026.utils.math.inches
import com.frcteam3636.frc2026.utils.math.meters
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class Climb : Subsystem {
    internal companion object Constants {
        private val CLIMBER_TOLERANCE = 0.2.inches
        private val HOOK_TOLERANCE = 5.degrees
    }

    enum class Position(val height: Distance) {
        STOWED(0.meters),
        GROUND_L1(0.5.meters),
        HANG_NEXT_BAR(0.4.meters)
        // ... Might be more complicated than just set heights, we'll see.
    }

    enum class HookPosition(val angle: Angle) {
        STOWED(0.degrees),
        EXTENDED(90.degrees)
    }

    private val io: ClimbIO = ClimbIOReal()

    val inputs = LoggedClimbInputs()

    var position = Position.STOWED
    var hookPosition = HookPosition.STOWED
    var targetHeight = 0.meters
    var hookTargetAngle = 0.degrees

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Climb", inputs)
    }

    fun goToHeight(setPosition: Position, slow: Boolean = false): Command = startEnd(
        {
            position = setPosition
            targetHeight = position.height
            io.goToHeight(position.height, slow)
        },
        {

        }
    ).until { isAtTarget() }

    fun hookGoToPosition(setPosition: HookPosition): Command = startEnd(
        {
            hookPosition = setPosition
            hookTargetAngle = setPosition.angle
            io.hookGoToAngle(setPosition.angle)
        },
        {

        }
    ).until { hookIsAtTarget() }

    fun climbL1NoHook(): Command = Commands.sequence(
        Commands.parallel(
            goToHeight(Position.GROUND_L1),
            hookGoToPosition(HookPosition.STOWED),
        ),
        // TODO: Wait until the robot is in the right position to climb. Ideally it should already be in that position, but just in case.
        goToHeight(Position.STOWED, true),
    )

    // Pulls down and hooks onto the L1 bar.
    fun climbL1(): Command = Commands.sequence(
        climbL1NoHook(),
        hookGoToPosition(HookPosition.EXTENDED)
    )

    // Climb to next bar is really hard to tell what we'll be doing ...
    // We'll have to see based on design.

    fun isAtTarget(): Boolean = abs((inputs.height - targetHeight).inMeters()) < CLIMBER_TOLERANCE.inMeters()
    fun hookIsAtTarget(): Boolean = abs((inputs.hookAngle - hookTargetAngle).inDegrees()) < HOOK_TOLERANCE.inDegrees()
}