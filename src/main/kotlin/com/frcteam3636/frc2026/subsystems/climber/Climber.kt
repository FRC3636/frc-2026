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

object Climber : Subsystem {
    private object Constants {
        val CLIMBER_TOLERANCE = 0.2.inches
    }

    enum class Position(val height: Distance) {
        STOWED(0.meters),
        GROUND_L1(0.5.meters),
        // ... Might be more complicated than just set heights, we'll see.
    }

    private val io: ClimberIO = when (Robot.model) {
        Robot.Model.COMPETITION -> ClimberIOReal()
        Robot.Model.SIMULATION -> ClimberIOSim()
    }

    val inputs = LoggedClimberInputs()

    var position = Position.STOWED
    var targetHeight = 0.meters

    val rgbTable = NetworkTableInstance.getDefault().getTable("RGB")
    val climbPosTopic = DoubleTopic(rgbTable.getTopic("/ClimbHeight")).publish()
    val climbVelTopic = DoubleTopic(rgbTable.getTopic("/ClimbVelocity")).publish()
    val climbTargetTopic = DoubleTopic(rgbTable.getTopic("/ClimbTarget")).publish()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Climb", inputs)
        climbPosTopic.set(inputs.height.inMeters())
        climbVelTopic.set(inputs.velocity.inMetersPerSecond())
        climbTargetTopic.set(targetHeight.inMeters())
    }

    fun goToHeight(setPosition: Position, slow: Boolean = false): Command = startEnd(
        {
            position = setPosition
            targetHeight = position.height
            io.goToHeight(position.height, slow)
        },
        {

        }
    ).until(isAtTarget())

    fun homeRoutine(): Command = Commands.sequence(
        Commands.runEnd(
            {
                io.setVoltage(1.0.volts)
            },
            {
                io.setVoltage(0.0.volts)
            }
        ).until({ inputs.current > 3.5.amps}),
        Commands.runOnce({ io.setEncoderPosition(0.0.meters) })
    )

    // Pulls down and hooks onto the L1 bar.
    fun climbL1(): Command = Commands.sequence(
        goToHeight(Position.GROUND_L1),
        // TODO: Wait until the robot is in the right position to climb. Ideally it should already be in that position, but just in case.
        // This is dependant on the auto system which to my knowledge isn't implemented yet.
        goToHeight(Position.STOWED, true),
    )

    fun isAtTarget(): Trigger =
        Trigger({ abs((inputs.height - targetHeight).inMeters()) < Constants.CLIMBER_TOLERANCE.inMeters() })
}
