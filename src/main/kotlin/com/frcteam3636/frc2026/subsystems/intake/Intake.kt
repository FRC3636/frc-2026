package com.frcteam3636.frc2026.subsystems.intake

import com.frcteam3636.frc2026.robot.Robot
import com.frcteam3636.frc2026.subsystems.indexer.Indexer
import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.inDegrees
import com.frcteam3636.frc2026.utils.math.volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

/*
 * 5. High-level Subsystem
 *
 * The intake is responsible for taking items off the ground. This intake consists of two components: The motor that
 * spins the intake and the motor that pivots it. For now, we don't have to worry about all those details, because the
 * io (which stands for input-output) field takes care of it.
 */

object Intake : Subsystem {

    /**
     * The exact angle of each position of the Intake.
     * Deployed means on the field, Stowed means pointing straight up, and Back is rotated as far as possible towards
     * the robot (this could be useful to push balls out of the intake and into the indexer.)
     */
    enum class Position(val angle: Angle) {
        Deployed(110.degrees),
        Stowed(75.degrees),
        Back(10.degrees),
    }

    /**
     * Here we first encounter the high-level intake and IO-level separation. The IO takes basic commands from this
     * class, and converts those commands to real action on the robot, regardless of whether the robot is in Simulation
     * or Competition mode. In technical terms, the IntakeIO *interface* is the same, regardless of the robot Model.
     *
     * For simplicity, this code doesn't have a simulation version of the IO.
     */
    private val io: IntakeIO =
        when (Robot.model) {
            Robot.Model.SIMULATION -> TODO()
            Robot.Model.COMPETITION -> IntakeIOReal()
        }

    /**
     * During the periodic function, this object is passed to the IO function updateInputs. We can then use some of this
     * data after it's processed by the IO.
     */
    private val inputs = LoggedIntakeInputs()

    /**
     * Periodic runs every ~20ms on every Subsystem (which is why the function is marked override).
     */
    override fun periodic() {
        io.updateInputs(inputs)
        // We also log all these inputs.
        Logger.processInputs("Intake", inputs)
    }

    /*
     * Our first encounter with a Trigger! If you don't remember how triggers work, there is a more in-depth explanation
     * in the book in Section 6 (Commands). This specific trigger checks if the intake pivot is within 3 degrees of the
     * pivot setpoint.
     */
    val atDesiredPivotAngle: Trigger =
        Trigger {
            abs((inputs.pivotAngle - inputs.pivotSetpoint).inDegrees()) < 3
        }

    /*
     * Here is a function which can generate a Command. It probably looks a little different from the Commands in the
     * book, because that was written for next year's version of WPILib, and this code was written for last year's
     * competition.
     */
    /**
     * Generates a command that moves the intake pivot to the desired position.
     */
    fun setPivotPosition(position: Position): Command =
        run {
            io.setPivotAngle(position.angle)
        }

    /*
     * One of the problems in robotics is figuring out the rotation of a mechanism. All of our motors have encoders on
     * them, but their rotation is set to zero every time the robot starts up. Because the information the encoders
     * give us may not be accurate, sometimes we have to zero the encoder manually. Mechanism slippage (when the amount
     * the motor moves and the amount the mechanism moves becomes disproportional) can also cause the encoder to report
     * an erroneous rotation.
     */
    fun zeroPivot(): Command = Commands.runOnce(
        {
            println("Zeroing Pivot!!!")
            io.zeroEncoder()
        }
    )

    fun setPivotVoltage(voltage: Voltage): Command = Commands.runEnd(
        {io.setPivotVoltage(voltage)},
        {io.setPivotVoltage(0.volts)}
    )

    /*
     * Here is a more complex command. This both deploys the intake, spins the intake, and spins the indexer at the
     * same time. For the rest of the commands, see if you can reason through what they do just based on the code.
     */
    fun intakeSequence(): Command =
        Commands.parallel(
            Commands.runEnd(
                { io.setPivotAngle(Position.Deployed.angle) },
                { io.setPivotAngle(Position.Stowed.angle) }
            ),
            intake(),
            Commands.parallel(
                Indexer.slowIndex(),
//                Feeder.slowFeed()
            ) // .until { Feeder.inputs.ballDetected }
        )

    fun manipulateSequence(): Command =
        Commands.parallel(
            Commands.runEnd(
                { io.setPivotAngle(Position.Back.angle) },
                { io.setPivotAngle(Position.Stowed.angle) }
            ),
            intake()
        )

    fun intake(): Command =
            runEnd(
                { io.setVoltage(10.0.volts) },
                { io.setVoltage(0.volts) }
            )

    fun outtake(): Command = runEnd(
        { io.setVoltage((-5.0).volts) },
        { io.setVoltage(0.volts) },
    )
}

/*
 * Now that we have read through some subsystem code, and seen how Commands and Triggers are used in practice, let's
 * move on to the IO portion of the Subsystem (which is in the adjacent IntakeIO.kt file).
 */