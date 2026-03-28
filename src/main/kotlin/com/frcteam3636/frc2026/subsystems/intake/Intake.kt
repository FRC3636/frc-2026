package com.frcteam3636.frc2026.subsystems.intake

import com.frcteam3636.frc2026.robot.Robot
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

object Intake : Subsystem {

    enum class Position(val angle: Angle) {
        Deployed(110.degrees),
        Stowed(75.degrees),
        Back(10.degrees),
    }

    private val io: IntakeIO =
        when (Robot.model) {
        Robot.Model.SIMULATION -> IntakeIOReal()
        Robot.Model.COMPETITION -> IntakeIOReal()

    }

    private val inputs = LoggedIntakeInputs()

    val atDesiredPivotAngle: Trigger =
        Trigger({
            abs((inputs.pivotAngle - inputs.pivotSetpoint).inDegrees()) < 3
        })

    fun setPivotPosition(position: Position): Command =
        run {
            io.setPivotAngle(position.angle)
        }

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

    fun intakeSequence(): Command =
        Commands.parallel(
            Commands.runEnd(
                { io.setPivotAngle(Position.Deployed.angle) },
                { io.setPivotAngle(Position.Stowed.angle) }
            ),
//            Commands.run({ io.setPivotAngle(Position.Deployed.angle) }),
            intake()
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
                { io.setWheelMotorVoltage(4.0.volts) },
                { io.setWheelMotorVoltage(0.volts) }
            )

    fun outtake(): Command = runEnd(
        { io.setWheelMotorVoltage((-5.0).volts) },
        { io.setWheelMotorVoltage(0.volts) },
    )

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }
}
