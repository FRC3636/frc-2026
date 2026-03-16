package com.frcteam3636.frc2026.subsystems.intake

import com.frcteam3636.frc2026.robot.Robot
import com.frcteam3636.frc2026.utils.math.amps
import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.rotations
import com.frcteam3636.frc2026.utils.math.seconds
import com.frcteam3636.frc2026.utils.math.volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Intake : Subsystem {

    var intakeDown = false

    enum class Position(val angle: Angle) {
        Stowed(0.degrees),
        Deployed(0.87.rotations),
    }

    private val io: IntakeIO =
        when (Robot.model) {
        Robot.Model.SIMULATION -> IntakeIOReal()
        Robot.Model.COMPETITION -> IntakeIOReal()

    }

    private val inputs = LoggedIntakeInputs()

    fun setPivotPosition(position: Position): Command =
        runOnce {
            Logger.recordOutput("Intake/Pivot/Active Setpoint", position.angle)
            io.setPivotAngle(position.angle)
        }

    fun setPercent(percent: Double): Command = Commands.runEnd(
        {io.setPivotSpeed(percent)},
        {io.setPivotSpeed(0.0)}
    )

    fun intake(): Command =
            runEnd(
                { io.setWheelMotorVoltage(5.0.volts) },
                { io.setWheelMotorVoltage(0.volts) }
            )

    fun outtake(): Command = runEnd(
        { io.setWheelMotorVoltage((-5.0).volts) },
        { io.setWheelMotorVoltage(0.volts) },
    )

    fun pivot(): Command = startEnd(
        { io.setPivotAngle(Position.Deployed.angle) },
        { io.setPivotAngle(Position.Stowed.angle) }
    ).onlyWhile { !intakeDown }

    fun pivotVoltage(): Command = startEnd(
        {
            Commands.sequence(
                run { io.setPivotMotorVoltage(3.6.volts) }.until { inputs.intakePivotMotorCurrent > 50.0.amps },
                run { io.setPivotMotorVoltage(1.0.volts) }
              )
        },
        {
            run { io.setPivotMotorVoltage(-1.0.volts) }
        }
    )

    fun maintainPosition(): Command = run {
        if (intakeDown) {
            io.setPivotMotorVoltage(-0.5.volts)
        } else {
            io.setPivotMotorVoltage(0.5.volts)
        }
    }

    fun deploy(): Command = runPivotVoltage(-1.0.volts).withTimeout(2.0.seconds)

    fun stow(): Command = runPivotVoltage(3.6.volts).withTimeout(2.0.seconds)

    fun runPivotVoltage(voltage: Voltage): Command = Commands.runEnd({io.setPivotMotorVoltage(voltage)}, {io.setPivotMotorVoltage(0.0.volts)}).until { inputs.intakePivotMotorCurrent > 50.0.amps }

//    val IntakeSimulation
//        get() = if (io is IntakeIOSim) { io.intakeSimulation }
//        else throw Throwable("Cannot access intake simulation out of sim")


    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }
}
