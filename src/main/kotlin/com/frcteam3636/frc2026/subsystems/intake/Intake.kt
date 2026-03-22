package com.frcteam3636.frc2026.subsystems.intake

import com.frcteam3636.frc2026.robot.Robot
import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.radians
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
        Deployed(7.degrees),
        Stowed(40.degrees),
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

    fun setVoltage(volts: Voltage): Command = Commands.runEnd(
        {io.setVoltage(volts)},
        {io.setVoltage(0.volts)}
    )

    fun intakeSequence(): Command = runEnd(
        {
            Commands.run({ io.setVoltage(-5.volts) }).withTimeout(1.0.seconds)
            io.setWheelMotorVoltage(6.volts)
        },
        {
            io.setVoltage(0.volts)
            io.setWheelMotorVoltage(0.volts)
        },
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
    ).onlyWhile { intakeDown }

//    val IntakeSimulation
//        get() = if (io is IntakeIOSim) { io.intakeSimulation }
//        else throw Throwable("Cannot access intake simulation out of sim")


    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }
}
