package com.frcteam3636.frc2026.subsystems.intake

import com.frcteam3636.frc2026.Robot
import com.frcteam3636.frc2026.subsystems.shooter.LoggedTurretInputs
import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Intake : Subsystem {

    var intakeDown = false

    enum class Position(val angle: Angle) {
        Stowed(0.degrees),
        Deployed(0.degrees),
    }

    private val io: IntakeIO =
        when (Robot.model) {
        Robot.Model.SIMULATION -> IntakeIOSim()
        Robot.Model.COMPETITION -> IntakeIOReal()

    }

    private val inputs = LoggedIntakeInputs()

    fun setPivotPosition(position: Position): Command =
        runOnce {
            Logger.recordOutput("Intake/Pivot/Active Setpoint", position.angle)
            io.setPivotAngle(position.angle)
        }

    fun intake(): Command =
        if (io is IntakeIOSim) {
            runEnd(
                {io.setRunning(true)},
                {io.setRunning(false)}
            )
        } else {
            runEnd(
                { io.setRunMotorVoltage(1.0.volts) },
                { io.setRunMotorVoltage(0.volts) }
            )
        }

    fun outtake(): Command = runEnd(
        { io.setRunMotorVoltage(-1.0.volts) },
        { io.setRunMotorVoltage(0.volts) }
    )

    fun pivot(): Command = startEnd(
        { io.setPivotAngle(Position.Deployed.angle) },
        { io.setPivotAngle(Position.Stowed.angle) }
    ).onlyWhile { intakeDown }

    val IntakeSimulation
        get() = if (io is IntakeIOSim) { io.intakeSimulation }
        else throw Throwable("Cannot access intake simulation out of sim")


    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }
}
