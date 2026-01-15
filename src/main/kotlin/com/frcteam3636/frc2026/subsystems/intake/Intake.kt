package com.frcteam3636.frc2026.subsystems.intake

import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import javax.swing.text.Position

object Intake : Subsystem {

    enum class Position(val angle: Angle) {
        Stowed(0.degrees),
        Deployed(0.degrees),
    }

    private val io: IntakeIO = IntakeIOReal()

    fun setPivotPosition(position: Position): Command =
        runOnce {
            Logger.recordOutput("Intake/Pivot/Active Setpoint", position.angle)
            io.setPivotAngle(position.angle)
        }

    fun intake(): Command = runEnd(
        {
            io.setRunMotorVoltage(0.volts)
            io.setPivotAngle(Position.Deployed.angle)
        },
        {
            io.setRunMotorVoltage(0.volts)
            io.setPivotAngle(Position.Deployed.angle)
        }
    )



}
