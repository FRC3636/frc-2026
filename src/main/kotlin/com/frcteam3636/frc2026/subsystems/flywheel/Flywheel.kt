package com.frcteam3636.frc2026.subsystems.flywheel

import com.frcteam3636.frc2026.Robot
import com.frcteam3636.frc2026.utils.math.volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem

object Flywheel: Subsystem {

    private val io: FlywheelIO = when (Robot.model) {
        Robot.Model.SIMULATION -> TODO()
        Robot.Model.COMPETITION -> FlywheelIOReal()
    }

    var inputs: FlywheelInputs = FlywheelInputs()

    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun setVoltage(volts: Voltage): Command = startEnd(
        {
            io.setMotorVoltage(volts)
        },
        {
            io.setMotorVoltage(0.volts)
        }
    )
}