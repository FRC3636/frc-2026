package com.frcteam3636.frc2026.subsystems.feeder

import com.frcteam3636.frc2026.Robot
import com.frcteam3636.frc2026.utils.math.volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem

object Feeder: Subsystem {

    private val io: FeederIO = when (Robot.model) {
        Robot.Model.SIMULATION -> TODO()
        Robot.Model.COMPETITION -> FeederIOReal()
    }

    var inputs: FeederInputs = FeederInputs()

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

    fun feed() : Command {
        return runEnd({
            setVoltage(10.0.volts)
        },{
            setVoltage(0.0.volts)
        })
    }
}