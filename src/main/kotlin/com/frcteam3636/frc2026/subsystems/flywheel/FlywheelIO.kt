package com.frcteam3636.frc2026.subsystems.flywheel

import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.utils.math.volts
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged
import com.frcteam3636.frc2026.TalonFX

@Logged
open class FlywheelInputs {
    var motorVolts = 0.volts
}

interface FlywheelIO {
    fun updateInputs(inputs: FlywheelInputs)
    fun setMotorVoltage(volts: Voltage)
}

class FlywheelIOReal : FlywheelIO {

    private val motor = TalonFX()

    override fun updateInputs(inputs: FlywheelInputs) {
        TODO("Not yet implemented")
    }

    override fun setMotorVoltage(volts: Voltage) {
        TODO("Not yet implemented")
    }
}