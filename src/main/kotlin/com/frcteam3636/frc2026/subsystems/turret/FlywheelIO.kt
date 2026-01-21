package com.frcteam3636.frc2026.subsystems.flywheel

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.utils.math.volts
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.amps
import com.frcteam3636.frc2026.utils.math.inVolts
import com.frcteam3636.frc2026.utils.math.radiansPerSecond

@Logged
open class FlywheelInputs {
    var motorVolts = 0.volts
    var angularVelocity = 0.radiansPerSecond
}

interface FlywheelIO {
    fun updateInputs(inputs: FlywheelInputs)
    fun setMotorVoltage(volts: Voltage)
    fun setSpeed(percentage: Double)
}

class FlywheelIOReal : FlywheelIO {

    private val motor = TalonFX(CTREDeviceId.FlywheelMotor).apply {
         val config = TalonFXConfiguration().apply {
            withCurrentLimits(CurrentLimitsConfigs().apply {
                withSupplyCurrentLimit(30.amps)
            })
         }
        configurator.apply(config)
    }

    override fun updateInputs(inputs: FlywheelInputs) {
        inputs.motorVolts = motor.motorVoltage.value
        inputs.angularVelocity = motor.velocity.value
    }

    override fun setMotorVoltage(volts: Voltage) {
        motor.setVoltage(volts.inVolts())
    }

    override fun setSpeed(percentage: Double) {
        motor.set(percentage)
    }
}