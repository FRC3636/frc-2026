package com.frcteam3636.frc2026.subsystems.flywheel

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.utils.math.volts
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.MotorFFGains
import com.frcteam3636.frc2026.utils.math.PIDGains
import com.frcteam3636.frc2026.utils.math.SimpleMotorFeedforward
import com.frcteam3636.frc2026.utils.math.amps
import com.frcteam3636.frc2026.utils.math.inMeters
import com.frcteam3636.frc2026.utils.math.inRPM
import com.frcteam3636.frc2026.utils.math.inRadiansPerSecond
import com.frcteam3636.frc2026.utils.math.inVolts
import com.frcteam3636.frc2026.utils.math.meters
import com.frcteam3636.frc2026.utils.math.metersPerSecond
import com.frcteam3636.frc2026.utils.math.metersPerSecondPerSecond
import com.frcteam3636.frc2026.utils.math.pidGains
import com.frcteam3636.frc2026.utils.math.radiansPerSecond
import com.frcteam3636.frc2026.utils.math.rpm
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Velocity

@Logged
open class FlywheelInputs {
    var motorVolts = 0.volts
    var angularVelocity = 0.radiansPerSecond
    var linearVelocity = MetersPerSecond.zero()!!
}

interface FlywheelIO {
    fun updateInputs(inputs: FlywheelInputs)
    fun setMotorVoltage(volts: Voltage)
    fun setSpeed(percentage: Double)
    fun setVelocity(velocity: AngularVelocity)
}

class FlywheelIOReal : FlywheelIO {

    private val motor = TalonFX(CTREDeviceId.FlywheelMotor).apply {
         val config = TalonFXConfiguration().apply {
            withCurrentLimits(CurrentLimitsConfigs().apply {
                withSupplyCurrentLimit(30.amps)
            }).withSlot0(
                Slot0.apply {
                    pidGains = PID_GAINS
                })
            )
         }
        configurator.apply(config)
    }

    private val flywheelController = SimpleMotorFeedforward(FF_GAINS)

    override fun updateInputs(inputs: FlywheelInputs) {
        inputs.motorVolts = motor.motorVoltage.value
        inputs.angularVelocity = motor.velocity.value
        inputs.linearVelocity = (motor.velocity.value.inRadiansPerSecond() * flywheelRadius.inMeters()).metersPerSecond
    }

    override fun setMotorVoltage(volts: Voltage) {
        motor.setVoltage(volts.inVolts())
    }

    override fun setSpeed(percentage : Double) {
        motor.set(percentage)
    }

    override fun setVelocity(velocity: AngularVelocity){
        assert(velocity in 0.rpm..6000.rpm)

    }

    companion object Constants{
        val flywheelRadius = 0.0505.meters
        val FF_GAINS = MotorFFGains(0.1, 0.0001, 0.0001)
        val PID_GAINS = PIDGains(5.0,0.0,0.0)
    }
}