package com.frcteam3636.frc2026.subsystems.flywheel

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.configs.VoltageConfigs
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2026.CTREDeviceId
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.*
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Velocity
import org.littletonrobotics.junction.inputs.LoggableInputs

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

    private val flyWheelMotor = TalonFX(CTREDeviceId.FlywheelMotor).apply {
        configurator.apply(TalonFXConfiguration().apply{
            MotorOutput.apply {
                Inverted = InvertedValue.Clockwise_Positive
                NeutralMode = NeutralModeValue.Coast
            }

            CurrentLimits.apply {
                SupplyCurrentLimit = 40.0
            }

            Slot0.apply{
                pidGains = PID_GAINS
            }

            MotionMagic.apply {
                MotionMagicCruiseVelocity = PROFILE_VELOCITY.inRotationsPerSecond()
                MotionMagicAcceleration = PROFILE_ACCELERATION.inRotationsPerSecondPerSecond()
                MotionMagicJerk = PROFILE_JERK
            }

        })
    }
//    private val flyWheelMotor = TalonFX(CTREDeviceId.FlywheelMotor).apply {
//         val config = TalonFXConfiguration().apply {
//            withCurrentLimits(CurrentLimitsConfigs().apply {
//                withSupplyCurrentLimit(30.amps)
//            }).withSlot0(
//                Slot0.apply {
//                    pidGains = PID_GAINS
//                }).withMotionMagic(
//                MotionMagicVelocityVoltage
//                )
//         }
//        configurator.apply(config)
//    }


    override fun updateInputs(inputs: FlywheelInputs) {
        inputs.motorVolts = flyWheelMotor.motorVoltage.value
        inputs.angularVelocity = flyWheelMotor.velocity.value
        inputs.linearVelocity = (flyWheelMotor.velocity.value.inRadiansPerSecond() * flywheelRadius.inMeters()).metersPerSecond
    }

    override fun setMotorVoltage(volts: Voltage) {
        flyWheelMotor.set(volts.inVolts())
    }

    override fun setSpeed(percentage : Double) {
        flyWheelMotor.set(percentage)
    }

    override fun setVelocity(velocity: AngularVelocity){
        assert(velocity in 0.rpm..6000.rpm)
        flyWheelMotor.setControl(MotionMagicVelocityVoltage(velocity.inRPM()))
    }

    companion object Constants{
        val flywheelRadius = 0.0505.meters
        val PID_GAINS = PIDGains(5.0,0.0,0.0)
        val PROFILE_ACCELERATION = 2.0.rotationsPerSecondPerSecond
        val PROFILE_VELOCITY = 2.0.rotationsPerSecond
        val PROFILE_JERK = 1.0
  }
}