package com.frcteam3636.frc2026.subsystems.climb

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.PIDGains
import com.frcteam3636.frc2026.utils.math.amps
import com.frcteam3636.frc2026.utils.math.inRotationsPerSecond
import com.frcteam3636.frc2026.utils.math.inVolts
import com.frcteam3636.frc2026.utils.math.inches
import com.frcteam3636.frc2026.utils.math.inchesPerSecond
import com.frcteam3636.frc2026.utils.math.meters
import com.frcteam3636.frc2026.utils.math.metersPerSecond
import com.frcteam3636.frc2026.utils.math.pidGains
import com.frcteam3636.frc2026.utils.math.toAngular
import com.frcteam3636.frc2026.utils.math.toLinear
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged

@Logged
open class ClimbInputs {
    var height = 0.meters
    var current = 0.amps
    var velocity = 0.metersPerSecond
}

interface ClimbIO {
    fun setVoltage(volts: Voltage)
    fun goToHeight(height: Distance, slow: Boolean = false)
    fun setEncoderPosition(position: Distance)
    fun updateInputs(inputs: ClimbInputs)
}

class ClimbIOReal : ClimbIO {
    internal companion object Constants {
        private val SPOOL_RADIUS = 1.0.inches // Not measured, approximation
        private val PID_GAINS = PIDGains(30.0, 0.0, 0.0) // Not measured, approximation
        private val ROTOR_TO_MECHANISM_GEAR_RATIO = 8.0 // Not sure what this does, taken from ElevatorIO@frc-2025

        // Below values taken from frc-2025, again, need to be tuned.
        private val FAST_PROFILE_VELOCITY = 100.inchesPerSecond.toAngular(SPOOL_RADIUS)
        private val FAST_PROFILE_ACCELERATION = 50.0
        private val FAST_PROFILE_JERK = 0.0
        private val SLOW_PROFILE_VELOCITY = 10.inchesPerSecond.toAngular(SPOOL_RADIUS)
        private val SLOW_PROFILE_ACCELERATION = 2.0
        private val SLOW_PROFILE_JERK = 0.0
    }

    private val motorConfig = TalonFXConfiguration()
    private val motor = TalonFX(CTREDeviceId.ClimbMotor)

    init {
        motorConfig.apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
            }

            Slot0.apply {
                pidGains = PID_GAINS
                // Gravity and FF gains?
            }

            Feedback.apply {
                SensorToMechanismRatio = ROTOR_TO_MECHANISM_GEAR_RATIO
            }
        }
        motor.configurator.apply(motorConfig)

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            motor.position,
            motor.velocity,
            motor.supplyCurrent
        )
        motor.optimizeBusUtilization()
    }

    override fun setVoltage(volts: Voltage) {
        motor.setVoltage(volts.inVolts())
    }

    override fun goToHeight(height: Distance, slow: Boolean) {
        Logger.recordOutput("Climb/Height Setpoint", height)
        motor.setControl(getMotionMagicVoltage(slow).withPosition(height.toAngular(SPOOL_RADIUS)))
    }

    override fun setEncoderPosition(position: Distance) {
        motor.setPosition(position.toAngular(SPOOL_RADIUS))
    }

    override fun updateInputs(inputs: ClimbInputs) {
        inputs.height = motor.getPosition(false).value.toLinear(SPOOL_RADIUS)
        inputs.velocity = motor.getVelocity(false).value.toLinear(SPOOL_RADIUS)
        inputs.current = motor.getSupplyCurrent(false).value
    }

    private fun getMotionMagicVoltage(slow: Boolean): DynamicMotionMagicVoltage {
        if (slow) {
            return DynamicMotionMagicVoltage(0.0, SLOW_PROFILE_VELOCITY.inRotationsPerSecond(), SLOW_PROFILE_ACCELERATION).withJerk(SLOW_PROFILE_JERK)
        }
        return DynamicMotionMagicVoltage(0.0, FAST_PROFILE_VELOCITY.inRotationsPerSecond(), FAST_PROFILE_ACCELERATION).withJerk(FAST_PROFILE_JERK)
    }
}

