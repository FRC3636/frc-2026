package com.frcteam3636.frc2026.subsystems.climber

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.PIDController
import com.frcteam3636.frc2026.utils.math.PIDGains
import com.frcteam3636.frc2026.utils.math.amps
import com.frcteam3636.frc2026.utils.math.volts
import com.frcteam3636.frc2026.utils.math.inMeters
import com.frcteam3636.frc2026.utils.math.inRotationsPerSecond
import com.frcteam3636.frc2026.utils.math.inVolts
import com.frcteam3636.frc2026.utils.math.inches
import com.frcteam3636.frc2026.utils.math.inchesPerSecond
import com.frcteam3636.frc2026.utils.math.meters
import com.frcteam3636.frc2026.utils.math.metersPerSecond
import com.frcteam3636.frc2026.utils.math.metersPerSecondPerSecond
import com.frcteam3636.frc2026.utils.math.pidGains
import com.frcteam3636.frc2026.utils.math.toAngular
import com.frcteam3636.frc2026.utils.math.toLinear
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged
import com.frcteam3636.frc2026.CANcoder
import com.ctre.phoenix6.configs.CANcoderConfiguration

@Logged
open class ClimberInputs {
    var height = 0.meters
    var current = 0.amps
    var velocity = 0.metersPerSecond
}

interface ClimberIO {
    fun setVoltage(volts: Voltage)
    fun goToHeight(height: Distance, slow: Boolean = false)
    fun setEncoderPosition(position: Distance)
    fun updateInputs(inputs: ClimberInputs)
}

class ClimberIOReal : ClimberIO {
    internal companion object Constants {
        private val SPOOL_RADIUS = 1.0.inches // Not measured, approximation
        private val PID_GAINS = PIDGains(5.0, 0.0, 0.0) // Not measured, approximation
        private val ROTOR_TO_MECHANISM_GEAR_RATIO = 0.1 // Not sure what this does, taken from ElevatorIO@frc-2025

        // Below values taken from frc-2025, again, need to be tuned.
        private val FAST_PROFILE_VELOCITY = 1.metersPerSecond.toAngular(SPOOL_RADIUS)
        private val FAST_PROFILE_ACCELERATION = 2.0.metersPerSecondPerSecond.toAngular(SPOOL_RADIUS)
        private val FAST_PROFILE_JERK = 0.0
        private val SLOW_PROFILE_VELOCITY = 0.1.metersPerSecond.toAngular(SPOOL_RADIUS)
        private val SLOW_PROFILE_ACCELERATION = 0.2.metersPerSecondPerSecond.toAngular(SPOOL_RADIUS)
        private val SLOW_PROFILE_JERK = 0.0
    }

    private val motorConfig = TalonFXConfiguration()
    private val motor = TalonFX(CTREDeviceId.ClimbMotor)
    private val encoder = CANcoder(CTREDeviceId.ClimbEncoder)
    private val pid_controller = PIDController(PID_GAINS)

    init {
        motorConfig.apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Coast
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

    val height: Distance
        get() = encoder.position.value.toLinear(SPOOL_RADIUS)

    override fun setVoltage(volts: Voltage) {
        motor.setVoltage(volts.inVolts())
    }

    override fun goToHeight(height: Distance, slow: Boolean) {
        Logger.recordOutput("Climb/Height Setpoint", height)
        val current_height = height
        setVoltage(pid_controller.calculate(height.inMeters(), current_height.inMeters()).volts);
        // motor.setControl(getMotionMagicVoltage(slow).withPosition(height.toAngular(SPOOL_RADIUS)))
    }

    override fun setEncoderPosition(position: Distance) {
        encoder.setPosition(position.toAngular(SPOOL_RADIUS))
    }

    override fun updateInputs(inputs: ClimberInputs) {
        inputs.height = height
        inputs.velocity = encoder.getVelocity(false).value.toLinear(SPOOL_RADIUS)
        inputs.current = motor.supplyCurrent.value
    }

    // private fun getMotionMagicVoltage(slow: Boolean): DynamicMotionMagicVoltage {
    //     if (slow) {
    //         return DynamicMotionMagicVoltage(0.0, SLOW_PROFILE_VELOCITY.inRotationsPerSecond(), SLOW_PROFILE_ACCELERATION).withJerk(SLOW_PROFILE_JERK)
    //     }
    //     return DynamicMotionMagicVoltage(0.0, FAST_PROFILE_VELOCITY.inRotationsPerSecond(), FAST_PROFILE_ACCELERATION).withJerk(FAST_PROFILE_JERK)
    // }
}

class ClimberIOSim : ClimberIO {
    internal companion object Constants {
        private val SPOOL_RADIUS = 1.0.inches // Not measured, approximation
        private val PID_GAINS = PIDGains(30.0, 0.0, 0.0) // Not measured, approximation
    }

    private val motor = DCMotor.getKrakenX60(1)
    private val system = LinearSystemId.createDCMotorSystem(motor, 1.0,1.0)
    private val sim = DCMotorSim(system, motor)

    private val pid = PIDController(PID_GAINS)

    override fun setVoltage(volts: Voltage) {
        sim.inputVoltage = volts.inVolts()
    }

    override fun goToHeight(height: Distance, slow: Boolean) {
        sim.inputVoltage = pid.calculate(sim.angularPosition.toLinear(SPOOL_RADIUS).inMeters())
    }

    override fun setEncoderPosition(position: Distance) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: ClimberInputs) {
        inputs.height = sim.angularPosition.toLinear(SPOOL_RADIUS)
        inputs.velocity = sim.angularVelocity.toLinear(SPOOL_RADIUS)
        inputs.current = sim.currentDrawAmps.amps
    }
}
