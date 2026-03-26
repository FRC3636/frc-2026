package com.frcteam3636.frc2026.subsystems.climber

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.frcteam3636.frc2026.CANcoder
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.PIDController
import com.frcteam3636.frc2026.utils.math.PIDGains
import com.frcteam3636.frc2026.utils.math.amps
import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.volts
import com.frcteam3636.frc2026.utils.math.inMeters
import com.frcteam3636.frc2026.utils.math.inVolts
import com.frcteam3636.frc2026.utils.math.inches
import com.frcteam3636.frc2026.utils.math.meters
import com.frcteam3636.frc2026.utils.math.metersPerSecond
import com.frcteam3636.frc2026.utils.math.toAngular
import com.frcteam3636.frc2026.utils.math.toLinear
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.team9432.annotation.Logged
import com.frcteam3636.frc2026.utils.math.pidGains
import org.littletonrobotics.junction.Logger

@Logged
open class ClimberInputs {
    var height = 0.meters
    var current = 0.amps
    var velocity = 0.metersPerSecond
}

interface ClimberIO {
    fun setVoltage(volts: Voltage)
    fun goToHeight(targetHeight: Distance)
    fun setEncoderPosition(position: Distance)
    fun updateInputs(inputs: ClimberInputs)
}

class ClimberIOReal : ClimberIO {
    internal companion object Constants {
        private val SPOOL_RADIUS = 0.25.inches // Half inch diameter hex shaft
        private val PID_GAINS = PIDGains(140.0, 0.01, 0.0)
        private const val MOTOR_TO_ENCODER_GEAR_RATIO = 20.0
    }

    private val motorConfig = TalonFXConfiguration()
    private val motor = TalonFX(CTREDeviceId.ClimberMotor)
    private val encoder =  CANcoder(CTREDeviceId.ClimberEncoder).apply {
        configurator.apply(CANcoderConfiguration().apply {
            MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive
        })
    }
    private val pidController = PIDController(PID_GAINS)

    init {
        motorConfig.apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
            }

            Slot0.apply {
                pidGains = PID_GAINS
            }

            Feedback.apply {
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
                FeedbackRemoteSensorID = CTREDeviceId.ClimberEncoder.num
                SensorToMechanismRatio = 1.0
                RotorToSensorRatio = MOTOR_TO_ENCODER_GEAR_RATIO // Encoder is on spool
            }

            MotorOutput.apply {
                Inverted = InvertedValue.Clockwise_Positive
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

    override fun goToHeight(targetHeight: Distance) {
        Logger.recordOutput("/Climb/TargetHeight", targetHeight)
        val currentHeight = height
        setVoltage(pidController.calculate(targetHeight.inMeters(), currentHeight.inMeters()).volts);
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
        private val PID_GAINS = PIDGains(5.0, 0.0, 0.0) // Not measured, approximation
    }

    private val motor = DCMotor.getKrakenX60(1)
    private val system = LinearSystemId.createDCMotorSystem(motor, 1.0,1.0)
    private val sim = DCMotorSim(system, motor)

    private val pid = PIDController(PID_GAINS)

    override fun setVoltage(volts: Voltage) {
        sim.inputVoltage = volts.inVolts()
    }

    override fun goToHeight(targetHeight: Distance) {
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
