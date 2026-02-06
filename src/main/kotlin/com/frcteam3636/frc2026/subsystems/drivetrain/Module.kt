package com.frcteam3636.frc2026.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.REVMotorControllerId
import com.frcteam3636.frc2026.SparkMax
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.*
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.FeedbackSensor
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.FeedForwardConfig
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.Logger
import kotlin.math.PI
import kotlin.math.roundToInt

interface SwerveModule {
    // This is the speed and the angle of the module.
    val state: SwerveModuleState
    // This is the wheel velocity that we're trying to get to.
    var desiredState: SwerveModuleState
    // This is the measured position of the module, a vector with direction equal to the current angle of the module,
    // and magnitude equal to the total signed distance traveled by the wheel.
    val position: SwerveModulePosition

    fun periodic() {}
    fun characterize(voltage: Voltage, measure: Angle?)
    fun getSignals(): Array<BaseStatusSignal> { return arrayOf() }
}

class GeneralSwerveModule(
    private val drivingMotor: DrivingMotor, private val turningMotor: TurningMotor, private val chassisAngle: Rotation2d
): SwerveModule {

    override val state: SwerveModuleState
        get() = SwerveModuleState(
            drivingMotor.velocity.inMetersPerSecond(), Rotation2d.fromRadians(turningMotor.position.inRadians()) + chassisAngle
        )

    override val position: SwerveModulePosition
        get() = SwerveModulePosition(
            drivingMotor.position, Rotation2d.fromRadians(turningMotor.position.inRadians()) + chassisAngle
        )

    override fun characterize(voltage: Voltage, measure: Angle?) {
        drivingMotor.setVoltage(voltage)
        turningMotor.setPosition(Radians.of(-chassisAngle.radians))
    }

    override var desiredState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d())
        get() = SwerveModuleState(field.speedMetersPerSecond, field.angle + chassisAngle)
        set(value) {
            // corrected is the module-relative angle
            val corrected = SwerveModuleState(value.speedMetersPerSecond, value.angle - chassisAngle)
            // optimize the state to avoid rotating more than 90 degrees
            corrected.optimize(Rotation2d.fromRadians(turningMotor.position.inRadians()))

            drivingMotor.setVelocity(corrected.speedMetersPerSecond.metersPerSecond)
            turningMotor.setPosition(Radians.of(corrected.angle.radians))

            field = corrected
        }

    override fun getSignals(): Array<BaseStatusSignal> {
        return turningMotor.getSignals() + drivingMotor.getSignals()
    }
}

interface DrivingMotor {
    val position: Distance
    val velocity: LinearVelocity
    fun setVoltage(voltage: Voltage)
    fun setVelocity(velocity: LinearVelocity)
    fun getSignals(): Array<BaseStatusSignal> {
        return arrayOf()
    }
}

interface TurningMotor {
    val position: Angle
    fun setPosition(position: Angle)
    fun getSignals(): Array<BaseStatusSignal> {
        return arrayOf()
    }
}

//This is a Kraken
class DrivingTalon(id: CTREDeviceId) : DrivingMotor {

    private val inner = TalonFX(id).apply {
        configurator.apply(
            TalonFXConfiguration().apply {
                Slot0.apply {
                    pidGains = DRIVING_PID_GAINS_TALON
                    motorFFGains = DRIVING_FF_GAINS_TALON
                }
                CurrentLimits.apply {
                    SupplyCurrentLimit = DRIVING_CURRENT_LIMIT.inAmps()
                    SupplyCurrentLimitEnable = true
                }
            }
        )
    }

    init {
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, inner.position, inner.velocity)
        inner.optimizeBusUtilization()
    }

    override val position: Distance
        get() = inner.position.value.toLinear(WHEEL_RADIUS) * DRIVING_GEAR_RATIO_TALON

    private var velocityControl = VelocityVoltage(0.0).apply {
        EnableFOC = true
    }

    override val velocity: LinearVelocity
        get() = inner.velocity.value.toLinear(WHEEL_RADIUS) * DRIVING_GEAR_RATIO_TALON

    override fun setVelocity(velocity: LinearVelocity) {
        inner.setControl(velocityControl.withVelocity(velocity.toAngular(WHEEL_RADIUS) / DRIVING_GEAR_RATIO_TALON))
    }

    private val voltageControl = VoltageOut(0.0).apply {
        EnableFOC = true
    }

    override fun setVoltage(voltage: Voltage) {
        inner.setControl(voltageControl.withOutput(voltage.inVolts()))
    }

    override fun getSignals(): Array<BaseStatusSignal> {
        return arrayOf(inner.getPosition(false), inner.getVelocity(false))
    }
}

//This is a Neo
class DrivingSparkMAX(private val id: REVMotorControllerId) : DrivingMotor {

    private val inner = SparkMax(id, SparkLowLevel.MotorType.kBrushless).apply {
        val innerConfig = SparkMaxConfig().apply {
            idleMode(IdleMode.kBrake)
            smartCurrentLimit(DRIVING_CURRENT_LIMIT.inAmps().toInt())
            inverted(false)

            encoder.apply {
                positionConversionFactor(WHEEL_CIRCUMFERENCE.inMeters() / DRIVING_GEAR_RATIO_NEO)
                velocityConversionFactor(WHEEL_CIRCUMFERENCE.inMeters() / DRIVING_GEAR_RATIO_NEO / 60.0)
            }

            closedLoop.apply {
                pid(DRIVING_PID_GAINS_NEO.p, DRIVING_PID_GAINS_NEO.i, DRIVING_PID_GAINS_NEO.d)
//                velocityFF(DRIVING_FF_GAINS_NEO.v)
                feedForward = FeedForwardConfig().apply { DRIVING_FF_GAINS_NEO } // Possibly completely incorrect
                feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            }
        }
        configure(innerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override val position: Distance
        get() = inner.absoluteEncoder.position.meters

    override val velocity: LinearVelocity
        get() = inner.encoder.velocity.metersPerSecond

    override fun setVelocity(velocity: LinearVelocity) {
        Logger.recordOutput("/Drivetrain/$id/OutputVel", velocity)
        inner.closedLoopController.setSetpoint(velocity.inMetersPerSecond(), SparkBase.ControlType.kVelocity)
    }

    override fun setVoltage(voltage: Voltage) {
        inner.setVoltage(voltage.inVolts())
    }
}

//This is for a Mk5n swerve module
class TurningTalon(id: CTREDeviceId, encoderID: CTREDeviceId, magnetOffset: Double): TurningMotor {

    private val encoder = com.ctre.phoenix6.hardware.CANcoder(encoderID.num, encoderID.bus).apply {
        configurator.apply(
            CANcoderConfiguration().apply {
                MagnetSensor.MagnetOffset = magnetOffset
            }
        )
    }

    private val inner = TalonFX(id).apply {
        configurator.apply(TalonFXConfiguration().apply {
            Slot0.apply {
                pidGains = TURNING_PID_GAINS_TALON
                motorFFGains = TURNING_FF_GAINS_TALON
            }
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
            }
            Feedback.apply {
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
                SensorToMechanismRatio = TURNING_CANCODER_TO_MECHANISM_RATIO
                RotorToSensorRatio = TURNING_MOTOR_TO_MECHANISM_RATIO
                FeedbackRemoteSensorID = encoderID.num
            }
            CurrentLimits.apply {
                StatorCurrentLimit = TURNING_CURRENT_LIMIT.inAmps()
                StatorCurrentLimitEnable = true
            }
        })
    }

    init {
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, inner.position)
        inner.optimizeBusUtilization()
    }

    override val position: Angle
        get() = Radians.of( inner.position.value.inRadians() * 2 * PI)

    override fun setPosition(position: Angle) {
        inner.setControl(positionControl.withPosition(position.inRadians() / TAU))
    }

    private val positionControl = PositionVoltage(0.0).apply {
        EnableFOC = true
    }

    override fun getSignals(): Array<BaseStatusSignal> {
        return arrayOf(inner.getPosition(false))
    }
}

//This is for a MAX swerve module
class TurningSparkMax(id: REVMotorControllerId): TurningMotor {

    private val inner = SparkMax(id, SparkLowLevel.MotorType.kBrushless).apply {
        configure(
            SparkMaxConfig().apply {
                idleMode(IdleMode.kBrake)
                smartCurrentLimit(TURNING_CURRENT_LIMIT.inAmps().roundToInt())

                absoluteEncoder.apply {
                    inverted(true)
                    positionConversionFactor(TAU)
                    velocityConversionFactor(TAU / 60)
                }

                closedLoop.apply {
                    pid(TURNING_PID_GAINS_NEO.p, TURNING_PID_GAINS_NEO.i, TURNING_PID_GAINS_NEO.d)
                    feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    positionWrappingEnabled(true)
                    positionWrappingMinInput(0.0)
                    positionWrappingMaxInput(TAU)
                }
            },
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        )
    }

    override val position: Angle
        get() {
            val rawPosition = inner.absoluteEncoder.position
            val correctedPosition = (rawPosition).mod(TAU)
            return Radians.of(correctedPosition)
        }

    override fun setPosition(position: Angle) {
        inner.closedLoopController.setSetpoint(
            (position.inRadians()).mod(TAU),
            SparkBase.ControlType.kPosition
        )
    }

    override fun getSignals(): Array<BaseStatusSignal>{
        return arrayOf()
    }

}

class SimSwerveModule(
    override val state: SwerveModuleState,
    override var desiredState: SwerveModuleState,
    override val position: SwerveModulePosition
) : SwerveModule {
    // TODO("Not yet implemented")
    override fun characterize(voltage: Voltage, measure: Angle?) {
        TODO("Not yet implemented")
    }
}

// Constants
internal val WHEEL_RADIUS = 1.5.inches
internal val WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * TAU

internal const val DRIVING_GEAR_RATIO_TALON = 1.0 / 3.56

private const val DRIVING_MOTOR_PINION_TEETH = 14
const val DRIVING_GEAR_RATIO_NEO = (45.0 * 22.0) / (DRIVING_MOTOR_PINION_TEETH * 15.0)

const val TURNING_CANCODER_TO_MECHANISM_RATIO = 1.0 // TODO: Fix values
const val TURNING_MOTOR_TO_MECHANISM_RATIO = 1.0

internal val NEO_FREE_SPEED = 5676.rpm
internal val NEO_DRIVING_FREE_SPEED = NEO_FREE_SPEED.toLinear(WHEEL_CIRCUMFERENCE) / DRIVING_GEAR_RATIO_NEO

internal val DRIVING_PID_GAINS_TALON: PIDGains = PIDGains(.19426, 0.0)
internal val DRIVING_PID_GAINS_NEO: PIDGains = PIDGains(0.04, 0.0, 0.0)
internal val DRIVING_FF_GAINS_TALON: MotorFFGains = MotorFFGains(0.22852, 0.1256, 0.022584)
internal val DRIVING_FF_GAINS_NEO: MotorFFGains = MotorFFGains(0.0, 1 / NEO_DRIVING_FREE_SPEED.inMetersPerSecond(), 0.0) // TODO: ensure this is right

internal val TURNING_PID_GAINS_NEO: PIDGains = PIDGains(1.7, 0.0, 0.125)
internal val TURNING_FF_GAINS_NEO: MotorFFGains = MotorFFGains(0.1, 2.66, 0.0) // TODO: I'm pretty sure we want only a PID controller on a turning motor
internal val TURNING_PID_GAINS_TALON: PIDGains = PIDGains(1.7, 0.0, 0.125)
internal val TURNING_FF_GAINS_TALON: MotorFFGains = MotorFFGains(0.1, 2.66, 0.0) // TODO: fix values

internal val DRIVING_CURRENT_LIMIT = 37.amps
internal val TURNING_CURRENT_LIMIT = 20.amps