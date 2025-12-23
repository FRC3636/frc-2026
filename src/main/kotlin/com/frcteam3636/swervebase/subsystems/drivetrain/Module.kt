package com.frcteam3636.swervebase.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import com.frcteam3636.swervebase.CANcoder
import com.frcteam3636.swervebase.CTREDeviceId
import com.frcteam3636.swervebase.Robot
import com.frcteam3636.swervebase.TalonFX
import com.frcteam3636.swervebase.utils.math.*
import com.frcteam3636.swervebase.utils.swerve.SwerveModuleTemperature
import com.frcteam3636.swervebase.utils.swerve.speed
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import java.util.*

interface SwerveModule {
    // The current "state" of the swerve module.
    //
    // This is essentially the velocity of the wheel,
    // and includes both the speed and the angle
    // in which the module is currently traveling.
    val state: SwerveModuleState

    // The desired state of the module.
    //
    // This is the wheel velocity that we're trying to get to.
    var desiredState: SwerveModuleState

    // The measured position of the module.
    //
    // This is a vector with direction equal to the current angle of the module,
    // and magnitude equal to the total signed distance traveled by the wheel.
    val position: SwerveModulePosition
    val positionRad: Angle
    var odometryTimestamps: DoubleArray
    var odometryTurnPositions: Array<Rotation2d>
    var odometryDrivePositions: DoubleArray
    var odometryPositions: Array<SwerveModulePosition>
    var validTimestamps: Int
    var temperatures: SwerveModuleTemperature
    val signals: Array<BaseStatusSignal>
        get() = emptyArray()

    fun periodic() {}
    fun characterize(voltage: Voltage, turningAngle: Angle?)
}

class Mk5nSwerveModule(
    val drivingMotor: SwerveDrivingMotor, val turningMotor: SwerveTurningMotor, private val chassisAngle: Rotation2d
) : SwerveModule {
    private var timestampQueue: Queue<Double> = PhoenixOdometryThread.getInstance().makeTimestampQueue()

    private val maxQueueSize = 100  // or however many timestamps we expect

    // preallocate to reduce GC pressure
    private val emptySwerveModulePosition = SwerveModulePosition()
    override var odometryTimestamps: DoubleArray = DoubleArray(maxQueueSize)
    override var odometryDrivePositions = doubleArrayOf()
    override var odometryTurnPositions: Array<Rotation2d> = Array(maxQueueSize) { Rotation2d.kZero }
    override var odometryPositions: Array<SwerveModulePosition> = Array(maxQueueSize) { emptySwerveModulePosition }
    override var validTimestamps: Int = 0
    override var temperatures: SwerveModuleTemperature = SwerveModuleTemperature(0.0.celsius, 0.0.celsius)

    override val state: SwerveModuleState
        get() = SwerveModuleState(
            drivingMotor.velocity,
            Rotation2d.fromRadians(turningMotor.position.inRadians()) + chassisAngle
        )

    override val position: SwerveModulePosition
        get() = SwerveModulePosition(
            drivingMotor.position, Rotation2d.fromRadians(turningMotor.position.inRadians()) + chassisAngle
        )

    override val positionRad: Angle
        get() = drivingMotor.positionRad

    override fun characterize(voltage: Voltage, turningAngle: Angle?) {
        drivingMotor.setVoltage(voltage)
        if (turningAngle != null) {
            turningMotor.position = turningAngle
        }
    }

    override var desiredState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d.kZero)
        get() = SwerveModuleState(field.speedMetersPerSecond, field.angle + chassisAngle)
        set(value) {
            val corrected = SwerveModuleState(value.speedMetersPerSecond, value.angle - chassisAngle)
            // optimize the state to avoid rotating more than 90 degrees
            corrected.optimize(
                Rotation2d(turningMotor.position)
            )

            corrected.cosineScale(Rotation2d(turningMotor.position))

            drivingMotor.velocity = corrected.speed + (turningMotor.velocity * COUPLING_RATIO).toLinear(WHEEL_RADIUS)
            turningMotor.position = corrected.angle.measure


            field = corrected
        }

    override val signals: Array<BaseStatusSignal>
        get() = turningMotor.signals + drivingMotor.signals

    override fun periodic() {
        validTimestamps = timestampQueue.size

        // Fill the odometryTimestamps array in-place
        for ((i, timestamp) in timestampQueue.withIndex()) {
            odometryTimestamps[i] = timestamp
        }

        drivingMotor.periodic()
        turningMotor.periodic()
        odometryTurnPositions = turningMotor.odometryTurnPositions
        odometryDrivePositions = drivingMotor.odometryDrivePositions

        // Update positions in-place
        @Suppress("EmptyRange") // for some reason intellij determines this using the default value lol
        for (index in 0..<validTimestamps) {
            val distance = (odometryDrivePositions[index].radians -
                    (odometryTurnPositions[index].rotations.rotations * COUPLING_RATIO)).toLinear(WHEEL_RADIUS)
            val angle = odometryTurnPositions[index] + chassisAngle

            val pos = odometryPositions[index]
            pos.distanceMeters = distance.inMeters()
            pos.angle = angle
        }

        timestampQueue.clear()
    }
}

interface SwerveDrivingMotor {
    val position: Distance
    val positionRad: Angle
    var velocity: LinearVelocity
    var odometryDrivePositions: DoubleArray
    val temperature: Temperature
    val signals: Array<BaseStatusSignal>
    fun setVoltage(voltage: Voltage)
    fun periodic() {}
}

interface SwerveTurningMotor {
    var position: Angle
    val velocity: AngularVelocity
    var odometryTurnPositions: Array<Rotation2d>
    val temperature: Temperature
    val signals: Array<BaseStatusSignal>

    fun periodic() {}
}

class DrivingTalon(id: CTREDeviceId) : SwerveDrivingMotor {

    private val inner = TalonFX(id).apply {
        configurator.apply(TalonFXConfiguration().apply {
            Slot0.apply {
                pidGains = DRIVING_PID_GAINS
                motorFFGains = DRIVING_FF_GAINS
            }
            CurrentLimits.apply {
                StatorCurrentLimit = DRIVING_CURRENT_LIMIT.inAmps()
                StatorCurrentLimitEnable = true
            }
            TorqueCurrent.apply {
                PeakForwardTorqueCurrent = DRIVING_CURRENT_LIMIT.inAmps()
                PeakReverseTorqueCurrent = -DRIVING_CURRENT_LIMIT.inAmps()
            }
            Feedback.apply {
                SensorToMechanismRatio = DRIVING_GEAR_RATIO
            }
        })
    }

    override var odometryDrivePositions = doubleArrayOf()

    private val positionSignal = inner.position
    private val velocitySignal = inner.velocity
    private val temperatureSignal = inner.deviceTemp

    private val positionQueue: Queue<Double> =
        PhoenixOdometryThread.getInstance().registerSignal(positionSignal.clone())

    init {
        BaseStatusSignal.setUpdateFrequencyForAll(250.0, positionSignal)
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, velocitySignal, temperatureSignal)
        inner.optimizeBusUtilization(0.0)
    }

    override val position: Distance
        get() = positionSignal.value.toLinear(WHEEL_RADIUS)

    override val positionRad: Angle
        get() = positionSignal.value

    private var velocityControl = VelocityVoltage(0.0).apply {
        EnableFOC = true
    }

    override var velocity: LinearVelocity
        get() = velocitySignal.value.toLinear(WHEEL_RADIUS)
        set(value) {
            inner.setControl(velocityControl.withVelocity(value.toAngular(WHEEL_RADIUS)))
        }

    override val temperature: Temperature
        get() = temperatureSignal.value

    private val voltageControl = VoltageOut(0.0).apply {
        EnableFOC = true
    }

    override fun setVoltage(voltage: Voltage) {
        inner.setControl(voltageControl.withOutput(voltage.inVolts()))
    }

    override val signals: Array<BaseStatusSignal> = arrayOf(positionSignal, velocitySignal, temperatureSignal)

    override fun periodic() {
        odometryDrivePositions = positionQueue.map { it.rotations.inRadians() }.toDoubleArray()
        positionQueue.clear()
    }
}

class TurningTalon(id: CTREDeviceId, encoderId: CTREDeviceId, magnetOffset: Double) : SwerveTurningMotor {

    private val inner = TalonFX(id).apply {
        configurator.apply(TalonFXConfiguration().apply {
            Slot0.apply {
                pidGains = TURNING_PID_GAINS
                motorFFGains = TURNING_FF_GAINS
                StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign
            }
            ClosedLoopGeneral.apply {
                ContinuousWrap = true
            }
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
            }
            Feedback.apply {
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
                RotorToSensorRatio = TURNING_GEAR_RATIO
                FeedbackRemoteSensorID = encoderId.num
            }
            MotionMagic.apply {
                MotionMagicCruiseVelocity = 100.0 / TURNING_GEAR_RATIO
                MotionMagicAcceleration = MotionMagicCruiseVelocity / 0.1
                MotionMagicExpo_kA = 0.1
                MotionMagicExpo_kV = 0.12 * TURNING_GEAR_RATIO
            }
        })
    }

    private val positionSignal = inner.position
    private val velocitySignal = inner.velocity
    private val temperatureSignal = inner.deviceTemp

    private val positionQueue: Queue<Double> =
        PhoenixOdometryThread.getInstance().registerSignal(positionSignal.clone())
    override var odometryTurnPositions: Array<Rotation2d> = emptyArray()

    init {
        CANcoder(encoderId).apply {
            configurator.apply(CANcoderConfiguration().apply {
                MagnetSensor.MagnetOffset = magnetOffset
            })
        }
        BaseStatusSignal.setUpdateFrequencyForAll(250.0, positionSignal)
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, velocitySignal, temperatureSignal)
        inner.optimizeBusUtilization(0.0)
    }

    private val positonControl = PositionVoltage(0.0).apply {
        EnableFOC = true
    }

    override val temperature: Temperature
        get() = temperatureSignal.value

    override var position: Angle
        set(value) {
            inner.setControl(positonControl.withPosition(value))
        }
        get() = positionSignal.value

    override val velocity: AngularVelocity
        get() = velocitySignal.value

    override val signals: Array<BaseStatusSignal> = arrayOf(positionSignal, temperatureSignal, velocitySignal)

    override fun periodic() {
        odometryTurnPositions = positionQueue.map { Rotation2d(it.rotations) }.toTypedArray()
        positionQueue.clear()
    }
}

class SimSwerveModule : SwerveModule {

    override var odometryDrivePositions: DoubleArray = doubleArrayOf()
    override var odometryTurnPositions: Array<Rotation2d> = emptyArray()
    override var odometryPositions: Array<SwerveModulePosition> = emptyArray()
    override var temperatures: SwerveModuleTemperature = SwerveModuleTemperature(0.0.celsius, 0.0.celsius)
    override var validTimestamps: Int = 0
    private val driveMotorSystem = LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60Foc(1),
        0.0001,
        DRIVING_GEAR_RATIO
    )
    private val driveMotor = DCMotorSim(driveMotorSystem, DCMotor.getKrakenX60Foc(1).withReduction(DRIVING_GEAR_RATIO))

    private val turnMotorSystem = LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60(1),
        0.01,
        TURNING_GEAR_RATIO
    )
    private val turnMotor = DCMotorSim(turnMotorSystem, DCMotor.getKrakenX60(1).withReduction(TURNING_GEAR_RATIO))

    private val drivingFeedforward = SimpleMotorFeedforward(MotorFFGains(v = 3.3))
    private val drivingFeedback = PIDController(PIDGains(1.0))

    private val turningFeedback = PIDController(PIDGains(15.0)).apply { enableContinuousInput(0.0, TAU) }

    override var odometryTimestamps: DoubleArray = doubleArrayOf()

    override val state: SwerveModuleState
        get() = SwerveModuleState(
            driveMotor.angularVelocity.toLinear(WHEEL_RADIUS),
            Rotation2d.fromRadians(turnMotor.angularPositionRad)
        )

    override val positionRad: Angle
        get() = driveMotor.angularPosition

    override var desiredState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d())
        set(value) {
            field = value.apply {
                optimize(state.angle)
            }
        }

    override val position: SwerveModulePosition
        get() = SwerveModulePosition(
            driveMotor.angularPosition.toLinear(WHEEL_RADIUS),
            Rotation2d.fromRadians(turnMotor.angularPositionRad)
        )

    override fun periodic() {
        turnMotor.update(Robot.period)
        driveMotor.update(Robot.period)
        // Set the new input voltages
        turnMotor.inputVoltage = turningFeedback.calculate(state.angle.radians, desiredState.angle.radians)
        driveMotor.inputVoltage =
            drivingFeedforward.calculate(desiredState.speedMetersPerSecond) + drivingFeedback.calculate(
                state.speedMetersPerSecond, desiredState.speedMetersPerSecond
            )
    }

    override fun characterize(voltage: Voltage, turningAngle: Angle?) {
        return // we don't need this in sim
    }
}

// take the known wheel diameter, divide it by two to get the radius, then get the
// circumference
internal val WHEEL_RADIUS = 1.976.inches

const val DRIVING_GEAR_RATIO = TunerConstants.kDriveGearRatio
const val TURNING_GEAR_RATIO = TunerConstants.kSteerGearRatio

internal val DRIVING_PID_GAINS: PIDGains = TunerConstants.driveGains!!.pidGains
internal val DRIVING_FF_GAINS: MotorFFGains = TunerConstants.driveGains!!.motorFFGains

internal val TURNING_PID_GAINS: PIDGains = TunerConstants.steerGains!!.pidGains
internal val TURNING_FF_GAINS: MotorFFGains = TunerConstants.steerGains!!.motorFFGains

internal val DRIVING_CURRENT_LIMIT = TunerConstants.kSlipCurrent // FIXME: Calculate

const val COUPLING_RATIO = 0.64