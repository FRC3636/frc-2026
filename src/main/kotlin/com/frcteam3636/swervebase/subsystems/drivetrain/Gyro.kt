package com.frcteam3636.swervebase.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.hardware.Pigeon2
import com.frcteam3636.swervebase.Robot
import com.frcteam3636.swervebase.utils.math.degreesPerSecond
import com.frcteam3636.swervebase.utils.math.radiansPerSecond
import com.frcteam3636.swervebase.utils.swerve.PerCorner
import com.studica.frc.AHRS
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.AngularVelocity
import org.littletonrobotics.junction.Logger
import java.util.*
import kotlin.math.sign


interface Gyro {
    /**
     * The current rotation of the robot.
     * This can be set to a different value to change the gyro's offset.
     */
    var rotation: Rotation2d

    /**
     * The rotational velocity of the robot on its yaw axis.
     */
    val velocity: AngularVelocity

    /** Whether the gyro is connected. */
    val connected: Boolean

    var odometryYawPositions: DoubleArray
    var odometryYawTimestamps: DoubleArray

    val signals: Array<BaseStatusSignal>
        get() = emptyArray()

    fun periodic() {}
}

@Suppress("unused") // and hopefully it stays that way
class GyroNavX(private val ahrs: AHRS) : Gyro {

    private var offset = Rotation2d.kZero

    init {
        Logger.recordOutput("NavXGyro/Offset", offset)
    }

    override var rotation: Rotation2d
        get() = offset + ahrs.rotation2d
        set(goal) {
            offset = goal - ahrs.rotation2d
            Logger.recordOutput("NavXGyro/Offset", offset)
        }

    override val velocity: AngularVelocity
        get() = ahrs.rate.degreesPerSecond

    override var odometryYawPositions: DoubleArray = doubleArrayOf()
    override var odometryYawTimestamps: DoubleArray = doubleArrayOf()

    override val connected
        get() = ahrs.isConnected
}

class GyroPigeon(private val pigeon: Pigeon2) : Gyro {
    private val yawSignal = pigeon.yaw
    private val pitchSignal = pigeon.pitch
    private val rollSignal = pigeon.roll
    private val angularVelocitySignal = pigeon.angularVelocityZWorld
    private var yawTimestampQueue: Queue<Double>
    private var yawPositionQueue: Queue<Double>

    init {
        BaseStatusSignal.setUpdateFrequencyForAll(
            250.0,
            yawSignal
        )
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            pitchSignal,
            rollSignal,
            angularVelocitySignal
        )
        pigeon.optimizeBusUtilization(0.0)
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue()
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yawSignal.clone())
    }

    override var rotation: Rotation2d
        // Basically just pigeon.rotation2d but bypasses the refresh
        get() = Rotation2d.fromDegrees(yawSignal.valueAsDouble)
        set(goal) {
            pigeon.setYaw(goal.measure)
        }

    override var odometryYawPositions: DoubleArray = doubleArrayOf()
    override var odometryYawTimestamps: DoubleArray = doubleArrayOf()

    override val velocity: AngularVelocity
        get() = angularVelocitySignal.value

    override val connected
        get() = yawSignal.status.isOK

    override val signals: Array<BaseStatusSignal>
        get() = arrayOf(
            yawSignal,
            pitchSignal,
            rollSignal,
            angularVelocitySignal
        )

    override fun periodic() {
        odometryYawTimestamps = yawTimestampQueue.toDoubleArray()
        odometryYawPositions = yawPositionQueue.toDoubleArray()
        yawTimestampQueue.clear()
        yawPositionQueue.clear()
    }
}

class GyroSim(private val modules: PerCorner<SwerveModule>) : Gyro {
    override var rotation: Rotation2d = Rotation2d.kZero
    override var velocity: AngularVelocity = 0.radiansPerSecond
    override val connected = true
    override var odometryYawPositions: DoubleArray = doubleArrayOf()
    override var odometryYawTimestamps: DoubleArray = doubleArrayOf()

    override fun periodic() {
        val moduleStates = modules.map { it.state }.toTypedArray()
        val velocityMap = moduleStates
            .map { Translation2d(it.speedMetersPerSecond, it.angle) }
            .reduce(Translation2d::plus)
            .div(moduleStates.size.toDouble())

        val referenceModule = modules.frontLeft.state
        val referenceModulePosition = Translation2d(
            TunerConstants.FrontLeft!!.LocationX,
            TunerConstants.FrontLeft.LocationY
        )

        val referenceModuleVelocity = Translation2d(referenceModule.speedMetersPerSecond, referenceModule.angle)
        val referenceRotationalVelocityComponent = referenceModuleVelocity.minus(velocityMap)

        val cross = referenceModulePosition.x * referenceRotationalVelocityComponent.y -
                referenceModulePosition.y * referenceRotationalVelocityComponent.x

        val turningDirection = cross.sign  // +1.0 for CCW, -1.0 for CW, 0.0 if no rotation

        val turnRateMagnitude = referenceRotationalVelocityComponent.norm / referenceModulePosition.norm
        val turnRate = Rotation2d.fromRadians(turnRateMagnitude * turningDirection)

        velocity = turnRate.degrees.degreesPerSecond
        rotation = rotation.plus(turnRate.times(Robot.period))
    }
}