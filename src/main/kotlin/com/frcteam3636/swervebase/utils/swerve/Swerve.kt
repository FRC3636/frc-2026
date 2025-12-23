package com.frcteam3636.swervebase.utils.swerve

import com.frcteam3636.swervebase.utils.math.celsius
import com.frcteam3636.swervebase.utils.math.inCelsius
import com.frcteam3636.swervebase.utils.math.inMetersPerSecond
import com.frcteam3636.swervebase.utils.math.metersPerSecond
import com.frcteam3636.swervebase.utils.math.radiansPerSecond
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import java.nio.ByteBuffer

enum class DrivetrainCorner {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
}

data class PerCorner<T>(var frontLeft: T, var frontRight: T, var backLeft: T, var backRight: T) :
    Collection<T> {
    operator fun get(corner: DrivetrainCorner): T =
        when (corner) {
            DrivetrainCorner.FRONT_LEFT -> frontLeft
            DrivetrainCorner.BACK_LEFT -> backLeft
            DrivetrainCorner.BACK_RIGHT -> backRight
            DrivetrainCorner.FRONT_RIGHT -> frontRight
        }

    // someone please give me a better way to do this
    operator fun get(index: Int): T =
        when (index) {
            0 -> frontLeft
            1 -> frontRight
            2 -> backLeft
            3 -> backRight
            else -> throw IndexOutOfBoundsException()
        }

    operator fun set(index: Int, value: T) = when (index) {
        0 -> frontLeft = value
        1 -> frontRight = value
        2 -> backLeft = value
        3 -> backRight = value
        else -> throw IndexOutOfBoundsException()
    }

    fun <U> map(block: (T) -> U): PerCorner<U> = mapWithCorner { x, _ -> block(x) }
    fun <U> mapWithCorner(block: (T, DrivetrainCorner) -> U): PerCorner<U> = generate { corner ->
        block(this[corner], corner)
    }

    fun <U> zip(second: PerCorner<U>): PerCorner<Pair<T, U>> = generate { corner ->
        Pair(this[corner], second[corner])
    }

    private fun sequence(): Sequence<T> = sequenceOf(frontLeft, frontRight, backLeft, backRight)
    override fun iterator(): Iterator<T> = sequence().iterator()

    override val size: Int = 4

    override fun isEmpty(): Boolean = false

    override fun containsAll(elements: Collection<T>): Boolean = elements.all { contains(it) }

    override fun contains(element: T): Boolean = sequence().contains(element)

    companion object {
        fun <T> generate(block: (DrivetrainCorner) -> T): PerCorner<T> =
            PerCorner(
                frontLeft = block(DrivetrainCorner.FRONT_LEFT),
                frontRight = block(DrivetrainCorner.FRONT_RIGHT),
                backLeft = block(DrivetrainCorner.BACK_LEFT),
                backRight = block(DrivetrainCorner.BACK_RIGHT),
            )

        fun <T> fromConventionalArray(array: Array<T>): PerCorner<T> =
            PerCorner(
                frontLeft = array[0],
                frontRight = array[1],
                backLeft = array[2],
                backRight = array[3],
            )
    }
}

fun SwerveDriveKinematics.toCornerSwerveModuleStates(
    speeds: ChassisSpeeds
): PerCorner<SwerveModuleState> = PerCorner.fromConventionalArray(toSwerveModuleStates(speeds))

fun SwerveDriveKinematics.cornerStatesToChassisSpeeds(
    states: PerCorner<SwerveModuleState>
): ChassisSpeeds = toChassisSpeeds(*states.toList().toTypedArray())

fun SwerveDriveKinematics(translations: PerCorner<Translation2d>) =
    SwerveDriveKinematics(*translations.toList().toTypedArray())

inline fun <T> PerCorner<T>.forEachCornerIndexed(block: (corner: DrivetrainCorner, index: Int, value: T) -> Unit) {
    block(DrivetrainCorner.FRONT_LEFT, 0, frontLeft)
    block(DrivetrainCorner.FRONT_RIGHT, 1, frontRight)
    block(DrivetrainCorner.BACK_LEFT, 2, backLeft)
    block(DrivetrainCorner.BACK_RIGHT, 3, backRight)
}

/** The speed of the swerve module. */
var SwerveModuleState.speed: LinearVelocity
    get() = speedMetersPerSecond.metersPerSecond
    set(value) {
        speedMetersPerSecond = value.inMetersPerSecond()
    }

/** This swerve module state as a `Translation2d`. */
val SwerveModuleState.translation2dPerSecond: Translation2d
    get() = Translation2d(speedMetersPerSecond, angle)

val ChassisSpeeds.translation2dPerSecond: Translation2d
    get() = Translation2d(vxMetersPerSecond, vyMetersPerSecond)

val ChassisSpeeds.angularVelocity: AngularVelocity
    get() = omegaRadiansPerSecond.radiansPerSecond

data class Corner(val position: Pose2d, val magnetOffset: Double)

data class SwerveModuleTemperature(
    val drivingMotorTemperature: Temperature,
    val turningMotorTemperature: Temperature
) : StructSerializable {
    companion object {
        @JvmField
        @Suppress("unused")
        val struct = SwerveModuleTemperatureStruct()
    }
}

class SwerveModuleTemperatureStruct : Struct<SwerveModuleTemperature> {
    override fun getTypeClass(): Class<SwerveModuleTemperature> = SwerveModuleTemperature::class.java
    override fun getTypeName(): String = "struct:SwerveModuleTemperature"
    override fun getTypeString(): String = "struct:SwerveModuleTemperature"
    override fun getSize(): Int =
        Struct.kSizeDouble * 2

    override fun getSchema(): String =
        "double drivingMotorTemperatureCelsius; double turningMotorTemperatureCelsius;"

    override fun unpack(bb: ByteBuffer): SwerveModuleTemperature =
        SwerveModuleTemperature(
            bb.double.celsius,
            bb.double.celsius
        )

    override fun pack(bb: ByteBuffer, value: SwerveModuleTemperature) {
        bb.putDouble(value.drivingMotorTemperature.inCelsius())
        bb.putDouble(value.turningMotorTemperature.inCelsius())
    }
}