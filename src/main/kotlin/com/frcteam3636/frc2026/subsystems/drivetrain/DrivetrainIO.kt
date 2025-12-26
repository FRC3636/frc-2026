package com.frcteam3636.frc2026.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.Diagnostics
import com.frcteam3636.frc2026.Pigeon2
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain.Constants.MODULE_POSITIONS
import com.frcteam3636.frc2026.utils.math.*
import com.frcteam3636.frc2026.utils.swerve.DrivetrainCorner
import com.frcteam3636.frc2026.utils.swerve.PerCorner
import com.frcteam3636.frc2026.utils.swerve.SwerveModuleTemperature
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.measure.Voltage
import org.photonvision.simulation.VisionSystemSim
import org.team9432.annotation.Logged
import kotlin.math.atan2

@Logged
open class DrivetrainInputs {
    var gyroRotation = Rotation2d.kZero!!
    var gyroVelocity = 0.degreesPerSecond
    var gyroConnected = true
    var measuredStates = PerCorner.generate { SwerveModuleState() }
    var measuredPositions = PerCorner.generate { SwerveModulePosition() }
    var moduleTemperatures = PerCorner.generate {
        SwerveModuleTemperature(0.0.celsius, 0.0.celsius)
    }
}

abstract class DrivetrainIO {
    protected abstract val gyro: Gyro
    abstract val modules: PerCorner<out SwerveModule>


    open fun updateInputs(inputs: DrivetrainInputs) {
        gyro.periodic()
        modules.forEachIndexed { i, module ->
            module.periodic()
            inputs.measuredStates[i] = module.state     // no allocation
            inputs.measuredPositions[i] = module.position   // no allocation
        }

        inputs.gyroRotation = gyro.rotation
        inputs.gyroVelocity = gyro.velocity
        inputs.gyroConnected = gyro.connected
        inputs.moduleTemperatures = modules.map { it.temperatures }
    }

    var desiredStates: PerCorner<SwerveModuleState>
        get() = modules.map { it.desiredState }
        set(value) {
            modules.zip(value).forEach { (module, state) -> module.desiredState = state }
        }

    fun runCharacterization(voltage: Voltage, shouldSpin: Boolean = false, shouldStraight: Boolean = false) {
        if (shouldSpin) {
            for (i in 0..<MODULE_POSITIONS.size) {
                val trans = MODULE_POSITIONS[i].position.translation
                var angle = atan2(trans.y, trans.x)
                if (MODULE_POSITIONS[i] == MODULE_POSITIONS.frontRight || MODULE_POSITIONS[i] == MODULE_POSITIONS.backRight) {
//                    angle -= 90.degrees.inRadians()
                    if (MODULE_POSITIONS[i] == MODULE_POSITIONS.backRight) {
                        modules[i].characterize(voltage, Rotation2d(angle.radians).unaryMinus().measure)
                    } else {
                        modules[i].characterize(
                            voltage,
                            Rotation2d(angle.radians).unaryMinus().measure + Rotation2d.k180deg.measure
                        )
                    }
                } else {
                    angle += 90.degrees.inRadians()
                    modules[i].characterize(voltage, Rotation2d(angle.radians).measure)
                }
            }
        } else if (shouldStraight) {
            for (i in 0..<MODULE_POSITIONS.size) {
                modules[i].characterize(voltage, MODULE_POSITIONS[i].position.rotation.unaryMinus().measure)
            }
        } else {
            // keep at same angle
            for (module in modules) {
                module.characterize(voltage, null)
            }
        }

    }

    val odometryPositions: PerCorner<Array<SwerveModulePosition>>
        get() = modules.map { it.odometryPositions }

    val odometryTimestamps: DoubleArray
        get() = modules[DrivetrainCorner.FRONT_LEFT].odometryTimestamps

    val odometryYawPositions: DoubleArray
        get() = gyro.odometryYawPositions

    val validTimestamps: Int
        get() = modules[DrivetrainCorner.FRONT_LEFT].validTimestamps

    val signals: Array<BaseStatusSignal>
        get() {
            var signals = arrayOf<BaseStatusSignal>()

            modules.forEach { module ->
                signals += module.signals
            }

            signals += gyro.signals
            return signals
        }
}

/** Drivetrain I/O layer that uses real swerve modules along with a NavX gyro. */
class DrivetrainIOReal(override val modules: PerCorner<SwerveModule>) : DrivetrainIO() {
    override val gyro = GyroPigeon(Pigeon2(CTREDeviceId.PigeonGyro))
}

/** Drivetrain I/O layer that uses simulated swerve modules along with a simulated gyro with an angle based off their movement. */
class DrivetrainIOSim : DrivetrainIO() {
    val vision = VisionSystemSim("main").apply {
        addAprilTags(FIELD_LAYOUT)
    }

    override val modules = PerCorner.generate { SimSwerveModule() }
    override val gyro = GyroSim(modules.map { it })
    override fun updateInputs(inputs: DrivetrainInputs) {
        super.updateInputs(inputs)
        vision.update(Drivetrain.poseEstimator.estimatedPosition)


        Diagnostics.report(gyro)
    }

    fun registerPoseProviders(providers: Iterable<AbsolutePoseProvider>) {
        for (provider in providers) {
            if (provider is CameraSimPoseProvider) {
                vision.addCamera(provider.sim, provider.chassisToCamera)
            }
        }
    }
}

val FIELD_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)