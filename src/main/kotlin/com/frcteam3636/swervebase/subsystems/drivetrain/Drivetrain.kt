package com.frcteam3636.swervebase.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.swervebase.CTREDeviceId
import com.frcteam3636.swervebase.Robot
import com.frcteam3636.swervebase.RobotState
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.BRAKE_POSITION
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.DRIVE_BASE_RADIUS
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.FREE_SPEED
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.JOYSTICK_DEADBAND
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.MODULE_POSITIONS
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.ROTATION_SENSITIVITY
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.TRANSLATION_SENSITIVITY
import com.frcteam3636.swervebase.utils.fieldRelativeTranslation2d
import com.frcteam3636.swervebase.utils.math.*
import com.frcteam3636.swervebase.utils.swerve.*
import com.frcteam3636.swervebase.utils.translation2d
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger
import java.util.concurrent.locks.ReentrantLock
import kotlin.jvm.optionals.getOrNull
import kotlin.math.*

/** A singleton object representing the drivetrain. */
object Drivetrain : Subsystem {
    private val io = when (Robot.model) {
        Robot.Model.SIMULATION -> DrivetrainIOSim()
        Robot.Model.COMPETITION -> DrivetrainIOReal(
            MODULE_POSITIONS.zip(Constants.MODULE_CAN_IDS)
                .map { (corner, ids) ->
                    val (driveId, turnId, encoderId) = ids
                    Mk5nSwerveModule(
                        DrivingTalon(driveId),
                        TurningTalon(turnId, encoderId, corner.magnetOffset),
                        corner.position.rotation
                    )
                })
    }
    val inputs = LoggedDrivetrainInputs()

    private val limiter = SlewRateLimiter(0.05)
    private var wheelRadiusModuleStates = DoubleArray(4)
    private var wheelRadiusLastAngle = Rotation2d.kZero
    private var wheelRadiusGyroDelta = 0.0

    @Suppress("unused")
    fun calculateWheelRadius(): Command = Commands.parallel(
        Commands.sequence(
            Commands.runOnce({
                Logger.recordOutput("Drivetrain/Wheel Radius Calculated/Running", true)
                limiter.reset(0.0)
            }),
            Commands.run({
                val speed = limiter.calculate(0.1)
                driveWithoutDeadband(Translation2d.kZero, Translation2d(0.0, speed))
            }, this)
        ),
        Commands.sequence(
            // Wait for modules to orient
            Commands.waitSeconds(1.0),
            Commands.runOnce({
                for (i in 0..3) {
                    wheelRadiusModuleStates[i] = io.modules.toTypedArray()[i].positionRad.inRadians()
                }
                wheelRadiusLastAngle = inputs.gyroRotation
                wheelRadiusGyroDelta = 0.0
            }),
            Commands.run({
                val rotation = inputs.gyroRotation
                wheelRadiusGyroDelta += abs(rotation.minus(wheelRadiusLastAngle).radians)
                wheelRadiusLastAngle = rotation
                Logger.recordOutput("Drivetrain/Wheel Radius Calculated/Gyro Delta", wheelRadiusGyroDelta)
            })
                .finallyDo { ->
                    var wheelDelta = 0.0
                    // Someone give me a better way to do this
                    for (i in 0..3) {
                        wheelDelta += abs(io.modules.toTypedArray()[i].positionRad.inRadians() - wheelRadiusModuleStates[i]) / 4.0
                        Logger.recordOutput(
                            "Drivetrain/Wheel Radius Calculated/Initial Wheel Position Rad/$i",
                            wheelRadiusModuleStates[i]
                        )
                        Logger.recordOutput(
                            "Drivetrain/Wheel Radius Calculated/Final Wheel Position Rad/$i",
                            io.modules.toTypedArray()[i].positionRad.inRadians()
                        )
                    }
                    val wheelRadius = ((wheelRadiusGyroDelta * DRIVE_BASE_RADIUS) / wheelDelta)
                    Logger.recordOutput("Drivetrain/Wheel Radius Calculated/Drive Base Radius", DRIVE_BASE_RADIUS)
                    Logger.recordOutput("Drivetrain/Wheel Radius Calculated/Wheel Delta", wheelDelta)
                    Logger.recordOutput("Drivetrain/Wheel Radius Calculated/Meters", wheelRadius)
                    Logger.recordOutput("Drivetrain/Wheel Radius Calculated/Inches", wheelRadius.meters.inInches())
                    Logger.recordOutput("Drivetrain/Wheel Radius Calculated/Running", false)
                }
        )
    )

    val odometryLock = ReentrantLock()

    private var rawGyroRotation = Rotation2d.kZero

    // someone please give me a better way to do this
    val lastModulePositions = arrayOf(
        SwerveModulePosition(),
        SwerveModulePosition(),
        SwerveModulePosition(),
        SwerveModulePosition()
    )

    private val absolutePoseIOs = when (Robot.model) {
        Robot.Model.SIMULATION -> mapOf(
            "Limelight" to CameraSimPoseProvider("limelight", Transform3d.kZero),
        )

        else -> mapOf(
            "Limelight Left" to LimelightPoseProvider(
                "limelight-left",
                {
                    poseEstimator.estimatedPosition.rotation
                },
                {
                    inputs.gyroVelocity
                },
                {
                    inputs.gyroConnected
                },
                true,
            ),
            "Limelight Right" to LimelightPoseProvider(
                "limelight-right",
                {
                    poseEstimator.estimatedPosition.rotation
                },
                {
                    inputs.gyroVelocity
                },
                {
                    inputs.gyroConnected
                },
                true,
            ),
        )
    }.mapValues { Pair(it.value, LoggedAbsolutePoseProviderInputs()) }

    /** Helper for converting a desired drivetrain velocity into the speeds and angles for each swerve module */
    private val kinematics =
        SwerveDriveKinematics(
            *MODULE_POSITIONS
                .map { it.position.translation }
                .toTypedArray()
        )


    private val acceptedPoses: MutableList<Pose2d> = mutableListOf()
    private val rejectedPoses: MutableList<Pose2d> = mutableListOf()

    /** Helper for estimating the location of the drivetrain on the field */
    val poseEstimator =
        SwerveDrivePoseEstimator(
            kinematics, // swerve drive kinematics
            inputs.gyroRotation, // initial gyro rotation
            inputs.measuredPositions.toTypedArray(), // initial module positions
            Pose2d.kZero, // initial pose
            VecBuilder.fill(0.02, 0.02, 0.005),
            // Overwrite each measurement
            VecBuilder.fill(0.0, 0.0, 0.0)
        )

    /** Whether every sensor used for pose estimation is connected. */
    val allPoseProvidersConnected
        get() = absolutePoseIOs.values.all { it.second.connected }

    init {
        if (io is DrivetrainIOSim) {
            io.registerPoseProviders(absolutePoseIOs.values.map { it.first })
        }

        PhoenixOdometryThread.getInstance().start()
    }

    val modulePositions = Array(4) { SwerveModulePosition() }
    val moduleDeltas = Array(4) { SwerveModulePosition() }

    override fun periodic() {
        if (Robot.model != Robot.Model.SIMULATION) {
            try {
                odometryLock.lock()
                io.updateInputs(inputs)
                Logger.processInputs("Drivetrain", inputs)
                val odometryTimestamps = io.odometryTimestamps
                val odometryPositions = io.odometryPositions
                val odometryYawPositons = io.odometryYawPositions
                val validTimestamps = io.validTimestamps
                Logger.recordOutput("Drivetrain/Odometry Positions Count", odometryPositions.first().size)
                for (i in 0..<validTimestamps) {
                    for (index in 0..3) {
                        val pos = odometryPositions[index][i]
                        modulePositions[index].distanceMeters = pos.distanceMeters
                        modulePositions[index].angle = pos.angle
                    }

                    // Compute deltas in-place
                    for (index in 0..3) {
                        val deltaDistance =
                            modulePositions[index].distanceMeters - lastModulePositions[index].distanceMeters
                        val deltaAngle = modulePositions[index].angle - lastModulePositions[index].angle
                        moduleDeltas[index].distanceMeters = deltaDistance
                        moduleDeltas[index].angle = deltaAngle

                        // Update last positions
                        lastModulePositions[index] = modulePositions[index]
                    }

                    rawGyroRotation = if (inputs.gyroConnected) {
                        Rotation2d(odometryYawPositons[i].degrees)
                    } else {
                        rawGyroRotation.plus(Rotation2d(kinematics.toTwist2d(*moduleDeltas).dtheta))
                    }
                    poseEstimator.updateWithTime(odometryTimestamps[i], rawGyroRotation, modulePositions)
                }
            } finally {
                odometryLock.unlock()
            }
        } else {
            io.updateInputs(inputs)
            Logger.processInputs("Drivetrain", inputs)
            rawGyroRotation = inputs.gyroRotation
            poseEstimator.update(
                rawGyroRotation,
                inputs.measuredPositions.toTypedArray()
            )
        }

        Logger.recordOutput("Drivetrain/Raw Gyro Rotation", rawGyroRotation)


        // Update absolute pose sensors and add their measurements to the pose estimator
        for ((name, ioPair) in absolutePoseIOs) {
            val (sensorIO, inputs) = ioPair

            sensorIO.updateInputs(inputs)
            Logger.processInputs("Drivetrain/Absolute Pose/$name", inputs)

            for (measurement in inputs.measurements) {
                if (!measurement.isLowQuality) {
                    if (measurement.pose.x < 0.0 || measurement.pose.y < 0.0) {
                        rejectedPoses.add(measurement.pose)
                        continue
                    } else if (measurement.pose.x > FIELD_LAYOUT.fieldLength || measurement.pose.y > FIELD_LAYOUT.fieldWidth) {
                        rejectedPoses.add(measurement.pose)
                        continue
                    } else if (abs(measurement.pose.rotation.degrees - estimatedPose.rotation.degrees) > 5 && !RobotState.beforeFirstEnable) {
                        rejectedPoses.add(measurement.pose)
                        continue
                    }
                    acceptedPoses.add(measurement.pose)
                    poseEstimator.addAbsolutePoseMeasurement(measurement)
                } else {
                    rejectedPoses.add(measurement.pose)
                }
            }

            Logger.recordOutput("Drivetrain/Absolute Pose/$name/Accepted Poses", *acceptedPoses.toTypedArray())
            Logger.recordOutput("Drivetrain/Absolute Pose/$name/Rejected Poses", *rejectedPoses.toTypedArray())
            acceptedPoses.clear()
            rejectedPoses.clear()
        }

        Logger.recordOutput("Drivetrain/Pose Estimator/Estimated Pose", poseEstimator.estimatedPosition)
        Logger.recordOutput("Drivetrain/Chassis Speeds", measuredChassisSpeeds)
        Logger.recordOutput("Drivetrain/Desired Chassis Speeds", desiredChassisSpeeds)
        Logger.recordOutput(
            "Drivetrain/Measured Velocity",
            measuredChassisSpeeds.translation2dPerSecond.norm.metersPerSecond
        )
        Logger.recordOutput(
            "Drivetrain/Desired Velocity",
            desiredChassisSpeeds.translation2dPerSecond.norm.metersPerSecond
        )

        Logger.recordOutput(
            "Drivetrain/TagPoses", *FIELD_LAYOUT.tags
                .filter { tag ->
                    absolutePoseIOs.values.any { it.second.observedTags.contains(tag.ID) }
                }
                .map { it.pose }
                .toTypedArray())
    }

    /** The desired speeds and angles of the swerve modules. */
    private var desiredModuleStates
        get() = io.desiredStates
        set(value) {
            val stateArr = value.toTypedArray()
            SwerveDriveKinematics.desaturateWheelSpeeds(stateArr, FREE_SPEED)

            io.desiredStates = PerCorner.fromConventionalArray(stateArr)
            Logger.recordOutput("Drivetrain/Desired States", *stateArr)
        }

    /**
     * The current speed of chassis relative to the ground,
     * assuming that the wheels have perfect traction with the ground.
     */
    val measuredChassisSpeeds get() = kinematics.cornerStatesToChassisSpeeds(inputs.measuredStates)

    /**
     * The chassis speeds that the drivetrain is attempting to move at.
     *
     * Note that the speeds are relative to the chassis, not the field.
     */
    private var desiredChassisSpeeds
        get() = kinematics.cornerStatesToChassisSpeeds(desiredModuleStates)
        set(value) {
            val discretized = ChassisSpeeds.discretize(value, Robot.period)
            desiredModuleStates = kinematics.toCornerSwerveModuleStates(discretized)
        }

    /** The estimated pose of the robot on the field, using the yaw value measured by the gyro. */
    var estimatedPose: Pose2d
        get() {
            return poseEstimator.estimatedPosition
        }
        private set(value) {
            poseEstimator.resetPosition(
                inputs.gyroRotation,
                inputs.measuredPositions.toTypedArray(),
                value
            )
        }

    val signals: Array<BaseStatusSignal>
        get() = io.signals

    private fun isInDeadband(translation: Translation2d) =
        abs(translation.x) < JOYSTICK_DEADBAND && abs(translation.y) < JOYSTICK_DEADBAND

    private fun drive(translationInput: Translation2d, rotationInput: Translation2d) {
        if (isInDeadband(translationInput) && isInDeadband(rotationInput)) {
            // No joystick input - stop moving!
            desiredModuleStates = BRAKE_POSITION
        } else {
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                calculateInputCurve(translationInput.x) * FREE_SPEED.inMetersPerSecond() * TRANSLATION_SENSITIVITY,
                calculateInputCurve(translationInput.y) * FREE_SPEED.inMetersPerSecond() * TRANSLATION_SENSITIVITY,
                rotationInput.y * TAU * ROTATION_SENSITIVITY,
                estimatedPose.rotation
            )
        }
    }

    @Suppress("SameParameterValue")
    private fun driveWithoutDeadband(translationInput: Translation2d, rotationInput: Translation2d) {
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            calculateInputCurve(translationInput.x) * FREE_SPEED.inMetersPerSecond() * TRANSLATION_SENSITIVITY,
            calculateInputCurve(translationInput.y) * FREE_SPEED.inMetersPerSecond() * TRANSLATION_SENSITIVITY,
            rotationInput.y * TAU * ROTATION_SENSITIVITY,
            estimatedPose.rotation
        )
    }

    private fun calculateInputCurve(input: Double): Double {
        val exponent = 1.7

        return input.absoluteValue.pow(exponent).withSign(input)
    }

    fun driveWithJoysticks(translationJoystick: Joystick, rotationJoystick: Joystick): Command =
        run {
            // Directly accessing Joystick.x/y gives inverted values - use a `Translation2d` instead.
            drive(translationJoystick.fieldRelativeTranslation2d, rotationJoystick.translation2d)
        }

    @Suppress("unused")
    fun driveWithController(controller: CommandXboxController): Command =
        run {
            val translationInput = Translation2d(controller.leftX, controller.leftY)
            val rotationInput = Translation2d(controller.rightX, controller.rightY)

            drive(translationInput, rotationInput)
        }

    fun zeroGyro(isReversed: Boolean = false, offset: Rotation2d = Rotation2d.kZero) {
        // Tell the gyro that the robot is facing the other alliance.
        var zeroPos = when (DriverStation.getAlliance().getOrNull()) {
            DriverStation.Alliance.Red -> Rotation2d.k180deg
            else -> Rotation2d.kZero
        }

        if (isReversed) {
            zeroPos += Rotation2d.k180deg
        }

        estimatedPose = Pose2d(estimatedPose.translation, zeroPos + offset)
//        io.setGyro(zeroPos)
    }

    @Suppress("unused")
    fun stop(): Command =
        runOnce {
            desiredModuleStates = BRAKE_POSITION
        }

    var sysID = SysIdRoutine(
        SysIdRoutine.Config(
            null, null, null
        ) {
            SignalLogger.writeString("state", it.toString())
        }, SysIdRoutine.Mechanism(
            io::runCharacterization,
            null,
            this,
        )
    )

    @Suppress("unused")
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction) = run {
        io.runCharacterization(0.volts, shouldStraight = true)
    }.withTimeout(2.0).andThen(sysID.quasistatic(direction))!!

    @Suppress("unused")
    fun sysIdDynamic(direction: SysIdRoutine.Direction) = run {
        io.runCharacterization(0.volts, shouldStraight = true)
    }.withTimeout(2.0).andThen(sysID.dynamic(direction))!!

    @Suppress("unused")
    fun sysIdQuasistaticSpin(direction: SysIdRoutine.Direction) = run {
        io.runCharacterization(0.volts, shouldSpin = true)
    }.withTimeout(2.0).andThen(sysID.quasistatic(direction))!!

    @Suppress("unused")
    fun sysIdDynamicSpin(direction: SysIdRoutine.Direction) = run {
        io.runCharacterization(0.volts, shouldSpin = true)
    }.withTimeout(2.0).andThen(sysID.dynamic(direction))!!

    @Suppress("unused")
    object Constants {
        // Translation/rotation coefficient for teleoperated driver controls
        /** Unit: Percent of max robot speed */
        const val TRANSLATION_SENSITIVITY = 1.0 // FIXME: Increase

        /** Unit: Rotations per second */
        const val ROTATION_SENSITIVITY = 0.8

        val ROBOT_LENGTH = 25.5.inches
        val ROBOT_WIDTH = 25.5.inches
        val TRACK_WIDTH = abs(TunerConstants.FrontLeft!!.LocationY - TunerConstants.FrontRight!!.LocationY)
        val WHEEL_BASE = abs(TunerConstants.FrontRight!!.LocationX - TunerConstants.BackRight!!.LocationX)
        const val WHEEL_COF = 1.8 // FIXME: figure this out man idk

        val BUMPER_WIDTH = 30.inches
        val BUMPER_LENGTH = 30.inches

        const val JOYSTICK_DEADBAND = 0.075

        val FRONT_LEFT_CONSTANTS = TunerConstants.FrontLeft!!
        val FRONT_RIGHT_CONSTANTS = TunerConstants.FrontRight!!
        val BACK_RIGHT_CONSTANTS = TunerConstants.BackRight!!
        val BACK_LEFT_CONSTANTS = TunerConstants.BackLeft!!

        val FRONT_LEFT_MAGNET_OFFSET = FRONT_LEFT_CONSTANTS.EncoderOffset
        val FRONT_RIGHT_MAGNET_OFFSET = FRONT_RIGHT_CONSTANTS.EncoderOffset
        val BACK_RIGHT_MAGNET_OFFSET = BACK_RIGHT_CONSTANTS.EncoderOffset
        val BACK_LEFT_MAGNET_OFFSET = BACK_LEFT_CONSTANTS.EncoderOffset

        val MODULE_POSITIONS = PerCorner(
            frontLeft = Corner(
                Pose2d(
                    Translation2d(FRONT_LEFT_CONSTANTS.LocationX, FRONT_LEFT_CONSTANTS.LocationY),
                    Rotation2d.kZero
                ), FRONT_LEFT_MAGNET_OFFSET
            ),
            frontRight = Corner(
                Pose2d(
                    Translation2d(FRONT_RIGHT_CONSTANTS.LocationX, FRONT_RIGHT_CONSTANTS.LocationY),
                    Rotation2d.fromDegrees(180.0)
                ), FRONT_RIGHT_MAGNET_OFFSET
            ),
            backLeft = Corner(
                Pose2d(
                    Translation2d(BACK_LEFT_CONSTANTS.LocationX, BACK_LEFT_CONSTANTS.LocationY),
                    Rotation2d.kZero
                ), BACK_LEFT_MAGNET_OFFSET
            ),
            backRight = Corner(
                Pose2d(
                    Translation2d(BACK_RIGHT_CONSTANTS.LocationX, BACK_RIGHT_CONSTANTS.LocationY),
                    Rotation2d.fromDegrees(180.0)
                ), BACK_RIGHT_MAGNET_OFFSET
            ),
        )

        val DRIVE_BASE_RADIUS = hypot(MODULE_POSITIONS.frontLeft.position.x, MODULE_POSITIONS.frontLeft.position.y)

        // Chassis Control
        val FREE_SPEED = TunerConstants.kSpeedAt12Volts

        val PATH_FOLLOWING_TRANSLATION_GAINS = PIDGains(8.0, 0.0, 0.0)
        val PATH_FOLLOWING_ROTATION_GAINS = PIDGains(10.0, 0.0, 0.0)

        // CAN IDs
        val MODULE_CAN_IDS =
            PerCorner(
                frontLeft =
                    Triple(
                        CTREDeviceId.FrontLeftDrivingMotor,
                        CTREDeviceId.FrontLeftTurningMotor,
                        CTREDeviceId.FrontLeftTurningEncoder
                    ),
                frontRight =
                    Triple(
                        CTREDeviceId.FrontRightDrivingMotor,
                        CTREDeviceId.FrontRightTurningMotor,
                        CTREDeviceId.FrontRightTurningEncoder
                    ),
                backLeft =
                    Triple(
                        CTREDeviceId.BackLeftDrivingMotor,
                        CTREDeviceId.BackLeftTurningMotor,
                        CTREDeviceId.BackLeftTurningEncoder
                    ),
                backRight =
                    Triple(
                        CTREDeviceId.BackRightDrivingMotor,
                        CTREDeviceId.BackRightTurningMotor,
                        CTREDeviceId.BackRightTurningEncoder
                    ),
            )

        /** A position with the modules radiating outwards from the center of the robot, preventing movement. */
        val BRAKE_POSITION =
            MODULE_POSITIONS.map { module -> SwerveModuleState(0.0, module.position.translation.angle) }
    }
}
