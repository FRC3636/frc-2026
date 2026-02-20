@file:Suppress("unused")

package com.frcteam3636.frc2026.subsystems.shooter

import com.frcteam3636.frc2026.Robot
import com.frcteam3636.frc2026.Robot.Model
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.drivetrain.DrivetrainIOSim
import com.frcteam3636.frc2026.subsystems.intake.Intake
import com.frcteam3636.frc2026.utils.FIELD_HEIGHT_METERS
import com.frcteam3636.frc2026.utils.FIELD_WIDTH_METERS
import com.frcteam3636.frc2026.utils.flipHorizontally
import com.frcteam3636.frc2026.utils.math.*
import com.frcteam3636.frc2026.utils.swerve.translation2dPerSecond
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import kotlin.math.*

object Shooter {
    object Turret : Subsystem {
        private var io = when (Robot.model) {
            Model.SIMULATION -> TurretIOSim()
            Model.COMPETITION -> TurretIOReal()
        }
        private var seeTagsRaw = false
            set(value) {
                inputs.seeTags = seeTagsDebouncer.calculate(value)
                field = value
            }

        private val inputs = LoggedTurretInputs()
        private val turretLimelight = NetworkTableInstance.getDefault().getTable("turret-limelight")
        private val seeTagsDebouncer = Debouncer(0.5)
        private val sysID = SysIdRoutine(
            SysIdRoutine.Config(
                null,
                null,
                null,
                {
                    state -> Logger.recordOutput("SysIdTestState", state.toString())
                }
            ),
            SysIdRoutine.Mechanism(
                io::setVoltage,
                null,
                this
            )
        )

        override fun periodic() {
            inputs.setPoint = shooterTarget.turretAngle
            io.updateInputs(inputs)
            seeTagsRaw = turretLimelight.getEntry("tv").equals(1)
            Logger.processInputs("Shooter/Turret", inputs)
            Logger.recordOutput("Shooter/Turret/DistanceToHub", translationToHub.norm)
            Logger.recordOutput("Shooter/Turret/TurretDistanceToHub", turretTranslationToHub.norm)
        }

        fun alignToHub(offset: Angle = 0.0.radians): Command =
            run {
                val camError = (turretLimelight.getEntry("tx").getDouble(0.0).degrees - offset.inDegrees().degrees).inRadians()
                // align with limelight
                if (inputs.seeTags) {
                    val kP = -0.1
                    io.turnToAngle(inputs.angle + (camError * kP).radians)
                }
            }

        fun getClosetTarget() : Translation2d {
            var ourAllianceZone = Zones.BlueAllianceZone
            var opposingAllianceZone = Zones.RedAllianceZone

            if (DriverStation.getAlliance().get() == Alliance.Red){
                ourAllianceZone = Zones.BlueAllianceZone
                opposingAllianceZone = Zones.RedAllianceZone
            }

            val target =  when (Drivetrain.estimatedPose.x) {
                in ourAllianceZone.startY.inMeters()..ourAllianceZone.endY.inMeters() -> hubTranslation.toTranslation2d()
                in opposingAllianceZone.startY.inMeters()..ourAllianceZone.endY.inMeters() -> FeedPose.RightSideNeutralZone.target
                else -> FeedPose.RightSideAllianceZone.target
            }

            if (Drivetrain.estimatedPose.translation.y < 4.035.meters.inMeters()) {
                return target.flipHorizontally()
            }

            return target
        }

        fun turnToTargetTurretAngle(): Command =
            run {
                io.turnToAngle(shooterTarget.turretAngle)
            }

        fun turretBrakeMode(): Command =
            run {
                io.setBrakeMode(true)
            }

        fun turretCoastMode(): Command =
            run {
                io.setBrakeMode(false)
            }

        fun sysIdQuasistatic(direction: Direction): Command = sysID.quasistatic(direction)

        fun sysIdDynamic(direction: Direction): Command = sysID.dynamic(direction)
    }

    object Hood: Subsystem {
        var fixedHood = false
        val atDesiredHoodAngle = Trigger {
            val error = abs((inputs.hoodAngle - shooterTarget.hoodAngle).inDegrees())
            Logger.recordOutput("Shooter/Hood/Angle Error", error)
            error < Constants.HOOD_ANGLE_TOLERANCE.inDegrees()
        }

        private var io = when (Robot.model) {
            Model.SIMULATION -> HoodIOSim()
            Model.COMPETITION -> HoodIOReal()
        }
        private val inputs = LoggedHoodInputs()
        private val sysID = SysIdRoutine(
            SysIdRoutine.Config(
                null,
                null,
                null,
                { state -> Logger.recordOutput("SysIdTestState", state.toString()) }
            ),
            SysIdRoutine.Mechanism(
                io::setVoltage,
                null,
                this
            )
        )

        // distance -> sin(2 * angle)
        // TODO: find values
        private val angleInterpolationTable =
            if (io is HoodIOReal) {
                InterpolatingTreeMap(
                    InverseInterpolator.forDouble(),
                    Interpolator.forDouble()
                ).apply {
                    put(3.5, 0.866025403784)
                    put(4.0, 0.939692620786)
                    put(4.5, 0.984807753012)
                    put(5.0, 1.0)
                }
            } else {
                // https://www.desmos.com/calculator/vv0icacepe
                InterpolatingTreeMap(
                    InverseInterpolator.forDouble(),
                    Interpolator.forDouble()
                ).apply {
                    put(2.12, 55.0)
                    put(2.9, 50.0)
                    put(3.7, 45.0)
                    put(4.5, 40.0)
                    put(5.3, 35.0)
                }
            }

        override fun periodic() {
            io.updateInputs(inputs)
            Logger.processInputs("Shooter/Hood", inputs)
            Logger.recordOutput("Shooter/Hood/Shooter Target", shooterTarget.toString())
            Logger.recordOutput("Shooter/Hood/Reference", shooterTarget.hoodAngle)
        }

        fun sysIdQuasistatic(direction: Direction): Command = sysID.quasistatic(direction)

        fun sysIdDynamic(direction: Direction): Command = sysID.dynamic(direction)

        fun getHoodAngle(distance: Distance): Angle = (angleInterpolationTable.get(distance.inMeters())).degrees

        fun turnToTargetHoodAngle(): Command =
            run {
                io.turnToAngle(shooterTarget.hoodAngle)
            }

        fun hoodBrakeMode(): Command =
            run {
                io.setBrakeMode(true)
            }

        fun hoodCoastMode(): Command =
            run {
                io.setBrakeMode(false)
            }

        fun fixedHoodMode(): Command =
            run {
                fixedHood = true
            }

        fun adjustableHoodMode(): Command =
            run {
                fixedHood = false
            }
    }

    object Flywheel: Subsystem {
        val atDesiredFlywheelVelocity = Trigger {
            val error = abs((inputs.angularVelocity - shooterTarget.angularVelocity).inRPM())
            Logger.recordOutput("Shooter/Flywheel/Velocity Error", error)
            error < Constants.FLYWHEEL_VELOCITY_TOLERANCE.inRPM()
        }

        private var inputs = LoggedFlywheelInputs()
        private val io = when (Robot.model) {
            Model.SIMULATION -> FlywheelIOSim()
            Model.COMPETITION -> FlywheelIOReal()
        }
        private var sysID = SysIdRoutine(
            SysIdRoutine.Config(
                null,
                null,
                null,
                { state ->
                    Logger.recordOutput("SysIdTestState", state.toString())
                }
            ),
            SysIdRoutine.Mechanism(
                io::setVoltage,
                null, // recorded by URCL
                this
            )
        )

        // distance -> velocity^2
        // TODO: find values
        private val velocityInterpolationTable =
            if (io is FlywheelIOReal) {
                InterpolatingTreeMap(
                    InverseInterpolator.forDouble(),
                    Interpolator.forDouble()
                ).apply {
                    put(3.5, (300.0.pow(2)))
                    put(4.0, 350.0.pow(2))
                    put(4.5, 400.0.pow(2))
                    put(5.0, 450.0.pow(2))
                }
            } else {
                InterpolatingTreeMap(
                    InverseInterpolator.forDouble(),
                    Interpolator.forDouble()
                ).apply {
                    put(2.12, (6.51 * Constants.ANGULAR_TO_LINEAR_RATIO).pow(2))
                    put(2.9, (7.05 * Constants.ANGULAR_TO_LINEAR_RATIO).pow(2))
                    put(3.7, (7.72 * Constants.ANGULAR_TO_LINEAR_RATIO).pow(2))
                    put(4.5, (8.52416 * Constants.ANGULAR_TO_LINEAR_RATIO).pow(2))
                    put(5.3, (9.52215 * Constants.ANGULAR_TO_LINEAR_RATIO).pow(2))
                }
            }

        override fun periodic() {
            io.updateInputs(inputs)
            Logger.processInputs("Flywheel", inputs)
            Logger.recordOutput("Shooter/Flywheel/Desired Velocity", shooterTarget.angularVelocity)
        }

        fun sysIdQuasistatic(direction: Direction): Command = sysID.quasistatic(direction)

        fun sysIdDynamic(direction: Direction): Command = sysID.dynamic(direction)

        fun getFlywheelVelocity(distance: Distance): AngularVelocity = velocityInterpolationTable.get(distance.inMeters()).rpm
        fun getSimFuelVelocity(distance: Distance): LinearVelocity = (sqrt(velocityInterpolationTable.get(distance.inMeters())) / Constants.ANGULAR_TO_LINEAR_RATIO).metersPerSecond

        // TODO: Command waits until the measured speed is at that amount (startEnd).
        fun toTargetSpeed(): Command = run {
            io.setVelocity(shooterTarget.angularVelocity)
        }
    }

    data class ShooterProfile(
        val turretError: Angle,
        val turretAngle: Angle,
        val hoodAngle: Angle,
        val angularVelocity: AngularVelocity,
    )

    val hoodTunable = LoggedNetworkNumber("/Tuning/HoodTestAngle", 40.0)
    val flywheelTunable = LoggedNetworkNumber("/Tuning/FlywheelSpeed", 1000.0)
    var shooterTarget = Target.STOWED.profile

    val hubTranslation
        get() = when (Robot.model) {
            // simulation defaults to red alliance
            Model.SIMULATION -> Translation3d(
                (16.54 - 4.62534).meters,
                (8.07 / 2).meters,
                1.83.meters,
            )
            Model.COMPETITION -> when (DriverStation.getAlliance().get()) {
                Alliance.Blue -> Translation3d(
                    4.62534.meters,
                    (8.07 / 2).meters,
                    1.83.meters,
                )
                Alliance.Red -> Translation3d(
                    (16.54 - 4.62534).meters,
                    (8.07 / 2).meters,
                    1.83.meters,
                )
            }
        }

    private val nearestHubTranslation: Translation3d
        get() = if (Drivetrain.estimatedPose.y < FIELD_HEIGHT_METERS /2.0){
            Translation3d(
            4.62534.meters,
            (8.07 / 2).meters,
            1.83.meters,
            )
        } else {
            Translation3d(
            (16.54 - 4.62534).meters,
            (8.07 / 2).meters,
            1.83.meters,
            )
        }

    private val translationToHub: Translation2d
        get() {
            if (Robot.model == Model.COMPETITION) {
                return hubTranslation.toTranslation2d() - Drivetrain.estimatedPose.translation
            }
            else {
                return hubTranslation.toTranslation2d() - Drivetrain.getSwerveDriveSimulation().simulatedDriveTrainPose.translation
            }
        }

    private val turretTranslationToHub: Translation2d
        get() {
            if (Robot.model == Model.COMPETITION) {
                val pose = Drivetrain.estimatedPose
                return hubTranslation.toTranslation2d() - (pose.translation + Constants.SHOOTER_OFFSET.rotateBy(pose.rotation))
            }
            else {
                val pose = Drivetrain.getSwerveDriveSimulation().simulatedDriveTrainPose
                return hubTranslation.toTranslation2d() - (pose.translation + Constants.SHOOTER_OFFSET.rotateBy(pose.rotation))
            }
        }


    fun setTarget(target: ShooterProfile): Command = Commands.run({
        shooterTarget = target
    })

    fun shootAtTranslation(target: Translation2d = Turret.getClosetTarget()): Command {
        var shooterProfile = getTurretProfileFromTranslation2d(target)
        if (target == hubTranslation.toTranslation2d()) {
            shooterProfile = Target.AIM_AT_HUB.profile
        }

        return Commands.sequence(
            setTarget(shooterProfile),
            Commands.parallel(
                Turret.turnToTargetTurretAngle(),
                Hood.turnToTargetHoodAngle(),
                Flywheel.toTargetSpeed()
            )
        )
    }

    fun shootSequence(target: Target): Command = Commands.sequence(
        setTarget(target.profile),
        Commands.parallel(
            Turret.alignToHub(target.profile.turretError),
            Turret.turnToTargetTurretAngle(),
            Hood.turnToTargetHoodAngle(),
            Flywheel.toTargetSpeed(),
        )
    )

    fun simSequence(): Command = Commands.run(
        {
//            if (Intake.IntakeSimulation.gamePiecesAmount == 0) {
//                Intake.IntakeSimulation.addGamePiecesToIntake(40)
//            }
            val adjustedVector = simAdjustedVector
            val turretAngle = (atan(adjustedVector[1, 0] / adjustedVector[2, 0]) - Drivetrain.getSwerveDriveSimulation().simulatedDriveTrainPose.rotation.radians)
            val velocity = adjustedVector.norm().metersPerSecond
            Logger.recordOutput("Shooter/Flywheel/velocity", velocity)
            if (Intake.IntakeSimulation.obtainGamePieceFromIntake()) {
                SimulatedArena.getInstance().addGamePieceProjectile(
                    RebuiltFuelOnFly(
                        Drivetrain.getSwerveDriveSimulation().simulatedDriveTrainPose.translation,
                        Constants.SHOOTER_OFFSET,
                        DrivetrainIOSim().swerveDriveSimulation.driveTrainSimulatedChassisSpeedsFieldRelative,
                        Rotation2d(turretAngle.radians),
                            0.3835.meters,
                            velocity,
                            Hood.getHoodAngle(turretTranslationToHub.norm.meters)
                    ).withTargetPosition {
                        hubTranslation
                    }.withTargetTolerance(
                        Translation3d(
                            (41.7 / 2).inches,
                            (41.7 / 2).inches,
                            0.inches
                        )
                    )// TODO: Fix logging the correct trajectory
                    //.withProjectileTrajectoryDisplayCallBack {
//                        (poses) -> Logger.recordOutput("successfulShotsTrajectory", poses.toArra
//                            (poses) -> Logger.recordOutput("successfulShotsTrajectory", poses.toArray(Pose3d[]::new))
//                    }.
                )
            }
        }
    )

    fun getTurretProfileFromTranslation2d(targetTranslation: Translation2d) : ShooterProfile{
        val toTarget = Drivetrain.estimatedPose.translation - targetTranslation
        var hubDistance = nearestHubTranslation.toTranslation2d().getDistance(Drivetrain.estimatedPose.translation)
        if(Drivetrain.estimatedPose.y < FIELD_WIDTH_METERS / 2.0){
            hubDistance = toTarget.norm / 2.0
        }
        val parabolicA = hubDistance / (toTarget.norm.pow(2.0) + (toTarget.norm * hubDistance))
        val parabolicB = (-parabolicA * toTarget.norm)
        val parabolicSolutions = Pair(
            -parabolicB - sqrt(parabolicB.pow(2) - (4 * parabolicA * hubTranslation.y)),
            -parabolicB + sqrt(parabolicB.pow(2) - (4 * parabolicA * hubTranslation.y))
        )
        val adjustedDistance = max(parabolicSolutions.first, parabolicSolutions.second).meters
        val targetLinearVelocity = Flywheel.getFlywheelVelocity(adjustedDistance).toLinear(Constants.FLYWHEEL_RADIUS)
        val targetHoodAngle = Hood.getHoodAngle(adjustedDistance)
        val horizontalVelocity = targetLinearVelocity.getHorizontalComponent(targetHoodAngle)
        val targetVelocityVector = VecBuilder.fill(
            horizontalVelocity.getHorizontalComponent(toTarget.angle.measure).inMetersPerSecond(),
            horizontalVelocity.getVerticalComponent(toTarget.angle.measure).inMetersPerSecond(),
            targetLinearVelocity.getVerticalComponent(targetHoodAngle).inMetersPerSecond()
        )
        val robotVelocity = Drivetrain.measuredChassisSpeedsRelativeToField.translation2dPerSecond
        val robotVelocityVector = VecBuilder.fill(robotVelocity.x, robotVelocity.y, 0.0)
        val adjustedVector = targetVelocityVector - robotVelocityVector
        val angleError = acos(adjustedVector.dot(targetVelocityVector) / (adjustedVector.norm() * targetVelocityVector.norm())).radians
        return getProfile(adjustedVector, angleError)
    } 

    val targetVelocityVector: Vector<N3>
        get() {
            if (Robot.model == Model.COMPETITION) {
                val targetHoodAngle = Hood.getHoodAngle(turretTranslationToHub.norm.meters)
                val targetLinearVelocity = Flywheel.getFlywheelVelocity(turretTranslationToHub.norm.meters).toLinear(Constants.FLYWHEEL_RADIUS)
                val horizontalVelocity = targetLinearVelocity.getHorizontalComponent(targetHoodAngle)
                return VecBuilder.fill(
                    horizontalVelocity.getHorizontalComponent(turretTranslationToHub.angle.measure).inMetersPerSecond(),
                    horizontalVelocity.getVerticalComponent(turretTranslationToHub.angle.measure).inMetersPerSecond(),
                    targetLinearVelocity.getVerticalComponent(targetHoodAngle).inMetersPerSecond()
                )
            } else {
                val targetHoodAngle = Hood.getHoodAngle(turretTranslationToHub.norm.meters)
                val targetLinearVelocity = Flywheel.getSimFuelVelocity(turretTranslationToHub.norm.meters)
                val horizontalVelocity = targetLinearVelocity.getHorizontalComponent(targetHoodAngle)
                return VecBuilder.fill(
                    horizontalVelocity.getHorizontalComponent(turretTranslationToHub.angle.measure).inMetersPerSecond(),
                    horizontalVelocity.getVerticalComponent(turretTranslationToHub.angle.measure).inMetersPerSecond(),
                    targetLinearVelocity.getVerticalComponent(targetHoodAngle).inMetersPerSecond()
                )
            }
        }

    val adjustedVector: Vector<N3>
        get() {
            val robotVelocity = Drivetrain.measuredChassisSpeedsRelativeToField.translation2dPerSecond
            val robotVelocityVector = VecBuilder.fill(robotVelocity.x, robotVelocity.y, 0.0)
            return targetVelocityVector - robotVelocityVector
        }

    val simAdjustedVector: Vector<N3>
        get() {
            val robotVelocity = Drivetrain.getSwerveDriveSimulation().linearVelocity
            val robotVelocityVector = VecBuilder.fill(robotVelocity.x, robotVelocity.y, 0.0)
            return targetVelocityVector - robotVelocityVector
        }

    val angleError: Angle
        get() {
            return acos(adjustedVector.dot(targetVelocityVector) / (adjustedVector.norm() * targetVelocityVector.norm())).radians
        }

    fun getProfile(vector: Vector<N3>, error: Angle): ShooterProfile {
        val turretAngle = (atan(vector[1,0] / vector[0,0]) - Drivetrain.estimatedPose.rotation.radians).radians
        val velocity = (vector.norm() / Constants.FLYWHEEL_RADIUS.inMeters() * TAU).rpm
        val hoodAngle = atan(vector[2,0]/(sqrt(vector[0,0].pow(2) + vector[1,0]))).radians
        return ShooterProfile(
            error,
            turretAngle,
            hoodAngle,
            velocity,
        )
    }

    fun fixedHoodProfile(vector: Vector<N3>, error: Angle): ShooterProfile {
        val turretAngle = (atan(vector[1,0] / vector[0,0]) - Drivetrain.estimatedPose.rotation.radians).radians
        val velocity = (sqrt(vector[0, 0].pow(2) + vector[1, 0].pow(2) + vector[2, 0].pow(2)) /
                Constants.FLYWHEEL_RADIUS.inMeters() * TAU).rpm
        return ShooterProfile(
            error,
            turretAngle,
            Constants.FIXED_HOOD_ANGLE,
            velocity
        )
    }

    fun registerSubsystems() {
        Hood.register()
        Flywheel.register()
        Turret.register()
    }

    enum class Target(val profile: ShooterProfile) {
        //AIM_AT_POSE(getTurretProfileFromTranslation2d(Turret.getClosetTarget())),
        AIM_AT_HUB(getProfile(targetVelocityVector, angleError)),
//        AIM_WITHOUT_HOOD(
//            vectorToFixedHoodShooterProfile(adjustedVelocityVectorAndError)
//        ),
        STOWED(ShooterProfile(
            0.0.radians,
            0.0.radians,
            40.degrees.inRadians().radians,
            0.rpm
        )),
        TUNING(ShooterProfile(
            0.0.radians,
            0.0.radians,
            hoodTunable.get().degrees,
            flywheelTunable.get().rpm
        ))
    }

    object Constants {
        val FLYWHEEL_RADIUS = 0.0505.meters
        val HOOD_ANGLE_TOLERANCE = 3.0.degrees
        val FLYWHEEL_VELOCITY_TOLERANCE = 100.rpm
        val FIXED_HOOD_ANGLE = 40.radians
        val SHOOTER_OFFSET = Translation2d(.184, -.184)
        val ANGULAR_TO_LINEAR_RATIO = 18.0 // arbitrary ratio between flywheel rpm and fuel mps
    }

    enum class FeedPose(val target : Translation2d) {
        LeftSideNeutralZone(Translation2d(3.0.meters,3.0.meters)),
        RightSideNeutralZone(Translation2d(3.0.meters,3.0.meters)),
        LeftSideAllianceZone(Translation2d(3.0.meters,3.0.meters)),
        RightSideAllianceZone(Translation2d(3.0.meters,3.0.meters)),
    }

    enum class Zones(val startY : Distance, val endY : Distance){
        BlueAllianceZone(0.meters, 4.03.meters),
        RedAllianceZone(4.03.meters, 11.22.meters),
        NeutralZone(11.22.meters, 15.23.meters),
    }

}




