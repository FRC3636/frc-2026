package com.frcteam3636.frc2026.subsystems.shooter

import com.frcteam3636.frc2026.Robot
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.shooter.Shooter.Turret.hubTranslation
import com.frcteam3636.frc2026.utils.math.*
import com.frcteam3636.frc2026.utils.swerve.translation2dPerSecond
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import kotlin.math.*

object Shooter {
    object Turret : Subsystem{

        private var io = when (Robot.model) {
            Robot.Model.SIMULATION -> TurretIOSim()
            Robot.Model.COMPETITION -> TurretIOReal()
        }

        private val inputs = LoggedTurretInputs()

        private val turretLimelight = NetworkTableInstance.getDefault().getTable("turret-limelight")
        private val seeTagsDebouncer = Debouncer(0.5)
        private var seeTagsRaw = false
            set(value) {
                inputs.seeTags = seeTagsDebouncer.calculate(value)
                field = value
            }

        override fun periodic() {
            io.updateInputs(inputs)
            seeTagsRaw = turretLimelight.getEntry("tv").equals(1)
            Logger.processInputs("Shooter/Turret", inputs)
            Logger.recordOutput("Shooter/Turret/Distance from hub", distanceToHub)
        }

        fun alignToHub(offset: Angle = 0.0.radians): Command =
            run {
                val camError = (turretLimelight.getEntry("tx").getDouble(0.0).degrees - offset.inDegrees().degrees).inRadians()
                // align with limelight
                if (inputs.seeTags) {
                    val kP = -0.1
                    io.turnToAngle(inputs.turretAngle + (camError * kP).radians)
                } else {
                    // if no tags are seen then align with estimated pose
                    val hubTranslation = DriverStation.getAlliance()
                        .orElse(DriverStation.Alliance.Blue)
                        .hubTranslation
                    val turretAngle = (atan((hubTranslation.y - Drivetrain.estimatedPose.y) / (hubTranslation.x - Drivetrain.estimatedPose.x)) +
                            offset.inRadians() -
                            Drivetrain.estimatedPose.rotation.radians).radians
                    turnToTargetTurretAngle(turretAngle)
            }
        }

        fun turnToTargetTurretAngle(angle: Angle): Command =
            run {
                io.turnToAngle(angle)
            }

        fun turretBrakeMode(): Command =
            run {
                io.setBrakeMode(true)
            }

        fun turretCoastMode(): Command =
            run {
                io.setBrakeMode(false)
            }

        val sysID = SysIdRoutine(
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

        fun sysIdQuasistatic(direction: Direction): Command = sysID.quasistatic(direction)

        fun sysIdDynamic(direction: Direction): Command = sysID.dynamic(direction)

        val DriverStation.Alliance.hubTranslation
            get() = when (this) {
                DriverStation.Alliance.Blue -> Translation3d(
                    4.62534.meters,
                    (8.07 / 2).meters,
                    1.83.meters,
                )

                else -> Translation3d(
                    (16.54 - 4.62534).meters,
                    (8.07 / 2).meters,
                    1.83.meters,
                )
            }
    }

    object Hood: Subsystem {
        private var io = when (Robot.model) {
            Robot.Model.SIMULATION -> HoodIOSim()
            Robot.Model.COMPETITION -> HoodIOReal()
        }

        val inputs = LoggedHoodInputs()
        var target = Target.STOWED

        // distance -> sin(2 * angle)
        // TODO: find values
        private val angleInterpolationTable = InterpolatingTreeMap(
            InverseInterpolator.forDouble(),
            Interpolator.forDouble()
        ).apply {
            put(1.0, 45.0)
            put(1.5, 40.0)
        }

        val atDesiredHoodAngle =
            Trigger {
                val error = abs((inputs.hoodAngle - target.profile.getAngle()).inDegrees())
                Logger.recordOutput("Shooter/Hood/Angle Error", error)
                error < Constants.HOOD_ANGLE_TOLERANCE.inDegrees()
            }

        override fun periodic() {
            io.updateInputs(inputs)
            Logger.processInputs("Shooter/Hood", inputs)
            Logger.recordOutput("Shooter/Hood/Active profile", target)
            Logger.recordOutput("Shooter/Hood/Reference", target.profile.getAngle())
        }

        val sysID = SysIdRoutine(
            SysIdRoutine.Config(
                null,
                null,
                null,
                {state -> Logger.recordOutput("SysIdTestState", state.toString())}
            ),
            SysIdRoutine.Mechanism(
                io::setVoltage,
                null,
                this
            )
        )

        fun sysIdQuasistatic(direction: Direction): Command = sysID.quasistatic(direction)

        fun sysIdDynamic(direction: Direction): Command = sysID.dynamic(direction)

        fun getHoodAngle(distance: Distance): Angle = (asin(angleInterpolationTable.get(distance.inMeters())) / 2).radians

        fun setTarget(target: Target): Command =
            runOnce {
                Hood.target = target
            }

        fun turnToTargetHoodAngle(): Command =
            run {
                io.turnToAngle(target.profile.getAngle())
            }

        fun hoodBrakeMode(): Command =
            run {
                io.setBrakeMode(true)
            }

        fun hoodCoastMode(): Command =
            run {
                io.setBrakeMode(false)
            }

    }

    object Flywheel: Subsystem {

        private val io: FlywheelIO = when (Robot.model) {
            Robot.Model.SIMULATION -> TODO()
            Robot.Model.COMPETITION -> FlywheelIOReal()
        }

        var inputs = FlywheelInputs()

        // distance -> velocity^2
        // TODO: find values
        val velocityInterpolationTable = InterpolatingTreeMap(
            InverseInterpolator.forDouble(),
            Interpolator.forDouble()
        ).apply {
            put(1.0, 3000.0)
            put(1.5, 4500.0)
        }

        val atDesiredFlywheelVelocity =
            Trigger {
                val error = abs((inputs.angularVelocity - Hood.target.profile.getVelocity()).inRPM())
                Logger.recordOutput("Shooter/Flywheel/Velocity Error", error)
                error < Constants.FLYWHEEL_VELOCITY_TOLERANCE.inRPM()
            }

        override fun periodic() {
            io.updateInputs(inputs)
//            Logger.processInputs("Flywheel", inputs)
            Logger.recordOutput("Shooter/Flywheel/Desired Velocity", Hood.target.profile.getVelocity())
        }

        @Suppress("Unused")
        var sysID = SysIdRoutine(
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

        @Suppress("unused")
        fun sysIdQuasistatic(direction: Direction): Command = sysID.quasistatic(direction)

        @Suppress("unused")
        fun sysIdDynamic(direction: Direction): Command = sysID.dynamic(direction)

        fun getFlywheelVelocity(distance: Distance): AngularVelocity = velocityInterpolationTable.get(distance.inMeters()).rpm

        fun setVoltage(volts: Voltage): Command = startEnd(
            {
                io.setVoltage(volts)
            },
            {
                io.setVoltage(0.volts)
            }
        )

        fun setSpeed(): Command = run {
            io.setVelocity(Hood.target.profile.getVelocity())
        }
    }

    fun shootSequence(target: Target): Command =
        Commands.sequence(
            Hood.setTarget(target),
            Commands.parallel(
                Turret.alignToHub(),
                Hood.turnToTargetHoodAngle(),
                Flywheel.setSpeed(),
            )
        )

    private val distanceToHub: Translation2d
        get() {
            return DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .hubTranslation.toTranslation2d() - Drivetrain.estimatedPose.translation
        }

    data class ShooterProfile(
        val getHoodOffset: () -> Angle,
        val getAngle: () -> Angle,
        val getVelocity: () -> AngularVelocity
    )

    val getAdjustedVelocityVectorAndError: Pair<Vector<N3>, Angle>
        get() {
            val distance = distanceToHub
            val targetHoodAngle = Hood.getHoodAngle(distance.norm.meters)
            val targetLinearVelocity =
                Flywheel.getFlywheelVelocity(distance.norm.meters).toLinear(Constants.FLYWHEEL_RADIUS)
            val horizontalVelocity = targetLinearVelocity.getHorizontalComponent(targetHoodAngle)
            val targetVelocityVector = VecBuilder.fill(
                horizontalVelocity.getHorizontalComponent(distance.angle.measure).inMetersPerSecond(),
                horizontalVelocity.getVerticalComponent(distance.angle.measure).inMetersPerSecond(),
                targetLinearVelocity.getVerticalComponent(targetHoodAngle).inMetersPerSecond()
            )
            val robotVelocity = Drivetrain.measuredChassisSpeedsRelativeToField.translation2dPerSecond
            val robotVelocityVector = VecBuilder.fill(robotVelocity.x, robotVelocity.y, 0.0)
            val adjustedVector = targetVelocityVector - robotVelocityVector
            val angleError = acos(adjustedVector.dot(targetVelocityVector) / (adjustedVector.norm() * targetVelocityVector.norm())).radians
            return Pair(adjustedVector, angleError)
        }

    fun vectorToShooterProfile(vectorAndAngle: Pair<Vector<N3>, Angle>): ShooterProfile {
        val (vector, error) = vectorAndAngle
        val turretOffset = atan(vector[1, 0] / vector[0, 0]).radians
        val velocity = (sqrt(vector[0, 0].pow(2) + vector[1, 0].pow(2) + vector[2, 0].pow(2)) /
                Constants.FLYWHEEL_RADIUS.inMeters() * TAU).rpm
        return ShooterProfile(
            {turretOffset},
            {error},
            {velocity},
        )
    }

    enum class Target(val profile: ShooterProfile) {

        AIM(vectorToShooterProfile(getAdjustedVelocityVectorAndError)),

        STOWED(
            ShooterProfile(
                {
                    0.0.radians
                },
                {
                    40.degrees.inRadians().radians
                },
                {
                    0.rpm
                }
            )
        ),

        TUNING(
            ShooterProfile(
                {
                    0.0.radians
                },
                {
                    hoodTunable.get().degrees.inRadians().radians
                },
                {
                    flywheelTunable.get().rpm
                }
            )
        )

    }

    val hoodTunable = LoggedNetworkNumber("/Tuning/HoodTestAngle", 40.0)
    val flywheelTunable = LoggedNetworkNumber("/Tuning/FlywheelSpeed", 1000.0)

    object Constants {
        val FLYWHEEL_RADIUS = 0.0505.meters
        val HOOD_ANGLE_TOLERANCE = 3.0.degrees
        val FLYWHEEL_VELOCITY_TOLERANCE = 100.rpm
    }
}
