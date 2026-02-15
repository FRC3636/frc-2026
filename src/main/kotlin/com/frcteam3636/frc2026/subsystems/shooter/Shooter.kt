package com.frcteam3636.frc2026.subsystems.shooter

import com.frcteam3636.frc2026.Robot
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.utils.flipHorizontally
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
import edu.wpi.first.wpilibj.DriverStation.Alliance
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
                }
        }
        fun getClosetTarget() : Translation2d{
            var ourAllianceZone = Zones.BlueAllianceZone
            var opposingAllianceZone = Zones.RedAllianceZone
            if(DriverStation.getAlliance().get() == Alliance.Red){
                ourAllianceZone = Zones.BlueAllianceZone
                opposingAllianceZone = Zones.RedAllianceZone
            }

            val target =  when(Drivetrain.estimatedPose.x){
                in ourAllianceZone.startX.inMeters()..ourAllianceZone.endX.inMeters() -> Constants.hubTranslation.toTranslation2d()
                in opposingAllianceZone.startX.inMeters()..ourAllianceZone.endX.inMeters() -> FeedPose.RightSideNeutralZone.target
                else -> FeedPose.RightSideAllianceZone.target
            }
            if(Drivetrain.estimatedPose.translation.y < 4.035.meters.inMeters()){
                return target.flipHorizontally()
            }
            return target
            }



        fun turnToTargetTurretAngle(): Command =
            run {
                io.turnToAngle(Hood.target.turretAngle)
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
        val test = FeedPose.RightSideNeutralZone as Translation2d
    }

    object Hood: Subsystem {
        private var io = when (Robot.model) {
            Robot.Model.SIMULATION -> HoodIOSim()
            Robot.Model.COMPETITION -> HoodIOReal()
        }

        val inputs = LoggedHoodInputs()
        var target = Target.STOWED.profile
        var fixedHood = false

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
                val error = abs((inputs.hoodAngle - target.hoodAngle).inDegrees())
                Logger.recordOutput("Shooter/Hood/Angle Error", error)
                error < Constants.HOOD_ANGLE_TOLERANCE.inDegrees()
            }

        override fun periodic() {
            io.updateInputs(inputs)
            Logger.processInputs("Shooter/Hood", inputs)
            Logger.recordOutput("Shooter/Hood/Shooter Target", target.target)
            Logger.recordOutput("Shooter/Hood/Reference", target.hoodAngle)
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

        fun setTarget(target: ShooterProfile): Command =
            runOnce {
                Hood.target = target
            }

        fun turnToTargetHoodAngle(): Command =
            run {
                io.turnToAngle(target.hoodAngle)
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
                val error = abs((inputs.angularVelocity - Hood.target.angularVelocity).inRPM())
                Logger.recordOutput("Shooter/Flywheel/Velocity Error", error)
                error < Constants.FLYWHEEL_VELOCITY_TOLERANCE.inRPM()
            }

        override fun periodic() {
            io.updateInputs(inputs)
//            Logger.processInputs("Flywheel", inputs)
            Logger.recordOutput("Shooter/Flywheel/Desired Velocity", Hood.target.angularVelocity)
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
            io.setVelocity(Hood.target.angularVelocity)
        }
    }
    fun shootAtTranslation(target: Translation2d = Turret.getClosetTarget()): Command {
        var shooterProfile = getTurretProfileFromTranslation2d(target)
        if(target == Constants.hubTranslation.toTranslation2d()){
          shooterProfile = Target.AIM_AT_HUB.profile
        }
        return Commands.sequence(
            Hood.setTarget(shooterProfile),
            Commands.parallel(
                Turret.turnToTargetTurretAngle(),
                Hood.turnToTargetHoodAngle(),
                Flywheel.setSpeed()
            )
        )
    }

    fun shootSequence(target: Target): Command =
        Commands.sequence(
            Hood.setTarget(target.profile),
            Commands.parallel(
                Turret.alignToHub(target.profile.turretError),
                Turret.turnToTargetTurretAngle(),
                Hood.turnToTargetHoodAngle(),
                Flywheel.setSpeed(),
            )
        )

//    fun simSequence(target: Target): Command =
//        Commands.sequence(
//            Hood.setTarget(target),
//            Commands.run({
//                SimulatedArena.getInstance()
//                    .addGamePiece(RebuiltFuelOnField(
//                        DrivetrainIOSim
//                    ))
//            })
//        )

    private val distanceToHub: Translation2d
        get() {
            return Constants.hubTranslation.toTranslation2d() - Drivetrain.estimatedPose.translation
        }

    data class ShooterProfile(
        val turretError: Angle,
        val turretAngle: Angle,
        val hoodAngle: Angle,
        val angularVelocity: AngularVelocity,
        val target: Translation2d
    )

    fun getTurretProfileFromTranslation2d(targetTranslation : Translation2d) : ShooterProfile{

        val distanceToPose = Drivetrain.estimatedPose.translation.minus(targetTranslation)
        val distanceToHub = Constants.hubTranslation.toTranslation2d().getDistance(Drivetrain.estimatedPose.translation)
        val parabolicA = (distanceToHub)/(distanceToPose.norm.meters.inMeters().pow(2) + (distanceToPose.norm.meters * distanceToHub).inMeters())
        val parabolicB = (-parabolicA * distanceToPose.norm)
        val parabolicSolutions = Pair(
            (-parabolicB - sqrt(parabolicB.pow(2) - (4 * parabolicA * Constants.hubTranslation.y)))
            ,(-parabolicB + sqrt(parabolicB.pow(2) - (4 * parabolicA * Constants.hubTranslation.y)))
        )
        val adjustedDistance = max(parabolicSolutions.first, parabolicSolutions.second).meters
        val targetLinearVelocity = Flywheel.getFlywheelVelocity(adjustedDistance).toLinear(Constants.FLYWHEEL_RADIUS)
        val targetHoodAngle = Hood.getHoodAngle(adjustedDistance)
        val horizontalVelocity = targetLinearVelocity.getHorizontalComponent(targetHoodAngle)
        val targetVelocityVector = VecBuilder.fill(
            horizontalVelocity.getHorizontalComponent(distanceToPose.angle.measure).inMetersPerSecond(),
            horizontalVelocity.getVerticalComponent(distanceToPose.angle.measure).inMetersPerSecond(),
            targetLinearVelocity.getVerticalComponent(targetHoodAngle).inMetersPerSecond()
        )
        val robotVelocity = Drivetrain.measuredChassisSpeedsRelativeToField.translation2dPerSecond
        val robotVelocityVector = VecBuilder.fill(robotVelocity.x, robotVelocity.y, 0.0)
        val adjustedVector = targetVelocityVector - robotVelocityVector
        val angleError = acos(adjustedVector.dot(targetVelocityVector) / (adjustedVector.norm() * targetVelocityVector.norm())).radians
        return vectorToShooterProfile(Triple(adjustedVector, angleError, targetTranslation))
    }

    val adjustedVelocityVectorAndError: Triple<Vector<N3>, Angle, Translation2d>
        get() {
            val target = Constants.hubTranslation.toTranslation2d()
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
            return Triple(adjustedVector, angleError, target)
        }



    fun vectorToShooterProfile(vectorAndAngle: Triple<Vector<N3>, Angle, Translation2d>): ShooterProfile {
        val (vector, error, target) = vectorAndAngle
        val turretAngle = (atan(vector[1,0] / vector[0,0]) - Drivetrain.estimatedPose.rotation.radians).radians
        val velocity = (vector.norm() / Constants.FLYWHEEL_RADIUS.inMeters() * TAU).rpm
        val hoodAngle = atan(vector[2,0]/(sqrt(vector[0,0].pow(2) + vector[1,0]))).radians
        return ShooterProfile(
            error,
            turretAngle,
            hoodAngle,
            velocity,
            target
        )
    }

    fun vectorToFixedHoodShooterProfile(vectorAndAngle: Triple<Vector<N3>, Angle, Translation2d>): ShooterProfile{
        val (vector,error, target) = vectorAndAngle
        val turretAngle = (atan(vector[1,0] / vector[0,0]) - Drivetrain.estimatedPose.rotation.radians).radians
        val velocity = (sqrt(vector[0, 0].pow(2) + vector[1, 0].pow(2) + vector[2, 0].pow(2)) /
                Constants.FLYWHEEL_RADIUS.inMeters() * TAU).rpm
        return ShooterProfile(
            error,
            turretAngle,
            Constants.FIXED_HOOD_ANGLE,
            velocity,
            target
        )
    }

    enum class Target(val profile: ShooterProfile) {
        
        AIM_AT_HUB(vectorToShooterProfile(adjustedVelocityVectorAndError)),

        AIM_WITHOUT_HOOD(
            vectorToFixedHoodShooterProfile(adjustedVelocityVectorAndError)
        ),


        STOWED(
            ShooterProfile(

                    0.0.radians
                ,

                    0.0.radians
                ,

                    40.degrees.inRadians().radians
                ,

                    0.rpm
                ,

                    Translation2d()

            )
        ),

        TUNING(
            ShooterProfile(

                    0.0.radians
                ,

                    0.0.radians
                ,

                    hoodTunable.get().degrees.inRadians().radians
                ,

                    flywheelTunable.get().rpm
                ,

                Translation2d()

            )
        )

    }

    val hoodTunable = LoggedNetworkNumber("/Tuning/HoodTestAngle", 40.0)
    val flywheelTunable = LoggedNetworkNumber("/Tuning/FlywheelSpeed", 1000.0)

    object Constants {
        val FLYWHEEL_RADIUS = 0.0505.meters
        val HOOD_ANGLE_TOLERANCE = 3.0.degrees
        val FLYWHEEL_VELOCITY_TOLERANCE = 100.rpm
        val FIXED_HOOD_ANGLE = 40.radians


        val hubTranslation
            get() = when (DriverStation.getAlliance().get()) {
                Alliance.Blue  -> Translation3d(
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

    enum class FeedPose(val target : Translation2d) {
        LeftSideNeutralZone(Translation2d(3.0.meters,3.0.meters)),
        RightSideNeutralZone(Translation2d(3.0.meters,3.0.meters)),
        LeftSideAllianceZone(Translation2d(3.0.meters,3.0.meters)),
        RightSideAllianceZone(Translation2d(3.0.meters,3.0.meters)),
    }

    enum class Zones(val startX : Distance, val endX : Distance){
        BlueAllianceZone(0.meters, 4.03.meters),
        RedAllianceZone(4.03.meters, 11.22.meters),
        NeutralZone(11.22.meters, 15.23.meters),
    }

}




