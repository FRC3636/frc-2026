package com.frcteam3636.frc2026.subsystems.drivetrain

import com.frcteam3636.frc2026.subsystems.climber.Climber
import com.frcteam3636.frc2026.subsystems.intake.Intake
import com.frcteam3636.frc2026.subsystems.shooter.Target
import com.frcteam3636.frc2026.subsystems.shooter.setShooterTarget
import com.frcteam3636.frc2026.subsystems.shooter.shoot
import com.frcteam3636.frc2026.utils.autos.APTargetWithTolerance
import com.frcteam3636.frc2026.utils.autos.alignToClimbLeft
import com.frcteam3636.frc2026.utils.autos.flipTarget
import com.frcteam3636.frc2026.utils.autos.isRedAlliance
import com.frcteam3636.frc2026.utils.math.inSeconds
import com.frcteam3636.frc2026.utils.math.meters
import com.frcteam3636.frc2026.utils.math.metersPerSecond
import com.frcteam3636.frc2026.utils.math.radians
import com.frcteam3636.frc2026.utils.math.seconds
import com.frcteam3636.frc2026.utils.math.volts
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import kotlin.math.PI

interface Auto {
    fun getPath(flipH: Boolean, flipV: Boolean): Command
}

object Lebron : Auto {
    override fun getPath(flipH: Boolean, flipV: Boolean): Command =
        Commands.sequence(
            runOnce({
                Drivetrain.poseEstimator.resetPose(
                    flipTarget(
                        Targets.Start.target,
                        flipV = flipV,
                        flipH = flipH
                    ).reference
                )
            }),
            runOnce({
                setShooterTarget(Target.STATIONARY_TURRET)
            }),
            Commands.parallel(
                Intake.setPivotVoltage(10.volts).withTimeout(0.1.seconds),
                Drivetrain.alignAndFlip(Targets.Start.target, flipH, flipV),
            ),
            Commands.race(
                Intake.intakeSequence(),
                Commands.sequence(
                    Drivetrain.alignAndFlip(Targets.Target8.target, flipH, flipV),
                    Drivetrain.alignAndFlip(Targets.Target3.target, flipH, flipV),
                    Drivetrain.alignAndFlip(Targets.Target4.target, flipH, flipV),
                    Drivetrain.alignAndFlip(Targets.Target5.target, flipH, flipV),
                    Drivetrain.alignAndFlip(Targets.Target8.target, flipH, flipV),
                    Drivetrain.alignAndFlip(Targets.Target9.target, flipH, flipV),
                    Drivetrain.alignAndFlip(Targets.Target10.target, flipH, flipV),
                )
            ),
            setShooterTarget(Target.PASS_NO_SOTM),
            Commands.parallel(
                Commands.run({Drivetrain.stop()}),
                shoot(),
            ).withTimeout(3.seconds),
            setShooterTarget(Target.STATIONARY_TURRET),
            Drivetrain.alignAndFlip(Targets.Target9.target, flipH, flipV),
            Commands.race(
                Intake.intakeSequence(),
                Commands.sequence(
                    Drivetrain.alignAndFlip(Targets.Target8.target, flipH, flipV),
                    Drivetrain.alignAndFlip(Targets.Target3.target, flipH, flipV),
                    Drivetrain.alignAndFlip(Targets.Target4.target, flipH, flipV),
                    Drivetrain.alignAndFlip(Targets.Target5.target, flipH, flipV),
                )
            )
        )

    enum class Targets(val target: APTargetWithTolerance) {
        Start(APTargetWithTolerance(Pose2d(4.364.meters, 0.500.meters, Rotation2d(3.142.radians)))),
        Target3(APTargetWithTolerance(Pose2d(7.621.meters, 1.425.meters, Rotation2d(PI.radians))).withVelocity(1.0.metersPerSecond)),
        Target4(APTargetWithTolerance(Pose2d(8.086.meters, 2.653.meters, Rotation2d(-1.571.radians))).withVelocity(1.500.metersPerSecond)),
        Target5(APTargetWithTolerance(Pose2d(7.741.meters, 3.506.meters, Rotation2d(-0.785.radians))).withVelocity(0.5.metersPerSecond)),
        Target6(APTargetWithTolerance(Pose2d(6.725.meters, 3.677.meters, Rotation2d(0.000.radians))).withVelocity(0.5.metersPerSecond)),
        Target7(APTargetWithTolerance(Pose2d(6.000.meters, 2.800.meters, Rotation2d(1.571.radians))).withVelocity(0.5.metersPerSecond)),
        Target8(APTargetWithTolerance(Pose2d(6.000.meters, 0.500.meters, Rotation2d(-3.142.radians))).withVelocity(0.5.metersPerSecond)),
        Target9(APTargetWithTolerance(Pose2d(3.000.meters, 0.500.meters, Rotation2d(3.142.radians)))),
        Target10(APTargetWithTolerance(Pose2d(2.500.meters, 1.700.meters, Rotation2d(0.785.radians))))
    }


}

object CenterStartDepotLeftShoot : Auto {
    override fun getPath(flipH: Boolean, flipV: Boolean): Command =
        Commands.sequence(
            resetOdom(Targets.Target1.target, flipH, flipV),
            setShooterTarget(Target.STATIONARY_TURRET),
            Commands.parallel(
                Drivetrain.alignAndFlip(Targets.Target1.target, flipH, flipV),
                Intake.setPivotVoltage(10.volts).withTimeout(0.1.seconds),
            ),
            Commands.race(
                Intake.intakeSequence(),
                Commands.sequence(
                    Drivetrain.alignAndFlip(Targets.Target2.target, flipH, flipV),
                    Drivetrain.alignAndFlip(Targets.Target3.target, flipH, flipV),
                )
            ),
            Drivetrain.alignAndFlip(Targets.Target4.target, flipH, flipV),
            stopAndShoot(2.seconds),
            Commands.race(
                Intake.intakeSequence(),
                Commands.sequence(
                    Drivetrain.alignAndFlip(Targets.Target5.target, flipH, flipV),
                    Drivetrain.alignAndFlip(Targets.Target6.target, flipH, flipV),
                    Drivetrain.alignAndFlip(Targets.Target7.target, flipH, flipV),
                    Drivetrain.alignAndFlip(Targets.Target8.target, flipH, flipV),
                    Drivetrain.alignAndFlip(Targets.Target9.target, flipH, flipV),
                )
            ),
            Drivetrain.alignAndFlip(Targets.Target10.target, flipH, flipV),
            Drivetrain.alignAndFlip(Targets.Target4.target, flipH, flipV),
            stopAndShoot(4.seconds),
        )

    enum class Targets(val target: APTargetWithTolerance) {
        Target1(APTargetWithTolerance(Pose2d(3.655.meters, 4.035.meters, Rotation2d(0.000.radians)))),
        Target2(APTargetWithTolerance(Pose2d(0.676.meters, 4.989.meters, Rotation2d(-1.300.radians)))),
        Target3(APTargetWithTolerance(Pose2d(0.590.meters, 6.898.meters, Rotation2d(-1.776.radians)))),
        Target4(APTargetWithTolerance(Pose2d(2.070.meters, 7.069.meters, Rotation2d(-0.736.radians)))),
        Target5(APTargetWithTolerance(Pose2d(4.360.meters, 7.478.meters, Rotation2d(3.132.radians)))),
        Target6(APTargetWithTolerance(Pose2d(7.150.meters, 7.222.meters, Rotation2d(2.749.radians)))),
        Target7(APTargetWithTolerance(Pose2d(5.910.meters, 3.984.meters, Rotation2d(1.386.radians)))),
        Target8(APTargetWithTolerance(Pose2d(6.272.meters, 5.756.meters, Rotation2d(-1.513.radians)))),
        Target9(APTargetWithTolerance(Pose2d(5.703.meters, 7.410.meters, Rotation2d(-0.451.radians)))),
        Target10(APTargetWithTolerance(Pose2d(3.345.meters, 7.478.meters, Rotation2d(3.142.radians))))
    }
}


object Climb : Auto {
    override fun getPath(flipH: Boolean, flipV: Boolean): Command {
        return Commands.sequence(
            Commands.runOnce({
                Drivetrain.poseEstimator.resetPose(
                    flipTarget(
                        Targets.Target1.target,
                        flipV = flipV,
                        flipH = flipH
                    ).reference
                )
            }),
            Drivetrain.alignAndFlip(Targets.Target2.target, flipH, flipV),
            shoot().withTimeout(5.seconds),
            alignToClimbLeft(isRedAlliance()),
//            Climber.setTargetPosition(Climber.Position.STOWED),
            Climber.climb(),
        )
    }

    enum class Targets(val target: APTargetWithTolerance) {
        Target1(APTargetWithTolerance(Pose2d(3.620.meters, 4.035.meters, Rotation2d(0.000.radians)))),
        Target2(APTargetWithTolerance(Pose2d(2.553.meters, 4.035.meters, Rotation2d(0.000.radians))))
    }
}


fun resetOdom(pose: APTargetWithTolerance,flipH: Boolean,flipV: Boolean): Command = runOnce({
    Drivetrain.poseEstimator.resetPose(
        flipTarget(pose,flipV,flipH).reference
    )
})

fun stopAndShoot(time: Time): Command = Commands.sequence(
    setShooterTarget(Target.PASS_NO_SOTM),
    Commands.parallel(
        Commands.run({Drivetrain.stop()}),
        shoot(),
    ).withTimeout(time.inSeconds()),
    setShooterTarget(Target.STATIONARY_TURRET),
)
