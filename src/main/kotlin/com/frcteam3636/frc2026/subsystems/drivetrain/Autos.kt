package com.frcteam3636.frc2026.subsystems.drivetrain

import com.frcteam3636.frc2026.subsystems.feeder.Feeder
import com.frcteam3636.frc2026.subsystems.indexer.Indexer
import com.frcteam3636.frc2026.subsystems.intake.Intake
import com.frcteam3636.frc2026.subsystems.shooter.ShooterCalculator
import com.frcteam3636.frc2026.subsystems.shooter.ShooterProfile
import com.frcteam3636.frc2026.subsystems.shooter.Target
import com.frcteam3636.frc2026.subsystems.shooter.flywheel.Flywheel
import com.frcteam3636.frc2026.subsystems.shooter.setShooterTarget
import com.frcteam3636.frc2026.utils.autos.APTargetWithTolerance
import com.frcteam3636.frc2026.utils.autos.alignToClimb
import com.frcteam3636.frc2026.utils.autos.alignToClimbLeft
import com.frcteam3636.frc2026.utils.autos.alignToClimbRight
import com.frcteam3636.frc2026.utils.autos.flipTarget
import com.frcteam3636.frc2026.utils.autos.isRedAlliance
import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.meters
import com.frcteam3636.frc2026.utils.math.metersPerSecond
import com.frcteam3636.frc2026.utils.math.radians
import com.frcteam3636.frc2026.utils.math.seconds
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import kotlin.math.PI

interface Auto {
    fun getPath(flipH: Boolean, flipV: Boolean): Command
}


object TwoScore : Auto {
    override fun getPath(flipH: Boolean, flipV: Boolean): Command {
        return Commands.sequence(
            Drivetrain.alignAndFlip(Targets.Target1.target, flipH, flipV),
            Drivetrain.alignAndFlip(Targets.Target2.target, flipH, flipV),
            Drivetrain.alignAndFlip(Targets.Target3.target, flipH, flipV),
            Commands.race(
                Drivetrain.alignAndFlip(Targets.Target4.target, flipH, flipV),
                Intake.intake()
            ),
            Drivetrain.alignAndFlip(Targets.Target5.target, flipH, flipV),
            Drivetrain.alignAndFlip(Targets.Target2.target, flipH, flipV),
            Commands.parallel(
                Drivetrain.alignAndFlip(Targets.Target1.target, flipH, flipV)
            ),
        )
    }

    private enum class Targets(val target: APTargetWithTolerance) {
        Target1(
            APTargetWithTolerance(
                Pose2d(
                    2.7136747875808314.meters,
                    6.008694768709806.meters,
                    Rotation2d((-0.7500147072732971).radians)
                )
            ).withTolerance(10.meters)
        ),
        Target2(
            APTargetWithTolerance(
                Pose2d(
                    3.8616252017874615.meters,
                    7.413649006992547.meters,
                    Rotation2d((-0.75).radians)
                )
            )
        ),
        Target3(
            APTargetWithTolerance(
                Pose2d(
                    7.0.meters,
                    7.208045947731659.meters,
                    Rotation2d(1.98448554007376.radians)
                )
            )
        ),

        //        Target4(APTarget(Pose2d(7.545346680211723.meters, 4.792210001416213.meters, Rotation2d(1.6926387638148142.radians)))),
        Target4(
            APTargetWithTolerance(
                Pose2d(
                    7.545346680211723.meters,
                    3.5.meters,
                    Rotation2d(1.6926387638148142.radians)
                )
            )
        ),
        Target5(
            APTargetWithTolerance(
                Pose2d(
                    6.7229344431681675.meters,
                    7.208045947731659.meters,
                    Rotation2d((-0.6043055068662304).radians)
                )
            )
        )
    }
}

object Middle : Auto {
    override fun getPath(flipH: Boolean, flipV: Boolean): Command {
        return Commands.sequence(
            Commands.runOnce({
                Drivetrain.poseEstimator.resetPose(
                    flipTarget(
                        Targets.StartPos.target,
                        flipV = flipV,
                        flipH = flipH
                    ).reference
                )
            }),
            Drivetrain.alignAndFlip(Targets.Target1.target, flipH, flipV),
            Commands.parallel(
                setShooterTarget(Target.AIM_AT_HUB),
                Flywheel.runAtTarget(),
                Indexer.index(),
                Feeder.feed(),
            ).withTimeout(2.seconds),
            Drivetrain.alignAndFlip(Targets.Target2.target, flipH, flipV),
            Commands.waitSeconds(3.0),
            Drivetrain.alignAndFlip(Targets.Target3.target, flipH, flipV),
            Commands.parallel(
                setShooterTarget(Target.AIM_AT_HUB),
                Flywheel.runAtTarget(),
                Indexer.index(),
                Feeder.feed(),
            ).withTimeout(2.seconds),
        )
    }

    enum class Targets(val target: APTargetWithTolerance) {
        StartPos(APTargetWithTolerance(Pose2d(3.60, 4.00, Rotation2d.kZero))),
        Target1(APTargetWithTolerance(Pose2d(2.200.meters, 4.100.meters, Rotation2d(0.000.radians)))),
        Target2(APTargetWithTolerance(Pose2d(0.400.meters, 0.700.meters, Rotation2d(0.000.radians)))),
        Target3(APTargetWithTolerance(Pose2d(1.200.meters, 1.400.meters, Rotation2d(0.785.radians))))
    }

}

object Climb : Auto {
    override fun getPath(flipH: Boolean, flipV: Boolean): Command {
        return Commands.sequence(
            Commands.runOnce({
                Drivetrain.poseEstimator.resetPose(
                    flipTarget(
                        Targets.Start.target,
                        flipV = flipV,
                        flipH = flipH
                    ).reference
                )
            }),
            Drivetrain.alignAndFlip(Targets.Shoot.target, flipH, flipV),
            Commands.sequence(
                Commands.parallel(
                    Flywheel.runAtTarget(),
                ).until(Flywheel.atDesiredFlywheelVelocity),
                Commands.parallel(
                    Flywheel.runAtTarget(),
                    Commands.parallel(
                        Feeder.feed(),
                        Indexer.index()
                    ).onlyWhile(Flywheel.atDesiredStandingFlywheelVelocity).repeatedly()
                )
            ).withTimeout(2.seconds),
            alignToClimbLeft(isRedAlliance())
        )
    }

    enum class Targets(val target: APTargetWithTolerance) {
        Start(APTargetWithTolerance(Pose2d(3.700.meters, 4.100.meters, Rotation2d(0.000.radians)))),
        Shoot(APTargetWithTolerance(Pose2d(1.97.meters, 4.73.meters, Rotation2d((-0.12).radians))))
    }
}

object Stem : Auto {
    override fun getPath(flipH: Boolean, flipV: Boolean): Command {
        return Commands.sequence(
            runOnce({
                Drivetrain.poseEstimator.resetPose(
                    flipTarget(
                        Targets.StartPos.target,
                        flipH = flipH,
                        flipV = flipV
                    ).reference
                )
            }),
            Intake.setPivotPosition(Intake.Position.Deployed).until(Intake.atDesiredPivotAngle),
            Commands.race(
                Drivetrain.alignAndFlip(Targets.Cycle1.target, flipH, flipV),
                Intake.intake(),
            ),
            Commands.race(
                Drivetrain.alignAndFlip(Targets.Center.target, flipH, flipV),
                Intake.intake(),
            ),
            Drivetrain.alignAndFlip(Targets.Cycle1.target, flipH, flipV),
            Drivetrain.alignAndFlip(Targets.Trench.target, flipH, flipV),
            Commands.parallel(
                setShooterTarget(Target.AIM_AT_HUB),
                Drivetrain.alignAndFlip(Targets.Safe.target, flipH, flipV),
            ),
            Commands.sequence(
                Commands.parallel(
                    Flywheel.runAtTarget(),
                ).until(Flywheel.atDesiredFlywheelVelocity),
                Commands.parallel(
                    Flywheel.runAtTarget(),
                    Commands.parallel(
                        Feeder.feed(),
                        Indexer.index()
                    ).onlyWhile(Flywheel.atDesiredStandingFlywheelVelocity).repeatedly()
                ),
            )
        )
    }

    enum class Targets(val target: APTargetWithTolerance) {
        StartPos(APTargetWithTolerance(Pose2d(3.778.meters, 0.606.meters, Rotation2d(PI.radians)))),
        Trench(
            APTargetWithTolerance(
                Pose2d(
                    4.484.meters,
                    0.588.meters,
                    Rotation2d(0.0.radians)
                )
            ).withVelocity(1.0.metersPerSecond).withTolerance(0.4.meters)
        ),
        Safe(
            APTargetWithTolerance(
                Pose2d(
                    2.576.meters,
                    0.761.meters,
                    Rotation2d(0.785.radians)
                )
            ).withVelocity(1.0.metersPerSecond)
        ),
        Center(
            APTargetWithTolerance(
                Pose2d(
                    7.811.meters,
                    1.970.meters,
                    Rotation2d(-1.571.radians)
                )
            ).withVelocity(1.0.metersPerSecond)
        ),
        Cycle1(
            APTargetWithTolerance(
                Pose2d(
                    6.612.meters,
                    0.588.meters,
                    Rotation2d(-2.749.radians)
                )
            ).withVelocity(1.0.metersPerSecond).withTolerance(0.4.meters)
        ),
    }
}


object TestAuto : Auto {
    override fun getPath(flipH: Boolean, flipV: Boolean): Command = Commands.sequence(
        Drivetrain.alignAndFlip(Targets.Target1.target, flipH, flipV),
        Drivetrain.alignAndFlip(Targets.Target2.target, flipH, flipV)
    )

    // Generated by autopilot path picker
//    enum class Targets(val target: APTarget) {
//        Target1(APTarget(Pose2d(2.3.meters, 2.9.meters, Rotation2d(1.579.radians.unaryMinus()))).withEntryAngle(Rotation2d(0.radians))),
//        Target2(APTarget(Pose2d(2.3.meters, 0.8.meters, Rotation2d(1.57.radians))))
//    )

    enum class Targets(val target: APTargetWithTolerance) {
        Target1(
            APTargetWithTolerance(
                Pose2d(
                    7.0313390320595.meters,
                    3.335854998318249.meters,
                    Rotation2d(2.8368720828252267.radians)
                )
            )
        ),
        Target2(
            APTargetWithTolerance(
                Pose2d(
                    7.014205443787759.meters,
                    4.4152710594379165.meters,
                    Rotation2d(-3.1262092518091977.radians)
                )
            )
        )
    }

}
