package com.frcteam3636.frc2026.subsystems.drivetrain

import com.frcteam3636.frc2026.utils.autos.flipTargetHorizontal
import com.frcteam3636.frc2026.utils.autos.flipTargetVertical
import com.frcteam3636.frc2026.utils.math.meters
import com.frcteam3636.frc2026.utils.math.radians
import com.therekrab.autopilot.APTarget
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

interface Auto {
    fun getPath(drivetrain: Drivetrain): Command
    fun getFlippedPath(drivetrain: Drivetrain): Command
    fun getVerticallyFlippedPath(drivetrain: Drivetrain): Command
}

object TestAuto: Auto {

    fun testAutoPath(target1: APTarget, target2: APTarget): Command = Commands.sequence(
        Drivetrain.alignWithAutopilot(target1),
        Drivetrain.alignWithAutopilot(target2)
    )

    enum class Targets(val target: APTarget) {
        Target1(
            APTarget(Pose2d(2.3.meters, 2.9.meters, Rotation2d(1.579.radians.unaryMinus()))).withEntryAngle(
                Rotation2d(0.radians)
            )
        ),
        Target2(APTarget(Pose2d(2.3.meters, 0.8.meters, Rotation2d(1.57.radians))))
    }

    override fun getPath(drivetrain: Drivetrain): Command = testAutoPath(
        Targets.Target1.target,
        Targets.Target2.target
    )

    override fun getFlippedPath(drivetrain: Drivetrain): Command =
        testAutoPath (
            flipTargetHorizontal(Targets.Target1.target),
            flipTargetHorizontal(Targets.Target2.target)
        )

    override fun getVerticallyFlippedPath(drivetrain: Drivetrain): Command =
        testAutoPath(
            flipTargetVertical(Targets.Target1.target),
            flipTargetVertical(Targets.Target2.target)
        )

}
