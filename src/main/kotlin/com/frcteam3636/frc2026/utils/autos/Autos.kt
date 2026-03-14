package com.frcteam3636.frc2026.utils.autos

import com.frcteam3636.frc2026.subsystems.climber.Climber
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.utils.math.*
import com.therekrab.autopilot.APTarget
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI

class APTargetWithTolerance(pose: Pose2d) : APTarget(pose) {
    var tolerance: Distance = 15.centimeters

    override fun clone(): APTargetWithTolerance {
        val target = APTargetWithTolerance(m_reference)
        target.m_velocity = m_velocity
        target.m_entryAngle = m_entryAngle
        target.m_rotationRadius = m_rotationRadius
        target.tolerance = tolerance
        return target
    }

    fun withVelocity(velocity: LinearVelocity): APTargetWithTolerance {
        val clone = this.clone()
        clone.m_velocity = velocity.inMetersPerSecond()
        return clone
    }

    override fun withReference(reference: Pose2d): APTargetWithTolerance {
        val target = this.clone()
        target.m_reference = reference
        return target
    }

    fun withTolerance(tolerance: Distance): APTargetWithTolerance {
        val target = this.clone()
        target.tolerance = tolerance
        return target
    }
}

fun flipTargetHorizontal(target: APTargetWithTolerance): APTargetWithTolerance {
    return target.withReference(
        Pose2d(
            Translation2d(
                FIELD_HEIGHT_METERS - target.reference.translation.x, target.reference.translation.y
            ), Rotation2d(PI - target.reference.rotation.radians)
        )
    )
}

fun flipTargetVertical(target: APTargetWithTolerance): APTargetWithTolerance {
    return target.withReference(
        Pose2d(
            Translation2d(
                target.reference.translation.x, FIELD_WIDTH_METERS - target.reference.translation.y.meters.inMeters()
            ), -target.reference.rotation
        )
    )
}

fun flipTarget(target: APTargetWithTolerance, flipH: Boolean = false, flipV: Boolean = false): APTargetWithTolerance {
    var new_target = target
    if (flipH) {
        new_target = flipTargetHorizontal(new_target)
    }
    if (flipV) {
        new_target = flipTargetVertical(new_target)
    }
    return new_target
}

fun flipPath(
    path: Array<APTargetWithTolerance>, flipH: Boolean = false, flipV: Boolean = false
): Array<APTargetWithTolerance> {
    if (!(flipH || flipV)) {
        return path
    }
    for (i in 0..path.size) {
        path[i] = flipTarget(path[i])
    }
    return path
}

internal val CLIMB_RIGHT_OFFSET = Translation2d(1.0864.meters, 2.837.meters)
internal val LEFT_OFFSET = 4.68.meters - 2.837.meters;

private enum class ClimbAlignTargets(val target: APTargetWithTolerance) {
    ClimbBlueRight(APTargetWithTolerance(Pose2d(CLIMB_RIGHT_OFFSET.measureX, CLIMB_RIGHT_OFFSET.measureY, Rotation2d(9.425.radians)))),
    ClimbBlueRunupRight(APTargetWithTolerance(Pose2d(CLIMB_RIGHT_OFFSET.measureX + 0.1.meters, CLIMB_RIGHT_OFFSET.measureY, Rotation2d(-3.142.radians)))),
    ClimbBlueLeft(APTargetWithTolerance(Pose2d(CLIMB_RIGHT_OFFSET.measureX, CLIMB_RIGHT_OFFSET.measureY + LEFT_OFFSET, Rotation2d(0.000.radians)))),
    ClimbBlueRunupLeft(APTargetWithTolerance(Pose2d(CLIMB_RIGHT_OFFSET.measureX + 0.1.meters, CLIMB_RIGHT_OFFSET.measureY + LEFT_OFFSET, Rotation2d(0.000.radians)))),
}

private fun alignToClimbLeft(red_alliance: Boolean): Command = Commands.sequence(
    Drivetrain.alignAndFlip(ClimbAlignTargets.ClimbBlueRunupLeft.target, flipH = red_alliance, flipV = red_alliance),
    Drivetrain.alignAndFlip(ClimbAlignTargets.ClimbBlueLeft.target, flipH = red_alliance, flipV = red_alliance)
)
private fun alignToClimbRight(red_alliance: Boolean): Command = Commands.sequence(
    Drivetrain.alignAndFlip(ClimbAlignTargets.ClimbBlueRunupRight.target, flipH = red_alliance, flipV = red_alliance),
    Drivetrain.alignAndFlip(ClimbAlignTargets.ClimbBlueRight.target, flipH = red_alliance, flipV = red_alliance)
)

fun alignToClimb(): Command  {
    var red_alliance = DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red
    var side = Drivetrain.field_side
    var path = when (side) {
        Drivetrain.FieldSide.Left -> alignToClimbLeft(red_alliance)
        Drivetrain.FieldSide.Right -> alignToClimbRight(red_alliance)
    }

    return Commands.sequence(Commands.parallel(
        path,
        Climber.setTargetPosition(Climber.Position.GROUND_L1)
    )) // Climber.setTargetPosition(Climber.Position.STOWED)
}
