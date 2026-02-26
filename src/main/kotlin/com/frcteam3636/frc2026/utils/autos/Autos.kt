package com.frcteam3636.frc2026.utils.autos

import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.utils.math.centimeters
import com.frcteam3636.frc2026.utils.math.inMeters
import com.frcteam3636.frc2026.utils.math.meters
import com.therekrab.autopilot.APTarget
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
import kotlin.math.PI

const val FIELD_HEIGHT_METERS = 16.54048
const val FIELD_WIDTH_METERS = 8.06958


class APTargetWithTolerance(pose: Pose2d) : APTarget(pose) {
    var tolerance: Distance = 5.centimeters

    override fun clone(): APTargetWithTolerance {
        val target = APTargetWithTolerance(m_reference)
        target.m_velocity = m_velocity
        target.m_entryAngle = m_entryAngle
        target.m_rotationRadius = m_rotationRadius
        target.tolerance = tolerance
        return target
    }

    fun withTolerance(tolerance: Distance): APTargetWithTolerance {
        val target = this.clone()
        target.tolerance = tolerance
        return target
    }
}

fun flipTargetHorizontal(target: APTargetWithTolerance): APTargetWithTolerance {
    return APTargetWithTolerance(
        Pose2d(
            Translation2d(
                FIELD_HEIGHT_METERS - target.reference.translation.x, target.reference.translation.y
            ), Rotation2d(PI - target.reference.rotation.radians)
        )
    )
}

fun flipTargetVertical(target: APTargetWithTolerance): APTargetWithTolerance {
    return APTargetWithTolerance(
        Pose2d(
            Translation2d(
                target.reference.translation.x, FIELD_WIDTH_METERS - target.reference.translation.y.meters.inMeters()
            ), -target.reference.rotation
        )
    )
}

fun flipPath(
    path: Array<APTargetWithTolerance>, flipH: Boolean = false, flipV: Boolean = false
): Array<APTargetWithTolerance> {
    if (!(flipH || flipV)) {
        return path
    }
    for (i in 0..path.size) {
        if (flipH) {
            path[i] = flipTargetHorizontal(path[i])
        }
        if (flipV) {
            path[i] = flipTargetVertical(path[i])
        }
    }
    return path
}
