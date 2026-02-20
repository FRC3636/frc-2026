package com.frcteam3636.frc2026.utils.autos

import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.utils.math.inMeters
import com.frcteam3636.frc2026.utils.math.meters
import com.therekrab.autopilot.APTarget
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import kotlin.math.PI

const val FIELD_HEIGHT_METERS = 16.54048
const val FIELD_WIDTH_METERS = 8.06958

fun flipTargetHorizontal(target: APTarget): APTarget {
    return APTarget(
        Pose2d (
            Translation2d (
                FIELD_HEIGHT_METERS - target.reference.translation.x,
                target.reference.translation.y
            ),
            Rotation2d(PI - target.reference.rotation.radians)
        )
    )
}

fun flipTargetVertical(target: APTarget): APTarget {
    return APTarget (
        Pose2d (
            Translation2d (
                target.reference.translation.x,
                FIELD_WIDTH_METERS - target.reference.translation.y.meters.inMeters()
            ),
            -target.reference.rotation
        )
    )
}

fun flipPath(path: Array<APTarget>, flipH: Boolean = false, flipV: Boolean = false): Array<APTarget> {
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