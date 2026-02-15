package com.frcteam3636.frc2026.utils

import com.frcteam3636.frc2026.utils.math.inMeters
import com.frcteam3636.frc2026.utils.math.meters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import kotlin.math.PI

const val FIELD_HEIGHT_METERS = 16.54048
const val FIELD_WIDTH_METERS = 8.06958


fun Pose2d.flipVertically(): Pose2d{
    return Pose2d(Translation2d(
        FIELD_HEIGHT_METERS - this.translation.x
        , this.translation.y),
        Rotation2d(PI- this.rotation.radians))
}

fun Pose2d.flipHorizontally(): Pose2d{
    return Pose2d(Translation2d(
        this.translation.x,
        FIELD_WIDTH_METERS - this.translation.y),
        Rotation2d(this.rotation.radians.unaryMinus())
    )
}

fun Translation2d.flipVertically(): Translation2d{
    return Translation2d(
        FIELD_WIDTH_METERS - this.x,
        this.y
    )
}

fun Translation2d.flipHorizontally(): Translation2d{
    return Translation2d(
        this.x,
        FIELD_WIDTH_METERS - this.y
    )
}