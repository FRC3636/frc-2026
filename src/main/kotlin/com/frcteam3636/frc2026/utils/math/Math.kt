@file:Suppress("unused", "UnusedReceiverParameter")

package com.frcteam3636.frc2026.utils.math

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Velocity
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

const val TAU = PI * 2
const val GRAVITY = 9.81

fun Translation2d.fromPolar(magnitude: Double, angle: Double): Translation2d {
    return Translation2d(magnitude * cos(angle), magnitude * sin(angle))
}

fun Translation2d.dot(other: Translation2d): Double {
    return x * other.x + y * other.y
}

fun Angle.toRotation2d(): Rotation2d {
    return Rotation2d(this)
}

fun AngularVelocity.toLinearVelocity(radius: Double): LinearVelocity {
    return (this.inRadiansPerSecond() / radius).metersPerSecond
}

fun LinearVelocity.getVerticalComponent(angle: Angle): LinearVelocity {
    return this * sin(angle.inRadians())
}

fun LinearVelocity.getHorizontalComponent(angle: Angle): LinearVelocity {
    return this * cos(angle.inRadians())
}