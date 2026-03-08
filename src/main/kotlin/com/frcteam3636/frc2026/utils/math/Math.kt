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
import kotlin.math.sqrt

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

fun Angle.clamp(minimum: Angle, maximum: Angle): Angle {
    if (this < minimum) {
        return minimum
    } else if (this > maximum) {
        return maximum
    } else {
        return this
    }
}
fun Angle.clamp_deadzone(minimum: Angle, maximum: Angle): Angle {
    if (this < maximum && this > minimum) {
        return this
    }

    val dist_to_minimum = (this.inRadians() - minimum.inRadians()).radians
    val dist_to_maximum = (this.inRadians() - maximum.inRadians()).radians

    if (dist_to_minimum < dist_to_maximum) {
        return minimum
    } else {
        return maximum
    }
}

fun LinearVelocity.getVerticalComponent(angle: Angle): LinearVelocity {
    return this * sin(angle.inRadians())
}

fun LinearVelocity.getHorizontalComponent(angle: Angle): LinearVelocity {
    return this * cos(angle.inRadians())
}

data class Vector3d(val x: Double, val y: Double, val z: Double) {
    val norm: Double get() = sqrt(x*x + y*y + z*z)
    operator fun minus(other: Vector3d) = Vector3d(x - other.x, y - other.y, z - other.z)
    operator fun plus(other: Vector3d) = Vector3d(x + other.x, y + other.y, z + other.z)
    operator fun times(scalar: Double) = Vector3d(x * scalar, y * scalar, z * scalar)
    operator fun div(scalar: Double) = Vector3d(x / scalar, y / scalar, z / scalar)
}
