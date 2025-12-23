package com.frcteam3636.swervebase.utils

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import kotlin.jvm.optionals.getOrNull


/**
 * Returns the translation of the joystick input, flipped to match the current alliance.
 */
val Joystick.fieldRelativeTranslation2d: Translation2d
    get() {
        val base = translation2d
        return when (DriverStation.getAlliance().getOrNull()) {
            Alliance.Red -> -base
            else -> base
        }
    }

/**
 * Returns the translation of the joystick input.
 */
val Joystick.translation2d: Translation2d
    // The field-space translation returned by this method is rotated 90 degrees from the joystick's
    // perspective. (x, y) -> (y, -x) The joystick's Y-axis is also inverted because of our physical
    // hardware.
    get() = Translation2d(
        -y, -x
    )

fun CommandXboxController.rumble(rumbleType: GenericHID.RumbleType, power: Double, time: Time): Command =
    Commands.startEnd({
        setRumble(rumbleType, power)
    }, {
        setRumble(rumbleType, 0.0)
    })
        .withTimeout(time)
