@file:JvmName("Main") // set the compiled Java class name to "Main" rather than "MainKt"
package com.frcteam3636.frc2026

import com.frcteam3636.frc2026.robot.Robot
import edu.wpi.first.wpilibj.RobotBase

/**
 * 1. Main file
 *
 * Welcome to the Robot code! This is the code for the 2026 FRC season,
 * "REBUILT". By reading these documentation comments you will get a tour
 * of the robot code! Below is the main function, where the program starts.
 * As you can see, all it does is start the robot code. This code is very
 * simple, so we can swiftly move on. To see the absolute highest level
 * overview of how we tell the robot to do things, head over to `robot.Bindings`
 * file (you don't need to look at the rest of this comment).
 *
 * Main initialization function. Do not perform any initialization here
 * other than calling `RobotBase.startRobot`. Do not modify this file
 * except to change the object passed to the `startRobot` call.
 *
 * If you change the package of this file, you must also update the
 * `ROBOT_MAIN_CLASS` variable in the gradle build file. Note that
 * this file has a `@file:JvmName` annotation so that its compiled
 * Java class name is "Main" rather than "MainKt". This is to prevent
 * any issues/confusion if this file is ever replaced with a Java class.
 *
 * If you change your main Robot object (name), change the parameter of the
 * `RobotBase.startRobot` call to the new name. (If you use the IDE's Rename
 * Refactoring when renaming the object, it will get changed everywhere
 * including here.)
 */
fun main() = RobotBase.startRobot { Robot }
