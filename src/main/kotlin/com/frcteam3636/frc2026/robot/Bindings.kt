package com.frcteam3636.frc2026.robot

import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.feeder.Feeder
import com.frcteam3636.frc2026.subsystems.indexer.Indexer
import com.revrobotics.util.StatusLogger
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine

val controller = CommandXboxController(2)
val joystickLeft = CommandJoystick(0)
val joystickRight = CommandJoystick(1)

@Suppress("unused")
val joystickDev = CommandJoystick(3)

@Suppress("unused")
val controllerDev = CommandXboxController(4)

fun configureBindings() {
    Drivetrain.defaultCommand = Drivetrain.driveWithJoysticks (
        joystickLeft.hid,
        joystickRight.hid
    )
    // (The button with the yellow tape on it)
    joystickLeft.button(8).onTrue(Commands.runOnce({
        println("Zeroing gyro.")
        Drivetrain.zeroGyro()
    }).ignoringDisable(true))

    controller.b().onTrue(Commands.runOnce( {
        Drivetrain.zeroGyro()
    }))

    controller.a().whileTrue(
        Commands.parallel(
            Indexer.index(),
            Feeder.feed()
        )
    )

    controller.x().whileTrue(
        Commands.parallel(
            Indexer.outdex(),
            Feeder.outtake()
        )
    )

    joystickRight.button(1).whileTrue(Drivetrain.alignWithAutopilot(Drivetrain.Constants.ALIGN_TARGET))

    // Angles robot for shooting, just in case the
    // turret stops working.
    // joystickRight.button(12).whileTrue(Drivetrain.alignToHub())


    if (Preferences.getBoolean("DeveloperMode", false)) {
        controllerDev.leftBumper().onTrue(
            Commands.runOnce(SignalLogger::start)
                .andThen(StatusLogger::start))
        controllerDev.rightBumper().onTrue(
            Commands.runOnce(SignalLogger::stop)
                .andThen(StatusLogger::stop)
        )

        controllerDev.y().whileTrue(Drivetrain.sysIdQuasistaticSpin(SysIdRoutine.Direction.kForward))
        controllerDev.a().whileTrue(Drivetrain.sysIdQuasistaticSpin(SysIdRoutine.Direction.kReverse))
        controllerDev.b().whileTrue(Drivetrain.sysIdDynamicSpin(SysIdRoutine.Direction.kForward))
        controllerDev.x().whileTrue(Drivetrain.sysIdDynamicSpin(SysIdRoutine.Direction.kReverse))

        controllerDev.povUp().whileTrue(Drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        controllerDev.povDown().whileTrue(Drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        controllerDev.povRight().whileTrue(Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward))
        controllerDev.povLeft().whileTrue(Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse))

        joystickDev.button(1).whileTrue(Drivetrain.calculateWheelRadius())
    }
}