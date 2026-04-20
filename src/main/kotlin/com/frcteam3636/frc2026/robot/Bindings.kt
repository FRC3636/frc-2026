package com.frcteam3636.frc2026.robot

import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.feeder.Feeder
import com.frcteam3636.frc2026.subsystems.indexer.Indexer
import com.frcteam3636.frc2026.subsystems.intake.Intake
import com.frcteam3636.frc2026.subsystems.shooter.Target
import com.frcteam3636.frc2026.subsystems.shooter.hood.Hood
import com.frcteam3636.frc2026.subsystems.shooter.setShooterTarget
import com.frcteam3636.frc2026.subsystems.shooter.shoot
import com.frcteam3636.frc2026.subsystems.shooter.turret.Turret
import com.frcteam3636.frc2026.utils.math.volts
import com.revrobotics.util.StatusLogger
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine

val joystickLeft = CommandJoystick(0)
val joystickRight = CommandJoystick(1)
val controller = CommandXboxController(2)

@Suppress("unused")
val joystickDev = CommandJoystick(3)

@Suppress("unused")
val controllerDev = CommandXboxController(4)

fun configureBindings() {

    /* main bindings */

    joystickLeft.button(1).whileTrue(
        Intake.intakeSequence()
    )

    joystickLeft.button(2).whileTrue(
        Intake.manipulateSequence()
    )

    joystickLeft.button(3).onTrue(
        Intake.setPivotVoltage(0.volts)
    )

    joystickLeft.povUp().whileTrue(
        Commands.parallel(
            Feeder.outtake(),
            Indexer.outdex()
        )
    )



    joystickRight.button(1).whileTrue(
        shoot()
    )

    joystickRight.button(2).whileTrue(
        setShooterTarget(Target.STATIONARY_TURRET)
    )

    joystickRight.button(3).onTrue(
        setShooterTarget(Target.TUNING)
    )

    joystickRight.button(4).onTrue(
        setShooterTarget(Target.AIM_AT_HUB_PASS)
    )

    /*  default commands   */

    Drivetrain.defaultCommand = Drivetrain.driveWithJoysticks (
        joystickLeft.hid,
        joystickRight.hid
    )

    Turret.defaultCommand = Turret.turnToSetpoint()
    Hood.defaultCommand = Hood.turnToTargetHoodAngle()

    /* zeroing commands */

    joystickLeft.button(8).onTrue(Commands.runOnce({
        Drivetrain.zeroGyro()
    }).ignoringDisable(true))

    joystickRight.button(8).onTrue(
        Commands.sequence(
            Commands.runOnce({
                println("Pre-match zeroing.")
                Drivetrain.zeroGyro()
            }).ignoringDisable(true),
            Intake.zeroPivot().ignoringDisable(true),
            Turret.zeroTurretEncoder().ignoringDisable(true),
            Hood.zeroEncoder().ignoringDisable(true),
        )
    )


    /* dev bindings */

    if (Preferences.getBoolean("DeveloperMode", false)) {
        controllerDev.leftBumper().onTrue(
            Commands.runOnce(SignalLogger::start)
                .andThen(StatusLogger::start)
        )
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
