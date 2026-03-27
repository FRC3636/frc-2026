package com.frcteam3636.frc2026.robot

import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.frc2026.subsystems.climber.Climber
import com.frcteam3636.frc2026.subsystems.shooter.Target
import com.frcteam3636.frc2026.subsystems.shooter.setShooterTarget
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.feeder.Feeder
import com.frcteam3636.frc2026.subsystems.shooter.hood.Hood
import com.frcteam3636.frc2026.subsystems.indexer.Indexer
import com.frcteam3636.frc2026.subsystems.intake.Intake
import com.frcteam3636.frc2026.subsystems.shooter.shoot
import com.frcteam3636.frc2026.subsystems.shooter.turret.Turret
import com.frcteam3636.frc2026.utils.autos.alignToClimb
import com.frcteam3636.frc2026.utils.math.meters
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


    controller.a().whileTrue(
        Intake.intakeSequence()
    )

    controller.b().whileTrue(
        Intake.manipulateSequence()
    )


    joystickLeft.button(1).whileTrue(
            Intake.intakeSequence()
    )

    joystickLeft.button(2).whileTrue(
        Intake.setPivotVoltage(9.0.volts)
    )

//    joystickLeft.button(3).onTrue(
//        Commands.runOnce({ Climber.targetPosition = Climber.Position.STOWED })
//    )
//
//    joystickLeft.button(4).onTrue(
//        Commands.runOnce({ Climber.targetPosition = Climber.Position.GROUND_L1 })
////        Climber.climb()
//    )
//    joystickLeft.button(5).whileTrue(
//        Climber.homeRoutine()
//    )

    joystickLeft.povUp().whileTrue(
        Commands.parallel(
            Feeder.outtake(),
            Indexer.outdex()
        )
    )



//    joystickRight.button(9).whileTrue(
//        Climber.setPosition(0.183.meters).ignoringDisable(true)
//    )

    joystickRight.button(3).onTrue(
        setShooterTarget(Target.TUNING)
    )

    joystickRight.button(4).onTrue(
        setShooterTarget(Target.AIM_AT_HUB_PASS)
    )

    joystickRight.button(11).onTrue(
        setShooterTarget(Target.AIM_AT_HUB_NO_PASS)
    )

    joystickRight.button(12).whileTrue(
        Commands.parallel(
            Indexer.outdex(),
            Intake.outtake()
        )
    )

//    joystickRight.button(2).whileTrue(alignToClimb())

    joystickRight.button(1).whileTrue(
        shoot()
    )

    /*  default commands   */

    Drivetrain.defaultCommand = Drivetrain.driveWithJoysticks (
        joystickLeft.hid,
        joystickRight.hid
    )

    Turret.defaultCommand = Turret.turnToSetpoint()
    Hood.defaultCommand = Hood.turnToTargetHoodAngle()

//    Climber.defaultCommand = Climber.goToTargetHeight()

    /* zeroing commands */

    joystickLeft.button(8).onTrue(Commands.runOnce({
        println("Zeroing gyro.")
        Drivetrain.zeroGyro()
    }).ignoringDisable(true))

    joystickLeft.button(14).onTrue(Commands.runOnce({
        println("Zeroing gyro.")
        Drivetrain.zeroGyro(isReversed = true)
    }).ignoringDisable(true))

    joystickRight.button(8).onTrue(
        Commands.parallel(
            Turret.zeroTurretEncoder().ignoringDisable(true),
            Commands.runOnce({
                println("Zeroing turret.")
            })
        ).ignoringDisable(true)

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
