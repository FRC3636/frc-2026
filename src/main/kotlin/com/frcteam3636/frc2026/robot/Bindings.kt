package com.frcteam3636.frc2026.robot

import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.feeder.Feeder
import com.frcteam3636.frc2026.subsystems.indexer.Indexer
import com.frcteam3636.frc2026.subsystems.shooter.Shooter
import com.frcteam3636.frc2026.subsystems.intake.Intake
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
    Shooter.Turret.defaultCommand = Shooter.Turret.turnToTargetTurretAngle()
     Shooter.Hood.defaultCommand = Shooter.Hood.turnToTargetHoodAngle()
    // (The button with the yellow tape on it)
    joystickLeft.button(8).onTrue(Commands.runOnce({
        println("Zeroing gyro.")
        Drivetrain.zeroGyro()
    }).ignoringDisable(true))

    joystickLeft.button(1).whileTrue(
        Intake.intake()
    )

    joystickRight.button(2).onTrue(
        Commands.runOnce({
            Shooter.shooterTarget = Shooter.Target.VECTOR_AIM_AT_HUB
        })
    )

    joystickRight.button(3).onTrue(
        Commands.runOnce({
            Shooter.shooterTarget = Shooter.Target.AIM_AT_HUB
        })
    )

    joystickRight.button(4).onTrue(
        Commands.runOnce({
            Shooter.shooterTarget = Shooter.Target.TUNING
        })
    )

    joystickRight.button(5).onTrue(
        Commands.runOnce({
            Shooter.shooterTarget = Shooter.Target.TUNING
        })
    )

    joystickRight.button(6).onTrue(
        Commands.runOnce({
            Shooter.shooterTarget = Shooter.Target.STOWED
        })
    )

    joystickRight.button(1).whileTrue(
        Commands.sequence(
            Commands.parallel(
                Shooter.Flywheel.runAtTarget().until(Shooter.Flywheel.atDesiredFlywheelVelocity),
                Shooter.Hood.turnToTargetHoodAngle().until(Shooter.Hood.atDesiredHoodAngle)
            ),
            Commands.parallel(
                Commands.parallel(
                    Feeder.feed(),
                    Indexer.index()
                ).onlyWhile(Shooter.Flywheel.atDesiredStandingFlywheelVelocity).repeatedly(),
                Shooter.Flywheel.runAtTarget(),
            ),
        )
    )
    joystickRight.button(8).onTrue(Commands.runOnce({
        println("Zeroing Turret Encoder")
        Shooter.Turret.zeroTurretEncoder()
    }).ignoringDisable(true))

    controller.leftBumper().onTrue(
        Commands.runOnce(SignalLogger::start)
            .andThen(StatusLogger::start))
    controller.rightBumper().onTrue(
        Commands.runOnce(SignalLogger::stop)
            .andThen(StatusLogger::stop)
    )

    controller.b().whileTrue(
        Shooter.Flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
    )
    controller.a().whileTrue(
        Shooter.Flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
    )
    controller.x().whileTrue(
        Shooter.Flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward)
    )
    controller.y().whileTrue(
        Shooter.Flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse)
    )



//    controller.a().whileTrue(
////        Shooter.Hood.turnToAngle(Shooter.Hood.angle + 5.degrees)
////        Shooter.Hood.turnToAngle(0.degrees)
//        Shooter.Flywheel.spinAtTargetSpeed(3000.rpm)
//    )

//    controller.x().whileTrue(
////        Shooter.Hood.turnToAngle(15.degrees)
//        Commands.parallel(
//            Indexer.index(),
//            Feeder.feed()
//        )
//    )

//    controller.povUp().onTrue(Intake.setPivotPosition(Intake.Position.Stowed))
//    controller.povDown().onTrue(Intake.setPivotPosition(Intake.Position.Deployed))

//    controller.a().whileTrue(
//        Commands.parallel(
//            Commands.run({ Shooter.shooterTarget = Shooter.Target.AIM_AT_HUB }),
//            Commands.sequence(
//                Shooter.Flywheel.runAtTarget().until(Shooter.Flywheel.atDesiredFlywheelVelocity),
//    //            Hood.turnToTargetHoodAngle().until(Shooter.Hood.atDesiredHoodAngle),
//                Commands.parallel(
//                    Commands.parallel(
////                        Indexer.index(),
//                        Feeder.feed(),
//                    ).onlyWhile(Shooter.Flywheel.atDesiredStandingFlywheelVelocity).repeatedly(),
//                    Shooter.Flywheel.runAtTarget()
//                ),
//            ),
//        )
//    )

//    controller.x().whileTrue(
//        Commands.parallel(
//            Indexer.outdex(),
//            Feeder.outtake()
//        )
//    )

//    joystickRight.button(1).whileTrue(Drivetrain.alignWithAutopilot(Drivetrain.Constants.ALIGN_TARGET))

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
