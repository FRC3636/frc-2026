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

/*
 * 2. The Joysticks
 *
 * Here are where we define the joysticks! Usually, the driver uses the left
 * and right joysticks and the operator (if there is one) uses the controller.
 * This season we didn't have an operator, so the controller is just another
 * way to control the robot.
 *
 * The port argument corresponds to a port on the driver station. You can use
 * the driver station application to change which virtual port a physical port
 * corresponds to.
 *
 * You can continue down to the configureBindings function.
 */

val joystickLeft = CommandJoystick(0)
val joystickRight = CommandJoystick(1)
val controller = CommandXboxController(2)

@Suppress("unused")
val joystickDev = CommandJoystick(3)

@Suppress("unused")
val controllerDev = CommandXboxController(4)

/*
 * This function is called from the Robot class.
 */
fun configureBindings() {

    /* main bindings */

    /*
     * Every joystick has buttons, which act as triggers. On those triggers we
     * can run different commands. In this case, we run Intake.intakeSequence()
     * *while* button 1 is pressed on the left joystick.
     *
     * All of the other triggers are fairly simple, because most of the logic
     * is in the subsystems like Intake instead of here. There's no need to
     * understand the zeroing commands or the dev bindings right now. You can
     * continue looking through them all the way to the bottom of the function.
     */
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
        setShooterTarget(Target.PASS)
    )

    /*  default commands   */

    Drivetrain.defaultCommand = Drivetrain.driveWithJoysticks (
        joystickLeft.hid,
        joystickRight.hid
    )

    Turret.defaultCommand = Turret.turnToSetpoint()
//    Hood.defaultCommand = Hood.turnToTargetHoodAngle()

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

/*
 * Now it's time to look at the Robot class! That is where all the bindings and
 * the Subsystems come together.
 */