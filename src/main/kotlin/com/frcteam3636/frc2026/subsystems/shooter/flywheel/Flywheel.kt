package com.frcteam3636.frc2026.subsystems.shooter.flywheel

import com.frcteam3636.frc2026.robot.Robot
import com.frcteam3636.frc2026.robot.Robot.Model
import com.frcteam3636.frc2026.subsystems.shooter.shooterProfile
import com.frcteam3636.frc2026.subsystems.shooter.shooterToHub
import com.frcteam3636.frc2026.utils.math.*
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import org.littletonrobotics.junction.Logger
import kotlin.math.abs
import kotlin.math.sqrt

object Flywheel: Subsystem {
    val atDesiredFlywheelVelocity = Trigger {
        val error = abs((inputs.angularVelocity - shooterProfile.angularVelocity).inRPM())
        Logger.recordOutput("Shooter/Flywheel/Velocity Error", error.rpm)
        error < Constants.FLYWHEEL_VELOCITY_TOLERANCE.inRPM()
    }

    val atDesiredStandingFlywheelVelocity = Trigger {
        val error = abs((inputs.angularVelocity - shooterProfile.angularVelocity).inRPM())
        Logger.recordOutput("Shooter/Flywheel/Velocity Error", error.rpm)
        error < Constants.STANDING_FLYWHEEL_VELOCITY_TOLERANCE.inRPM()
    }

    private var inputs = LoggedFlywheelInputs()
    private val io = when (Robot.model) {
        Model.SIMULATION -> FlywheelIOSim()
        Model.COMPETITION -> FlywheelIOReal()
    }


    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Flywheel", inputs)
        Logger.recordOutput("Shooter/Flywheel/Desired Velocity", shooterProfile.angularVelocity)
        Logger.recordOutput("Shooter/Flywheel/atDesiredFlywheelVelocity", atDesiredFlywheelVelocity)
    }

    fun calculateFlywheelVelocity(distance: Distance): AngularVelocity {
        // https://www.desmos.com/calculator/504yoxmqbr
        return (188.86324 * distance.inMeters() + 1866.16506).rpm
    }
    fun getSimFuelVelocity(distance: Distance): LinearVelocity = (sqrt(calculateFlywheelVelocity(distance).inRPM()) / Constants.ANGULAR_TO_LINEAR_RATIO).metersPerSecond

    fun runAtTarget(): Command = runEnd(
        {
            io.setVelocity(shooterProfile.angularVelocity)
        },
        {
            io.setVoltage(0.0.volts)
        })

    fun spinAtTargetSpeed(velocity: AngularVelocity): Command = run {
        Logger.recordOutput("Shooter/Flywheel/target velocity", velocity)
        io.setVelocity(velocity)
    }

    fun runAtVoltage(voltage: Voltage): Command = runEnd(
        {
            io.setVoltage(voltage)
        },
        {
            io.setVoltage(0.0.volts)
        }
    )

    private var sysID = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            null,
            null,
            { state ->
                Logger.recordOutput("SysIdTestState", state.toString())
            }
        ),
        SysIdRoutine.Mechanism(
            io::setVoltage,
            null, // recorded by URCL
            this
        )
    )

    fun sysIdQuasistatic(direction: Direction): Command = sysID.quasistatic(direction)

    fun sysIdDynamic(direction: Direction): Command = sysID.dynamic(direction)
}

object Constants {
    val FLYWHEEL_RADIUS = 0.0505.meters
    val FLYWHEEL_VELOCITY_TOLERANCE = 50.rpm
    val STANDING_FLYWHEEL_VELOCITY_TOLERANCE = 150.rpm
    const val ANGULAR_TO_LINEAR_RATIO = 18.0 // arbitrary ratio between flywheel rpm and fuel mps
    const val FLYWHEEL_TO_FUEL_RATIO = 0.5 // hypothetical ratio between flywheel tangential velocity and fuel velocity
}
