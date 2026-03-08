package com.frcteam3636.frc2026.subsystems.hood

import com.frcteam3636.frc2026.robot.Robot
import com.frcteam3636.frc2026.robot.Robot.Model
import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.inDegrees
import com.frcteam3636.frc2026.utils.math.volts
import com.frcteam3636.frc2026.shooter.shooterProfile
import com.frcteam3636.frc2026.shooter.shooterTarget
import com.frcteam3636.frc2026.utils.math.inMeters
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import org.littletonrobotics.junction.Logger
import kotlin.math.abs
import kotlin.math.log

object Hood: Subsystem {
    private var io = when (Robot.model) {
        Model.SIMULATION -> HoodIOSim()
        Model.COMPETITION -> HoodIOReal()
    }

    private val inputs = LoggedHoodInputs()

    val atDesiredHoodAngle = Trigger {
        val error = abs((inputs.hoodAngle - shooterProfile.hoodAngle).inDegrees())
        Logger.recordOutput("Shooter/Hood/Angle Error", error.degrees)
        error < Constants.HOOD_ANGLE_TOLERANCE.inDegrees()
    }

    val angle: Angle
        get() = inputs.hoodAngle

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Shooter/Hood", inputs)
        Logger.recordOutput("Shooter/Shooter Target", shooterTarget.toString())
        Logger.recordOutput("Shooter/Hood/Reference", shooterProfile.hoodAngle)
    }

    fun calculateHoodAngle(distance: Distance): Angle {
        // https://www.desmos.com/calculator/504yoxmqbr
        // When we retune this probably isn't going to be logarithmic
        return (330.7608 * log(distance.inMeters(), 0.149) + 767.97283).degrees
    }

    fun turnToTargetHoodAngle(): Command =
        run {
            io.turnToAngle(clamp(shooterProfile.hoodAngle.inDegrees(), 25.0, 45.0).degrees)
        }

    fun turnToAngle(angle: Angle): Command =
        run {
            Logger.recordOutput("Shooter/Hood/setpoint", angle)
            io.turnToAngle(angle)
        }

    fun setVoltage(voltage: Voltage): Command =
        runEnd(
            {io.setVoltage(voltage)},
            {io.setVoltage(0.volts)}
        )

    fun hoodBrakeMode(brake: Boolean): Command =
        run {
            io.setBrakeMode(brake)
        }


    private val sysID = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            null,
            null,
            { state -> Logger.recordOutput("SysIdTestState", state.toString()) }
        ),
        SysIdRoutine.Mechanism(
            io::setVoltage,
            null,
            this
        )
    )

    fun sysIdQuasistatic(direction: Direction): Command = sysID.quasistatic(direction)

    fun sysIdDynamic(direction: Direction): Command = sysID.dynamic(direction)
}

object Constants {
    val HOOD_ANGLE_TOLERANCE = 6.0.degrees
}