package com.frcteam3636.frc2026.subsystems.shooter.turret

import com.frcteam3636.frc2026.robot.Robot
import com.frcteam3636.frc2026.robot.Robot.Model
import com.frcteam3636.frc2026.subsystems.shooter.shooterFieldPose
import com.frcteam3636.frc2026.subsystems.shooter.shooterProfile
import com.frcteam3636.frc2026.subsystems.shooter.shooterTarget
import com.frcteam3636.frc2026.subsystems.shooter.shooterToHub
import com.frcteam3636.frc2026.utils.math.inDegrees
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

object Turret : Subsystem {
    private var io = when (Robot.model) {
        Model.SIMULATION -> TurretIOSim()
        Model.COMPETITION -> TurretIOReal()
    }

    private val inputs = LoggedTurretInputs()
    private val sysID = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            null,
            null,
            {
                    state -> Logger.recordOutput("SysIdTestState", state.toString())
            }
        ),
        SysIdRoutine.Mechanism(
            io::setVoltage,
            null,
            this
        )
    )

    override fun periodic() {
        shooterProfile = shooterTarget.profile()
        inputs.setPoint = shooterProfile.turretAngle
        Logger.processInputs("Shooter/Turret", inputs)
        io.updateInputs(inputs)

        Logger.recordOutput("Shooter/Turret/TurretDistanceToHub", shooterToHub.norm)
        Logger.recordOutput("Shooter/Shooter Pose", shooterFieldPose)
        Logger.recordOutput("Shooter/Turret/atDesiredTurretAngle", atTargetTurretAngle)
    }

    val turretAngle: Rotation2d
        get() {
            return Rotation2d(inputs.angle)
        }

    val atTargetTurretAngle: Trigger = Trigger { abs(inputs.angle.inDegrees() - shooterProfile.turretAngle.inDegrees()) < Constants.TURRET_TOLERANCE }

    fun setTargetAngle(angle: Angle): Command =
        run {
            io.turnToAngle(angle)
        }

    fun turnToSetpoint(): Command =
        run {
            io.turnToAngle(shooterProfile.turretAngle)
        }


    fun zeroTurretEncoder() : Command =
        runOnce {
            println("zeroing turret")
            io.zeroEncoder()
        }

    fun turretBrakeMode(): Command =
        run {
            io.setBrakeMode(true)
        }

    fun turretCoastMode(): Command =
        run {
            io.setBrakeMode(false)
        }

    fun sysIdQuasistatic(direction: Direction): Command = sysID.quasistatic(direction)

    fun sysIdDynamic(direction: Direction): Command = sysID.dynamic(direction)
}
object Constants {
    val SHOOTER_OFFSET = Translation2d(.184, -.184)
    val TURRET_TOLERANCE = 3.0
}