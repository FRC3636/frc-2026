package com.frcteam3636.frc2026.subsystems.shooter.turret

import com.frcteam3636.frc2026.robot.Robot
import com.frcteam3636.frc2026.robot.Robot.Model
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.shooter.FeedTranslation
import com.frcteam3636.frc2026.subsystems.shooter.Zones
import com.frcteam3636.frc2026.subsystems.shooter.hubTranslation
import com.frcteam3636.frc2026.subsystems.shooter.shooterFieldPose
import com.frcteam3636.frc2026.subsystems.shooter.shooterProfile
import com.frcteam3636.frc2026.subsystems.shooter.shooterTarget
import com.frcteam3636.frc2026.subsystems.shooter.shooterToHub
import com.frcteam3636.frc2026.utils.autos.flipHorizontally
import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.inDegrees
import com.frcteam3636.frc2026.utils.math.inMeters
import com.frcteam3636.frc2026.utils.math.meters
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import org.littletonrobotics.junction.Logger
import kotlin.jvm.optionals.getOrNull
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

    fun getClosetTarget() : Translation2d {
        var ourAllianceZone = Zones.BlueAllianceZone
        var opposingAllianceZone = Zones.RedAllianceZone

        if (DriverStation.getAlliance().getOrNull() == Alliance.Red){
            ourAllianceZone = Zones.RedAllianceZone
            opposingAllianceZone = Zones.BlueAllianceZone
        }

        val target =  when (Drivetrain.estimatedPose.x) {
            in ourAllianceZone.startY.inMeters()..ourAllianceZone.endY.inMeters() -> hubTranslation.toTranslation2d()
            in opposingAllianceZone.startY.inMeters()..ourAllianceZone.endY.inMeters() -> FeedTranslation.RightSideNeutralZone.target
            else -> FeedTranslation.RightSideAllianceZone.target
        }

        if (Drivetrain.estimatedPose.translation.y < 4.035.meters.inMeters()) {
            return target.flipHorizontally()
        }

        return target
    }

    fun setTargetAngle(angle: Angle): Command =
        run {
            io.turnToAngle(angle)
        }

    private val maxAngle = 90.0.degrees
    private val minAngle = (-85.0).degrees
    fun turnToTargetTurretAngle(): Command =
        run {
            val angleSetpoint = if(shooterProfile.turretAngle < maxAngle && shooterProfile.turretAngle > minAngle) {
                shooterProfile.turretAngle
            } else if (shooterProfile.turretAngle > (maxAngle + minAngle).div(2.0) + 180.0.degrees){
                maxAngle
            } else {
                minAngle
            }
            io.turnToAngle(angleSetpoint)
        }

    fun turnToTargetHubAngle(): Command =
        run {
            setTargetAngle(shooterToHub.angle.measure - Drivetrain.estimatedPose.rotation.measure)
        }

    fun zeroTurretEncoder() : Command =
        run{
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