package com.frcteam3636.frc2026.utils.shooting

import com.frcteam3636.frc2026.robot.Robot
import com.frcteam3636.frc2026.robot.Robot.Model
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.utils.autos.FIELD_WIDTH_METERS
import com.frcteam3636.frc2026.utils.math.meters
import com.frcteam3636.frc2026.utils.math.radians
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import kotlin.jvm.optionals.getOrNull
import kotlin.math.cos
import kotlin.math.sin

fun translatePose(pose: Pose2d, vx: Double, vy: Double, vtheta: Double, ax: Double, ay: Double, atheta: Double, tof: Double) : Pose2d {

    // These functions find the position at a given time

    fun dxdt(time: Double) : Double {
        val thetat = vtheta * time + 0.5 * atheta * time * time
        val vxt = vx + ax * time
        val vyt = vy + ay * time
        return vxt * cos(thetat) - vyt * sin(thetat)
    }

    fun dydt(time: Double) : Double {
        val thetat = vtheta * time + 0.5 * atheta * time * time
        val vxt = vx + ax * time
        val vyt = vy + ay * time
        return vxt * sin(thetat) + vyt * cos(thetat)
    }

    // This is using the Runge–Kutta method

    val t0 = 0.0
    val t1 = tof

    val k1x = dxdt(t0)
    val k1y = dydt(t0)

    val k2x = dxdt(t0 + tof/2)
    val k2y = dydt(t0 + tof/2)

    val k3x = dxdt(t0 + tof/2)
    val k3y = dydt(t0 + tof/2)

    val k4x = dxdt(t1)
    val k4y = dydt(t1)

    val dx = (tof / 6) * (k1x + 2*k2x + 2*k3x + k4x)
    val dy = (tof / 6) * (k1y + 2*k2y + 2*k3y + k4y)

    // Rotation has a closed‑form solution

    val dtheta = Rotation2d((vtheta * tof + 0.5 * atheta * tof * tof).radians)

    return pose + Transform2d(dx, dy, dtheta)

}



val hubTranslation
    get() = when (Robot.model) {
        // simulation defaults to red alliance
        Model.SIMULATION -> Translation3d(
            4.62534.meters,
            (8.07 / 2).meters,
            1.83.meters,
        )
        Model.COMPETITION -> when (DriverStation.getAlliance().orElse(Alliance.Blue)) {
            Alliance.Blue -> Translation3d(
                4.62534.meters,
                (8.07 / 2).meters,
                1.83.meters,
            )
            Alliance.Red -> Translation3d(
                (16.54 - 4.62534).meters,
                (8.07 / 2).meters,
                1.83.meters,
            )
        }
    }

val targetPassTranslation: Translation2d
    get() {
        val alliance = DriverStation.getAlliance().getOrNull()
        val pose = Drivetrain.estimatedPose.translation

        if (alliance == Alliance.Blue){
            if (pose.inZone(Zones.TopNeutralZone) || pose.inZone(Zones.TopRedAllianceZone)) {
                return Translation2d(4.meters, (FIELD_WIDTH_METERS / 4 ).meters)
            }
            else if (pose.inZone(Zones.BottomNeutralZone) || pose.inZone(Zones.BottomRedAllianceZone)){
                return Translation2d(4.meters, (FIELD_WIDTH_METERS * 3/4).meters)
            }
            else {
                return hubTranslation.toTranslation2d()
            }
        }
        else {
            if (pose.inZone(Zones.TopNeutralZone) || pose.inZone(Zones.TopBlueAllianceZone)) {
                return Translation2d(12.6.meters, (FIELD_WIDTH_METERS / 4 ).meters)
            }
            else if (pose.inZone(Zones.BottomNeutralZone) || pose.inZone(Zones.BottomBlueAllianceZone)){
                return Translation2d(12.6.meters, (FIELD_WIDTH_METERS * 3/4).meters)
            }
            else {
                return hubTranslation.toTranslation2d()
            }
        }
    }

enum class Zones(val startX : Distance, val endX : Distance, val startY: Distance, val endY : Distance) {
    TopBlueAllianceZone(0.meters, 4.03.meters, 0.meters, 4.meters),
    BottomBlueAllianceZone(0.meters, 4.03.meters, 4.meters, 8.meters),
    TopRedAllianceZone(11.22.meters, 16.20.meters, 0.meters, 4.meters),
    BottomRedAllianceZone(11.22.meters, 16.20.meters, 4.meters, 8.meters),
    TopNeutralZone(4.03.meters, 12.22.meters, 0.meters, 4.meters),
    BottomNeutralZone(4.03.meters, 12.22.meters, 4.meters, 8.meters),
}

fun Translation2d.inZone(target: Zones): Boolean {
    return this.x.meters in target.startX..<target.endX && this.y.meters in target.startY..<target.endY
}