package com.frcteam3636.frc2026.subsystems.objectDetection

import com.frcteam3636.frc2026.robot.Robot
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain.estimatedPose
import com.frcteam3636.frc2026.utils.autos.APTargetWithTolerance
import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.inMeters
import com.frcteam3636.frc2026.utils.math.inRadians
import com.frcteam3636.frc2026.utils.math.meters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.floor
import kotlin.math.tan

object ObjectDetection : Subsystem {

    private val io = when(Robot.model){
        Robot.Model.COMPETITION -> ObjectDetectionIOReal()
        Robot.Model.SIMULATION -> ObjectDetectionIOSim()
    }

    var inputs = LoggedObjectDetectionInputs()

    override fun periodic() {
        io.updateResult()
        io.updateInputs(inputs)
    }

    val targets: List<PhotonTrackedTarget> get() = io.newestResults.targets

    val fuelYaws : ArrayList<Double>
        get() {
            val yaws = ArrayList<Double>(targets.size)
            for(i in 0 until targets.size){
                yaws[i] = targets[i].yaw
            }
            return yaws
        }

    val fuelPitches : Array<Double>
        get() {
            val pitches = Array(targets.size) {0.0}
            for(i in 0 until targets.size){
                pitches[i] = targets[i].pitch
            }
            return pitches
        }

    fun driveToLargestFuelCluster() : Command = Commands.run({
        if(fuelYaws.isNotEmpty()){

            val groupedYaws = fuelYaws.
            groupBy { floor(it / 9.0)}

            val largestCluster = groupedYaws.maxBy { it.value.size }.value
            val smallestPitch = largestCluster.minBy { it }
            val distance = (Constant.CAMERA_OFFSET.z.meters - Constant.FUEL_RADIUS/ tan(smallestPitch.degrees.inRadians() + Constant.CAMERA_PITCH.inRadians()))
            Logger.recordOutput("photonvision/Largest Fuel Cluster Distance", distance)

            val targetHorizontalAngle = largestCluster.average()

            Logger.recordOutput("photonvision/Largest Fuel Cluster Yaw", targetHorizontalAngle)

            val targetPose = Pose2d(
                estimatedPose.translation + Translation2d(distance.inMeters(), Rotation2d(targetHorizontalAngle)),
                Rotation2d.k180deg
            )

            Logger.recordOutput("photonvision/Target Fuel Pose", targetPose )
            Drivetrain.alignWithAutopilot(APTargetWithTolerance(targetPose))
        }
    })
}
object Constant{
    val CAMERA_OFFSET = Translation3d(0.6477.meters, 0.08.meters, 0.04.meters)
    val CAMERA_PITCH = (-10).degrees
    val FUEL_RADIUS = 0.075.meters

}
