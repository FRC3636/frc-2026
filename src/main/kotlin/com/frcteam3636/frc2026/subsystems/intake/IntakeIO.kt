package com.frcteam3636.frc2026.subsystems.intake

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2026.CANcoder
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.PIDGains
import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.inRotations
import com.frcteam3636.frc2026.utils.math.inRotationsPerSecond
import com.frcteam3636.frc2026.utils.math.inRotationsPerSecondPerSecond
import com.frcteam3636.frc2026.utils.math.inVolts
import com.frcteam3636.frc2026.utils.math.pidGains
import com.frcteam3636.frc2026.utils.math.rotations
import com.frcteam3636.frc2026.utils.math.rotationsPerSecond
import com.frcteam3636.frc2026.utils.math.rotationsPerSecondPerSecond
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged
import kotlin.apply

@Logged
open class IntakeInputs {
    var intakeMotorVelocity = 0.rotationsPerSecond
    var intakeMotorCurrent = Amps.zero()!!
    var pivotAngle = 0.degrees
    var intakePivotMotorCurrent = Amps.zero()!!
    var rightPivotMotorCurrent = Amps.zero()!!
}

interface IntakeIO {
    fun setSpeed(percent: Double)
    fun setWheelMotorVoltage(voltage: Voltage)
    fun setPivotAngle(angle: Angle)
    fun updateInputs(inputs: IntakeInputs)
}

class IntakeIOReal : IntakeIO {
    companion object Constants {
        val PID_GAINS = PIDGains(25.0, 0.0, 0.0)
        val PROFILE_CRUISE_VELOCITY = 1.0.rotationsPerSecond
        val PROFILE_ACCELERATION = (6.7 / 2.0).rotationsPerSecondPerSecond
        val PROFILE_JERK = 0.0
        val ENCODER_TO_PIVOT_GEAR_RATIO = 2.0
        val MOTOR_TO_ENCODER_GEAR_RATIO = 4.0
        val MAGNET_OFFSET = 0.1777.rotations

        val LEFT_MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive
        val RIGHT_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive
    }

    private val pivotMotorConfig = TalonFXConfiguration()

    private val intakePivotMotor = TalonFX(CTREDeviceId.IntakePivotMotor).apply {
        configurator.apply(TalonFXConfiguration().apply {
            Slot0.apply {
                pidGains = PID_GAINS
            }
            MotionMagic.apply {
                MotionMagicCruiseVelocity = PROFILE_CRUISE_VELOCITY.inRotationsPerSecond()
                MotionMagicAcceleration = PROFILE_ACCELERATION.inRotationsPerSecondPerSecond()
                MotionMagicJerk = PROFILE_JERK
            }
            Feedback.apply {
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
                FeedbackRemoteSensorID = CTREDeviceId.IntakePivotEncoder.num
                SensorToMechanismRatio = ENCODER_TO_PIVOT_GEAR_RATIO
                RotorToSensorRatio = MOTOR_TO_ENCODER_GEAR_RATIO
            }
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
        })
    }
    private val intakeMotor = TalonFX(CTREDeviceId.IntakeMotor).apply {
        configurator.apply(TalonFXConfiguration().apply { MotorOutput.Inverted = LEFT_MOTOR_DIRECTION })
    }


    init {
        CANcoder(CTREDeviceId.IntakePivotEncoder).apply {
            configurator.apply(CANcoderConfiguration().apply {
                MagnetSensor.apply {
                    MagnetOffset = MAGNET_OFFSET.inRotations()
                }
//                ExternalFeedbackConfigs().apply {
//                    SensorToMechanismRatio = ENCODER_TO_PIVOT_GEAR_RATIO
//                    RotorToSensorRatio = MOTOR_TO_ENCODER_GEAR_RATIO
//                }
            })
        }
    }

    override fun setSpeed(percent: Double) {
        intakeMotor.set(percent)
    }

    override fun setWheelMotorVoltage(voltage: Voltage) {
        intakeMotor.setVoltage(voltage.inVolts())
    }

    override fun setPivotAngle(angle: Angle) {
        Logger.recordOutput("Intake/Pivot Setpoint", angle)
        val controlRequest = MotionMagicVoltage(angle)
        intakePivotMotor.setControl(controlRequest.withPosition(angle))
    }

    override fun updateInputs(inputs: IntakeInputs) {
        inputs.intakeMotorVelocity = intakeMotor.velocity.value
        inputs.intakeMotorCurrent = intakeMotor.supplyCurrent.value

        inputs.intakePivotMotorCurrent = intakePivotMotor.supplyCurrent.value
//        inputs.rightPivotMotorCurrent = rightPivotMotor.supplyCurrent.value
        inputs.pivotAngle = intakePivotMotor.position.value

    }
}

//class IntakeIOSim: IntakeIO {
//
//    val intakeSimulation = IntakeSimulation.OverTheBumperIntake(
//        "Fuel",
//        Drivetrain.getSwerveDriveSimulation(),
//        Drivetrain.Constants.BUMPER_WIDTH,
//        0.182.meters,
//        IntakeSimulation.IntakeSide.BACK,
//        40
//    )!!
//
//    fun setRunning(runIntake: Boolean) {
//        if (runIntake) {
//            intakeSimulation.startIntake()
//        } else {
//            intakeSimulation.stopIntake()
//        }
//    }
//
//    val isFuelInsideIntake: Boolean
//        get() {
//            return intakeSimulation.gamePiecesAmount != 0
//        }
//
//    override fun setSpeed(percent: Double) {
//        TODO("Not yet implemented")
//    }
//
//    override fun setRunMotorVoltage(voltage: Voltage) {
//        TODO("Not yet implemented")
//    }
//
//    override fun setPivotAngle(angle: Angle) {
//        TODO("Not yet implemented")
//    }
//
//    override fun updateInputs(inputs: IntakeInputs) {
//        Logger.recordOutput("FieldSimulation/NumberOfFuel", intakeSimulation.gamePiecesAmount)
//    }
//}