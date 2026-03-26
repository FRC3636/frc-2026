package com.frcteam3636.frc2026.subsystems.intake

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2026.CANcoder
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.PIDGains
import com.frcteam3636.frc2026.utils.math.amps
import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.inRotationsPerSecond
import com.frcteam3636.frc2026.utils.math.inRotationsPerSecondPerSecond
import com.frcteam3636.frc2026.utils.math.inVolts
import com.frcteam3636.frc2026.utils.math.pidGains
import com.frcteam3636.frc2026.utils.math.rotationsPerSecond
import com.frcteam3636.frc2026.utils.math.rotationsPerSecondPerSecond
import com.frcteam3636.frc2026.utils.math.volts
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Volts
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
    var pivotSetpoint = 0.degrees
    var intakePivotMotorCurrent = Amps.zero()!!
    var intakePivotMotorVoltage = Volts.zero()!!
    var intakePivotMotorSupplyVoltage = 0.0.volts
    var rightPivotMotorCurrent = Amps.zero()!!
}

interface IntakeIO {
    fun setSpeed(percent: Double)
    fun setPivotVoltage(voltage: Voltage)
    fun setPivotSpeed(pivot: Double)
    fun setWheelMotorVoltage(voltage: Voltage)
    fun setPivotAngle(angle: Angle)
    fun updateInputs(inputs: IntakeInputs)
}

class IntakeIOReal : IntakeIO {
    companion object Constants {
        val PID_GAINS = PIDGains(128.0, 0.0, 0.0)
        val PROFILE_CRUISE_VELOCITY = 20.0.rotationsPerSecond
        val PROFILE_ACCELERATION = 20.rotationsPerSecondPerSecond
        val PROFILE_JERK = 20.0
        val ENCODER_TO_PIVOT_GEAR_RATIO = 32.0 / 12.0
        val MOTOR_TO_ENCODER_GEAR_RATIO = 4.0
        val DISCONTINUITY_POINT = 0.999
        val MAGNET_OFFSET = 0.130859375

        val PIVOT_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive
        val WHEEL_MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive
    }

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
                FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
                FeedbackRemoteSensorID = CTREDeviceId.IntakePivotEncoder.num
                SensorToMechanismRatio = ENCODER_TO_PIVOT_GEAR_RATIO
                RotorToSensorRatio = MOTOR_TO_ENCODER_GEAR_RATIO
            }
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = PIVOT_MOTOR_DIRECTION
            }
        })
    }
    private val intakeMotor = TalonFX(CTREDeviceId.IntakeMotor).apply {
        configurator.apply(TalonFXConfiguration().apply { MotorOutput.Inverted = WHEEL_MOTOR_DIRECTION })
    }

    init {
        CANcoder(CTREDeviceId.IntakePivotEncoder).apply {
            configurator.apply(CANcoderConfiguration().apply {
                MagnetSensor.apply {
                    AbsoluteSensorDiscontinuityPoint = DISCONTINUITY_POINT
                    MagnetOffset = MAGNET_OFFSET
                }
            })
        }
    }

    override fun setSpeed(percent: Double) {
        intakeMotor.set(percent)
    }

    override fun setPivotSpeed(pivot: Double) {
        intakePivotMotor.set(pivot)
    }

    override fun setPivotVoltage(voltage: Voltage) {
        Logger.recordOutput("Intake/Pivot Attempted Voltage", voltage)
        intakePivotMotor.setVoltage(voltage.inVolts())
    }

    override fun setWheelMotorVoltage(voltage: Voltage) {
        intakeMotor.setVoltage(voltage.inVolts())
    }

    private val positionControl = MotionMagicVoltage(0.0)

    override fun setPivotAngle(angle: Angle) {
        Logger.recordOutput("Intake/Pivot Setpoint", angle)
        intakePivotMotor.setControl(positionControl.withPosition(angle))
    }

    var setpoint = 0.degrees

    override fun updateInputs(inputs: IntakeInputs) {
        inputs.intakeMotorVelocity = intakeMotor.velocity.value
        inputs.intakeMotorCurrent = intakeMotor.supplyCurrent.value

        inputs.intakePivotMotorCurrent = intakePivotMotor.supplyCurrent.value
        inputs.intakePivotMotorVoltage = intakePivotMotor.motorVoltage.value
        inputs.intakePivotMotorSupplyVoltage = intakeMotor.supplyVoltage.value
//        inputs.rightPivotMotorCurrent = rightPivotMotor.supplyCurrent.value
        inputs.pivotAngle = intakePivotMotor.position.value
        inputs.pivotSetpoint = setpoint
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