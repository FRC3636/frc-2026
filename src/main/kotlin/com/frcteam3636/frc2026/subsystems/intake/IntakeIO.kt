package com.frcteam3636.frc2026.subsystems.intake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.drivetrain.DrivetrainIOSim
import com.frcteam3636.frc2026.utils.math.PIDGains
import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.inRotationsPerSecond
import com.frcteam3636.frc2026.utils.math.inRotationsPerSecondPerSecond
import com.frcteam3636.frc2026.utils.math.inVolts
import com.frcteam3636.frc2026.utils.math.meters
import com.frcteam3636.frc2026.utils.math.pidGains
import com.frcteam3636.frc2026.utils.math.rotationsPerSecond
import com.frcteam3636.frc2026.utils.math.rotationsPerSecondPerSecond
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import org.ironmaple.simulation.IntakeSimulation
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged
import kotlin.apply

@Logged
open class IntakeInputs {
    var intakeMotorVelocity = 0.rotationsPerSecond
    var intakeMotorCurrent = Amps.zero()!!
    var pivotAngle = 0.degrees
    var leftPivotMotorCurrent = Amps.zero()!!
    var rightPivotMotorCurrent = Amps.zero()!!


}

interface IntakeIO {
    fun setSpeed(percent: Double)
    fun setRunMotorVoltage(voltage: Voltage)
    fun setPivotAngle(angle: Angle)
    fun updateInputs(inputs: IntakeInputs)
}

class IntakeIOReal : IntakeIO {
    companion object Constants {
        val PID_GAINS = PIDGains(25.0, 0.0, 0.0)
        val PROFILE_CRUISE_VELOCITY = 1.0.rotationsPerSecond
        val PROFILE_ACCELERATION = (6.7 / 2.0).rotationsPerSecondPerSecond
        val PROFILE_JERK = 0.0
        val ENCODER_TO_PIVOT_GEAR_RATIO = 2.25
        val MOTOR_TO_ENCODER_GEAR_RATIO = 4.0

        val LEFT_MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive
        val RIGHT_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive
    }

    private val pivotMotorConfig = TalonFXConfiguration()

    private val intakePivotMotor = TalonFX(CTREDeviceId.IntakeMotor).apply {
        configurator.apply(TalonFXConfiguration().apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
        })
    }
    private val leftPivotMotor = TalonFX(CTREDeviceId.LeftPivotMotor).apply {
        configurator.apply(TalonFXConfiguration().apply { MotorOutput.Inverted = LEFT_MOTOR_DIRECTION })
    }
    private val rightPivotMotor = TalonFX(CTREDeviceId.RightPivotMotor).apply {
        configurator.apply(TalonFXConfiguration().apply { MotorOutput.Inverted = RIGHT_MOTOR_DIRECTION })
    }


    init {
        pivotMotorConfig.apply {
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
            }
        }
        leftPivotMotor.configurator.apply(pivotMotorConfig)
        rightPivotMotor.configurator.apply(pivotMotorConfig)
    }

    override fun setSpeed(percent: Double) {
        intakePivotMotor.set(percent)
    }

    override fun setRunMotorVoltage(voltage: Voltage) {
        intakePivotMotor.setVoltage(voltage.inVolts())
    }

    override fun setPivotAngle(angle: Angle) {
        Logger.recordOutput("Intake/Pivot Setpoint", angle)
        val controlRequest = MotionMagicVoltage(angle)
        rightPivotMotor.setControl(controlRequest.withPosition(angle))
    }

    override fun updateInputs(inputs: IntakeInputs) {
        inputs.intakeMotorVelocity = intakePivotMotor.velocity.value
        inputs.intakeMotorCurrent = intakePivotMotor.supplyCurrent.value

        inputs.leftPivotMotorCurrent = leftPivotMotor.supplyCurrent.value
        inputs.rightPivotMotorCurrent = rightPivotMotor.supplyCurrent.value
        inputs.pivotAngle = leftPivotMotor.position.value

    }
}

class IntakeIOSim: IntakeIO {

    val intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        "Fuel",
        Drivetrain.getSwerveDriveSimulation(),
        Drivetrain.Constants.BUMPER_WIDTH,
        0.182.meters,
        IntakeSimulation.IntakeSide.BACK,
        40
    )!!

    fun setRunning(runIntake: Boolean) {
        if (runIntake) {
            intakeSimulation.startIntake()
        } else {
            intakeSimulation.stopIntake()
        }
    }

    val isFuelInsideIntake: Boolean
        get() {
            return intakeSimulation.gamePiecesAmount != 0
        }

    override fun setSpeed(percent: Double) {
        TODO("Not yet implemented")
    }

    override fun setRunMotorVoltage(voltage: Voltage) {
        TODO("Not yet implemented")
    }

    override fun setPivotAngle(angle: Angle) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: IntakeInputs) {
        Logger.recordOutput("FieldSimulation/NumberOfFuel", intakeSimulation.gamePiecesAmount)
    }
}