package com.frcteam3636.frc2026.subsystems.intake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.PIDGains
import com.frcteam3636.frc2026.utils.math.inRotationsPerSecond
import com.frcteam3636.frc2026.utils.math.inRotationsPerSecondPerSecond
import com.frcteam3636.frc2026.utils.math.inVolts
import com.frcteam3636.frc2026.utils.math.pidGains
import com.frcteam3636.frc2026.utils.math.rotationsPerSecond
import com.frcteam3636.frc2026.utils.math.rotationsPerSecondPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged
import kotlin.apply

@Logged
open class IntakeInputs {
    var intakeMotorVelocity = 0.rotationsPerSecond
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
        val ENCODER_MAGNET_OFFSET = -0.17529
        val ENCODER_TO_PIVOT_GEAR_RATIO = 2.25
        val MOTOR_TO_ENCODER_GEAR_RATIO = 4.0
        val ABSOLUTE_SENSOR_DISCONTINUITY_POINT = 0.6
    }

    private val runMotor = TalonFX(CTREDeviceId.IntakeRunMotor).apply {
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

    private val leftMotor = TalonFX(CTREDeviceId.LeftPivotMotor).apply {
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

    private val rightMotor = TalonFX(CTREDeviceId.RightPivotMotor).apply {
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

    override fun setSpeed(percent: Double) {
        TODO("Not yet implemented")
    }

    override fun setRunMotorVoltage(voltage: Voltage) {
        runMotor.setVoltage(voltage.inVolts())
    }

    override fun setPivotAngle(angle: Angle) {
        val controlRequest = MotionMagicVoltage(angle)

    }

    override fun updateInputs(inputs: IntakeInputs) {
        TODO("Not yet implemented")
    }


}