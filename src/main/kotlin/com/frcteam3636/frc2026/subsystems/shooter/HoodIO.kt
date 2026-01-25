package com.frcteam3636.frc2026.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.frcteam3636.frc2026.CANcoder
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.*
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

@Logged
open class HoodInputs {
    var hoodAngle = Radians.zero()!!
    var hoodVelocity = RadiansPerSecond.zero()!!
    var hoodCurrent = Amps.zero()!!
    var setPoint = Radians.zero()!!
    var hoodMotorTemperature = Celsius.zero()!!
    var brakeMode = false
    var fixedHood = false
}

interface HoodIO {
    fun turnToAngle(angle: Angle)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: HoodInputs)
    fun setBrakeMode(enabled: Boolean)

    val signals: Array<BaseStatusSignal>
        get() = emptyArray()
}

class HoodIOReal: HoodIO {
    private var brakeMode = false

    private val hoodMotor = TalonFX(CTREDeviceId.HoodMotor).apply {
        configurator.apply(TalonFXConfiguration().apply {

            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
            Slot0.apply {
                pidGains = PID_GAINS
            }
            MotionMagic.apply {
                MotionMagicAcceleration = PROFILE_ACCELERATION.inRotationsPerSecondPerSecond()
                MotionMagicJerk = PROFILE_JERK
            }
            Feedback.apply {
                FeedbackRemoteSensorID = CTREDeviceId.HoodMotor.num
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
                SensorToMechanismRatio = SENSOR_TO_MECHANISM_GEAR_RATIO
                RotorToSensorRatio = ROTOR_TO_SENSOR_GEAR_RATIO
            }

        })
    }

    private val positionSignal = hoodMotor.position
    private val velocitySignal = hoodMotor.velocity
    private val currentSignal = hoodMotor.supplyCurrent
    private val temperatureSignal = hoodMotor.deviceTemp

    init {
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            positionSignal,
            velocitySignal,
            currentSignal,
            temperatureSignal
        )
        hoodMotor.optimizeBusUtilization()

        CANcoder(CTREDeviceId.HoodEncoder).apply {
            CANcoderConfiguration().apply {
                MagnetSensor.MagnetOffset = MAGNET_OFFSET
                MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive
            }
        }
    }

    val positionControl: MotionMagicVoltage = MotionMagicVoltage(0.0).apply {
        UpdateFreqHz = 0.0
    }

    override fun turnToAngle(angle: Angle) {
        assert(angle.inRadians() in MIN_HOOD_ANGLE..MAX_HOOD_ANGLE)
        hoodMotor.setControl(positionControl.withPosition(angle))
    }

    override fun setVoltage(voltage: Voltage) {
        assert(voltage in 0.volts..12.volts)
        hoodMotor.setVoltage(voltage.inVolts())
    }

    override fun updateInputs(inputs: HoodInputs) {
        inputs.hoodAngle = positionSignal.value
        inputs.hoodVelocity = velocitySignal.value
        inputs.hoodCurrent = currentSignal.value
        inputs.hoodMotorTemperature = temperatureSignal.value
        inputs.brakeMode = brakeMode
    }

    override fun setBrakeMode(enabled: Boolean) {
        brakeMode = enabled
        hoodMotor.setNeutralMode(
            if (enabled) {
                NeutralModeValue.Brake
            } else {
                NeutralModeValue.Coast
            }
        )
    }


    companion object Constants {
        private val MAGNET_OFFSET = 0.0
        private val PID_GAINS = PIDGains(5.0, 0.0, 0.0)
        private const val SENSOR_TO_MECHANISM_GEAR_RATIO = 10.0
        private const val ROTOR_TO_SENSOR_GEAR_RATIO = 10.0
        private val PROFILE_ACCELERATION = 2.0.rotationsPerSecondPerSecond
        private val PROFILE_JERK = 0.0
        private val MAX_HOOD_ANGLE = 50.degrees.inRadians()
        private val MIN_HOOD_ANGLE = 30.degrees.inRadians()
    }
}

class HoodIOSim: HoodIO {

    override fun turnToAngle(angle: Angle) {
        TODO("Not yet implemented")
    }

    override fun setVoltage(voltage: Voltage) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: HoodInputs) {
        TODO("Not yet implemented")
    }

    override fun setBrakeMode(enabled: Boolean) {
        TODO("Not yet implemented")
    }

}