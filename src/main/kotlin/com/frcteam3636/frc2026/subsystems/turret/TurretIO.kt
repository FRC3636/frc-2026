package com.frcteam3636.frc2026.subsystems.turret

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.configs.TalonFXConfigurator
import com.ctre.phoenix6.configs.TalonFXSConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.RainbowAnimation
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.frcteam3636.frc2026.CANcoder
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.utils.math.PIDGains
import com.frcteam3636.frc2026.utils.math.amps
import com.frcteam3636.frc2026.utils.math.inRotationsPerSecondPerSecond
import com.frcteam3636.frc2026.utils.math.pidGains
import com.frcteam3636.frc2026.utils.math.radians
import com.frcteam3636.frc2026.utils.math.rotationsPerSecond
import com.frcteam3636.frc2026.utils.math.rotationsPerSecondPerSecond
import com.frcteam3636.frc2026.utils.math.toRotation2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.Angle
import jdk.jfr.Enabled
import org.team9432.annotation.Logged

@Logged
open class TurretInputs{
    var turretAngle = Radians.zero()!!
    var turretCurrent = Amps.zero()!!
    var setPoint = Radians.zero()!!
    var turretVelocity = RadiansPerSecond.zero()!!
    var turretMotorTemperature = Celsius.zero()!!
    var brakeMode = false
}

interface TurretIO{
    fun turnToAngle(angle: Angle)
    fun updateInputs(inputs: TurretInputs)
    fun setBrakeMode(enabled: Boolean)

    val signals: Array<BaseStatusSignal>
        get() = emptyArray()
}

class TurretIOReal : TurretIO {
    private var brakeMode = false

    private val turretTurningMotor = TalonFX(CTREDeviceId.TurretTurningMotor).apply {
        configurator.apply(TalonFXConfiguration().apply {

            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
            Slot0.apply {
                pidGains = PID_GAINS
            }
            MotionMagic.apply {
                //MotionMagicCruiseVelocity = PROFILE_CRUISE_VELOCITY.inRotationsPerSecond()
                MotionMagicAcceleration = PROFILE_ACCELERATION.inRotationsPerSecondPerSecond()
                MotionMagicJerk = PROFILE_JERK
            }
            Feedback.apply {
                FeedbackRemoteSensorID = CTREDeviceId.TurretTurningEncoder.num
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
                SensorToMechanismRatio = SENSOR_TO_MECHANISM_GEAR_RATIO
                RotorToSensorRatio = ROTOR_TO_SENSOR_GEAR_RATIO
            }
            CurrentLimits.apply {
                SupplyCurrentLowerLimit = 30.0
                SupplyCurrentLimit = 40.0
                SupplyCurrentLowerTime = 1.0
            }

        })
    }

    private val positionSignal = turretTurningMotor.position
    private val velocitySignal = turretTurningMotor.velocity
    private val currentSignal = turretTurningMotor.supplyCurrent
    private val temperatureSignal = turretTurningMotor.deviceTemp


    init{
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            positionSignal,
            currentSignal,
            velocitySignal,
            temperatureSignal
        )
        turretTurningMotor.optimizeBusUtilization()

        CANcoder(CTREDeviceId.TurretTurningEncoder).apply {
            configurator.apply(CANcoderConfiguration().apply {
                MagnetSensor.MagnetOffset = MAGNET_OFFSET
                MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive
            })
        }
    }
    val positionControl: MotionMagicVoltage = MotionMagicVoltage(0.0).apply {
        UpdateFreqHz = 0.0
    }

    override fun turnToAngle(angle: Angle) {
        turretTurningMotor.setControl(positionControl.withPosition(angle))
    }

    override fun updateInputs(inputs: TurretInputs) {
        inputs.turretAngle = positionSignal.value
        inputs.turretCurrent = currentSignal.value
        inputs.turretVelocity = velocitySignal.value
        inputs.turretMotorTemperature = temperatureSignal.value
        inputs.brakeMode = brakeMode
    }

    override fun setBrakeMode(enabled: Boolean) {
        brakeMode = enabled
        turretTurningMotor.setNeutralMode(
            if (enabled) {
                NeutralModeValue.Brake
            } else {
                NeutralModeValue.Coast
            }
        )
    }


    companion object Constants{
        private val PID_GAINS = PIDGains(62.0, 0.0, 0.0)
        private const val SENSOR_TO_MECHANISM_GEAR_RATIO = 1.0
        private const val ROTOR_TO_SENSOR_GEAR_RATIO = 37.25
        private const val MAGNET_OFFSET = -0.191650390625
        private val PROFILE_ACCELERATION = 2.0.rotationsPerSecondPerSecond
        private val PROFILE_JERK = 0.0
        private val PROFILE_VELOCITY = 12.0.rotationsPerSecond
    }
}

class TurretIOSim: TurretIO {
    override fun turnToAngle(angle: Angle) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: TurretInputs) {
        TODO("Not yet implemented")
    }

    override fun setBrakeMode(enabled: Boolean) {
        TODO("Not yet implemented")
    }

}

    //kraken x44
    // WCP ThroughBore Encoder


