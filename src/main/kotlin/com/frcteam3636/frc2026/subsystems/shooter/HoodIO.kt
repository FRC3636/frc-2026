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
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Torque
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.team9432.annotation.Logged

@Logged
open class HoodInputs {
    var hoodAngle = Radians.zero()!!
    var hoodVelocity = RadiansPerSecond.zero()!!
    var hoodCurrent = Amps.zero()!!
    var setPoint = Radians.zero()!!
    var motorTemperature = Celsius.zero()!!
    var brakeMode = false
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
    private var fixedHood = false

    private val motor = TalonFX(CTREDeviceId.HoodMotor).apply {
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

    private val positionSignal = motor.position
    private val velocitySignal = motor.velocity
    private val currentSignal = motor.supplyCurrent
    private val temperatureSignal = motor.deviceTemp

    init {
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            positionSignal,
            velocitySignal,
            currentSignal,
            temperatureSignal
        )
        motor.optimizeBusUtilization()

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
        motor.setControl(positionControl.withPosition(angle))
    }

    override fun setVoltage(voltage: Voltage) {
        assert(voltage in 0.volts..12.volts)
        motor.setVoltage(voltage.inVolts())
    }

    override fun updateInputs(inputs: HoodInputs) {
        inputs.hoodAngle = positionSignal.value
        inputs.hoodVelocity = velocitySignal.value
        inputs.hoodCurrent = currentSignal.value
        inputs.motorTemperature = temperatureSignal.value
        inputs.brakeMode = brakeMode
    }

    override fun setBrakeMode(enabled: Boolean) {
        brakeMode = enabled
        motor.setNeutralMode(
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
    private val motor = DCMotor.getKrakenX60(1)
    private val system = LinearSystemId.createDCMotorSystem(motor, 1.0,1.0)
    private val sim = DCMotorSim(system, motor)
    private var breakMode = false

    override fun turnToAngle(angle: Angle) {
        if(!breakMode){
            sim.setAngle(angle.inRadians())
        }

    }

    override fun setVoltage(voltage: Voltage) {
        sim.inputVoltage = voltage.inVolts()
    }

    override fun updateInputs(inputs: HoodInputs) {
        inputs.hoodAngle = sim.angularPosition
        inputs.brakeMode = breakMode
        inputs.hoodCurrent = motor.getCurrent(sim.torqueNewtonMeters).amps
    }

    override fun setBrakeMode(enabled: Boolean) {
        breakMode = enabled
    }

}