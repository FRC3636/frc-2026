package com.frcteam3636.frc2026.subsystems.shooter.turret

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
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged

@Logged
open class TurretInputs {
    var angle = Rotations.zero()!!
    var motorCurrent = Amps.zero()!!
    var setPoint = Rotations.zero()!!
    var motorVelocity = RadiansPerSecond.zero()!!
    var motorTemperature = Celsius.zero()!!
    var seeTags = false
    var brakeMode = false
}

interface TurretIO {
    fun turnToAngle(angle: Angle)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: TurretInputs)
    fun setBrakeMode(enabled: Boolean)
    fun zeroEncoder()

    val signals: Array<BaseStatusSignal>
        get() = emptyArray()
}

class TurretIOReal : TurretIO {
    private var brakeMode = false

    private val motor = TalonFX(CTREDeviceId.TurretTurningMotor).apply {
        configurator.apply(TalonFXConfiguration().apply {

            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.CounterClockwise_Positive
            }
            Slot0.apply {
                pidGains = PID_GAINS
            }
            MotionMagic.apply {
                MotionMagicCruiseVelocity = PROFILE_VELOCITY.inRotationsPerSecond()
                MotionMagicAcceleration = PROFILE_ACCELERATION.inRotationsPerSecondPerSecond()
                MotionMagicJerk = PROFILE_JERK
            }
            Feedback.apply {
                FeedbackRemoteSensorID = CTREDeviceId.TurretTurningEncoder.num
                FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
                SensorToMechanismRatio = SENSOR_TO_MECHANISM_GEAR_RATIO
            }
            CurrentLimits.apply {
                SupplyCurrentLowerLimit = 30.0
                SupplyCurrentLimit = 40.0
                SupplyCurrentLowerTime = 1.0
            }

        })
    }

    private val encoder =  CANcoder(CTREDeviceId.TurretTurningEncoder).apply {
        configurator.apply(CANcoderConfiguration().apply {
            MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive
        })
        setPosition(0.degrees)
    }

    private val positionSignal = motor.position
    private val velocitySignal = motor.velocity
    private val currentSignal = motor.supplyCurrent
    private val temperatureSignal = motor.deviceTemp
    private val positionControl: MotionMagicVoltage = MotionMagicVoltage(0.0).apply {
        UpdateFreqHz = 0.0
    }
    private var setPoint = 0.0.degrees

    init {
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            positionSignal,
            currentSignal,
            velocitySignal,
            temperatureSignal
        )
        motor.optimizeBusUtilization()
    }


    override fun turnToAngle(angle: Angle) {
        // This is fine to do because the PID acts on a linear system, meaning that there will be no wrapping through the bad angles.
        val clampedAngle = angle.coerceIn(TURRET_MIN_ANGLE..TURRET_MAX_ANGLE)

        setPoint = clampedAngle
        Logger.recordOutput("Shooter/Turret/Setpoint", setPoint)

        motor.setControl(positionControl.withPosition(setPoint))
    }

    override fun setVoltage(voltage: Voltage) {
        assert(voltage in 0.volts..12.volts)
        motor.setVoltage(voltage.inVolts())
    }

    override fun updateInputs(inputs: TurretInputs) {
        inputs.angle = positionSignal.value
        inputs.motorCurrent = currentSignal.value
        inputs.motorVelocity = velocitySignal.value
        inputs.motorTemperature = temperatureSignal.value
        inputs.brakeMode = brakeMode
        inputs.setPoint = setPoint
    }

    override fun zeroEncoder() {
        encoder.setPosition(0.degrees)
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

    companion object Constants{
        val TURRET_MAX_ANGLE = 90.degrees
        val TURRET_MIN_ANGLE = (-100).degrees

        private val PID_GAINS = PIDGains(80.0, 0.0, 1.2)
        private const val SENSOR_TO_MECHANISM_GEAR_RATIO = 155.0 / 30.0
        private const val ROTOR_TO_SENSOR_GEAR_RATIO = 1.0
        private const val MAGNET_OFFSET = -0.19140625
        private val PROFILE_ACCELERATION = 10.0.rotationsPerSecondPerSecond
        private const val PROFILE_JERK = 0.0
        private val PROFILE_VELOCITY = 14.0.rotationsPerSecond
    }
}

class TurretIOSim: TurretIO {
    private val motor = DCMotor.getKrakenX60(1)
    private val system = LinearSystemId.createDCMotorSystem(motor, 1.0,1.0)
    private val sim = DCMotorSim(system, motor)
    private var brakeMode = false

    override fun turnToAngle(angle: Angle) {
        sim.setAngle(angle.inRadians())
    }

    override fun setVoltage(voltage: Voltage) {
        sim.inputVoltage = voltage.inVolts()
    }

    override fun updateInputs(inputs: TurretInputs) {
        inputs.angle = sim.angularPosition
        inputs.brakeMode = brakeMode
        inputs.motorCurrent = motor.getCurrent(sim.torqueNewtonMeters).amps
        Logger.recordOutput("Shooter/Turret/SimAngle", sim.angularPosition)
    }

    override fun setBrakeMode(enabled: Boolean) {
        brakeMode = enabled
    }

    override fun zeroEncoder() {
        TODO()
    }

}
