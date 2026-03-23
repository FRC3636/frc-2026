package com.frcteam3636.frc2026.subsystems.shooter.flywheel

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2026.CTREDeviceId
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.*
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import java.util.logging.Logger

@Logged
open class FlywheelInputs {
    var motorVolts = 0.volts
    var angularVelocity = RPM.zero()!!
//    var linearVelocity = MetersPerSecond.zero()!!
    var targetAngularVelocity = RPM.zero()!!
    var angle = Rotations.zero()!!
    var current = Amps.zero()!!
}

interface FlywheelIO {
    fun updateInputs(inputs: FlywheelInputs)
    fun setVoltage(volts: Voltage)
    fun setSpeed(percentage: Double)
    fun setVelocity(velocity: AngularVelocity)
}

class FlywheelIOReal : FlywheelIO {
    private val motor = TalonFX(CTREDeviceId.FlywheelMotor).apply {
        configurator.apply(TalonFXConfiguration().apply{
            MotorOutput.apply {
                Inverted = InvertedValue.Clockwise_Positive
                NeutralMode = NeutralModeValue.Coast
            }

            CurrentLimits.apply {
                SupplyCurrentLimit = 40.0
                CurrentLimits = CurrentLimitsConfigs()
            }

            Slot0.apply{
                pidGains = PID_GAINS
                motorFFGains = FEED_FORWARD_GAINS
            }
        })
    }

    private val ffController = SimpleMotorFeedforward(FEED_FORWARD_GAINS)
    private val pidController = PIDController(PID_GAINS)

    private var targetVelocity: AngularVelocity = 0.0.rpm
    // mitigate noise in flywheel data
    private val velocityFilter = MedianFilter(10)

    override fun updateInputs(inputs: FlywheelInputs) {
        inputs.motorVolts = motor.motorVoltage.value
        inputs.angularVelocity = velocityFilter.calculate(motor.velocity.value.inRPM()).rpm
        inputs.angle = motor.position.value
        inputs.targetAngularVelocity = targetVelocity
        inputs.current = motor.supplyCurrent.value
    }

    override fun setVoltage(volts: Voltage) {
        motor.setVoltage(volts.inVolts())
    }

    override fun setSpeed(percentage : Double) {
        motor.set(percentage)
    }

    override fun setVelocity(velocity: AngularVelocity){
        targetVelocity = velocity.inRPM().coerceIn(0.0..6000.0).rpm
//        val output = ffController.calculate(velocity.inRPM()) + pidController.calculate(motor.velocity.value.inRPM(), velocity.inRPM())
//        org.littletonrobotics.junction.Logger.recordOutput("Shooter/Flywheel/controller output", output.volts)
//        motor.setVoltage(output)
        motor.setControl(VelocityVoltage(targetVelocity))
    }

    companion object Constants{
        val PID_GAINS = PIDGains(4E-1,0.0, 6E-3)
        val FEED_FORWARD_GAINS = MotorFFGains(0.24428, 0.0021578280449554683, 0.03)
    }
}

class FlywheelIOSim: FlywheelIO {
    private val motor = DCMotor.getKrakenX60(1)
    private val sim: FlywheelSim = FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            motor,
            1.0,
            5.0
        ),
        motor,
    )

    override fun updateInputs(inputs: FlywheelInputs) {
        inputs.motorVolts = sim.inputVoltage.volts
        inputs.angularVelocity = sim.angularVelocity
//        inputs.linearVelocity = sim.angularVelocity.toLinear(FlywheelIOReal.Constants.FLYWHEEL_RADIUS)
    }

    override fun setVoltage(volts: Voltage) {
        sim.inputVoltage = volts.inVolts()
    }

    override fun setSpeed(percentage: Double) {
        TODO("How would you do this on a simulated motor")
    }

    override fun setVelocity(velocity: AngularVelocity) {
        sim.setAngularVelocity(velocity.inRPM())
    }

}