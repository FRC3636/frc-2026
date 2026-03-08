package com.frcteam3636.frc2026.subsystems.flywheel

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2026.CTREDeviceId
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.simulation.FlywheelSim

@Logged
open class FlywheelInputs {
    var motorVolts = 0.volts
    var angularVelocity = RPM.zero()!!
//    var linearVelocity = MetersPerSecond.zero()!!
    var targetAngularVelocity = RPM.zero()!!
    var angle = Rotations.zero()!!
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
            }

            Slot0.apply{
                pidGains = PID_GAINS
            }
        })
    }

    private var ffController = SimpleMotorFeedforward(FEED_FORWARD_GAINS)

    private var targetVelocity: AngularVelocity = 0.0.rpm

    override fun updateInputs(inputs: FlywheelInputs) {
        inputs.motorVolts = motor.motorVoltage.value
        inputs.angularVelocity = motor.velocity.value
//        inputs.linearVelocity = motor.velocity.value.toLinear(Constants.FLYWHEEL_RADIUS)
        inputs.angle = motor.position.value
        inputs.targetAngularVelocity = targetVelocity
    }

    override fun setVoltage(volts: Voltage) {
        motor.setVoltage(volts.inVolts())
    }

    override fun setSpeed(percentage : Double) {
        motor.set(percentage)
    }

    override fun setVelocity(velocity: AngularVelocity){
        assert(velocity in 0.rpm..6000.rpm)
        targetVelocity = velocity
        motor.setVoltage(ffController.calculate(velocity.inRPM()))
    }

    companion object Constants{
        val PID_GAINS = PIDGains(0.001,0.0,0.0)//0.0005)
        val FEED_FORWARD_GAINS = MotorFFGains(0.24428, 0.0021606693158032443, 0.03)
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