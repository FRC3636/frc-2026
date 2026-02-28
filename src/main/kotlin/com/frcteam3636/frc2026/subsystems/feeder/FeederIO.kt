package com.frcteam3636.frc2026.subsystems.feeder

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.amps
import com.frcteam3636.frc2026.utils.math.rotationsPerSecond
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.LinearSystemSim
import org.team9432.annotation.Logged

@Logged
open class FeederInputs {
    var feederVelocity = 0.0.rotationsPerSecond
    var feederCurrent = 0.0.amps
}

interface FeederIO {
    fun setSpeed(percent: Double)
    fun updateInputs(inputs: FeederInputs)
}

class FeederIOReal : FeederIO {
    private val feederMotor = TalonFX(CTREDeviceId.FeederMotor)
    private val feederMotorConfig = TalonFXConfiguration()

    init {
        feederMotorConfig.apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
        }
        feederMotor.configurator.apply(feederMotorConfig)

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            feederMotor.position,
            feederMotor.velocity,
            feederMotor.supplyCurrent
        )
        feederMotor.optimizeBusUtilization()
    }


    override fun setSpeed(percent: Double) {
        assert(percent in -1.0..1.0)
        feederMotor.set(percent)
    }

    override fun updateInputs(inputs: FeederInputs) {
        inputs.feederVelocity = feederMotor.velocity.value
        inputs.feederCurrent = feederMotor.supplyCurrent.value
    }
}

class FeederIOSim : FeederIO {
//    private val motor = DCMotor.getKrakenX60(1)

    override fun setSpeed(percent: Double) {
//        motor = percent * 12.0
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: FeederInputs) {
//        inputs.feederVelocity = feederMotor.angularVelocity
//        inputs.feederCurrent = feederMotor.currentDrawAmps.amps
    }
}
