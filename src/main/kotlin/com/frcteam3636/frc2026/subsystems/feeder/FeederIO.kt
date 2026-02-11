package com.frcteam3636.frc2026.subsystems.feeder

import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.amps
import com.frcteam3636.frc2026.utils.math.rotationsPerSecond
import edu.wpi.first.epilogue.Logged

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

    override fun setSpeed(percent: Double) {
        assert(percent in -1.0..1.0)
        feederMotor.set(percent)
    }

    override fun updateInputs(inputs: FeederInputs) {
        inputs.feederVelocity = feederMotor.velocity.value
        inputs.feederCurrent = feederMotor.supplyCurrent.value
    }
}

//class FeederIOSim : FeederIO {
//    private val feederMotorSystem = LinearSystemId.createDCMotorSystem()
//    private val feederMotor = DCMotorSim(feederMotorSystem, )
//
//    override fun setSpeed(percent: Double) {
//        feederMotor.inputVoltage = percent * 12.0
//    }
//
//    override fun updateInputs(inputs: FeederInputs) {
//        inputs.feederVelocity = feederMotor.angularVelocity
//        inputs.feederCurrent = feederMotor.currentDrawAmps.amps
//    }
//}