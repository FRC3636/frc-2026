package com.frcteam3636.frc2026.subsystems.indexer

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.amps
import com.frcteam3636.frc2026.utils.math.inAmps
import com.frcteam3636.frc2026.utils.math.inVolts
import com.frcteam3636.frc2026.utils.math.rotationsPerSecond
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

@Logged
open class IndexerInputs {
    var motorVelocity = 0.rotationsPerSecond
    var motorCurrent = 0.amps
}

interface IndexerIO {
    fun setSpeed(percent: Double)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: IndexerInputs)
}

class IndexerIOReal : IndexerIO {
    private var indexerMotor = TalonFX(CTREDeviceId.IndexerMotor).apply {
        configurator.apply(
            TalonFXConfiguration().apply {
                MotorOutput.apply {
                    NeutralMode = NeutralModeValue.Coast
                    Inverted = InvertedValue.CounterClockwise_Positive
                }

                CurrentLimits.apply {
                    SupplyCurrentLimit = MOTOR_CURRENT_LIMIT.inAmps()
                    SupplyCurrentLimitEnable = true
                }
            }
        )
    }

    init {
        indexerMotor.optimizeBusUtilization()
    }

    override fun setSpeed(percent: Double) {
        assert(percent in -1.0..1.0)
        indexerMotor.set(percent)
    }

    override fun setVoltage(voltage: Voltage) {
        assert(voltage.inVolts() in -12.0..12.0)
        indexerMotor.setVoltage(voltage.inVolts())
    }

    override fun updateInputs(inputs: IndexerInputs) {
        inputs.motorVelocity = indexerMotor.velocity.value
        inputs.motorCurrent = indexerMotor.supplyCurrent.value
    }

    internal companion object Constants {
        private val MOTOR_CURRENT_LIMIT = 35.amps
    }

}