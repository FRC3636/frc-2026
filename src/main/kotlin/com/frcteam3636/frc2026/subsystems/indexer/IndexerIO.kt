package com.frcteam3636.frc2026.subsystems.indexer

import com.frcteam3636.frc2026.utils.math.amps
import com.frcteam3636.frc2026.utils.math.rotations
import com.frcteam3636.frc2026.utils.math.rotationsPerSecond
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

@Logged
open class IndexerInputs {
    var position = 0.rotations
    var velocity = 0.rotationsPerSecond
    var current = 0.amps
}

interface IndexerIO {
    fun setSpeed(percent: Double)
    fun setVoltage(volts: Voltage)
    fun updateInputs(inputs: IndexerInputs)
}

class IndexerIOReal : IndexerIO {
    override fun setSpeed(percent: Double) {
        TODO("Not yet implemented")
    }

    override fun setVoltage(volts: Voltage) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: IndexerInputs) {
        TODO("Not yet implemented")
    }
}