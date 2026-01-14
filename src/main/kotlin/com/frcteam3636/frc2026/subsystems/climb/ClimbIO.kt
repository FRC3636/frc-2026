package com.frcteam3636.frc2026.subsystems.climb

import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.amps
import com.frcteam3636.frc2026.utils.math.meters
import com.frcteam3636.frc2026.utils.math.metersPerSecond
import com.frcteam3636.frc2026.utils.math.toLinear
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged

@Logged
open class ClimbInputs {
    var height = 0.meters
    var current = 0.amps
    var velocity = 0.metersPerSecond
}

interface ClimbIO {
    fun setSpeed(percent: Double)
    fun setVoltage(volts: Voltage)
    fun goToHeight(height: Distance)
    fun updateInputs(inputs: ClimbInputs)
}

class ClimbIOReal : ClimbIO {
    private val motor = TalonFX(CTREDeviceId.ClimbMotor)

    init {
        // TODO: Configure motor stuff here.
    }

    override fun setSpeed(percent: Double) {
        TODO("Not yet implemented")
    }

    override fun setVoltage(volts: Voltage) {
        TODO("Not yet implemented")
    }

    override fun goToHeight(height: Distance) {
        Logger.recordOutput("Climb/Height Setpoint", height)
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: ClimbInputs) {
        inputs.height = motor.getPosition(false).value.toLinear(SPOOL_RADIUS)
        inputs.velocity = motor.getVelocity(false).value.toLinear(SPOOL_RADIUS)
        inputs.current = motor.getSupplyCurrent(false).value
    }
}

