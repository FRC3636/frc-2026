package com.frcteam3636.frc2026.subsystems.flywheel

import com.frcteam3636.frc2026.Robot
import edu.wpi.first.wpilibj2.command.Subsystem

object Flywheel: Subsystem {

    private val io: FlywheelIO = when (Robot.model) {
        Robot.Model.SIMULATION -> TODO()
        Robot.Model.COMPETITION -> FlywheelIOReal()
    }

}