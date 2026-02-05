package com.frcteam3636.frc2026

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object Dashboard {
    val autoChooser = SendableChooser<AutoModes>().apply {
        for (autoMode in AutoModes.entries) {
            if (autoMode == AutoModes.None)
                setDefaultOption(autoMode.autoName, autoMode)
            else if (Preferences.getBoolean(
                    "developerMode",
                    true
                ) && autoMode.developerAuto && !DriverStation.isFMSAttached()
            ) {
                addOption(autoMode.autoName, autoMode)
            } else if (!autoMode.developerAuto)
                addOption(autoMode.autoName, autoMode)
        }
    }

    fun initialize() {
        SmartDashboard.putData(autoChooser)
    }
}

enum class AutoModes(val autoName: String, val developerAuto: Boolean = false) {
    None("None"),
    TestAuto("Test Auto")
}