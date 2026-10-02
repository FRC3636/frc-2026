package com.frcteam3636.frc2026

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object Dashboard {
    val autoChooser = SendableChooser<AutoModes>().apply {
        setDefaultOption(AutoModes.None.autoName, AutoModes.None)
        for (autoMode in AutoModes.entries) {
            if (autoMode == AutoModes.None) continue
            if (DriverStation.isFMSAttached() && autoMode.developerAuto) continue
            if (autoMode.developerAuto && !Preferences.getBoolean("DeveloperMode", false)) continue
            addOption(autoMode.autoName, autoMode)
        }
    }

    fun initialize() {
        SmartDashboard.putData("Auto Chooser", autoChooser)
    }
}

enum class AutoModes(val autoName: String, val developerAuto: Boolean = false) {
    None("None"),
    CenterStartDepotLeftShoot("CenterStartDepotLeftShoot"),
    Lebron("Lebron"),
    LebronLeft("LebronLeft"),
}