package com.frcteam3636.frc2026

import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.utils.math.seconds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object Dashboard {
    var hubActive = true
    var shiftTimeRemaining = 0.0


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
    fun updatePhase(){
        val gameMessage = DriverStation.getGameSpecificMessage()

        if(gameMessage.isEmpty()) return

        val activeAllianceHub = when (gameMessage.first()) {
            'R'-> DriverStation.Alliance.Red
            'B' -> DriverStation.Alliance.Blue
            else -> "red"
        }

        val wonHub = (activeAllianceHub == DriverStation.getAlliance().get())
        val matchTime = DriverStation.getMatchTime()

        if (Shifts.Phase1.hasElapsed()) {
            hubActive = true
            shiftTimeRemaining = matchTime - Shifts.Phase1.endTime
        } else if (Shifts.Phase2.hasElapsed()) {
            hubActive = wonHub
            shiftTimeRemaining = matchTime - Shifts.Phase2.endTime
        } else if(Shifts.Phase3.hasElapsed()) {
            hubActive = !wonHub
            shiftTimeRemaining = matchTime - Shifts.Phase3.endTime
        } else if(Shifts.Phase4.hasElapsed()) {
            hubActive = wonHub
            shiftTimeRemaining = matchTime - Shifts.Phase4.endTime
        } else if (Shifts.Phase5.hasElapsed()) {
            hubActive = !wonHub
            shiftTimeRemaining = matchTime - Shifts.Phase5.endTime
        } else {
            hubActive = true
        }

    }
    fun initialize(){
        SmartDashboard.putData(autoChooser)
        SmartDashboard.putData(Drivetrain.field)
        SmartDashboard.putBoolean("Hub Active", hubActive)
        SmartDashboard.putNumber("Time Remaining", shiftTimeRemaining)
    }

    fun periodic(){
        updatePhase()
        SmartDashboard.updateValues();

    }
}

enum class AutoModes(val autoName: String, val developerAuto: Boolean = false) {
    None("None"),
    Climb("Climb"),
    Lebron("Lebron"),
    LebronLeft("LebronLeft"),
}

enum class Shifts(val endTime: Double ){
    Phase1(130.0),
    Phase2(105.0),
    Phase3(80.0),
    Phase4(55.0),
    Phase5(30.0);

    fun hasElapsed() : Boolean{
        return this.endTime < DriverStation.getMatchTime()
    }
}