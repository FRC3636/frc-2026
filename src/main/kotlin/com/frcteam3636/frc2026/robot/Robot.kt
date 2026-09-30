package com.frcteam3636.frc2026.robot

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.StatusSignalCollection
import com.frcteam3636.frc2026.AutoModes
import com.frcteam3636.frc2026.Dashboard
import com.frcteam3636.frc2026.Diagnostics
import com.frcteam3636.frc2026.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2026.subsystems.feeder.Feeder
import com.frcteam3636.frc2026.subsystems.shooter.flywheel.Flywheel
import com.frcteam3636.frc2026.subsystems.shooter.hood.Hood
import com.frcteam3636.frc2026.subsystems.indexer.Indexer
import com.frcteam3636.frc2026.subsystems.intake.Intake
import com.frcteam3636.frc2026.subsystems.shooter.turret.Turret
import com.frcteam3636.frc2026.subsystems.climber.Climber
import com.frcteam3636.frc2026.subsystems.drivetrain.Climb
import com.frcteam3636.frc2026.subsystems.drivetrain.Lebron
import com.frcteam3636.version.BUILD_DATE
import com.frcteam3636.version.DIRTY
import com.frcteam3636.version.GIT_BRANCH
import com.frcteam3636.version.GIT_SHA
import com.revrobotics.util.StatusLogger
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj.Threads
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import java.util.concurrent.locks.ReentrantLock
import kotlin.io.path.Path
import kotlin.io.path.exists
import kotlin.jvm.optionals.getOrNull

/*
 * 3. Robot
 *
 * Welcome to the Robot class! Here, we can see that Robot inherits a
 * LoggedRobot class. LoggedRobot is not a part of WPILib, but rather part of
 * the larger AdvantageKit system that we use for logging.
 *
 * You can ignore the comment below.
 */

/**
 *
 * The VM is configured to automatically run this object (which basically functions as a singleton
 * class), and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. This is written as an object rather than a class since there should only ever be a
 * single instance, and it cannot take any constructor arguments. This makes it a natural fit to be
 * an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also
 * update the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when
 * renaming the object or package, it will get changed everywhere.)
 */
object Robot : LoggedRobot() {
    /** The current autonomous command, if one is running, otherwise null. **/
    private var autoCommand: Command? = null
    /** Used by the auto selector. If the last selected auto is different from
     * the current selected auto, the auto will be changed.
     **/
    private var lastSelectedAuto = AutoModes.None

    /*
     * We'll learn more about how CAN buses work in the future.
     */

    private val rioCANBus = CANBus("rio")
    private val canivore = CANBus("*")

    /*
     * Same for these.
     */

    val statusSignals = StatusSignalCollection()
    val odometryLock = ReentrantLock()

    /**
     * A model of robot, depending on where we're deployed to.
     * Simulation is rairly used, it's for when we are testing the robot in a fully virtual environment.
     * Competition is the most common, it's used whenever the robot is running physically. It doesn't have to be a
     * real competition for the robot to be in competition mode.
     **/
    enum class Model {
        SIMULATION, COMPETITION
    }

    /** The model of this robot. */
    val model: Model = if (isSimulation()) {
        Model.SIMULATION
    } else {
        when (val key = Preferences.getString("Model", "competition")) {
            "competition" -> Model.COMPETITION
            else -> throw AssertionError("Invalid model found in preferences: $key")
        }
    }

    /**
     * This function runs when the robot starts up, or after code is pushed. It does not run when the robot is enabled.
     */
    override fun robotInit() {
        // Report the use of the Kotlin Language for "FRC Usage Report" statistics
        HAL.report(
            FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version
        )

        SignalLogger.enableAutoLogging(false)
        StatusLogger.disableAutoLogging()

        // Joysticks are likely to be missing in simulation, which usually isn't a problem.
        DriverStation.silenceJoystickConnectionWarning(model != Model.COMPETITION)

        configureAdvantageKit()
        configureSubsystems()
        configureAutos()
        configureBindings()
        configureDashboard()
        Dashboard.initialize()

        statusSignals.addSignals(*Drivetrain.signals)

        // BIG WARNING BIG WARNING BIG WARNING
        // hi there. if you're a team looking at copying some code (which we are flattered)
        // (hi 6696)
        // then please do not copy this unless you know what it does.
        // if you do know what it does then please ensure your loop times are 10ms max.
        // if you are above 10ms or are experiencing loop overruns, this is not the magic fix to your loop times.
        // sorry.
        // we would recommend profiling your code with VisualVM first.
        // this code will improve your loop times yes, but it will starve vendor threads
        // and you will start seeing random things like CAN errors appear.
        // thanks, 3636
        Threads.setCurrentThreadPriority(true, 1)
    }

    /*
     * No need to understand how exactly this is working. It's just setting up the logging system.
     */
    /** Start logging or pull replay logs from a file */
    private fun configureAdvantageKit() {
        Logger.recordMetadata("Git SHA", GIT_SHA)
        Logger.recordMetadata("Build Date", BUILD_DATE)
        @Suppress("SimplifyBooleanWithConstants")
        Logger.recordMetadata("Git Tree Dirty", (DIRTY == 1).toString())
        Logger.recordMetadata("Git Branch", GIT_BRANCH)
        Logger.recordMetadata("Model", model.name)

        if (isReal()) {
            Logger.addDataReceiver(WPILOGWriter()) // Log to a USB stick
            if (!Path("/U").exists()) {
                Alert(
                    "The Log USB drive is not connected to the roboRIO, so a match replay will not be saved. (If convenient, insert it and restart robot code.)",
                    Alert.AlertType.kInfo
                )
                    .set(true)
            }
            Logger.addDataReceiver(NT4Publisher()) // Publish data to NetworkTables
            // Enables power distribution logging
            PowerDistribution(
                1, PowerDistribution.ModuleType.kRev
            )
        } else {
            val logPath = try {
                // Pull the replay log from AdvantageScope (or prompt the user)
                LogFileUtil.findReplayLog()
            } catch (_: NoSuchElementException) {
                null
            }

            if (logPath == null) {
                // No replay log, so perform physics simulation
                Logger.addDataReceiver(NT4Publisher())
            } else {
                // Replay log exists, so replay data
                setUseTiming(false) // Run as fast as possible
                Logger.setReplaySource(WPILOGReader(logPath)) // Read replay log
                Logger.addDataReceiver(
                    WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))
                ) // Save outputs to a new log
            }
        }
        Logger.start() // Start logging! No more data receivers, replay sources, or metadata values may be added.
    }

    /**
     * Start robot subsystems so that their periodic tasks are run. If a subsystem isn't registered here, it won't be
     * setup properly, and probably won't work. So if a subsystem's periodic function isn't running, you should first
     * check here! */
    private fun configureSubsystems() {
        Drivetrain.register()
        Feeder.register()
        Flywheel.register()
        Hood.register()
        Indexer.register()
        Intake.register()
        Turret.register()
//        Climber.register()
    }


    /** Expose commands for autonomous routines to use and display an auto picker in Shuffleboard. */
    private fun configureAutos() {}

    /**
     * Runs every ~20ms when the robot is disabled. All that is done in this function is the logic for selecting an
     * auto. When a new auto is added, this is one of the places you have to add some code. A comment in this function
     * shows exactly how to do that.
     */
    override fun disabledPeriodic() {
        val selectedAuto = Dashboard.autoChooser.selected
        val alliance = DriverStation.getAlliance().getOrNull()
        val flipH = alliance == DriverStation.Alliance.Red
        val flipToSide = { side: Drivetrain.FieldSide -> if (alliance == DriverStation.Alliance.Blue) {
            side == Drivetrain.FieldSide.Left
        } else {
            side == Drivetrain.FieldSide.Right
        }}
        if (lastSelectedAuto != selectedAuto) {
            lastSelectedAuto = selectedAuto
            autoCommand = when (selectedAuto) {
                AutoModes.None -> Commands.none()
                AutoModes.Climb -> Climb.getPath(flipH = flipH, flipV = false)
                AutoModes.Lebron -> Lebron.getPath(flipH = flipH, flipV = flipToSide(Drivetrain.FieldSide.Right))
                AutoModes.LebronLeft -> Lebron.getPath(flipH = flipH, flipV = flipToSide(Drivetrain.FieldSide.Left))
                //        ^^ name of auto ^^     always the same ^^    if left or right specific ^^
            }
        }
    }

    /** Add data to the driver station dashboard. */
    private fun configureDashboard() {}

    private fun reportDiagnostics() {
        Diagnostics.periodic()
        Diagnostics.report(rioCANBus)
        Diagnostics.report(canivore)
        Diagnostics.reportDSPeripheral(joystickLeft.hid, isController = false)
        Diagnostics.reportDSPeripheral(joystickRight.hid, isController = false)
        Diagnostics.reportDSPeripheral(controller.hid, isController = true)
    }

    /**
     * Runs every ~20ms whenever the Robot is on (even if it's disabled). No code that could possibly move the robot
     * should ever go in here!
     */
    override fun robotPeriodic() {
        statusSignals.refreshAll()

        reportDiagnostics()
        Diagnostics.send()

        CommandScheduler.getInstance().run()
    }

    /**
     * Runs once when the autonomous is started.
     */
    override fun autonomousInit() {
//        val selectedAuto = Dashboard.autoChooser.selected
        if (!RobotState.beforeFirstEnable)
            RobotState.beforeFirstEnable = false
        CommandScheduler.getInstance().schedule(autoCommand)
    }

    override fun autonomousExit() {
        autoCommand?.cancel()
        Drivetrain.stop()
    }

    /**
     * Runs once when the teleoperated phase is started (this is the normal way to enable the robot).
     */
    override fun teleopInit() {
        if (!RobotState.beforeFirstEnable)
            RobotState.beforeFirstEnable = false
    }

    override fun testInit() {
    }

    override fun testExit() {
    }

    override fun simulationInit() {
        SimulatedArena.getInstance().resetFieldForAuto()
    }

    override fun simulationPeriodic() {
//        SimulatedArena.getInstance().simulationPeriodic()
//        Drivetrain.fuelPoses = SimulatedArena.getInstance()
//            .getGamePiecesArrayByType("Fuel")
//        Logger.recordOutput("FieldSimulation/FuelPositions", *fuelPoses)
//        Intake.periodic()
    }
}

/*
 * That's the Robot! It's a decent amount of code, but in itself doesn't do that much. Before we can see how exactly to
 * build a subsystem, let's look at how subsystems communicate with the physical motors on the robot. All of that is
 * specified in the CAN.kt file (not the one in the utils folder!).
 */