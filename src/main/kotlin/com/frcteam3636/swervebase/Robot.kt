package com.frcteam3636.swervebase

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.StatusSignalCollection
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain
import com.frcteam3636.version.BUILD_DATE
import com.frcteam3636.version.DIRTY
import com.frcteam3636.version.GIT_BRANCH
import com.frcteam3636.version.GIT_SHA
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import kotlin.io.path.Path
import kotlin.io.path.exists


/**
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
    private val controller = CommandXboxController(2)
    private val joystickLeft = CommandJoystick(0)
    private val joystickRight = CommandJoystick(1)

    @Suppress("unused")
    private val joystickDev = CommandJoystick(3)

    @Suppress("unused")
    private val controllerDev = CommandXboxController(4)

    private var autoCommand: Command? = null

    private val rioCANBus = CANBus("rio")
    private val canivore = CANBus("*")

    val statusSignals = StatusSignalCollection()

    override fun robotInit() {
        // Report the use of the Kotlin Language for "FRC Usage Report" statistics
        HAL.report(
            tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version
        )

        SignalLogger.enableAutoLogging(false)

        // Joysticks are likely to be missing in simulation, which usually isn't a problem.
        DriverStation.silenceJoystickConnectionWarning(model != Model.COMPETITION)

        configureAdvantageKit()
        configureSubsystems()
        configureAutos()
        configureBindings()
        configureDashboard()

//        Diagnostics.reportLimelightsInBackground(arrayOf("limelight-left", "limelight-right"))

        statusSignals.addSignals(*Drivetrain.signals)
    }

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
            if (model == Model.COMPETITION) {
                PowerDistribution(
                    1, PowerDistribution.ModuleType.kRev
                )
            } else {
                PowerDistribution(
                    1, PowerDistribution.ModuleType.kCTRE
                )
            }
        } else {
            val logPath = try {
                // Pull the replay log from AdvantageScope (or prompt the user)
                LogFileUtil.findReplayLog()
            } catch (_: java.util.NoSuchElementException) {
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

    /** Start robot subsystems so that their periodic tasks are run */
    private fun configureSubsystems() {
        Drivetrain.register()
    }

    /** Expose commands for autonomous routines to use and display an auto picker in Shuffleboard. */
    private fun configureAutos() {
//        NamedCommands.registerCommand(
//            "revAim",
//            Commands.parallel(
//                Shooter.Pivot.followMotionProfile(Shooter.Pivot.Target.AIM),
//                Shooter.Flywheels.rev(580.0, 0.0)
//            )
//        )
    }

    /** Configure which commands each joystick button triggers. */
    private fun configureBindings() {
        Drivetrain.defaultCommand = Drivetrain.driveWithJoysticks(joystickLeft.hid, joystickRight.hid)
        // (The button with the yellow tape on it)
        joystickLeft.button(8).onTrue(Commands.runOnce({
            println("Zeroing gyro.")
            Drivetrain.zeroGyro()
        }).ignoringDisable(true))


        if (Preferences.getBoolean("DeveloperMode", false)) {
            controllerDev.leftBumper().onTrue(Commands.runOnce(SignalLogger::start))
            controllerDev.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop))

            controllerDev.y().whileTrue(Drivetrain.sysIdQuasistaticSpin(SysIdRoutine.Direction.kForward))
            controllerDev.a().whileTrue(Drivetrain.sysIdQuasistaticSpin(SysIdRoutine.Direction.kReverse))
            controllerDev.b().whileTrue(Drivetrain.sysIdDynamicSpin(SysIdRoutine.Direction.kForward))
            controllerDev.x().whileTrue(Drivetrain.sysIdDynamicSpin(SysIdRoutine.Direction.kReverse))

            controllerDev.povUp().whileTrue(Drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
            controllerDev.povDown().whileTrue(Drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
            controllerDev.povRight().whileTrue(Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward))
            controllerDev.povLeft().whileTrue(Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse))

            joystickDev.button(1).whileTrue(Drivetrain.calculateWheelRadius())
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

    override fun robotPeriodic() {
        statusSignals.refreshAll()

        reportDiagnostics()
        Diagnostics.send()

        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {
        if (!RobotState.beforeFirstEnable)
            RobotState.beforeFirstEnable = false
        CommandScheduler.getInstance().schedule(autoCommand)
    }

    override fun autonomousExit() {
        autoCommand?.cancel()
    }

    override fun teleopInit() {
        if (!RobotState.beforeFirstEnable)
            RobotState.beforeFirstEnable = false
    }

    override fun testInit() {
    }

    override fun testExit() {
    }

    /** A model of robot, depending on where we're deployed to. */
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
}
