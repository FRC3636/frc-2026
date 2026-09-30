package com.frcteam3636.frc2026.subsystems.intake

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.frcteam3636.frc2026.CANcoder
import com.frcteam3636.frc2026.CTREDeviceId
import com.frcteam3636.frc2026.TalonFX
import com.frcteam3636.frc2026.utils.math.PIDGains
import com.frcteam3636.frc2026.utils.math.degrees
import com.frcteam3636.frc2026.utils.math.degreesPerSecond
import com.frcteam3636.frc2026.utils.math.degreesPerSecondPerSecond
import com.frcteam3636.frc2026.utils.math.inRotationsPerSecond
import com.frcteam3636.frc2026.utils.math.inRotationsPerSecondPerSecond
import com.frcteam3636.frc2026.utils.math.inVolts
import com.frcteam3636.frc2026.utils.math.pidGains
import com.frcteam3636.frc2026.utils.math.rotationsPerSecond
import com.frcteam3636.frc2026.utils.math.volts
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged
import kotlin.apply

/*
 * 6. The IO
 *
 * Welcome to the IO file! There is a lot of complicated-looking code here, but hopefully you should understand it well
 * by the time you've read through.
 */

/*
 * The first class here is IntakeInputs. It corresponds to the input field in the Intake subsystem in the previous file.
 * Notice that the type of the inputs field was *LoggedIntakeInputs*. This is because of the Logged annotation below
 * ("@Logged"). That is part of the AdvantageKit logging system, which generates a new class with "Logged" before the
 * original name. This new class is a value that can be logged.
 */
@Logged
open class IntakeInputs {
    var intakeMotorVelocity = 0.rotationsPerSecond
    var intakeMotorCurrent = Amps.zero()!!
    var pivotAngle = 0.degrees
    var pivotSetpoint = 0.degrees
    var intakePivotMotorCurrent = Amps.zero()!!
    var intakePivotMotorVoltage = Volts.zero()!!
    var intakePivotMotorSupplyVoltage = 0.0.volts
    var rightPivotMotorCurrent = Amps.zero()!!
}

/*
 * In Kotlin, an interface is similar but quite different from a class. Other classes can derive from it, and those
 * classes must have all the methods described below. However, an interface can't be directly created, only its
 * implementations. You'll see an example of an implementation below, in the IntakeIOReal class. This is the
 * only implementation we'll cover in this guide, and the only one you really need to worry about.
 */
interface IntakeIO {
    // All these functions have to be implemented for any Intake IO.
    fun setSpeed(percent: Double)
    fun setPivotVoltage(voltage: Voltage)
    fun setPivotSpeed(pivot: Double)
    fun setVoltage(voltage: Voltage)
    fun setPivotAngle(angle: Angle)
    fun updateInputs(inputs: IntakeInputs)
    fun zeroEncoder()
}

/*
 * Whereas the high-level Subsystem dealt with commands, the IO deals with the motor objects themselves. The
 * IntakeIO is responsible for setting the speed/voltage of motors, and moving to a setpoint. It also has a function
 * for updating the inputs which we saw at the top of the file.
 */
class IntakeIOReal : IntakeIO {
    /*
     * Here are all the intake Constants (values that never change when the program is run). These are hardcoded values,
     * which are usually obtained by tuning the robot.
     *
     * You don't need to know what a companion object is for now, just think as it as a folder where we can store all
     * the constants.
     */
    companion object Constants {
        val PID_GAINS = PIDGains(120.0, 0.0, 0.5)
        val PROFILE_CRUISE_VELOCITY = 320.0.degreesPerSecond
        val PROFILE_ACCELERATION = 400.degreesPerSecondPerSecond
        val PROFILE_JERK = 20.0
        val ENCODER_TO_PIVOT_GEAR_RATIO = 32.0 / 12.0
        val MOTOR_TO_ENCODER_GEAR_RATIO = 4.0
        val DISCONTINUITY_POINT = 0.999
        val MAGNET_OFFSET = 0.130859375
        val GRAVITY_COMPENSATION_GAIN = 1.0

        val PIVOT_MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive
        val WHEEL_MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive
    }

    /*
     * Now we create our first motor! This is the motor that spins the intake, and we'll go through how it works line
     * by line.
     *
     * The motor is a TalonFX object. TalonFX is the underlying software that controls the Kraken motors, which are the
     * ones we have on our intake. We pass to it the ID of the motor (recall that we defined this in CAN.kt).
     *
     * Next, we use the apply function. This is a function that can be called on every Kotlin class, and IntelliJ lets
     * us know that it's special by italicizing it and giving it a special color. The apply function lets us write code
     * that will run as if it's inside the class we are calling it on.
     */
    private val intakeMotor = TalonFX(CTREDeviceId.IntakeMotor).apply {
        /*
         * "configurator" is a member variable of the TalonFX object. Because we're in an apply block, we can use it.
         * When we call apply on configurator, this is actually a different apply function than the one we used before
         * (notice that it is not highlighted by IntelliJ). This apply function lets us add some configuration settings
         * to the motor.
         *
         * In this case, we are setting the motor inversion to be the same as the constant WHEEL_MOTOR_DIRECTION, which
         * was defined in the Constants object above.
         */
        configurator.apply(TalonFXConfiguration().apply { MotorOutput.Inverted = WHEEL_MOTOR_DIRECTION })
    }

    /*
     * This motor seems a lot more complex, but really it's the same format with more configuration options. This is the
     * pivot motor, so instead of just spinning it has to accurately move to a setpoint. That's a capability that
     * requires a lot more configuration.
     */
    private val intakePivotMotor = TalonFX(CTREDeviceId.IntakePivotMotor).apply {
        // We apply a configuration.
        configurator.apply(TalonFXConfiguration().apply {
            /*
             * We are working on a section of the book that will help you understand what a PID is. But for now, just
             * know that it specifies how fast the motor should move to get to a certain point.
             */
            Slot0.apply {
                pidGains = PID_GAINS
            }
            /*
             * MotionMagic works on top of the PID, and is unique to the TalonFX controller. It makes the movement
             * smoother and more efficient.
             */
            MotionMagic.apply {
                MotionMagicCruiseVelocity = PROFILE_CRUISE_VELOCITY.inRotationsPerSecond()
                MotionMagicAcceleration = PROFILE_ACCELERATION.inRotationsPerSecondPerSecond()
                // Just like velocity is change in position, and acceleration is change in velocity, jerk is change in
                // acceleration. Subsequently, there is snap/jounce, crackle, and pop.
                MotionMagicJerk = PROFILE_JERK
            }
            //To know how close it is to a point, the motor needs to know where it actually is. To do this we provide ...
            Feedback.apply {
                // The type of sensor. A CANcoder is an encoder on the CAN.
                FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
                // The ID of the encoder, specified as well in the CTREDeviceId enum.
                FeedbackRemoteSensorID = CTREDeviceId.IntakePivotEncoder.num
                // The ratio between the rotation of the encoder and the physical intake.
                SensorToMechanismRatio = ENCODER_TO_PIVOT_GEAR_RATIO
                // The ratio between the rotation of the encoder and the rotation of the motor.
                RotorToSensorRatio = MOTOR_TO_ENCODER_GEAR_RATIO
            }
            // And we specify some other things.
            MotorOutput.apply {
                // When the motor isn't told what to do, it should stop.
                NeutralMode = NeutralModeValue.Brake
                Inverted = PIVOT_MOTOR_DIRECTION
            }
        })
    }

    /*
     * We also create an encoder object for our own uses, setting the direction to be Clockwise, and zeroing it when
     * it's created.
     */
    private val encoder =  CANcoder(CTREDeviceId.IntakePivotEncoder).apply {
        configurator.apply(CANcoderConfiguration().apply {
            MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive
        })
        setPosition(0.degrees)
    }

    /**
     * Sets the speed of the intake motor.
     */
    override fun setSpeed(percent: Double) {
        intakeMotor.set(percent)
    }

    /**
     * Sets the voltage of the intake motor.
     */
    override fun setVoltage(voltage: Voltage) {
        intakeMotor.setVoltage(voltage.inVolts())
    }

    /*
     * Both of the next few functions aren't currently used.
     */
    override fun setPivotSpeed(pivot: Double) {
        intakePivotMotor.set(pivot)
    }

    override fun setPivotVoltage(voltage: Voltage) {
        Logger.recordOutput("Intake/Pivot Attempted Voltage", voltage)
        intakePivotMotor.setVoltage(voltage.inVolts())
    }

    /*
     * The lower level function for zeroing the encoder.
     */
    override fun zeroEncoder() {
        encoder.setPosition(0.degrees)
    }

    /*
     * The positionControl (a MotionMagicVoltage object) is where we specify the position we want the pivot to go to.
     * We only need to set the position once for the motor to continuously go to that point.
     */
    private val positionControl = MotionMagicVoltage(0.0)

    /**
     * Applies the positionControl to the intakePivotMotor at the specified angle.
     */
    override fun setPivotAngle(angle: Angle) {
        Logger.recordOutput("Intake/Pivot Setpoint", angle)
        intakePivotMotor.setControl(
            positionControl.withPosition(angle)
        )
    }

    var setpoint = 0.degrees

    /**
     * Updates all the inputs. This function is called from the higher-level Subsystem.
     */
    override fun updateInputs(inputs: IntakeInputs) {
        inputs.intakeMotorVelocity = intakeMotor.velocity.value
        inputs.intakeMotorCurrent = intakeMotor.supplyCurrent.value

        inputs.intakePivotMotorCurrent = intakePivotMotor.supplyCurrent.value
        inputs.intakePivotMotorVoltage = intakePivotMotor.motorVoltage.value
        inputs.intakePivotMotorSupplyVoltage = intakeMotor.supplyVoltage.value
        inputs.pivotAngle = intakePivotMotor.position.value
        inputs.pivotSetpoint = setpoint
    }
}

/*
 * Congratulations! We have gone through every section of the robot code you will need to know to write your own
 * subsystem, or even your own Robot!
 *
 * TODO: Introduce some kind of final project such as writing code for Iroh.
 */