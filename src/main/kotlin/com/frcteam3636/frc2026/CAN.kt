package com.frcteam3636.frc2026

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX

/*
 * 4. CAN
 *
 * CAN is an acronym that stands for Controller Area Network, although we rarely use the acronym in its non-abbreviated
 * form. Most things are on the canivoreBus, but sometimes things can be on the rioCANBus (for which there is a variable
 * in the Robot class). Every component on a CAN bus has a numerical ID, which is set with another software which is
 * used for configuring many parts of the robot. The enum CTREDeviceID groups the ID and bus and gives them a human-
 * readable name.
 *
 * In the Subsystems, we will use this enum. Now that you've learned about it, we can go on to see how to actually build
 * a Subsystem! Not all the subsystems are fully documented in this tutorial, so make sure to go to the Intake, which is
 * in the subsystems/intake folder. Make sure not to go to IntakeIO first!
 */

private val canivoreBus = CANBus("*")

enum class CTREDeviceId(val num: Int, val bus: CANBus) {

    // Note to electrical:
    // shooter is front, intake is back,
    // left and right are from a bird's eye view

    FrontLeftDrivingMotor(1, canivoreBus),
    BackLeftDrivingMotor(2, canivoreBus),
    BackRightDrivingMotor(3, canivoreBus),
    FrontRightDrivingMotor(4, canivoreBus),

    FrontLeftTurningMotor(5, canivoreBus),
    BackLeftTurningMotor(6, canivoreBus),
    BackRightTurningMotor(7, canivoreBus),
    FrontRightTurningMotor(8, canivoreBus),

    FrontLeftTurningEncoder(9, canivoreBus),
    BackLeftTurningEncoder(10, canivoreBus),
    BackRightTurningEncoder(11, canivoreBus),
    FrontRightTurningEncoder(12, canivoreBus),

    TurretTurningMotor(17, canivoreBus),
    TurretTurningEncoder(18, canivoreBus),
    HoodMotor(15, canivoreBus),
    HoodEncoder(16, canivoreBus),
    FlywheelMotor(19, canivoreBus),

    PigeonGyro(20, canivoreBus),

    IndexerMotor(22, canivoreBus),
    FeederMotor(21, canivoreBus),

    IntakeMotor(40, canivoreBus),
    IntakePivotMotor(41, canivoreBus),
    IntakePivotEncoder(43, canivoreBus),

    ClimberMotor(30, canivoreBus),
    ClimberEncoder(31, canivoreBus),

    CanRange(32, canivoreBus),
}

fun CANcoder(id: CTREDeviceId) = CANcoder(id.num, id.bus)
fun TalonFX(id: CTREDeviceId) = TalonFX(id.num, id.bus)
fun Pigeon2(id: CTREDeviceId) = Pigeon2(id.num, id.bus)
