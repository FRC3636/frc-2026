package com.frcteam3636.swervebase

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX

private val canivoreBus = CANBus("*")

enum class CTREDeviceId(val num: Int, val bus: CANBus) {
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

    PigeonGyro(20, canivoreBus),
}

fun CANcoder(id: CTREDeviceId) = CANcoder(id.num, id.bus)
fun TalonFX(id: CTREDeviceId) = TalonFX(id.num, id.bus)
fun Pigeon2(id: CTREDeviceId) = Pigeon2(id.num, id.bus)
