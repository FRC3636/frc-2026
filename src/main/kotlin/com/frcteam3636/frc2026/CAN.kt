package com.frcteam3636.frc2026

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkFlex

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
enum class REVMotorControllerId(val num: Int) {
    FrontLeftDrivingMotor(1),
    BackLeftDrivingMotor(2),
    BackRightDrivingMotor(3),
    FrontRightDrivingMotor(4),

    FrontLeftTurningMotor(5),
    BackLeftTurningMotor(6),
    BackRightTurningMotor(7),
    FrontRightTurningMotor(8),

}

fun CANcoder(id: CTREDeviceId) = CANcoder(id.num, id.bus)
fun TalonFX(id: CTREDeviceId) = TalonFX(id.num, id.bus)
fun Pigeon2(id: CTREDeviceId) = Pigeon2(id.num, id.bus)
fun SparkMax(id: REVMotorControllerId, type: SparkLowLevel.MotorType) = SparkMax(id.num, type)
fun SparkFlex(id: REVMotorControllerId, type: SparkLowLevel.MotorType) = SparkFlex(id.num, type)
