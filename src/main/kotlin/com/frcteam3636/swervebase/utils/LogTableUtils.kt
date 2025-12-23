package com.frcteam3636.swervebase.utils

import com.frcteam3636.swervebase.utils.swerve.PerCorner
import edu.wpi.first.util.struct.StructSerializable
import org.littletonrobotics.junction.LogTable

object LogTableUtils {
    inline fun <reified T : StructSerializable> LogTable.kPut(key: String, value: PerCorner<T>) =
        put(key, *value.toTypedArray())

    inline fun <reified T : StructSerializable> LogTable.kGet(key: String, defaultValue: PerCorner<T>) =
        PerCorner.fromConventionalArray(get(key, *defaultValue.toTypedArray()))
}
