package com.frcteam3636.swervebase.utils

import com.ctre.phoenix6.CANBus
import com.frcteam3636.swervebase.utils.math.advanceIfElapsed
import com.frcteam3636.swervebase.utils.math.seconds
import edu.wpi.first.wpilibj.Timer

private val cachedBusStatuses = HashMap<String, Pair<Timer, CANBus.CANBusStatus>>()
private val canRefreshPeriod = 1.seconds

/** Returns the status of the specified CAN bus, which may be up to 1 second out of date. */
val CANBus.cachedStatus: CANBus.CANBusStatus
    get() {
        val cached = cachedBusStatuses[name]
        val timer = cached?.first ?: Timer().apply { start() }

        if (cached != null) {
            val isOutdated = timer.advanceIfElapsed(canRefreshPeriod)
            if (!isOutdated) {
                return cached.second
            }
        }

        // This can block for up to 0.001 seconds, which isn't a lot,
        // but might not be good to do in a loop.
        val status = this.getStatus()
        cachedBusStatuses[name] = timer to status

        return status
    }
