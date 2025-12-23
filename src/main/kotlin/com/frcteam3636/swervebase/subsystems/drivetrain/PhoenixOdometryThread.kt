package com.frcteam3636.swervebase.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.odometryLock
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotController
import java.util.*
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock

class PhoenixOdometryThread : Thread("PhoenixOdometry") {
    init {
        isDaemon = true
    }

    private val timestampQueues: MutableList<Queue<Double>> = ArrayList()
    private val signalsLock: Lock = ReentrantLock() // Prevents conflicts when registering signals
    private var phoenixSignals: Array<BaseStatusSignal> = emptyArray() // Phoenix API does not accept a mutable list
    private val phoenixQueues = ArrayList<Queue<Double>>()

    override fun start() {
        if (!timestampQueues.isEmpty()) {
            super.start()
        }
    }

    fun registerSignal(signal: StatusSignal<Angle>): Queue<Double> {
        val queue: Queue<Double> = ArrayBlockingQueue(20)

        signalsLock.lock()
        odometryLock.lock()
        try {
            val newSignals = Array(phoenixSignals.size + 1) { i ->
                if (i < phoenixSignals.size) phoenixSignals[i] else signal
            }
            phoenixSignals = newSignals
            phoenixQueues.add(queue)
        } finally {
            signalsLock.unlock()
            odometryLock.unlock()
        }

        return queue
    }

    fun makeTimestampQueue(): Queue<Double> {
        val queue = ArrayBlockingQueue<Double>(20)
        odometryLock.lock()
        try {
            timestampQueues.add(queue)
        } finally {
            odometryLock.unlock()
        }
        return queue
    }

    override fun run() {
        while (true) {
            signalsLock.lock()
            try {
                if (!phoenixSignals.isEmpty()) {
                    BaseStatusSignal.waitForAll(2.0 / 250.0, *phoenixSignals)
                }
            } catch (e: InterruptedException) {
                e.printStackTrace()
            } finally {
                signalsLock.unlock()
            }

            odometryLock.lock()
            try {
                var timestamp = RobotController.getFPGATime() / 1e6
                var latency = 0.0
                for (signal in phoenixSignals) {
                    latency += signal.timestamp.latency
                }
                if (!phoenixSignals.isEmpty())
                    timestamp -= latency / phoenixSignals.size

                for (i in 0..<phoenixSignals.size) {
                    phoenixQueues[i].offer(phoenixSignals[i].valueAsDouble)
                }

                for (i in 0..<timestampQueues.size) {
                    timestampQueues[i].offer(timestamp)
                }
            } finally {
                odometryLock.unlock()
            }
        }
    }

    companion object {
        private var instance: PhoenixOdometryThread? = null

        fun getInstance(): PhoenixOdometryThread {
            if (instance == null)
                instance = PhoenixOdometryThread()
            return instance!!
        }
    }
}