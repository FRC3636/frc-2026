package com.frcteam3636.frc2026.subsystems.objectDetection

import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import org.team9432.annotation.Logged


@Logged
open class ObjectDetectionInputs {
    var lastCheckedResults: PhotonPipelineResult = PhotonPipelineResult()
    var connected: Boolean = false
}

interface ObjectDetectionIO{
    fun updateInputs(inputs : ObjectDetectionInputs)
    fun updateResult()
    val newestResults : PhotonPipelineResult
}

class ObjectDetectionIOReal: ObjectDetectionIO{
    private val camera = PhotonCamera("color camera")

    private var unReadResults = List<PhotonPipelineResult>(1) { PhotonPipelineResult() }
    override var newestResults = PhotonPipelineResult()

    override fun updateResult() {
        unReadResults = camera.allUnreadResults
        if (unReadResults.isNotEmpty()) {
            newestResults = unReadResults.last()
        }
        newestResults = PhotonPipelineResult()
    }

    override fun updateInputs(inputs: ObjectDetectionInputs) {
        inputs.lastCheckedResults = newestResults
        inputs.connected = camera.isConnected
    }
 }

class ObjectDetectionIOSim: ObjectDetectionIO{
    private var unReadResults = List<PhotonPipelineResult>(1) { PhotonPipelineResult() }
    val camera = PhotonCamera("color camera")
    override var newestResults = PhotonPipelineResult()

    override fun updateResult() {
        unReadResults = camera.allUnreadResults
        if (unReadResults.isNotEmpty()) {
            newestResults = unReadResults.last()
        }
        newestResults = PhotonPipelineResult()
    }

    override fun updateInputs(inputs: ObjectDetectionInputs) {
        inputs.lastCheckedResults = newestResults
        inputs.connected = camera.isConnected
    }
}


