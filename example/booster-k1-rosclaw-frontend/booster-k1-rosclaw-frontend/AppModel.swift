import Foundation
import SwiftUI
import simd

@MainActor
@Observable
final class AppModel {
    enum WorkflowStepStatus: Equatable {
        case pending
        case active
        case complete
    }

    struct ImmersiveSceneSnapshot: Equatable {
        let immersiveSpaceState: ImmersiveSpaceState
        let hasFloorEstimate: Bool
        let isAligned: Bool
        let pointerSource: PointerSource?
        let pointerRayOriginWorldPoint: SIMD3<Float>?
        let pointerWorldPoint: SIMD3<Float>?
        let robotAnchorWorldPoint: SIMD3<Float>?
        let sceneFloorCenterWorldPoint: SIMD3<Float>?
        let sceneOperatorWorldPoint: SIMD3<Float>?
        let sceneOperatorYawRadians: Float?
        let sceneRobotPreviewWorldPoint: SIMD3<Float>?
        let sceneRobotPreviewYawRadians: Float?
        let pendingGoalWorldPoint: SIMD3<Float>?
    }

    struct WorkflowStep: Identifiable, Sendable {
        let id: String
        let title: String
        let detail: String
        let status: WorkflowStepStatus
    }

    enum ImmersiveSpaceState: Equatable {
        case closed
        case inTransition
        case open
    }

    enum PointerSource: String, Equatable {
        case hand
        case head
    }

    let immersiveSpaceID = "k1-immersive-space"
    let backendClient: K1BackendClient
    let spatial = SpatialMappingSession()

    var immersiveSpaceState = ImmersiveSpaceState.closed
    var backendState = BackendStateResponse.empty
    var lastError: String?
    var isRefreshing = false
    var isSendingGoal = false
    var isChangingAutonomy = false
    var isRequestingReport = false
    var lastCommandSummary = "No command issued yet."
    var pointerSource: PointerSource?
    var pointerRayOriginWorldPoint: SIMD3<Float>?
    var pointerWorldPoint: SIMD3<Float>?
    var robotAnchorWorldPoint: SIMD3<Float>?
    var sceneFloorCenterWorldPoint: SIMD3<Float>?
    var sceneOperatorWorldPoint: SIMD3<Float>?
    var sceneOperatorYawRadians: Float?
    var sceneRobotPreviewWorldPoint: SIMD3<Float>?
    var sceneRobotPreviewYawRadians: Float?
    var pendingGoalWorldPoint: SIMD3<Float>?
    var pendingGoalMapPoint: SIMD3<Double>?
    var yawCalibrationOffsetRadians: Float = 0.0

    private var hasStarted = false
    @ObservationIgnored private var pollingTask: Task<Void, Never>?
    @ObservationIgnored private var pointerProjectionTask: Task<Void, Never>?
    @ObservationIgnored private var lastPinchSubmissionDate = Date.distantPast

    init(backendClient: K1BackendClient = .configuredFromEnvironment()) {
        self.backendClient = backendClient
    }

    deinit {
        pollingTask?.cancel()
        pointerProjectionTask?.cancel()
    }

    var backendURLDescription: String {
        backendClient.baseURL.absoluteString
    }

    var hasTrackedDevicePose: Bool {
        spatial.worldTrackingState == "running" && spatial.latestDeviceTransform != nil
    }

    var hasFloorEstimate: Bool {
        spatial.estimatedFloorY != nil
    }

    var hasRobotAnchor: Bool {
        robotAnchorWorldPoint != nil
    }

    var isAligned: Bool {
        spatial.alignment != nil && backendState.mapping.alignmentState == "aligned"
    }

    var canAlignHeadset: Bool {
        previewAlignment != nil
    }

    var canMarkRobotAnchor: Bool {
        immersiveSpaceState == .open &&
            hasFloorEstimate &&
            backendState.robotPose != nil &&
            pointerWorldPoint != nil &&
            !isAligned
    }

    var canSendTapGoal: Bool {
        immersiveSpaceState == .open && isAligned && !isSendingGoal
    }

    var canRotatePreview: Bool {
        hasRobotAnchor && !isAligned
    }

    var autonomyEnabled: Bool {
        backendState.autonomy.enabled
    }

    var liveOperatorWorldPoint: SIMD3<Float>? {
        sceneOperatorWorldPoint
    }

    var liveOperatorYawRadians: Float? {
        sceneOperatorYawRadians
    }

    var previewAlignment: HeadsetAlignment? {
        guard let worldOrigin = robotAnchorWorldPoint,
              let floorY = spatial.estimatedFloorY,
              let robotPose = backendState.robotPose,
              let deviceTransform = spatial.latestDeviceTransform
        else {
            return nil
        }

        return HeadsetAlignment(
            worldOrigin: worldOrigin,
            worldFloorY: floorY,
            worldYawRadians: yawRadians(of: deviceTransform) + yawCalibrationOffsetRadians,
            mapOrigin: robotPose.position.simdDouble,
            mapFloorY: robotPose.position.y,
            mapYawRadians: robotPose.yawRadians
        )
    }

    var displayedAlignment: HeadsetAlignment? {
        spatial.alignment ?? previewAlignment
    }

    var yawCalibrationDegrees: Double {
        Double(yawCalibrationOffsetRadians) * 180.0 / .pi
    }

    var immersiveSceneSnapshot: ImmersiveSceneSnapshot {
        ImmersiveSceneSnapshot(
            immersiveSpaceState: immersiveSpaceState,
            hasFloorEstimate: hasFloorEstimate,
            isAligned: isAligned,
            pointerSource: pointerSource,
            pointerRayOriginWorldPoint: pointerRayOriginWorldPoint,
            pointerWorldPoint: pointerWorldPoint,
            robotAnchorWorldPoint: robotAnchorWorldPoint,
            sceneFloorCenterWorldPoint: sceneFloorCenterWorldPoint,
            sceneOperatorWorldPoint: sceneOperatorWorldPoint,
            sceneOperatorYawRadians: sceneOperatorYawRadians,
            sceneRobotPreviewWorldPoint: sceneRobotPreviewWorldPoint,
            sceneRobotPreviewYawRadians: sceneRobotPreviewYawRadians,
            pendingGoalWorldPoint: pendingGoalWorldPoint
        )
    }

    var floorReadinessDescription: String {
        if let floorY = spatial.estimatedFloorY,
           let deviceHeight = spatial.deviceHeightAboveFloor {
            return String(
                format: "Floor locked %.2fm below the headset at world y %.2f.",
                deviceHeight,
                floorY
            )
        }
        if hasTrackedDevicePose {
            return "World tracking is live. Scan the floor and nearby walls to lock a ground plane."
        }
        return "Open the immersive workspace and look around so world tracking can start."
    }

    var reticleDescription: String {
        if let point = pointerWorldPoint {
            return String(
                format: "%@ ray hit: x %.2f  y %.2f  z %.2f",
                (pointerSource == .hand ? "Hand-ray" : "Head-ray"),
                point.x,
                point.y,
                point.z
            )
        }
        if immersiveSpaceState != .open {
            return "Reticle hidden until the immersive workspace is open."
        }
        if !hasTrackedDevicePose {
            return "Reticle unavailable until world tracking is running."
        }
        if !hasFloorEstimate {
            return "Reticle will appear after the app detects the floor."
        }
        if !hasRobotAnchor {
            return "Use the hand ray to place the floor target on the robot's feet, then mark that spot."
        }
        if !isAligned {
            return "The raycast target is live. Rotate the robot preview until it sits on the real robot, then align."
        }
        return "Point your hand lower toward the floor to place the raycast target."
    }

    var fusionConfidenceDescription: String {
        if let lastError, isTimeoutError(lastError) {
            return "Bridge timeout"
        }
        if spatial.worldTrackingState != "running" {
            return "Tracking reacquiring"
        }
        if !hasFloorEstimate {
            return "Floor scan in progress"
        }
        if !hasRobotAnchor {
            return "Waiting for robot anchor"
        }
        if !isAligned {
            return "Preview only"
        }
        return "Continuous overlay live"
    }

    var operatorHeadline: String {
        if immersiveSpaceState != .open {
            return "Open the immersive workspace"
        }
        if !hasTrackedDevicePose {
            return "Wait for world tracking to lock"
        }
        if !hasFloorEstimate {
            return "Scan the floor to place the workspace"
        }
        if !hasRobotAnchor {
            return "Mark the robot on the floor"
        }
        if !isAligned {
            return "Rotate the preview, then align"
        }
        if isSendingGoal {
            return "Sending the move request"
        }
        if let lastError, isTimeoutError(lastError) {
            return "The backend is timing out"
        }
        return "Aim with your hand ray and pinch once to move"
    }

    var operatorDetail: String {
        if let lastError, isTimeoutError(lastError) {
            return "The bridge at \(backendURLDescription) is not responding quickly enough. Wait for the timeout to clear, then try one move request at a time."
        }
        if immersiveSpaceState != .open {
            return "The reticle and floor projection only appear inside the immersive space."
        }
        if !hasTrackedDevicePose {
            return "The app is still waiting for visionOS world tracking. The repeated device-anchor log lines happen before this reaches the running state."
        }
        if !hasFloorEstimate {
            return "Look down at the ground and slowly pan across the room so plane detection can classify the floor."
        }
        if !hasRobotAnchor {
            return "Point at the robot's feet until the floor target sits under the real robot, then press Mark Robot Position. The orange operator marker should follow you continuously while the ray beam connects your hand to the floor target."
        }
        if !isAligned {
            return "The cyan robot marker is a live preview. Nudge it left or right until it overlays the real robot, then commit the alignment."
        }
        if isSendingGoal {
            return "The current reticle target has been sent. The app is temporarily blocking repeated goal requests until the backend responds."
        }
        return "The cyan hand ray shows the target floor point. Pinch with pointer finger and thumb once, then wait for the command summary before sending another move."
    }

    var workflowSteps: [WorkflowStep] {
        [
            WorkflowStep(
                id: "immersive",
                title: "Open immersive workspace",
                detail: immersiveSpaceState == .open
                    ? "Mixed-space view is open."
                    : "Open the immersive scene so the app can project the floor reticle around you.",
                status: immersiveSpaceState == .open ? .complete : .active
            ),
            WorkflowStep(
                id: "tracking",
                title: "Lock tracking and detect the floor",
                detail: hasTrackedDevicePose && hasFloorEstimate
                    ? floorReadinessDescription
                    : (!hasTrackedDevicePose
                        ? "Look around the room until world tracking starts."
                        : "Sweep the floor and nearby walls so the app can find the ground plane."),
                status: hasTrackedDevicePose && hasFloorEstimate ? .complete : (immersiveSpaceState == .open ? .active : .pending)
            ),
            WorkflowStep(
                id: "robot-anchor",
                title: "Mark the robot position",
                detail: hasRobotAnchor
                    ? "Robot anchor locked on the floor."
                    : (backendState.robotPose == nil
                        ? "Waiting for a robot pose from the backend."
                        : "Aim the reticle at the robot's feet and press Mark Robot Position."),
                status: hasRobotAnchor ? .complete : (hasFloorEstimate ? .active : .pending)
            ),
            WorkflowStep(
                id: "alignment",
                title: "Rotate preview and align",
                detail: isAligned
                    ? "Handset calibration committed to the robot map."
                    : (hasRobotAnchor
                        ? String(format: "Yaw trim %.0f°. Adjust until the cyan robot overlay matches the real robot.", yawCalibrationDegrees)
                        : "Yaw trim unlocks after you mark the robot position."),
                status: isAligned ? .complete : (hasRobotAnchor ? .active : .pending)
            ),
            WorkflowStep(
                id: "move",
                title: "Aim reticle and pinch once",
                detail: isSendingGoal
                    ? "Move request in flight. Additional taps are blocked until it finishes."
                    : (canSendTapGoal
                        ? reticleDescription
                        : "Tap-to-move unlocks after alignment."),
                status: isSendingGoal ? .active : (canSendTapGoal ? .active : .pending)
            ),
        ]
    }

    func startIfNeeded() async {
        guard !hasStarted else { return }
        hasStarted = true

        await spatial.start(
            observationUploadHandler: { [weak self] observation in
                guard let self else { return }
                do {
                    _ = try await self.backendClient.postObservation(observation)
                } catch {
                    await MainActor.run {
                        self.lastError = error.localizedDescription
                    }
                }
            },
            errorHandler: { [weak self] message in
                self?.lastError = message
            }
        )

        await refreshState()

        pollingTask = Task { [weak self] in
            while !Task.isCancelled {
                guard let self else { return }
                await self.refreshState()
                try? await Task.sleep(for: .seconds(2))
            }
        }

        pointerProjectionTask = Task { [weak self] in
            while !Task.isCancelled {
                guard let self else { return }
                self.refreshLiveReticle()
                self.refreshSceneState()
                self.processPinchGestureIfNeeded()
                try? await Task.sleep(for: .milliseconds(33))
            }
        }
    }

    func refreshState() async {
        guard !isRefreshing else { return }
        isRefreshing = true
        defer { isRefreshing = false }

        do {
            backendState = try await backendClient.fetchState()
            lastError = nil
            if let lastGoal = backendState.lastGoal {
                pendingGoalMapPoint = lastGoal.position.simdDouble
            } else if !isSendingGoal {
                pendingGoalMapPoint = nil
                pendingGoalWorldPoint = nil
            }
            if let alignment = spatial.alignment,
               let pendingGoalMapPoint {
                pendingGoalWorldPoint = alignment.worldPoint(for: pendingGoalMapPoint)
            }
            refreshLiveReticle()
            refreshSceneState()
        } catch {
            lastError = error.localizedDescription
        }
    }

    func alignHeadsetToRobotPose() async {
        guard let alignment = previewAlignment,
              let robotPose = backendState.robotPose
        else {
            lastError = "World tracking, floor detection, robot pose, or robot anchor is not ready."
            return
        }

        spatial.alignment = alignment
        pendingGoalMapPoint = nil
        pendingGoalWorldPoint = nil
        refreshLiveReticle()
        refreshSceneState()

        let request = AlignmentRequest(
            originWorld: Vector3Payload(alignment.worldOrigin),
            worldYawRadians: Double(alignment.worldYawRadians),
            mapOrigin: robotPose.position,
            mapYawRadians: robotPose.yawRadians
        )

        do {
            let response = try await backendClient.postAlignment(request)
            if applyCommandResponse(response) {
                lastCommandSummary = "Aligned the marked robot floor anchor to the current K1 map pose."
                await refreshState()
            }
        } catch {
            lastError = error.localizedDescription
        }
    }

    func markRobotAnchorFromReticle() {
        guard let pointerWorldPoint else {
            lastError = "Point at the robot's feet before marking the robot position."
            return
        }

        robotAnchorWorldPoint = pointerWorldPoint
        yawCalibrationOffsetRadians = 0.0
        lastError = nil
        lastCommandSummary = "Robot floor anchor captured from the live reticle."
        refreshSceneState()
    }

    func resetAlignmentCalibration() {
        spatial.alignment = nil
        robotAnchorWorldPoint = nil
        yawCalibrationOffsetRadians = 0.0
        pendingGoalMapPoint = nil
        pendingGoalWorldPoint = nil
        lastCommandSummary = "Alignment reset. Re-mark the robot and align again."
        refreshLiveReticle()
        refreshSceneState()
    }

    func adjustAlignmentYaw(byDegrees degrees: Float) {
        yawCalibrationOffsetRadians += degrees * (.pi / 180.0)
        lastCommandSummary = String(format: "Preview yaw trim set to %.0f°.", yawCalibrationDegrees)
        refreshLiveReticle()
        refreshSceneState()
    }

    func updatePointerWorldPoint(_ worldPoint: SIMD3<Float>?) {
        pointerWorldPoint = worldPoint
    }

    func sendGroundTap(worldPoint: SIMD3<Float>) async {
        guard !isSendingGoal else {
            lastError = "A move request is already in flight. Wait for it to finish before sending another."
            return
        }

        guard let alignment = spatial.alignment else {
            lastError = "Align the headset before sending tap goals."
            return
        }

        let mapPoint = alignment.mapPoint(for: worldPoint)
        pointerWorldPoint = worldPoint
        pendingGoalWorldPoint = worldPoint
        pendingGoalMapPoint = mapPoint
        isSendingGoal = true
        defer { isSendingGoal = false }

        do {
            let result = try await backendClient.sendTapGoal(
                TapGoalRequest(
                    mapPoint: Vector3Payload(mapPoint),
                    frame: "map"
                )
            )
            if applyCommandResponse(result) {
                await refreshState()
            }
        } catch {
            lastError = error.localizedDescription
            refreshSceneState()
        }
    }

    func toggleAutonomy() async {
        isChangingAutonomy = true
        defer { isChangingAutonomy = false }

        do {
            let response = try await backendClient.setAutonomy(
                AutonomyToggleRequest(enabled: !autonomyEnabled)
            )
            if applyCommandResponse(response) {
                await refreshState()
            }
        } catch {
            lastError = error.localizedDescription
        }
    }

    func requestSceneReport() async {
        isRequestingReport = true
        defer { isRequestingReport = false }

        do {
            let response = try await backendClient.requestSceneReport()
            if applyCommandResponse(response) {
                await refreshState()
            }
        } catch {
            lastError = error.localizedDescription
        }
    }

    func emergencyStop() async {
        do {
            let response = try await backendClient.stopRobot()
            if applyCommandResponse(response) {
                pendingGoalMapPoint = nil
                pendingGoalWorldPoint = nil
                await refreshState()
            }
        } catch {
            lastError = error.localizedDescription
            refreshSceneState()
        }
    }

    @discardableResult
    private func applyCommandResponse(_ response: CommandResponse) -> Bool {
        lastCommandSummary = response.summary
        if response.ok {
            lastError = nil
            return true
        }
        lastError = response.summary
        return false
    }

    private func refreshLiveReticle() {
        guard immersiveSpaceState == .open,
              let floorY = spatial.alignment?.worldFloorY ?? spatial.estimatedFloorY
        else {
            pointerSource = nil
            pointerRayOriginWorldPoint = nil
            pointerWorldPoint = nil
            return
        }

        if let handPointer = spatial.latestHandPointer,
           let floorPoint = floorIntersectionPoint(
               origin: handPointer.pinchMidpointWorld,
               direction: handPointer.directionWorld,
               floorY: floorY
           ),
           isInsideInteractionBounds(floorPoint) {
            pointerSource = .hand
            pointerRayOriginWorldPoint = handPointer.pinchMidpointWorld
            pointerWorldPoint = floorPoint
            return
        }

        guard let deviceTransform = spatial.latestDeviceTransform else {
            pointerSource = nil
            pointerRayOriginWorldPoint = nil
            pointerWorldPoint = nil
            return
        }

        let origin = translation(of: deviceTransform)
        let direction = forwardDirection(of: deviceTransform)
        guard let floorPoint = floorIntersectionPoint(origin: origin, direction: direction, floorY: floorY),
              isInsideInteractionBounds(floorPoint)
        else {
            pointerSource = nil
            pointerRayOriginWorldPoint = nil
            pointerWorldPoint = nil
            return
        }

        pointerSource = .head
        pointerRayOriginWorldPoint = origin
        pointerWorldPoint = floorPoint
    }

    private func refreshSceneState() {
        guard immersiveSpaceState == .open else {
            sceneFloorCenterWorldPoint = nil
            sceneOperatorWorldPoint = nil
            sceneOperatorYawRadians = nil
            sceneRobotPreviewWorldPoint = nil
            sceneRobotPreviewYawRadians = nil
            return
        }

        let displayedAlignment = displayedAlignment
        let floorY = displayedAlignment?.worldFloorY ?? spatial.estimatedFloorY

        if let floorY,
           let latestDeviceTransform = spatial.latestDeviceTransform {
            let operatorPoint = floorPoint(for: latestDeviceTransform, floorY: floorY)
            sceneOperatorWorldPoint = operatorPoint
            sceneOperatorYawRadians = yawRadians(of: latestDeviceTransform)
            sceneFloorCenterWorldPoint = displayedAlignment?.worldOrigin ?? operatorPoint
        } else {
            sceneFloorCenterWorldPoint = displayedAlignment?.worldOrigin
            sceneOperatorWorldPoint = nil
            sceneOperatorYawRadians = nil
        }

        if let alignment = displayedAlignment,
           let robotPose = backendState.robotPose {
            sceneRobotPreviewWorldPoint = alignment.worldPoint(for: robotPose.position.simdDouble)
            sceneRobotPreviewYawRadians = alignment.worldYaw(for: robotPose.yawRadians)
        } else if let anchor = robotAnchorWorldPoint {
            sceneRobotPreviewWorldPoint = anchor
            sceneRobotPreviewYawRadians = sceneOperatorYawRadians ?? 0.0
        } else {
            sceneRobotPreviewWorldPoint = nil
            sceneRobotPreviewYawRadians = nil
        }
    }

    private func isInsideInteractionBounds(_ point: SIMD3<Float>) -> Bool {
        let center: SIMD3<Float>
        if let operatorPoint = liveOperatorWorldPoint {
            center = operatorPoint
        } else if let previewAlignment {
            center = previewAlignment.worldOrigin
        } else {
            return false
        }

        return abs(point.x - center.x) <= 5.0 && abs(point.z - center.z) <= 5.0
    }

    private func processPinchGestureIfNeeded() {
        guard pointerSource == .hand,
              canSendTapGoal,
              let handPointer = spatial.latestHandPointer,
              let pointerWorldPoint,
              handPointer.pinchDistance < 0.030
        else {
            return
        }

        let now = Date()
        guard now.timeIntervalSince(lastPinchSubmissionDate) > 1.0 else { return }
        lastPinchSubmissionDate = now

        Task { await sendGroundTap(worldPoint: pointerWorldPoint) }
    }

    private func isTimeoutError(_ message: String) -> Bool {
        message.localizedCaseInsensitiveContains("timed out")
    }
}
