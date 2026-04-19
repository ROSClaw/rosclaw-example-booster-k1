import ARKit
import Foundation
import QuartzCore
import simd

struct HandPointerSample: Sendable, Equatable {
    let chirality: HandAnchor.Chirality
    let originWorld: SIMD3<Float>
    let directionWorld: SIMD3<Float>
    let pinchMidpointWorld: SIMD3<Float>
    let pinchDistance: Float
}

@MainActor
@Observable
final class SpatialMappingSession {
    var latestDeviceTransform: simd_float4x4?
    var alignment: HeadsetAlignment?
    var sampleCount = 0
    var floorPointCount = 0
    var recentFloorPoints: [SIMD3<Float>] = []
    var worldSensingState = "idle"
    var worldTrackingState = "initialized"
    var sceneReconstructionState = "initialized"
    var planeDetectionState = "initialized"
    var handTrackingState = HandTrackingProvider.isSupported ? "initialized" : "unsupported"
    var estimatedFloorY: Float?
    var deviceHeightAboveFloor: Float?

    private let arkitSession = ARKitSession()
    private let worldTracking = WorldTrackingProvider()
    private let planeDetection = PlaneDetectionProvider(alignments: [.horizontal])
    private let sceneReconstruction = SceneReconstructionProvider()
    private let handTracking = HandTrackingProvider()

    @ObservationIgnored private var providerTask: Task<Void, Never>?
    @ObservationIgnored private var planeTask: Task<Void, Never>?
    @ObservationIgnored private var meshTask: Task<Void, Never>?
    @ObservationIgnored private var deviceTask: Task<Void, Never>?
    @ObservationIgnored private var handTask: Task<Void, Never>?
    @ObservationIgnored private var uploadTask: Task<Void, Never>?
    @ObservationIgnored private var onObservationUpload: (@Sendable (SpatialObservationUpload) async -> Void)?
    @ObservationIgnored private var onError: ((String) -> Void)?
    @ObservationIgnored private var leftHandPointer: HandPointerSample?
    @ObservationIgnored private var rightHandPointer: HandPointerSample?
    @ObservationIgnored private var handTrackingAuthorized = false

    deinit {
        providerTask?.cancel()
        planeTask?.cancel()
        meshTask?.cancel()
        deviceTask?.cancel()
        handTask?.cancel()
        uploadTask?.cancel()
    }

    func start(
        observationUploadHandler: @escaping @Sendable (SpatialObservationUpload) async -> Void,
        errorHandler: @escaping (String) -> Void
    ) async {
        guard providerTask == nil else { return }
        onObservationUpload = observationUploadHandler
        onError = errorHandler

        providerTask = Task { [weak self] in
            guard let self else { return }
            await self.configureSession()
        }
    }

    var latestRightHandPointer: HandPointerSample? {
        rightHandPointer
    }

    var latestLeftHandPointer: HandPointerSample? {
        leftHandPointer
    }

    private func configureSession() async {
        do {
            var authorizations: [ARKitSession.AuthorizationType] = [.worldSensing]
            if HandTrackingProvider.isSupported {
                authorizations.append(.handTracking)
            }

            worldSensingState = "requesting_world_sensing_authorization"
            let results = await arkitSession.requestAuthorization(for: authorizations)
            guard results[.worldSensing] == .allowed else {
                worldSensingState = "world_sensing_denied"
                onError?("VisionOS world sensing permission was denied.")
                return
            }

            handTrackingAuthorized = HandTrackingProvider.isSupported && results[.handTracking] == .allowed
            if HandTrackingProvider.isSupported && !handTrackingAuthorized {
                handTrackingState = "denied"
            }

            worldSensingState = "starting_tracking_providers"
            var providers: [any DataProvider] = [worldTracking, planeDetection, sceneReconstruction]
            if handTrackingAuthorized {
                providers.append(handTracking)
            }
            try await arkitSession.run(providers)
            refreshProviderStates()

            deviceTask = Task { [weak self] in
                await self?.pollDeviceAnchor()
            }
            planeTask = Task { [weak self] in
                await self?.consumePlaneUpdates()
            }
            meshTask = Task { [weak self] in
                await self?.consumeMeshUpdates()
            }
            if handTrackingAuthorized {
                handTask = Task { [weak self] in
                    await self?.consumeHandUpdates()
                }
            }
            uploadTask = Task { [weak self] in
                await self?.uploadObservations()
            }
        } catch {
            worldSensingState = "failed"
            onError?("ARKit session failed: \(error.localizedDescription)")
        }
    }

    private func pollDeviceAnchor() async {
        while !Task.isCancelled {
            refreshProviderStates()

            guard worldTracking.state == .running else {
                latestDeviceTransform = nil
                deviceHeightAboveFloor = nil
                worldSensingState = "waiting_for_world_tracking"
                try? await Task.sleep(for: .milliseconds(120))
                continue
            }

            if let anchor = worldTracking.queryDeviceAnchor(atTimestamp: CACurrentMediaTime()),
               anchor.isTracked {
                latestDeviceTransform = anchor.originFromAnchorTransform
                updateDeviceHeightFromLatestPose()
                refreshProviderStates()
            } else {
                worldSensingState = estimatedFloorY == nil ? "scanning_floor" : "acquiring_device_pose"
            }
            try? await Task.sleep(for: .milliseconds(33))
        }
    }

    private func consumeHandUpdates() async {
        for await update in handTracking.anchorUpdates {
            if Task.isCancelled { return }

            let sample = handPointerSample(from: update.anchor)
            switch update.anchor.chirality {
            case .left:
                leftHandPointer = sample
            case .right:
                rightHandPointer = sample
            @unknown default:
                break
            }
        }
    }

    private func consumePlaneUpdates() async {
        for await update in planeDetection.anchorUpdates {
            if Task.isCancelled { return }

            let anchor = update.anchor
            guard anchor.alignment == .horizontal else { continue }

            if #available(visionOS 26.0, *) {
                guard anchor.surfaceClassification == .floor else { continue }
            } else {
                guard anchor.classification == .floor else { continue }
            }

            let point = translation(of: anchor.originFromAnchorTransform)
            ingestFloorPoint(point)
        }
    }

    private func consumeMeshUpdates() async {
        for await update in sceneReconstruction.anchorUpdates {
            if Task.isCancelled { return }
            let point = translation(of: update.anchor.originFromAnchorTransform)
            sampleCount += 1

            guard let latestDeviceTransform else { continue }

            let deviceY = translation(of: latestDeviceTransform).y
            let expectedFloorY = estimatedFloorY ?? (deviceY - 1.3)

            if point.y < deviceY - 0.15,
               abs(point.y - expectedFloorY) < 0.45 {
                floorPointCount += 1
                ingestFloorPoint(point)
            }
        }
    }

    private func uploadObservations() async {
        let formatter = ISO8601DateFormatter()
        while !Task.isCancelled {
            guard let onObservationUpload, let latestDeviceTransform else {
                try? await Task.sleep(for: .seconds(1))
                continue
            }

            let observation = SpatialObservationUpload(
                timestamp: formatter.string(from: Date()),
                devicePosition: Vector3Payload(translation(of: latestDeviceTransform)),
                deviceYawRadians: Double(yawRadians(of: latestDeviceTransform)),
                floorPoints: recentFloorPoints.suffix(24).map(Vector3Payload.init),
                sampleCount: sampleCount
            )
            await onObservationUpload(observation)
            try? await Task.sleep(for: .seconds(1))
        }
    }

    private func ingestFloorPoint(_ point: SIMD3<Float>) {
        if let latestDeviceTransform {
            let deviceY = translation(of: latestDeviceTransform).y
            guard point.y < deviceY - 0.1, point.y > deviceY - 2.5 else { return }
        }

        recentFloorPoints.append(point)
        if recentFloorPoints.count > 64 {
            recentFloorPoints.removeFirst(recentFloorPoints.count - 64)
        }

        let sortedHeights = recentFloorPoints.map(\.y).sorted()
        if let medianHeight = sortedHeights[safe: sortedHeights.count / 2] {
            estimatedFloorY = medianHeight
        }
        floorPointCount = recentFloorPoints.count
        updateDeviceHeightFromLatestPose()
        refreshProviderStates()
    }

    private func updateDeviceHeightFromLatestPose() {
        guard let latestDeviceTransform, let estimatedFloorY else {
            deviceHeightAboveFloor = nil
            return
        }

        let deviceY = translation(of: latestDeviceTransform).y
        deviceHeightAboveFloor = max(0.0, deviceY - estimatedFloorY)
    }

    private func refreshProviderStates() {
        worldTrackingState = worldTracking.state.description
        planeDetectionState = planeDetection.state.description
        sceneReconstructionState = sceneReconstruction.state.description
        if HandTrackingProvider.isSupported {
            handTrackingState = handTrackingAuthorized ? handTracking.state.description : handTrackingState
        }

        if worldTracking.state != .running {
            worldSensingState = "waiting_for_world_tracking"
        } else if latestDeviceTransform == nil {
            worldSensingState = "acquiring_device_pose"
        } else if estimatedFloorY == nil {
            worldSensingState = "scanning_floor"
        } else {
            worldSensingState = "ready_for_alignment"
        }
    }

    private func handPointerSample(from anchor: HandAnchor) -> HandPointerSample? {
        guard anchor.isTracked,
              let wrist = worldJointPosition(.wrist, in: anchor),
              let knuckle = worldJointPosition(.indexFingerKnuckle, in: anchor),
              let tip = worldJointPosition(.indexFingerTip, in: anchor),
              let thumb = worldJointPosition(.thumbTip, in: anchor)
        else {
            return nil
        }

        let rawDirection = (tip - knuckle) + ((tip - wrist) * 0.35) + SIMD3<Float>(0.0, -0.12, 0.0)
        guard simd_length_squared(rawDirection) > 0.0001 else {
            return nil
        }

        return HandPointerSample(
            chirality: anchor.chirality,
            originWorld: knuckle,
            directionWorld: simd_normalize(rawDirection),
            pinchMidpointWorld: (tip + thumb) * 0.5,
            pinchDistance: simd_distance(tip, thumb)
        )
    }

    private func worldJointPosition(_ jointName: HandSkeleton.JointName, in anchor: HandAnchor) -> SIMD3<Float>? {
        guard let skeleton = anchor.handSkeleton else { return nil }
        let joint = skeleton.joint(jointName)
        guard joint.isTracked else { return nil }
        let worldTransform = anchor.originFromAnchorTransform * joint.anchorFromJointTransform
        return translation(of: worldTransform)
    }
}

private extension Array {
    subscript(safe index: Int) -> Element? {
        indices.contains(index) ? self[index] : nil
    }
}
