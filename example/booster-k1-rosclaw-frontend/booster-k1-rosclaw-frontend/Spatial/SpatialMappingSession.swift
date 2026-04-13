import ARKit
import Foundation
import QuartzCore
import simd

@MainActor
@Observable
final class SpatialMappingSession {
    var latestDeviceTransform: simd_float4x4?
    var alignment: HeadsetAlignment?
    var sampleCount = 0
    var floorPointCount = 0
    var recentFloorPoints: [SIMD3<Float>] = []
    var worldSensingState = "idle"

    private let arkitSession = ARKitSession()
    private let worldTracking = WorldTrackingProvider()
    private let sceneReconstruction = SceneReconstructionProvider()

    @ObservationIgnored private var providerTask: Task<Void, Never>?
    @ObservationIgnored private var meshTask: Task<Void, Never>?
    @ObservationIgnored private var deviceTask: Task<Void, Never>?
    @ObservationIgnored private var uploadTask: Task<Void, Never>?
    @ObservationIgnored private var onObservationUpload: (@Sendable (SpatialObservationUpload) async -> Void)?
    @ObservationIgnored private var onError: ((String) -> Void)?
    @ObservationIgnored private var seededRobotPose: RobotPose?

    deinit {
        providerTask?.cancel()
        meshTask?.cancel()
        deviceTask?.cancel()
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

    func refreshAlignmentMapPose(_ robotPose: RobotPose) {
        alignment?.updateRobotPose(robotPose)
    }

    func seedRobotPose(_ robotPose: RobotPose) {
        seededRobotPose = robotPose
    }

    private func configureSession() async {
        do {
            let results = await arkitSession.requestAuthorization(for: [.worldSensing])
            guard results[.worldSensing] == .allowed else {
                worldSensingState = "world_sensing_denied"
                onError?("VisionOS world sensing permission was denied.")
                return
            }

            try await arkitSession.run([worldTracking, sceneReconstruction])
            worldSensingState = "running"

            deviceTask = Task { [weak self] in
                await self?.pollDeviceAnchor()
            }
            meshTask = Task { [weak self] in
                await self?.consumeMeshUpdates()
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
            if let anchor = worldTracking.queryDeviceAnchor(atTimestamp: CACurrentMediaTime()) {
                latestDeviceTransform = anchor.originFromAnchorTransform
            }
            try? await Task.sleep(for: .milliseconds(200))
        }
    }

    private func consumeMeshUpdates() async {
        for await update in sceneReconstruction.anchorUpdates {
            if Task.isCancelled { return }
            let point = translation(of: update.anchor.originFromAnchorTransform)
            sampleCount += 1

            if abs(point.y - (latestDeviceTransform.map { translation(of: $0).y - 1.2 } ?? point.y)) < 1.5 {
                floorPointCount += 1
                recentFloorPoints.append(point)
                if recentFloorPoints.count > 48 {
                    recentFloorPoints.removeFirst(recentFloorPoints.count - 48)
                }
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
}
