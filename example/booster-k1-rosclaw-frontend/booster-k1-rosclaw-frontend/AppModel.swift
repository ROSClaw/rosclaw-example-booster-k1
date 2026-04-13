import Foundation
import SwiftUI
import simd

@MainActor
@Observable
final class AppModel {
    enum ImmersiveSpaceState {
        case closed
        case inTransition
        case open
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
    var pendingGoalWorldPoint: SIMD3<Float>?
    var pendingGoalMapPoint: SIMD3<Double>?

    private var hasStarted = false
    @ObservationIgnored private var pollingTask: Task<Void, Never>?

    init(backendClient: K1BackendClient = .configuredFromEnvironment()) {
        self.backendClient = backendClient
    }

    deinit {
        pollingTask?.cancel()
    }

    var backendURLDescription: String {
        backendClient.baseURL.absoluteString
    }

    var canAlignHeadset: Bool {
        spatial.latestDeviceTransform != nil && backendState.robotPose != nil
    }

    var canSendTapGoal: Bool {
        spatial.alignment != nil && backendState.mapping.alignmentState == "aligned"
    }

    var autonomyEnabled: Bool {
        backendState.autonomy.enabled
    }

    func startIfNeeded() async {
        guard !hasStarted else { return }
        hasStarted = true

        await spatial.start(
            observationUploadHandler: { [weak self] observation in
                guard let self else { return }
                do {
                    try await self.backendClient.postObservation(observation)
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
    }

    func refreshState() async {
        guard !isRefreshing else { return }
        isRefreshing = true
        defer { isRefreshing = false }

        do {
            backendState = try await backendClient.fetchState()
            if let robotPose = backendState.robotPose {
                spatial.seedRobotPose(robotPose)
                spatial.refreshAlignmentMapPose(robotPose)
            }
            if let alignment = spatial.alignment,
               let pendingGoalMapPoint {
                pendingGoalWorldPoint = alignment.worldPoint(for: pendingGoalMapPoint)
            }
        } catch {
            lastError = error.localizedDescription
        }
    }

    func alignHeadsetToRobotPose() async {
        guard let deviceTransform = spatial.latestDeviceTransform,
              let robotPose = backendState.robotPose
        else {
            lastError = "World tracking or robot pose is not ready."
            return
        }

        let alignment = HeadsetAlignment(deviceTransform: deviceTransform, robotPose: robotPose)
        spatial.alignment = alignment
        pendingGoalMapPoint = nil
        pendingGoalWorldPoint = nil

        let request = AlignmentRequest(
            originWorld: Vector3Payload(alignment.worldOrigin),
            worldYawRadians: Double(alignment.worldYawRadians),
            mapOrigin: robotPose.position,
            mapYawRadians: robotPose.yawRadians
        )

        do {
            _ = try await backendClient.postAlignment(request)
            lastCommandSummary = "Aligned headset origin to the current K1 map pose."
            await refreshState()
        } catch {
            lastError = error.localizedDescription
        }
    }

    func sendGroundTap(worldPoint: SIMD3<Float>) async {
        guard let alignment = spatial.alignment else {
            lastError = "Align the headset before sending tap goals."
            return
        }

        let mapPoint = alignment.mapPoint(for: worldPoint)
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
            lastCommandSummary = result.summary
            await refreshState()
        } catch {
            lastError = error.localizedDescription
        }
    }

    func toggleAutonomy() async {
        isChangingAutonomy = true
        defer { isChangingAutonomy = false }

        do {
            let response = try await backendClient.setAutonomy(
                AutonomyToggleRequest(enabled: !autonomyEnabled)
            )
            lastCommandSummary = response.summary
            await refreshState()
        } catch {
            lastError = error.localizedDescription
        }
    }

    func requestSceneReport() async {
        isRequestingReport = true
        defer { isRequestingReport = false }

        do {
            let response = try await backendClient.requestSceneReport()
            lastCommandSummary = response.summary
            await refreshState()
        } catch {
            lastError = error.localizedDescription
        }
    }

    func emergencyStop() async {
        do {
            let response = try await backendClient.stopRobot()
            lastCommandSummary = response.summary
            pendingGoalMapPoint = nil
            pendingGoalWorldPoint = nil
            await refreshState()
        } catch {
            lastError = error.localizedDescription
        }
    }
}
