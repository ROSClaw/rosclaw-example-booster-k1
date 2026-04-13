import Foundation
import simd

struct Vector3Payload: Codable, Sendable, Equatable {
    var x: Double
    var y: Double
    var z: Double

    init(x: Double, y: Double, z: Double) {
        self.x = x
        self.y = y
        self.z = z
    }

    init(_ value: SIMD3<Float>) {
        self.init(x: Double(value.x), y: Double(value.y), z: Double(value.z))
    }

    init(_ value: SIMD3<Double>) {
        self.init(x: value.x, y: value.y, z: value.z)
    }

    var simdFloat: SIMD3<Float> {
        SIMD3<Float>(Float(x), Float(y), Float(z))
    }

    var simdDouble: SIMD3<Double> {
        SIMD3<Double>(x, y, z)
    }
}

struct RobotPose: Codable, Sendable, Equatable {
    var frame: String
    var position: Vector3Payload
    var yawRadians: Double
}

struct AutonomyState: Codable, Sendable, Equatable {
    var enabled: Bool
    var mode: String
    var navState: String
    var lastOutcome: String
}

struct MappingState: Codable, Sendable, Equatable {
    var alignmentState: String
    var sharedMapStatus: String
    var observationCount: Int
    var floorPointCount: Int
}

struct GoalState: Codable, Sendable, Equatable {
    var frame: String
    var position: Vector3Payload
    var summary: String
}

struct ReportItem: Codable, Sendable, Equatable, Identifiable {
    var id: String
    var timestamp: String
    var summary: String
    var labels: [String]
}

struct BackendStateResponse: Codable, Sendable, Equatable {
    var ok: Bool
    var connectionState: String
    var robotPose: RobotPose?
    var autonomy: AutonomyState
    var mapping: MappingState
    var currentIntent: String
    var statusSummary: String
    var lastGoal: GoalState?
    var reports: [ReportItem]

    static let empty = BackendStateResponse(
        ok: false,
        connectionState: "disconnected",
        robotPose: nil,
        autonomy: AutonomyState(enabled: false, mode: "MANUAL", navState: "idle", lastOutcome: ""),
        mapping: MappingState(
            alignmentState: "collecting",
            sharedMapStatus: "waiting_for_spatial_data",
            observationCount: 0,
            floorPointCount: 0
        ),
        currentIntent: "Waiting for backend",
        statusSummary: "No backend state yet.",
        lastGoal: nil,
        reports: []
    )
}

struct AlignmentRequest: Codable, Sendable {
    var originWorld: Vector3Payload
    var worldYawRadians: Double
    var mapOrigin: Vector3Payload
    var mapYawRadians: Double
}

struct SpatialObservationUpload: Codable, Sendable {
    var timestamp: String
    var devicePosition: Vector3Payload
    var deviceYawRadians: Double
    var floorPoints: [Vector3Payload]
    var sampleCount: Int
}

struct TapGoalRequest: Codable, Sendable {
    var mapPoint: Vector3Payload
    var frame: String
}

struct AutonomyToggleRequest: Codable, Sendable {
    var enabled: Bool
}

struct CommandResponse: Codable, Sendable, Equatable {
    var ok: Bool
    var summary: String
}
