import Foundation
import simd

func translation(of matrix: simd_float4x4) -> SIMD3<Float> {
    SIMD3<Float>(matrix.columns.3.x, matrix.columns.3.y, matrix.columns.3.z)
}

func yawRadians(of matrix: simd_float4x4) -> Float {
    atan2f(matrix.columns.0.z, matrix.columns.0.x)
}

func rotate2D(_ value: SIMD3<Double>, yawRadians: Double) -> SIMD3<Double> {
    let cosYaw = cos(yawRadians)
    let sinYaw = sin(yawRadians)
    return SIMD3<Double>(
        value.x * cosYaw - value.z * sinYaw,
        value.y,
        value.x * sinYaw + value.z * cosYaw
    )
}

struct HeadsetAlignment: Sendable, Equatable {
    let worldOrigin: SIMD3<Float>
    let worldYawRadians: Float
    var mapOrigin: SIMD3<Double>
    var mapYawRadians: Double

    init(deviceTransform: simd_float4x4, robotPose: RobotPose) {
        self.worldOrigin = translation(of: deviceTransform)
        self.worldYawRadians = yawRadians(of: deviceTransform)
        self.mapOrigin = robotPose.position.simdDouble
        self.mapYawRadians = robotPose.yawRadians
    }

    func mapPoint(for worldPoint: SIMD3<Float>) -> SIMD3<Double> {
        let delta = SIMD3<Double>(
            Double(worldPoint.x - worldOrigin.x),
            Double(worldPoint.y - worldOrigin.y),
            Double(worldPoint.z - worldOrigin.z)
        )
        let rotated = rotate2D(delta, yawRadians: mapYawRadians - Double(worldYawRadians))
        return mapOrigin + rotated
    }

    func worldPoint(for mapPoint: SIMD3<Double>) -> SIMD3<Float> {
        let delta = mapPoint - mapOrigin
        let rotated = rotate2D(delta, yawRadians: Double(worldYawRadians) - mapYawRadians)
        return SIMD3<Float>(
            Float(rotated.x) + worldOrigin.x,
            Float(rotated.y) + worldOrigin.y,
            Float(rotated.z) + worldOrigin.z
        )
    }

    mutating func updateRobotPose(_ robotPose: RobotPose) {
        self.mapOrigin = robotPose.position.simdDouble
        self.mapYawRadians = robotPose.yawRadians
    }
}
