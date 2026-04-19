import Foundation
import simd

func translation(of matrix: simd_float4x4) -> SIMD3<Float> {
    SIMD3<Float>(matrix.columns.3.x, matrix.columns.3.y, matrix.columns.3.z)
}

func yawRadians(of matrix: simd_float4x4) -> Float {
    atan2f(matrix.columns.0.z, matrix.columns.0.x)
}

func forwardDirection(of matrix: simd_float4x4) -> SIMD3<Float> {
    simd_normalize(
        SIMD3<Float>(
            -matrix.columns.2.x,
            -matrix.columns.2.y,
            -matrix.columns.2.z
        )
    )
}

func floorIntersectionPoint(
    origin: SIMD3<Float>,
    direction: SIMD3<Float>,
    floorY: Float
) -> SIMD3<Float>? {
    guard abs(direction.y) > 0.0001 else { return nil }
    let distance = (floorY - origin.y) / direction.y
    guard distance > 0.0, distance < 12.0 else { return nil }
    return origin + direction * distance
}

func floorPoint(for matrix: simd_float4x4, floorY: Float) -> SIMD3<Float> {
    let point = translation(of: matrix)
    return SIMD3<Float>(point.x, floorY, point.z)
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
    let worldFloorY: Float
    let worldYawRadians: Float
    var mapOrigin: SIMD3<Double>
    var mapFloorY: Double
    var mapYawRadians: Double

    init(
        worldOrigin: SIMD3<Float>,
        worldFloorY: Float,
        worldYawRadians: Float,
        mapOrigin: SIMD3<Double>,
        mapFloorY: Double,
        mapYawRadians: Double
    ) {
        self.worldOrigin = worldOrigin
        self.worldFloorY = worldFloorY
        self.worldYawRadians = worldYawRadians
        self.mapOrigin = mapOrigin
        self.mapFloorY = mapFloorY
        self.mapYawRadians = mapYawRadians
    }

    init(deviceTransform: simd_float4x4, robotPose: RobotPose, worldFloorY: Float? = nil) {
        let devicePosition = translation(of: deviceTransform)
        let resolvedWorldFloorY = worldFloorY ?? devicePosition.y
        self.init(
            worldOrigin: SIMD3<Float>(devicePosition.x, resolvedWorldFloorY, devicePosition.z),
            worldFloorY: resolvedWorldFloorY,
            worldYawRadians: yawRadians(of: deviceTransform),
            mapOrigin: robotPose.position.simdDouble,
            mapFloorY: robotPose.position.y,
            mapYawRadians: robotPose.yawRadians
        )
    }

    func mapPoint(for worldPoint: SIMD3<Float>) -> SIMD3<Double> {
        let delta = SIMD3<Double>(
            Double(worldPoint.x - worldOrigin.x),
            0.0,
            Double(worldPoint.z - worldOrigin.z)
        )
        let rotated = rotate2D(delta, yawRadians: mapYawRadians - Double(worldYawRadians))
        return SIMD3<Double>(
            mapOrigin.x + rotated.x,
            mapFloorY + Double(worldPoint.y - worldFloorY),
            mapOrigin.z + rotated.z
        )
    }

    func worldPoint(for mapPoint: SIMD3<Double>) -> SIMD3<Float> {
        let delta = SIMD3<Double>(
            mapPoint.x - mapOrigin.x,
            0.0,
            mapPoint.z - mapOrigin.z
        )
        let rotated = rotate2D(delta, yawRadians: Double(worldYawRadians) - mapYawRadians)
        return SIMD3<Float>(
            Float(rotated.x) + worldOrigin.x,
            worldFloorY + Float(mapPoint.y - mapFloorY),
            Float(rotated.z) + worldOrigin.z
        )
    }

    func worldYaw(for mapYawRadians: Double) -> Float {
        worldYawRadians + Float(mapYawRadians - self.mapYawRadians)
    }
}
