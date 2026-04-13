import Testing
@testable import booster_k1_rosclaw_frontend
import simd

struct AlignmentTests {
    @Test
    func worldPointRoundTripsThroughAlignment() async throws {
        let pose = RobotPose(
            frame: "map",
            position: Vector3Payload(x: 2.0, y: 0.0, z: -1.0),
            yawRadians: .pi / 4.0
        )

        var matrix = matrix_identity_float4x4
        matrix.columns.3 = SIMD4<Float>(1.0, 0.0, 3.0, 1.0)

        let alignment = HeadsetAlignment(deviceTransform: matrix, robotPose: pose)
        let worldPoint = SIMD3<Float>(1.5, 0.0, 4.2)
        let mapped = alignment.mapPoint(for: worldPoint)
        let resolved = alignment.worldPoint(for: mapped)

        #expect(abs(resolved.x - worldPoint.x) < 0.001)
        #expect(abs(resolved.y - worldPoint.y) < 0.001)
        #expect(abs(resolved.z - worldPoint.z) < 0.001)
    }
}
