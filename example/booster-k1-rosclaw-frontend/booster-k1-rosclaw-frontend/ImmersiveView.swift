import RealityKit
import SwiftUI
import simd

struct ImmersiveView: View {
    @Environment(AppModel.self) private var appModel

    @State private var rootEntity = Entity()
    @State private var floorEntity = ModelEntity()
    @State private var robotEntity = ModelEntity()
    @State private var goalEntity = ModelEntity()
    @State private var hasConfiguredScene = false

    var body: some View {
        RealityView { content in
            if !hasConfiguredScene {
                configureScene()
                content.add(rootEntity)
                hasConfiguredScene = true
            }
            updateScene()
        } update: { _ in
            updateScene()
        }
        .gesture(
            SpatialTapGesture()
                .targetedToAnyEntity()
                .onEnded { value in
                    guard value.entity.name == "map-floor" else { return }
                    let point = value.convert(value.location3D, from: .local, to: value.entity.parent!)
                    let worldPoint = SIMD3<Float>(Float(point.x), Float(point.y), Float(point.z))
                    Task { await appModel.sendGroundTap(worldPoint: worldPoint) }
                }
        )
    }

    private func configureScene() {
        floorEntity = ModelEntity(
            mesh: .generateBox(size: [6.0, 0.02, 6.0]),
            materials: [SimpleMaterial(color: .init(white: 0.15, alpha: 0.55), isMetallic: false)]
        )
        floorEntity.name = "map-floor"
        floorEntity.components.set(CollisionComponent(shapes: [.generateBox(size: [6.0, 0.02, 6.0])]))
        floorEntity.components.set(InputTargetComponent())
        floorEntity.position = [0.0, -1.2, -1.5]

        robotEntity = ModelEntity(
            mesh: .generateSphere(radius: 0.12),
            materials: [SimpleMaterial(color: .cyan, isMetallic: false)]
        )
        robotEntity.name = "robot-marker"
        robotEntity.isEnabled = false

        goalEntity = ModelEntity(
            mesh: .generateSphere(radius: 0.09),
            materials: [SimpleMaterial(color: .orange, isMetallic: false)]
        )
        goalEntity.name = "goal-marker"
        goalEntity.isEnabled = false

        rootEntity.addChild(floorEntity)
        rootEntity.addChild(robotEntity)
        rootEntity.addChild(goalEntity)
    }

    private func updateScene() {
        if let alignment = appModel.spatial.alignment {
            floorEntity.position = alignment.worldOrigin + SIMD3<Float>(0.0, -0.01, 0.0)
        }

        if let alignment = appModel.spatial.alignment,
           let robotPose = appModel.backendState.robotPose {
            let worldPoint = alignment.worldPoint(for: robotPose.position.simdDouble)
            robotEntity.position = worldPoint + SIMD3<Float>(0.0, 0.12, 0.0)
            robotEntity.isEnabled = true
        } else {
            robotEntity.isEnabled = false
        }

        if let goalPoint = appModel.pendingGoalWorldPoint {
            goalEntity.position = goalPoint + SIMD3<Float>(0.0, 0.12, 0.0)
            goalEntity.isEnabled = true
        } else {
            goalEntity.isEnabled = false
        }

        floorEntity.isEnabled = appModel.canSendTapGoal
    }
}

#Preview(immersionStyle: .mixed) {
    ImmersiveView()
        .environment(AppModel())
}
