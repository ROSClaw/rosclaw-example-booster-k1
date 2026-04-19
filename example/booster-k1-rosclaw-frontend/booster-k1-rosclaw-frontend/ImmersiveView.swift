import RealityKit
import SwiftUI
import UIKit
import simd

struct ImmersiveView: View {
    @Environment(AppModel.self) private var appModel

    @State private var rootEntity = Entity()
    @State private var floorEntity = ModelEntity()
    @State private var operatorEntity = Entity()
    @State private var operatorBaseEntity = ModelEntity()
    @State private var operatorHeadingEntity = ModelEntity()
    @State private var robotEntity = Entity()
    @State private var robotBaseEntity = ModelEntity()
    @State private var robotHeadingEntity = ModelEntity()
    @State private var robotShadowEntity = ModelEntity()
    @State private var robotAnchorEntity = ModelEntity()
    @State private var pointerReticleEntity = Entity()
    @State private var pointerReticleDiskEntity = ModelEntity()
    @State private var pointerReticleCenterEntity = ModelEntity()
    @State private var pointerReticleColumnEntity = ModelEntity()
    @State private var pointerBeamGlowEntity = ModelEntity()
    @State private var pointerBeamCoreEntity = ModelEntity()
    @State private var pointerSourceEntity = ModelEntity()
    @State private var goalEntity = ModelEntity()
    @State private var hasConfiguredScene = false

    var body: some View {
        let scene = appModel.immersiveSceneSnapshot

        RealityView { content in
            if !hasConfiguredScene {
                configureScene()
                content.add(rootEntity)
                hasConfiguredScene = true
            }
            updateScene(with: scene)
        } update: { _ in
            updateScene(with: scene)
        }
        .simultaneousGesture(
            SpatialTapGesture()
                .targetedToAnyEntity()
                .onEnded { value in
                    guard value.entity.name == "map-floor",
                          appModel.pointerSource != .hand,
                          appModel.canSendTapGoal,
                          let worldPoint = appModel.pointerWorldPoint
                    else {
                        return
                    }

                    Task { await appModel.sendGroundTap(worldPoint: worldPoint) }
                }
        )
        .overlay(alignment: .top) {
            ImmersiveWorkflowHUD()
                .environment(appModel)
        }
    }

    private func configureScene() {
        floorEntity = ModelEntity(
            mesh: .generateBox(size: [10.0, 0.005, 10.0]),
            materials: [SimpleMaterial(color: .init(white: 1.0, alpha: 0.04), isMetallic: false)]
        )
        floorEntity.name = "map-floor"
        floorEntity.components.set(CollisionComponent(shapes: [.generateBox(size: [10.0, 0.02, 10.0])]))
        floorEntity.components.set(InputTargetComponent())
        floorEntity.position = [0.0, -1.2, -1.5]

        operatorEntity = Entity()
        operatorEntity.name = "operator-marker"
        operatorEntity.isEnabled = false

        operatorBaseEntity = ModelEntity(
            mesh: .generateCylinder(height: 0.015, radius: 0.16),
            materials: [SimpleMaterial(color: .systemOrange.withAlphaComponent(0.92), isMetallic: true)]
        )
        operatorBaseEntity.position = [0.0, 0.0075, 0.0]

        operatorHeadingEntity = ModelEntity(
            mesh: .generateBox(size: [0.34, 0.018, 0.05]),
            materials: [SimpleMaterial(color: .white, isMetallic: true)]
        )
        operatorHeadingEntity.position = [0.19, 0.018, 0.0]

        operatorEntity.addChild(operatorBaseEntity)
        operatorEntity.addChild(operatorHeadingEntity)

        robotEntity = Entity()
        robotEntity.name = "robot-marker"
        robotEntity.isEnabled = false

        robotShadowEntity = ModelEntity(
            mesh: .generateCylinder(height: 0.008, radius: 0.22),
            materials: [SimpleMaterial(color: .cyan.withAlphaComponent(0.35), isMetallic: false)]
        )
        robotShadowEntity.position = [0.0, 0.004, 0.0]

        robotBaseEntity = ModelEntity(
            mesh: .generateCylinder(height: 0.16, radius: 0.10),
            materials: [SimpleMaterial(color: .cyan.withAlphaComponent(0.95), isMetallic: true)]
        )
        robotBaseEntity.position = [0.0, 0.08, 0.0]

        robotHeadingEntity = ModelEntity(
            mesh: .generateBox(size: [0.38, 0.03, 0.06]),
            materials: [SimpleMaterial(color: .white.withAlphaComponent(0.95), isMetallic: true)]
        )
        robotHeadingEntity.position = [0.23, 0.06, 0.0]

        robotEntity.addChild(robotShadowEntity)
        robotEntity.addChild(robotBaseEntity)
        robotEntity.addChild(robotHeadingEntity)

        robotAnchorEntity = ModelEntity(
            mesh: .generateCylinder(height: 0.012, radius: 0.12),
            materials: [SimpleMaterial(color: .systemOrange.withAlphaComponent(0.90), isMetallic: true)]
        )
        robotAnchorEntity.name = "robot-anchor"
        robotAnchorEntity.isEnabled = false

        pointerReticleEntity = Entity()
        pointerReticleEntity.name = "pointer-reticle"
        pointerReticleEntity.isEnabled = false

        pointerReticleDiskEntity = ModelEntity(
            mesh: .generateCylinder(height: 0.004, radius: 0.18),
            materials: [SimpleMaterial(color: .systemMint.withAlphaComponent(0.22), isMetallic: false)]
        )
        pointerReticleDiskEntity.position = [0.0, 0.002, 0.0]

        pointerReticleCenterEntity = ModelEntity(
            mesh: .generateCylinder(height: 0.01, radius: 0.055),
            materials: [SimpleMaterial(color: .systemMint.withAlphaComponent(0.92), isMetallic: false)]
        )
        pointerReticleCenterEntity.position = [0.0, 0.005, 0.0]

        pointerReticleColumnEntity = ModelEntity(
            mesh: .generateCylinder(height: 0.20, radius: 0.014),
            materials: [SimpleMaterial(color: .white.withAlphaComponent(0.88), isMetallic: false)]
        )
        pointerReticleColumnEntity.position = [0.0, 0.10, 0.0]
        pointerReticleColumnEntity.orientation = simd_quatf(angle: .pi / 18.0, axis: SIMD3<Float>(1.0, 0.0, 0.0))

        pointerReticleEntity.addChild(pointerReticleDiskEntity)
        pointerReticleEntity.addChild(pointerReticleCenterEntity)
        pointerReticleEntity.addChild(pointerReticleColumnEntity)

        pointerBeamGlowEntity = ModelEntity(
            mesh: .generateCylinder(height: 1.0, radius: 0.005),
            materials: [SimpleMaterial(color: .systemMint.withAlphaComponent(0.38), isMetallic: false)]
        )
        pointerBeamGlowEntity.name = "pointer-beam-glow"
        pointerBeamGlowEntity.isEnabled = false

        pointerBeamCoreEntity = ModelEntity(
            mesh: .generateCylinder(height: 1.0, radius: 0.0024),
            materials: [SimpleMaterial(color: .white.withAlphaComponent(0.98), isMetallic: false)]
        )
        pointerBeamCoreEntity.name = "pointer-beam-core"
        pointerBeamCoreEntity.isEnabled = false

        pointerSourceEntity = ModelEntity(
            mesh: .generateSphere(radius: 0.034),
            materials: [SimpleMaterial(color: .white.withAlphaComponent(0.92), isMetallic: false)]
        )
        pointerSourceEntity.name = "pointer-source"
        pointerSourceEntity.isEnabled = false

        goalEntity = ModelEntity(
            mesh: .generateSphere(radius: 0.09),
            materials: [SimpleMaterial(color: .orange, isMetallic: false)]
        )
        goalEntity.name = "goal-marker"
        goalEntity.isEnabled = false

        rootEntity.addChild(floorEntity)
        rootEntity.addChild(operatorEntity)
        rootEntity.addChild(robotEntity)
        rootEntity.addChild(robotAnchorEntity)
        rootEntity.addChild(pointerBeamGlowEntity)
        rootEntity.addChild(pointerBeamCoreEntity)
        rootEntity.addChild(pointerSourceEntity)
        rootEntity.addChild(pointerReticleEntity)
        rootEntity.addChild(goalEntity)
    }

    private func updateScene(with scene: AppModel.ImmersiveSceneSnapshot) {
        if let floorCenter = scene.sceneFloorCenterWorldPoint {
            floorEntity.position = floorCenter
        }

        updateOperatorMarker(with: scene)
        updateRobotMarker(with: scene)
        updatePointerAppearance(with: scene)
        updatePointerPreview(with: scene)
        updateGoalMarker(with: scene)

        floorEntity.isEnabled = scene.immersiveSpaceState == .open && scene.hasFloorEstimate
    }

    private func updateOperatorMarker(with scene: AppModel.ImmersiveSceneSnapshot) {
        guard let operatorPoint = scene.sceneOperatorWorldPoint,
              let operatorYaw = scene.sceneOperatorYawRadians
        else {
            operatorEntity.isEnabled = false
            return
        }

        operatorEntity.position = operatorPoint
        operatorEntity.orientation = simd_quatf(angle: operatorYaw, axis: SIMD3<Float>(0.0, 1.0, 0.0))
        operatorEntity.isEnabled = true
    }

    private func updateRobotMarker(with scene: AppModel.ImmersiveSceneSnapshot) {
        if let worldPoint = scene.sceneRobotPreviewWorldPoint,
           let yawRadians = scene.sceneRobotPreviewYawRadians {
            robotEntity.position = worldPoint
            robotEntity.orientation = simd_quatf(
                angle: yawRadians,
                axis: SIMD3<Float>(0.0, 1.0, 0.0)
            )
            robotEntity.isEnabled = true
        } else {
            robotEntity.isEnabled = false
        }

        if let anchorPoint = scene.robotAnchorWorldPoint, !scene.isAligned {
            robotAnchorEntity.position = anchorPoint + SIMD3<Float>(0.0, 0.005, 0.0)
            robotAnchorEntity.isEnabled = true
        } else {
            robotAnchorEntity.isEnabled = false
        }
    }

    private func updatePointerPreview(with scene: AppModel.ImmersiveSceneSnapshot) {
        guard let pointerPoint = scene.pointerWorldPoint else {
            pointerReticleEntity.isEnabled = false
            pointerBeamGlowEntity.isEnabled = false
            pointerBeamCoreEntity.isEnabled = false
            pointerSourceEntity.isEnabled = false
            return
        }

        let beamTarget = pointerPoint + SIMD3<Float>(0.0, 0.10, 0.0)
        pointerReticleEntity.position = pointerPoint
        pointerReticleEntity.isEnabled = true

        if let beamOrigin = scene.pointerRayOriginWorldPoint {
            pointerSourceEntity.position = beamOrigin
            pointerSourceEntity.isEnabled = true
            setBeamTransform(pointerBeamGlowEntity, from: beamOrigin, to: beamTarget)
            setBeamTransform(pointerBeamCoreEntity, from: beamOrigin, to: beamTarget)
        } else {
            pointerBeamGlowEntity.isEnabled = false
            pointerBeamCoreEntity.isEnabled = false
            pointerSourceEntity.isEnabled = false
        }
    }

    private func updateGoalMarker(with scene: AppModel.ImmersiveSceneSnapshot) {
        if let goalPoint = scene.pendingGoalWorldPoint {
            goalEntity.position = goalPoint + SIMD3<Float>(0.0, 0.12, 0.0)
            goalEntity.isEnabled = true
        } else {
            goalEntity.isEnabled = false
        }
    }

    private func updatePointerAppearance(with scene: AppModel.ImmersiveSceneSnapshot) {
        let reticleColor: UIColor
        let beamGlowColor: UIColor
        let beamCoreColor: UIColor

        switch scene.pointerSource {
        case .hand:
            reticleColor = scene.isAligned ? .cyan : .systemOrange
            beamGlowColor = scene.isAligned ? .cyan : .systemOrange
            beamCoreColor = .white
        case .head:
            reticleColor = .systemYellow
            beamGlowColor = .systemOrange
            beamCoreColor = .white
        case nil:
            reticleColor = .systemGray3
            beamGlowColor = .systemGray4
            beamCoreColor = .systemGray2
        }

        pointerReticleDiskEntity.model?.materials = [SimpleMaterial(color: reticleColor.withAlphaComponent(0.18), isMetallic: false)]
        pointerReticleCenterEntity.model?.materials = [SimpleMaterial(color: reticleColor.withAlphaComponent(0.92), isMetallic: false)]
        pointerReticleColumnEntity.model?.materials = [SimpleMaterial(color: beamCoreColor.withAlphaComponent(0.88), isMetallic: false)]
        pointerBeamGlowEntity.model?.materials = [SimpleMaterial(color: beamGlowColor.withAlphaComponent(0.36), isMetallic: false)]
        pointerBeamCoreEntity.model?.materials = [SimpleMaterial(color: beamCoreColor.withAlphaComponent(0.98), isMetallic: false)]
        pointerSourceEntity.model?.materials = [SimpleMaterial(color: beamCoreColor.withAlphaComponent(0.92), isMetallic: false)]
    }

    private func setBeamTransform(_ beam: ModelEntity, from start: SIMD3<Float>, to end: SIMD3<Float>) {
        let delta = end - start
        let length = simd_length(delta)
        guard length > 0.001 else {
            beam.isEnabled = false
            return
        }

        beam.position = (start + end) * 0.5
        beam.scale = SIMD3<Float>(1.0, length, 1.0)
        beam.orientation = simd_quatf(from: SIMD3<Float>(0.0, 1.0, 0.0), to: simd_normalize(delta))
        beam.isEnabled = true
    }
}

private struct ImmersiveWorkflowHUD: View {
    @Environment(AppModel.self) private var appModel

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack(spacing: 10) {
                Text(appModel.operatorHeadline)
                    .font(.headline)
                if appModel.isSendingGoal {
                    ProgressView()
                        .controlSize(.small)
                }
            }

            Text(appModel.operatorDetail)
                .font(.subheadline)
                .foregroundStyle(.secondary)

            Divider()

            Text(appModel.floorReadinessDescription)
                .font(.footnote)
                .foregroundStyle(.secondary)

            Text(appModel.reticleDescription)
                .font(.footnote)
                .foregroundStyle(.secondary)

            Text("Fusion: \(appModel.fusionConfidenceDescription)")
                .font(.footnote)
                .foregroundStyle(.secondary)

            controls
        }
        .padding(16)
        .frame(maxWidth: 640, alignment: .leading)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 22, style: .continuous))
        .padding(.horizontal, 24)
        .padding(.top, 24)
    }

    @ViewBuilder
    private var controls: some View {
        HStack(spacing: 10) {
            if appModel.canMarkRobotAnchor {
                Button("Mark Robot") {
                    appModel.markRobotAnchorFromReticle()
                }
                .buttonStyle(.borderedProminent)
            }

            if appModel.canRotatePreview {
                Button("Rotate Left") {
                    appModel.adjustAlignmentYaw(byDegrees: -10.0)
                }

                Button("Rotate Right") {
                    appModel.adjustAlignmentYaw(byDegrees: 10.0)
                }

                Button("Align") {
                    Task { await appModel.alignHeadsetToRobotPose() }
                }
                .buttonStyle(.borderedProminent)
            }

            if appModel.hasRobotAnchor || appModel.isAligned {
                Button("Reset") {
                    appModel.resetAlignmentCalibration()
                }
            }
        }
        .font(.footnote)
    }
}

#Preview(immersionStyle: .mixed) {
    ImmersiveView()
        .environment(AppModel())
}
