import SwiftUI

struct ContentView: View {
    @Environment(AppModel.self) private var appModel
    @Environment(\.openImmersiveSpace) private var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {
                header
                connectionSection
                robotSection
                mappingSection
                reportsSection
                controlSection
            }
            .padding(24)
        }
        .task {
            await appModel.startIfNeeded()
        }
    }

    private var header: some View {
        VStack(alignment: .leading, spacing: 8) {
            Text("Booster K1 Spatial Operator")
                .font(.largeTitle)
                .fontWeight(.semibold)
            Text("Headset-guided mapping, tap-to-move, and autonomous roaming through the OpenClaw backend.")
                .foregroundStyle(.secondary)
            Text("Backend: \(appModel.backendURLDescription)")
                .font(.footnote)
                .foregroundStyle(.secondary)
        }
    }

    private var connectionSection: some View {
        dashboardSection("Connection") {
            LabeledContent("Gateway state", value: appModel.backendState.connectionState)
            LabeledContent("Intent", value: appModel.backendState.currentIntent)
            LabeledContent("Status", value: appModel.backendState.statusSummary)
            if let lastError = appModel.lastError {
                Text(lastError)
                    .foregroundStyle(.red)
                    .font(.footnote)
            }
        }
    }

    private var robotSection: some View {
        dashboardSection("Robot") {
            if let pose = appModel.backendState.robotPose {
                LabeledContent(
                    "Pose",
                    value: String(
                        format: "(%.2f, %.2f, %.2f) %@",
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        pose.frame
                    )
                )
                LabeledContent("Heading", value: String(format: "%.1f°", pose.yawRadians * 180.0 / .pi))
            } else {
                Text("Robot pose is not available yet.")
                    .foregroundStyle(.secondary)
            }
            LabeledContent("Autonomy mode", value: appModel.backendState.autonomy.mode)
            LabeledContent("Nav state", value: appModel.backendState.autonomy.navState)
            LabeledContent(
                "Last outcome",
                value: appModel.backendState.autonomy.lastOutcome.isEmpty ? "None" : appModel.backendState.autonomy.lastOutcome
            )
            if let goal = appModel.backendState.lastGoal {
                LabeledContent(
                    "Last goal",
                    value: String(
                        format: "(%.2f, %.2f, %.2f) %@",
                        goal.position.x,
                        goal.position.y,
                        goal.position.z,
                        goal.frame
                    )
                )
            }
        }
    }

    private var mappingSection: some View {
        dashboardSection("Mapping") {
            LabeledContent("Alignment", value: appModel.backendState.mapping.alignmentState)
            LabeledContent("Shared map", value: appModel.backendState.mapping.sharedMapStatus)
            LabeledContent("Uploaded observations", value: "\(appModel.backendState.mapping.observationCount)")
            LabeledContent("Floor points", value: "\(appModel.backendState.mapping.floorPointCount)")
            LabeledContent("Headset sensing", value: appModel.spatial.worldSensingState)

            Button("Align Headset To Robot Pose") {
                Task { await appModel.alignHeadsetToRobotPose() }
            }
            .buttonStyle(.borderedProminent)
            .disabled(!appModel.canAlignHeadset)
        }
    }

    private var reportsSection: some View {
        dashboardSection("Reports") {
            if appModel.backendState.reports.isEmpty {
                Text("No autonomous reports yet.")
                    .foregroundStyle(.secondary)
            } else {
                ForEach(appModel.backendState.reports.prefix(3)) { report in
                    VStack(alignment: .leading, spacing: 4) {
                        Text(report.summary)
                        Text(report.timestamp)
                            .font(.caption)
                            .foregroundStyle(.secondary)
                        if !report.labels.isEmpty {
                            Text(report.labels.joined(separator: ", "))
                                .font(.caption)
                                .foregroundStyle(.secondary)
                        }
                    }
                    .padding(.vertical, 4)
                }
            }
        }
    }

    private var controlSection: some View {
        dashboardSection("Controls") {
            HStack(spacing: 12) {
                Button("Refresh State") {
                    Task { await appModel.refreshState() }
                }
                .disabled(appModel.isRefreshing)

                Button(appModel.autonomyEnabled ? "Disable Autonomy" : "Enable Autonomy") {
                    Task { await appModel.toggleAutonomy() }
                }
                .disabled(appModel.isChangingAutonomy)

                Button("Request Report") {
                    Task { await appModel.requestSceneReport() }
                }
                .disabled(appModel.isRequestingReport)

                Button("Emergency Stop", role: .destructive) {
                    Task { await appModel.emergencyStop() }
                }
            }

            Button(appModel.immersiveSpaceState == .open ? "Close Immersive Workspace" : "Open Immersive Workspace") {
                Task {
                    if appModel.immersiveSpaceState == .open {
                        appModel.immersiveSpaceState = .inTransition
                        await dismissImmersiveSpace()
                    } else {
                        appModel.immersiveSpaceState = .inTransition
                        await openImmersiveSpace(id: appModel.immersiveSpaceID)
                    }
                }
            }
            .buttonStyle(.borderedProminent)

            Text(appModel.lastCommandSummary)
                .font(.footnote)
                .foregroundStyle(.secondary)

            Text("Tap-to-move is only enabled after alignment. In the immersive space, tap the ground plane to send a map-frame goal through OpenClaw.")
                .font(.footnote)
                .foregroundStyle(.secondary)
        }
    }

    private func dashboardSection<Content: View>(_ title: String, @ViewBuilder content: () -> Content) -> some View {
        VStack(alignment: .leading, spacing: 12) {
            Text(title)
                .font(.title3)
                .fontWeight(.semibold)
            content()
        }
        .padding(18)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 24, style: .continuous))
    }
}

#Preview(windowStyle: .automatic) {
    ContentView()
        .environment(AppModel())
}
