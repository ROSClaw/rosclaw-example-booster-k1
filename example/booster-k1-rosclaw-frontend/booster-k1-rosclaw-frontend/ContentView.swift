import SwiftUI

struct ContentView: View {
    @Environment(AppModel.self) private var appModel
    @Environment(\.openImmersiveSpace) private var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {
                header
                workflowSection
                connectionSection
                robotSection
                mappingSection
                agentActivitySection
                reportsSection
                controlSection
            }
            .padding(24)
        }
        .task {
            await appModel.startIfNeeded()
        }
        .safeAreaInset(edge: .bottom) {
            if appModel.immersiveSpaceState == .open,
               appModel.canMarkRobotAnchor || appModel.canRotatePreview || appModel.hasRobotAnchor || appModel.isAligned {
                alignmentDock
                    .padding(.horizontal, 24)
                    .padding(.bottom, 20)
            }
        }
    }

    private var workflowSection: some View {
        dashboardSection("Guided Setup") {
            Text(appModel.operatorHeadline)
                .font(.title2)
                .fontWeight(.semibold)

            Text(appModel.operatorDetail)
                .foregroundStyle(.secondary)

            ForEach(appModel.workflowSteps) { step in
                HStack(alignment: .top, spacing: 12) {
                    Image(systemName: iconName(for: step.status))
                        .foregroundStyle(color(for: step.status))
                        .frame(width: 22)

                    VStack(alignment: .leading, spacing: 4) {
                        Text(step.title)
                            .fontWeight(.semibold)
                        Text(step.detail)
                            .font(.footnote)
                            .foregroundStyle(.secondary)
                    }
                }
                .padding(.vertical, 2)
            }

            workflowActionRow

            if let point = appModel.pointerWorldPoint {
                Text(
                    String(
                        format: "Current reticle target: (%.2f, %.2f, %.2f)",
                        point.x,
                        point.y,
                        point.z
                    )
                )
                .font(.footnote)
                .foregroundStyle(.secondary)
            }

            if let point = appModel.robotAnchorWorldPoint {
                Text(
                    String(
                        format: "Robot anchor: (%.2f, %.2f, %.2f)  Yaw trim: %.0f°",
                        point.x,
                        point.y,
                        point.z,
                        appModel.yawCalibrationDegrees
                    )
                )
                .font(.footnote)
                .foregroundStyle(.secondary)
            }
        }
    }

    private var workflowActionRow: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack(spacing: 12) {
                if appModel.immersiveSpaceState != .open {
                    Button("Open Immersive Workspace") {
                        Task {
                            appModel.immersiveSpaceState = .inTransition
                            await openImmersiveSpace(id: appModel.immersiveSpaceID)
                        }
                    }
                    .buttonStyle(.borderedProminent)
                } else if appModel.canSendTapGoal {
                    Label("Aim right, left-pinch once", systemImage: "hand.tap")
                        .font(.footnote)
                        .foregroundStyle(.secondary)
                }

                if appModel.immersiveSpaceState == .open {
                    if appModel.hasRobotAnchor || appModel.isAligned {
                        Button("Reset Alignment") {
                            appModel.resetAlignmentCalibration()
                        }
                    }

                    Button("Close Immersive Workspace") {
                        Task {
                            appModel.immersiveSpaceState = .inTransition
                            await dismissImmersiveSpace()
                        }
                    }
                }
            }

            if appModel.immersiveSpaceState == .open,
               (appModel.canMarkRobotAnchor || (appModel.hasRobotAnchor && !appModel.isAligned)) {
                Text("Use the alignment dock at the bottom of this window for Mark Robot, Rotate, and Align while keeping the robot in view.")
                    .font(.footnote)
                    .foregroundStyle(.secondary)
            }
        }
    }

    private var alignmentDock: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text(alignmentDockTitle)
                .font(.headline)

            Text(alignmentDockDetail)
                .font(.footnote)
                .foregroundStyle(.secondary)

            HStack(spacing: 12) {
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
        }
        .padding(16)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 24, style: .continuous))
    }

    private var alignmentDockTitle: String {
        if appModel.canMarkRobotAnchor {
            return "Mark Robot Position"
        }
        if appModel.canRotatePreview {
            return "Preview Alignment"
        }
        if appModel.isAligned {
            return "Alignment Locked"
        }
        return "Alignment"
    }

    private var alignmentDockDetail: String {
        if appModel.canMarkRobotAnchor {
            return "Aim the right-hand ray at the real robot's feet, then left-pinch or press Mark Robot."
        }
        if appModel.canRotatePreview {
            return String(
                format: "Rotate the cyan preview until it sits on the real robot, then align. Current yaw trim %.0f°.",
                appModel.yawCalibrationDegrees
            )
        }
        if appModel.isAligned {
            return "The headset and robot map are aligned. Use Reset if you need to recalibrate."
        }
        return "Alignment controls are available while immersive mode is open."
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
            if appModel.backendState.nav2Ready == false {
                Text("Tap-to-move is blocked because the backend does not currently have a live `/navigate_to_pose` action server.")
                    .foregroundStyle(.orange)
                    .font(.footnote)
            }
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
            if let nav2Ready = appModel.backendState.nav2Ready {
                LabeledContent("Nav2 action", value: nav2Ready ? "ready" : "unavailable")
            }
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
            LabeledContent("World tracking", value: appModel.spatial.worldTrackingState)
            LabeledContent("Plane detection", value: appModel.spatial.planeDetectionState)
            LabeledContent("Hand tracking", value: appModel.spatial.handTrackingState)
            Text(appModel.floorReadinessDescription)
                .font(.footnote)
                .foregroundStyle(.secondary)

            Button(appModel.hasRobotAnchor ? "Reset Alignment" : "Mark Robot Position") {
                if appModel.hasRobotAnchor {
                    appModel.resetAlignmentCalibration()
                } else {
                    appModel.markRobotAnchorFromReticle()
                }
            }
            .buttonStyle(.borderedProminent)
            .disabled(!(appModel.canMarkRobotAnchor || appModel.hasRobotAnchor))
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

    private var agentActivitySection: some View {
        dashboardSection("OpenClaw Activity") {
            if appModel.backendState.agentActivity.isEmpty {
                Text("No OpenClaw decisions recorded yet.")
                    .foregroundStyle(.secondary)
            } else {
                ForEach(appModel.backendState.agentActivity.prefix(8)) { item in
                    VStack(alignment: .leading, spacing: 8) {
                        HStack(spacing: 8) {
                            Text(activityRoleLabel(item.role))
                                .font(.caption2)
                                .fontWeight(.bold)
                                .padding(.horizontal, 8)
                                .padding(.vertical, 4)
                                .background(activityRoleColor(item.role).opacity(0.18), in: Capsule())
                                .foregroundStyle(activityRoleColor(item.role))

                            Text(item.summary)
                                .fontWeight(.semibold)

                            Spacer()

                            Text(item.timestamp)
                                .font(.caption)
                                .foregroundStyle(.secondary)
                        }

                        if !item.detail.isEmpty {
                            Text(item.detail)
                                .font(.footnote)
                                .foregroundStyle(.secondary)
                        }

                        Text(activityStatusLabel(item.status))
                            .font(.caption)
                            .foregroundStyle(activityStatusColor(item.status))
                    }
                    .padding(.vertical, 6)
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

            Text("The right hand drives the floor ray target while the left-hand pinch confirms it. Mark the robot first, trim the preview until it lines up, align, then left-pinch once to send each move request.")
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

    private func iconName(for status: AppModel.WorkflowStepStatus) -> String {
        switch status {
        case .pending:
            return "circle"
        case .active:
            return "arrow.trianglehead.clockwise"
        case .complete:
            return "checkmark.circle.fill"
        }
    }

    private func color(for status: AppModel.WorkflowStepStatus) -> Color {
        switch status {
        case .pending:
            return .secondary
        case .active:
            return .orange
        case .complete:
            return .green
        }
    }

    private func activityRoleLabel(_ role: String) -> String {
        switch role.lowercased() {
        case "operator":
            return "USER"
        case "tool":
            return "TOOL"
        case "assistant":
            return "AGENT"
        default:
            return role.uppercased()
        }
    }

    private func activityRoleColor(_ role: String) -> Color {
        switch role.lowercased() {
        case "operator":
            return .orange
        case "tool":
            return .cyan
        default:
            return .green
        }
    }

    private func activityStatusLabel(_ status: String) -> String {
        switch status.lowercased() {
        case "success":
            return "Completed"
        case "error":
            return "Failed"
        case "pending":
            return "Pending"
        default:
            return status.capitalized
        }
    }

    private func activityStatusColor(_ status: String) -> Color {
        switch status.lowercased() {
        case "success":
            return .green
        case "error":
            return .red
        default:
            return .secondary
        }
    }
}

#Preview(windowStyle: .automatic) {
    ContentView()
        .environment(AppModel())
}
