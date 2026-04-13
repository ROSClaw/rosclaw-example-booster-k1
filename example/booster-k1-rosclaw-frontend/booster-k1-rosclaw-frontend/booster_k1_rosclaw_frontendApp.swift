//
//  booster_k1_rosclaw_frontendApp.swift
//  booster-k1-rosclaw-frontend
//
//  Created by Marcus Arnett on 4/13/26.
//

import SwiftUI

@main
struct booster_k1_rosclaw_frontendApp: App {

    @State private var appModel = AppModel()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environment(appModel)
                .task {
                    await appModel.startIfNeeded()
                }
        }

        ImmersiveSpace(id: appModel.immersiveSpaceID) {
            ImmersiveView()
                .environment(appModel)
                .onAppear {
                    appModel.immersiveSpaceState = .open
                }
                .onDisappear {
                    appModel.immersiveSpaceState = .closed
                }
        }
        .immersionStyle(selection: .constant(.mixed), in: .mixed, .full)
    }
}
