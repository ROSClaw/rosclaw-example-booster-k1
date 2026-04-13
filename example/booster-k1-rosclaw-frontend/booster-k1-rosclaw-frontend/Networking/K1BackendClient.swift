import Foundation

struct K1BackendClient {
    let baseURL: URL

    private let decoder: JSONDecoder
    private let encoder: JSONEncoder

    init(baseURL: URL) {
        self.baseURL = baseURL

        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        self.decoder = decoder

        let encoder = JSONEncoder()
        encoder.keyEncodingStrategy = .convertToSnakeCase
        self.encoder = encoder
    }

    static func configuredFromEnvironment() -> K1BackendClient {
        let env = ProcessInfo.processInfo.environment
        let rawURL =
            env["K1_VISIONOS_BACKEND_URL"] ??
            env["OPENCLAW_VISIONOS_BACKEND_URL"] ??
            "http://127.0.0.1:8088"

        guard let baseURL = URL(string: rawURL) else {
            return K1BackendClient(baseURL: URL(string: "http://127.0.0.1:8088")!)
        }
        return K1BackendClient(baseURL: baseURL)
    }

    func fetchState() async throws -> BackendStateResponse {
        try await request(path: "/api/state", method: "GET", body: Optional<String>.none)
    }

    func postAlignment(_ payload: AlignmentRequest) async throws -> CommandResponse {
        try await request(path: "/api/map/alignment", method: "POST", body: payload)
    }

    func postObservation(_ payload: SpatialObservationUpload) async throws -> CommandResponse {
        try await request(path: "/api/map/observations", method: "POST", body: payload)
    }

    func sendTapGoal(_ payload: TapGoalRequest) async throws -> CommandResponse {
        try await request(path: "/api/goals/tap", method: "POST", body: payload)
    }

    func setAutonomy(_ payload: AutonomyToggleRequest) async throws -> CommandResponse {
        try await request(path: "/api/autonomy", method: "POST", body: payload)
    }

    func requestSceneReport() async throws -> CommandResponse {
        try await request(path: "/api/report", method: "POST", body: Optional<String>.none)
    }

    func stopRobot() async throws -> CommandResponse {
        try await request(path: "/api/stop", method: "POST", body: Optional<String>.none)
    }

    private func request<Response: Decodable, Body: Encodable>(
        path: String,
        method: String,
        body: Body?
    ) async throws -> Response {
        var request = URLRequest(url: baseURL.appending(path: path))
        request.httpMethod = method
        request.setValue("application/json", forHTTPHeaderField: "Accept")

        if let body {
            request.httpBody = try encoder.encode(body)
            request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        }

        let (data, response) = try await URLSession.shared.data(for: request)
        guard let httpResponse = response as? HTTPURLResponse else {
            throw BackendClientError.invalidResponse
        }

        guard (200...299).contains(httpResponse.statusCode) else {
            let message = String(data: data, encoding: .utf8) ?? "HTTP \(httpResponse.statusCode)"
            throw BackendClientError.httpFailure(message)
        }

        return try decoder.decode(Response.self, from: data)
    }
}

enum BackendClientError: LocalizedError {
    case invalidResponse
    case httpFailure(String)

    var errorDescription: String? {
        switch self {
        case .invalidResponse:
            return "Backend returned an invalid response."
        case .httpFailure(let message):
            return message
        }
    }
}
