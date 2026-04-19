import Foundation

struct K1BackendClient {
    let baseURL: URL

    private let decoder: JSONDecoder
    private let encoder: JSONEncoder
    private let session: URLSession

    init(baseURL: URL, session: URLSession = .shared) {
        self.baseURL = baseURL
        self.session = session

        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        self.decoder = decoder

        let encoder = JSONEncoder()
        encoder.keyEncodingStrategy = .convertToSnakeCase
        self.encoder = encoder
    }

    nonisolated static func configuredFromEnvironment() -> K1BackendClient {
        let env = ProcessInfo.processInfo.environment
        let rawURL =
            env["K1_VISIONOS_BACKEND_URL"] ??
            env["OPENCLAW_VISIONOS_BACKEND_URL"] ??
            "http://192.168.2.126:8088"

        guard let baseURL = URL(string: rawURL) else {
            return K1BackendClient(baseURL: URL(string: "http://127.0.0.1:8088")!)
        }
        return K1BackendClient(baseURL: baseURL)
    }

    func fetchState() async throws -> BackendStateResponse {
        try await request(path: "/api/state", method: "GET", body: Optional<String>.none, timeout: 6.0)
    }

    func postAlignment(_ payload: AlignmentRequest) async throws -> CommandResponse {
        try await request(path: "/api/map/alignment", method: "POST", body: payload, timeout: 8.0)
    }

    func postObservation(_ payload: SpatialObservationUpload) async throws -> CommandResponse {
        try await request(path: "/api/map/observations", method: "POST", body: payload, timeout: 8.0)
    }

    func sendTapGoal(_ payload: TapGoalRequest) async throws -> CommandResponse {
        try await request(path: "/api/goals/tap", method: "POST", body: payload, timeout: 12.0)
    }

    func setAutonomy(_ payload: AutonomyToggleRequest) async throws -> CommandResponse {
        try await request(path: "/api/autonomy", method: "POST", body: payload, timeout: 12.0)
    }

    func requestSceneReport() async throws -> CommandResponse {
        try await request(path: "/api/report", method: "POST", body: Optional<String>.none, timeout: 15.0)
    }

    func stopRobot() async throws -> CommandResponse {
        try await request(path: "/api/stop", method: "POST", body: Optional<String>.none, timeout: 8.0)
    }

    private func request<Response: Decodable, Body: Encodable>(
        path: String,
        method: String,
        body: Body?,
        timeout: TimeInterval
    ) async throws -> Response {
        var request = URLRequest(url: baseURL.appending(path: path))
        request.httpMethod = method
        request.timeoutInterval = timeout
        request.setValue("application/json", forHTTPHeaderField: "Accept")

        if let body {
            request.httpBody = try encoder.encode(body)
            request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        }

        let (data, response): (Data, URLResponse)
        do {
            (data, response) = try await session.data(for: request)
        } catch let error as URLError where error.code == .timedOut {
            throw BackendClientError.timeout(baseURL)
        } catch let error as URLError {
            throw BackendClientError.transportFailure(error.localizedDescription)
        }

        guard let httpResponse = response as? HTTPURLResponse else {
            throw BackendClientError.invalidResponse
        }

        guard (200...299).contains(httpResponse.statusCode) else {
            let message =
                (try? decoder.decode(BackendErrorResponse.self, from: data).summary) ??
                (String(data: data, encoding: .utf8) ?? "HTTP \(httpResponse.statusCode)")
            throw BackendClientError.httpFailure(message)
        }

        return try decoder.decode(Response.self, from: data)
    }
}

private struct BackendErrorResponse: Decodable {
    var summary: String
}

enum BackendClientError: LocalizedError {
    case invalidResponse
    case httpFailure(String)
    case timeout(URL)
    case transportFailure(String)

    var errorDescription: String? {
        switch self {
        case .invalidResponse:
            return "Backend returned an invalid response."
        case .httpFailure(let message):
            return message
        case .timeout(let url):
            return "Timed out waiting for the backend at \(url.absoluteString). Make sure the bridge is reachable and avoid sending repeated move requests while OpenClaw is busy."
        case .transportFailure(let message):
            return message
        }
    }
}
