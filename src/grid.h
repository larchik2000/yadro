#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <vector>

std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n[");
    size_t end = s.find_last_not_of(" \t\r\n]");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}

// Функция загрузки .ini
std::map<std::string, std::string> loadConfig(const std::string& filename) {
    std::map<std::string, std::string> config;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open config file: " << filename << std::endl;
        return {};
    }
    std::string line;
    // std::ifstream file(filename);


    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        auto pos = line.find('=');
        if (pos == std::string::npos) continue;
        // std::string key = line.substr(0, pos);
        // std::string value = line.substr(pos + 1);
        std::string key = trim(line.substr(0, pos));
        std::string value = trim(line.substr(pos + 1));
        config[key] = value;

    }

    return config;
}

// чтения заданий
void loadTasks(const std::string& taskFilename) {
    std::ifstream taskFile(taskFilename);
    if (!taskFile.is_open()) {
        std::cerr << "Failed to open task file: " << taskFilename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(taskFile, line)) {
        std::istringstream iss(line);
        int startId, endId;
        if (iss >> startId >> endId) {
            std::cout << "Task: from " << startId << " to " << endId << "\n";
        }
    }
}

std::vector<int> parseIntList(const std::string& value) {
    std::vector<int> result;
    std::regex pattern(R"(\d+)");
    std::smatch match;
    std::string::const_iterator searchStart(value.cbegin());

    while (std::regex_search(searchStart, value.cend(), match, pattern)) {
        result.push_back(std::stoi(match[0]));
        searchStart = match.suffix().first;
    }

    return result;
}

std::vector<std::pair<int, int>> parsePositions(const std::string& value) {
    std::vector<std::pair<int, int>> positions;
    std::regex pattern(R"(\[(\d+),\s*(\d+)\])");
    std::smatch match;
    std::string::const_iterator searchStart(value.cbegin());

    while (std::regex_search(searchStart, value.cend(), match, pattern)) {
        int x = std::stoi(match[1]);
        int y = std::stoi(match[2]);
        positions.emplace_back(x, y);
        searchStart = match.suffix().first;
    }

    return positions;
}
void processTasks(const std::string& taskFile,
                  const std::vector<int>& pointIds,
                  const std::vector<std::pair<int, int>>& posPoints) {
    std::ifstream task(taskFile);
    if (!task.is_open()) {
        std::cerr << "Failed to open task file.\n";
        return;
    }

    std::string line;
    while (std::getline(task, line)) {
        std::istringstream iss(line);
        int startId, endId;
        if (iss >> startId >> endId) {
            std::cout << "Task: " << startId << " → " << endId << "\n";

            auto itStart = std::find(pointIds.begin(), pointIds.end(), startId);
            auto itEnd = std::find(pointIds.begin(), pointIds.end(), endId);

            if (itStart != pointIds.end() && itEnd != pointIds.end()) {
                int indexStart = std::distance(pointIds.begin(), itStart);
                int indexEnd = std::distance(pointIds.begin(), itEnd);

                auto [sx, sy] = posPoints[indexStart];
                auto [ex, ey] = posPoints[indexEnd];

                std::cout << "  Start at: (" << sx << ", " << sy << ")\n";
                std::cout << "  End at: (" << ex << ", " << ey << ")\n";
            } else {
                std::cout << "  Point ID not found in num_point.\n";
            }
        }
    }
}
