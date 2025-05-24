#include <string>
#include <vector>
#include <regex>

std::vector<std::pair<int, int>> parseRobotPositions(const std::string& value) {
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
