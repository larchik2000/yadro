#pragma once

#include "robot.h"
#include "grid.h"
#include <vector>
#include <queue>
#include <fstream>
#include <map>
#include <set>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <sstream>

struct RobotFeature {
    int id;
    std::pair<int, int> position;
    std::pair<int, int> target = {-1, -1};
    std::string status = "Свободен";
    int waitCounter = 0;
    int waitAtStart = 0;
    int waitAtEnd = 0;
    std::vector<std::pair<int, int>> path;
    int currentPathIndex = 0;
    int assignedTaskStart = -1;
    int assignedTaskEnd = -1;
};

class RuleRobot {
public:
    RuleRobot(int w, int h, int waitTicks,
              const std::vector<std::pair<int, int>>& robotPos,
              const std::vector<int>& pointIds,
              const std::vector<std::pair<int, int>>& posPoints,
              const std::vector<std::pair<int, int>>& tasks)
        : width(w), height(h), wait(waitTicks), points(pointIds), positions(posPoints), taskList(tasks) {

        for (size_t i = 0; i < robotPos.size(); ++i) {
            RobotFeature robot;
            robot.id = i;
            robot.position = robotPos[i];
            robots.push_back(robot);
        }

        // Инициализируем очередь задач
        for (auto& t : taskList)
            pendingTasks.push(t);
    }

    // Один шаг симуляции
    void stepSimulation() {
        assignTasks(pendingTasks);

        for (auto& robot : robots) {
            if (robot.status == "Свободен") continue;

            if (robot.currentPathIndex < robot.path.size()) {
                auto next = robot.path[robot.currentPathIndex];
                if (!isOccupied(next)) {
                    robot.position = next;
                    ++robot.currentPathIndex;
                }
            } else {
                if (robot.waitAtStart < wait && robot.target == getPointCoords(robot.assignedTaskStart)) {
                    robot.waitAtStart++;
                } else if (robot.waitAtStart >= wait && robot.target == getPointCoords(robot.assignedTaskStart)) {
                    robot.target = getPointCoords(robot.assignedTaskEnd);
                    robot.path = aStar(robot.position, robot.target);
                    robot.currentPathIndex = 0;
                } else if (robot.target == getPointCoords(robot.assignedTaskEnd)) {
                    if (robot.waitAtEnd < wait) {
                        robot.waitAtEnd++;
                    } else {
                        robot.status = "Свободен";
                        robot.assignedTaskStart = -1;
                        robot.assignedTaskEnd = -1;
                        robot.waitAtStart = 0;
                        robot.waitAtEnd = 0;
                        robot.path.clear();
                        robot.currentPathIndex = 0;
                        robot.target = {-1, -1};
                    }
                }
            }
        }
        tick++;
    }

    // Проверяем, есть ли ещё задачи или занятые роботы
    bool hasTasksOrBusyRobots() const {
        if (!pendingTasks.empty()) return true;
        for (const auto& r : robots) {
            if (r.status != "Свободен") return true;
        }
        return false;
    }

    // Получаем текущее состояние роботов для визуализации
    const std::vector<RobotFeature>& getRobots() const {
        return robots;
    }

private:
    int width, height, wait;
    int tick = 0;
    std::vector<int> points;
    std::vector<std::pair<int, int>> positions;
    std::vector<std::pair<int, int>> taskList;
    std::vector<RobotFeature> robots;
    std::queue<std::pair<int,int>> pendingTasks;
    bool anyRobotBusy() {
        for (auto& r : robots)
            if (r.status != "Свободен") return true;
        return false;
    }

    bool isOccupied(std::pair<int, int> pos) {
        for (auto& r : robots)
            if (r.position == pos) return true;
        return false;
    }

    std::pair<int, int> getPointCoords(int pointId) {
        for (size_t i = 0; i < points.size(); ++i) {
            if (points[i] == pointId) return positions[i];
        }
        return {-1, -1};
    }

    void assignTasks(std::queue<std::pair<int, int>>& taskQueue) {
        std::vector<int> freeRobots;
        for (size_t i = 0; i < robots.size(); ++i)
            if (robots[i].status == "Свободен") freeRobots.push_back(i);

        while (!taskQueue.empty() && !freeRobots.empty()) {
            auto [startId, endId] = taskQueue.front();
            std::pair<int, int> startCoord = getPointCoords(startId);
            int closest = -1;
            int minDist = INT_MAX;

            for (int idx : freeRobots) {
                int dist = std::abs(robots[idx].position.first - startCoord.first) +
                           std::abs(robots[idx].position.second - startCoord.second);
                if (dist < minDist) {
                    minDist = dist;
                    closest = idx;
                }
            }

            if (closest != -1) {
                auto& robot = robots[closest];
                robot.assignedTaskStart = startId;
                robot.assignedTaskEnd = endId;
                robot.status = "Выполняет задание";
                robot.target = startCoord;
                robot.path = aStar(robot.position, robot.target);
                robot.currentPathIndex = 0;
                freeRobots.erase(std::remove(freeRobots.begin(), freeRobots.end(), closest), freeRobots.end());
                taskQueue.pop();
            } else {
                break;
            }
        }
    }

    std::vector<std::pair<int, int>> aStar(std::pair<int, int> start, std::pair<int, int> goal) {
        std::vector<std::pair<int, int>> path;
        std::set<std::pair<int, int>> closed;
        std::map<std::pair<int, int>, std::pair<int, int>> cameFrom;
        std::map<std::pair<int, int>, int> gScore;
        auto cmp = [&](auto a, auto b) {
            return gScore[a] + heuristic(a, goal) > gScore[b] + heuristic(b, goal);
        };

        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, decltype(cmp)> open(cmp);
        gScore[start] = 0;
        open.push(start);

        std::vector<std::pair<int, int>> directions = {{0,1},{1,0},{0,-1},{-1,0}};

        while (!open.empty()) {
            auto current = open.top(); open.pop();
            if (current == goal) {
                while (cameFrom.count(current)) {
                    path.push_back(current);
                    current = cameFrom[current];
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            closed.insert(current);

            for (auto [dx, dy] : directions) {
                std::pair<int, int> neighbor = {current.first + dx, current.second + dy};
                if (neighbor.first < 0 || neighbor.second < 0 || neighbor.first >= width || neighbor.second >= height)
                    continue;
                if (closed.count(neighbor)) continue;
                int tentative = gScore[current] + 1;
                if (!gScore.count(neighbor) || tentative < gScore[neighbor]) {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentative;
                    open.push(neighbor);
                }
            }
        }

        return path;
    }

    int heuristic(std::pair<int, int> a, std::pair<int, int> b) {
        return std::abs(a.first - b.first) + std::abs(a.second - b.second);
    }
};

void writeLog(const std::string& filename) {
    std::ofstream log(filename);
    if (log) {
        log << "ПрИвет\n";
        log.close();
    }
}

void readLog(const std::string& filename) {
    std::ifstream log(filename);
    if (log) {
        std::string line;
        std::cout << "Log content:\n";
        while (std::getline(log, line)) {
            std::cout << line << '\n';
        }
    }
}
