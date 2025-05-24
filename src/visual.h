#pragma once

#include "simulation.h"
#include <vector>
#include <string>
#include <iostream>
#include <map>
#include <iomanip>

// ANSI-цвета для терминала
const std::vector<std::string> colors = {
    "\033[31m", // красный (тусклый)
    "\033[32m", // зелёный
    "\033[33m", // жёлтый
    "\033[34m", // синий
    "\033[35m", // фиолетовый
    "\033[36m", // бирюзовый
    "\033[91m", // ярко-красный
    "\033[92m", // ярко-зелёный
    "\033[93m", // ярко-жёлтый
    "\033[94m", // ярко-синий
    "\033[0m"   // сброс
};

class Visualizer {
public:
    Visualizer(int width, int height,
           const std::vector<int>& pointIds,
           const std::vector<std::pair<int, int>>& pointCoords)
    : w(width), h(height) {
        for (size_t i = 0; i < pointIds.size(); ++i) {
            idMap[pointCoords[i]] = pointIds[i];
        }
    }


    void drawGrid(const std::vector<RobotFeature>& robots) {
        std::vector<std::vector<std::string>> grid(h, std::vector<std::string>(w, ". "));

        // Отметим активные точки номерами
        for (const auto& [pos, id] : idMap) {
            int x = pos.first;
            int y = pos.second;
            if (inBounds(x, y)) {
                grid[y][x] = std::to_string(id);
                if (grid[y][x].size() == 1) grid[y][x] += " ";
            }
        }

        // Прокрасим пути (используем тусклый цвет)
        for (const auto& r : robots) {
            std::string dullColor = getDullColor(r.id);
            for (const auto& cell : r.path) {
                int x = cell.first, y = cell.second;
                if (inBounds(x, y)) {
                    grid[y][x] = dullColor + "**" + colors.back();  // тусклый цветной путь
                }
            }
        }

        // Отметим роботов (яркий цвет)
        for (const auto& r : robots) {
            int x = r.position.first;
            int y = r.position.second;
            if (inBounds(x, y)) {
                std::string idStr = "R" + std::to_string(r.id);
                if (idStr.size() == 2) idStr += " ";
                grid[y][x] = getColor(r.id) + idStr + colors.back();
            }
        }

        // Вывод
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                std::cout << std::setw(3) << grid[y][x];
            }
            std::cout << '\n';
        }
        std::cout << "------------------------------\n";
    }

private:
    int w, h;
    std::map<std::pair<int, int>, int> idMap;

    // Яркие цвета — для роботов (индексы 6..9)
    std::string getColor(int robotId) const {
        return colors[6 + (robotId % 4)];
    }

    // Тусклые цвета — для путей (индексы 0..5)
    std::string getDullColor(int robotId) const {
        return colors[robotId % 6];
    }

    bool inBounds(int x, int y) const {
        return x >= 0 && y >= 0 && x < w && y < h;
    }
};
