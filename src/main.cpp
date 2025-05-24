// #include "simulation.h"
#include "visual.h"
#include <chrono>
#include <thread>

int main() {
    int width = 30;
    int height = 20;
    int wait = 3;
    std::vector<std::pair<int, int>> robots = {{1,1}, {3,4}, {5,6}};
    std::vector<int> pointIds = {1,2,3,4,5,6,7,8};
    std::vector<std::pair<int, int>> pointPos = {{0,0},{29,19},{0,15},{15,19},{19,1},{10,0},{29,10},{15,0}};
    std::vector<std::pair<int, int>> tasks = {{1,2}, {3,4}, {5,6}, {7,8}};

    RuleRobot sim(width, height, wait, robots, pointIds, pointPos, tasks);
    Visualizer visualizer(width, height, pointIds, pointPos);

    // Цикл по шагам симуляции с визуализацией
    while (sim.hasTasksOrBusyRobots()) {
        sim.stepSimulation();  // выполняем один шаг симуляции
        visualizer.drawGrid(sim.getRobots());  // визуализируем текущих роботов

        std::this_thread::sleep_for(std::chrono::milliseconds(300));  // небольшой таймаут, чтобы видеть обновления
    }

    return 0;
}
