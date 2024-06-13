#include <iostream>
#include <vector>
#include <string>
#include <list>
#include <limits> // for numeric_limits
#include <set>
#include <utility> // for pair
#include <algorithm>
#include <iterator>
#include <chrono>
#include <thread>

typedef int vertex_t;
typedef double weight_t;

const weight_t max_weight = std::numeric_limits<double>::infinity();

struct neighbor {
    vertex_t target;
    weight_t weight;
    neighbor(vertex_t arg_target, weight_t arg_weight)
        : target(arg_target), weight(arg_weight) { }
};

typedef std::vector<std::vector<neighbor>> adjacency_list_t;

void DijkstraComputePaths(vertex_t source,
    const adjacency_list_t& adjacency_list,
    std::vector<weight_t>& min_distance,
    std::vector<vertex_t>& previous) {
    int n = adjacency_list.size();
    min_distance.clear();
    min_distance.resize(n, max_weight);
    min_distance[source] = 0;
    previous.clear();
    previous.resize(n, -1);
    std::set<std::pair<weight_t, vertex_t>> vertex_queue;
    vertex_queue.insert(std::make_pair(min_distance[source], source));

    while (!vertex_queue.empty()) {
        weight_t dist = vertex_queue.begin()->first;
        vertex_t u = vertex_queue.begin()->second;
        vertex_queue.erase(vertex_queue.begin());

        // Visit each edge exiting u
        const std::vector<neighbor>& neighbors = adjacency_list[u];
        for (std::vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
            neighbor_iter != neighbors.end();
            neighbor_iter++) {
            vertex_t v = neighbor_iter->target;
            weight_t weight = neighbor_iter->weight;
            weight_t distance_through_u = dist + weight;
            if (distance_through_u < min_distance[v]) {
                vertex_queue.erase(std::make_pair(min_distance[v], v));
                min_distance[v] = distance_through_u;
                previous[v] = u;
                vertex_queue.insert(std::make_pair(min_distance[v], v));
            }
        }
    }
}

std::list<vertex_t> DijkstraGetShortestPathTo(vertex_t vertex, const std::vector<vertex_t>& previous) {
    std::list<vertex_t> path;
    for (; vertex != -1; vertex = previous[vertex])
        path.push_front(vertex);
    return path;
}

void printGrid(const std::vector<std::string>& grid) {
    for (const auto& row : grid) {
        std::cout << row << std::endl;
    }
}

int main() {
    const int grid_height = 20;
    const int grid_width = 40;
    adjacency_list_t adjacency_list(grid_height * grid_width);

    // Create a grid with uniform weights
    for (int i = 0; i < grid_height; ++i) {
        for (int j = 0; j < grid_width; ++j) {
            int index = i * grid_width + j;
            if (i > 0) adjacency_list[index].push_back(neighbor(index - grid_width, 1));
            if (i < grid_height - 1) adjacency_list[index].push_back(neighbor(index + grid_width, 1));
            if (j > 0) adjacency_list[index].push_back(neighbor(index - 1, 1));
            if (j < grid_width - 1) adjacency_list[index].push_back(neighbor(index + 1, 1));
        }
    }

    int ax, ay, bx, by;
    std::cout << "Enter coordinates for point A (x y): ";
    std::cin >> ax >> ay;
    std::cout << "Enter coordinates for point B (x y): ";
    std::cin >> bx >> by;

    int start = ay * grid_width + ax;
    int end = by * grid_width + bx;

    std::vector<weight_t> min_distance;
    std::vector<vertex_t> previous;
    DijkstraComputePaths(start, adjacency_list, min_distance, previous);
    std::list<vertex_t> path = DijkstraGetShortestPathTo(end, previous);

    // Create the grid and mark the path
    std::vector<std::string> grid(grid_height, std::string(grid_width, '*'));
    grid[ay][ax] = 'A';
    grid[by][bx] = 'B';

    // Show the grid with the path updating
    for (vertex_t vertex : path) {
        int y = vertex / grid_width;
        int x = vertex % grid_width;

        if (vertex != start && vertex != end) {
            grid[y][x] = '/';
        }

        printGrid(grid);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "\033[H\033[J"; // Clear the console (Linux/Mac)
    }

    // Print the final grid
    printGrid(grid);

    return 0;
}
