#include <iostream>
#include <vector>
#include <string>
#include <list>
#include <limits>
#include <set>
#include <utility>
#include <algorithm>
#include <iterator>
#include <chrono>
#include <thread>

using namespace std;
using namespace chrono;
using namespace this_thread;

typedef int vertex_t;
typedef double weight_t;

const weight_t max_weight = std::numeric_limits<double>::infinity();

struct neighbor {
    vertex_t target;
    weight_t weight;
    neighbor(vertex_t arg_target, weight_t arg_weight)
        : target(arg_target), weight(arg_weight) { }
};

typedef std::vector<std::vector<neighbor> > adjacency_list_t;

void DijkstraComputePaths(vertex_t source,
    const adjacency_list_t& adjacency_list,
    std::vector<weight_t>& min_distance,
    std::vector<vertex_t>& previous)
{
    int n = adjacency_list.size();
    min_distance.clear();
    min_distance.resize(n, max_weight);
    min_distance[source] = 0;
    previous.clear();
    previous.resize(n, -1);
    std::set<std::pair<weight_t, vertex_t> > vertex_queue;
    vertex_queue.insert(std::make_pair(min_distance[source], source));

    while (!vertex_queue.empty())
    {
        weight_t dist = vertex_queue.begin()->first;
        vertex_t u = vertex_queue.begin()->second;
        vertex_queue.erase(vertex_queue.begin());

        const std::vector<neighbor>& neighbors = adjacency_list[u];
        for (std::vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
            neighbor_iter != neighbors.end();
            neighbor_iter++)
        {
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

std::list<vertex_t> DijkstraGetShortestPathTo(
    vertex_t vertex, const std::vector<vertex_t>& previous)
{
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

int main()
{
    const int gridWidth = 40;
    const int gridHeight = 20;
    const int middleRow = gridHeight / 2;
    const int middleCol = gridWidth / 2;

    // dimenzije zida '+'
    const int plusWidth = 18;
    const int plusHeight = 10;

    adjacency_list_t adjacency_list(gridHeight * gridWidth);

    // Grid sa +-om
    vector<string> grid(gridHeight, string(gridWidth, '*'));

    for (int i = middleRow - plusHeight / 2; i <= middleRow + plusHeight / 2; ++i) {
        grid[i][middleCol] = '#';
    }

    for (int j = middleCol - plusWidth / 2; j <= middleCol + plusWidth / 2; ++j) {
        grid[middleRow][j] = '#';
    }


    for (int i = 0; i < gridHeight; ++i) {
        for (int j = 0; j < gridWidth; ++j) {
            int index = i * gridWidth + j;
            if (grid[i][j] == '#') continue;

            if (i > 0 && grid[i - 1][j] != '#')
                adjacency_list[index].push_back(neighbor(index - gridWidth, 1));
            if (i < gridHeight - 1 && grid[i + 1][j] != '#')
                adjacency_list[index].push_back(neighbor(index + gridWidth, 1));
            if (j > 0 && grid[i][j - 1] != '#')
                adjacency_list[index].push_back(neighbor(index - 1, 1));
            if (j < gridWidth - 1 && grid[i][j + 1] != '#')
                adjacency_list[index].push_back(neighbor(index + 1, 1));
        }
    }

    int ax, ay, bx, by;

    // koordinate
    cout << "Coordinates for point A (x y): ";
    cin >> ax >> ay;
    cout << endl;

    cout << "Coordinates for point B (x y): ";
    cin >> bx >> by;
    cout << endl;

    int start = ay * gridWidth + ax;
    int end = by * gridWidth + bx;

    grid[ay][ax] = 'A';
    grid[by][bx] = 'B';

    vector<weight_t> min_distance;
    vector<vertex_t> previous;
    DijkstraComputePaths(start, adjacency_list, min_distance, previous);
    list<vertex_t> path = DijkstraGetShortestPathTo(end, previous);

    // grid update
    for (vertex_t vertex : path) {
        int y = vertex / gridWidth;
        int x = vertex % gridWidth;
        if (vertex != start && vertex != end) {
            grid[y][x] = '/';
        }
        printGrid(grid);
        sleep_for(milliseconds(500));
        system("cls");
    }

    printGrid(grid);

    return 0;
}
