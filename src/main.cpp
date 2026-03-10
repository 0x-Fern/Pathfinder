#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace {

// One cell coordinate inside the grid.
struct Position {
    // Zero-based row index.
    int row = 0;
    // Zero-based column index.
    int col = 0;

    // Defaulted equality keeps comparisons concise throughout the solver.
    auto operator==(const Position&) const -> bool = default;
};

// Full outcome of one A* run.
struct SearchResult {
    // True only when the goal was actually reached.
    bool path_found = false;
    // Ordered final path from start to goal.
    std::vector<Position> path;
    // Closed set used for visualization.
    std::vector<std::vector<bool>> closed_nodes;
    // Count of nodes permanently expanded by the algorithm.
    std::size_t explored_nodes = 0;
    // Count of queue insertions, useful for demo statistics.
    std::size_t queued_nodes = 0;
};

// Convert a signed grid coordinate into a vector index.
// The solver checks bounds before indexing, so this helper is only about making
// the conversion explicit and warning-free.
std::size_t toIndex(int value) {
    return static_cast<std::size_t>(value);
}

// Owns the parsed map and basic grid queries.
class Grid {
  public:
    // Load a grid from a text file.
    static Grid loadFromFile(const fs::path& path) {
        std::ifstream input(path);
        if (!input) {
            throw std::runtime_error("Unable to open grid file: " + path.string());
        }

        // The file is parsed into raw character rows first, then validated by
        // the private constructor.
        std::vector<std::string> rows;
        std::string line;
        while (std::getline(input, line)) {
            std::string sanitized;
            for (char ch : line) {
                // Allowing whitespace in the text file makes example grids much
                // easier to read and edit by hand. The parser strips spacing so
                // both `S..#E` and `S . . # E` are accepted.
                if (ch != ' ' && ch != '\t' && ch != '\r') {
                    sanitized.push_back(ch);
                }
            }

            if (!sanitized.empty()) {
                rows.push_back(std::move(sanitized));
            }
        }

        return Grid(std::move(rows));
    }

    [[nodiscard]] int rows() const {
        return static_cast<int>(cells_.size());
    }

    // Number of columns in each row.
    [[nodiscard]] int cols() const {
        return cells_.empty() ? 0 : static_cast<int>(cells_.front().size());
    }

    // Start cell marked with `S`.
    [[nodiscard]] const Position& start() const {
        return start_;
    }

    // Goal cell marked with `E`.
    [[nodiscard]] const Position& end() const {
        return end_;
    }

    // Bounds check for candidate moves.
    [[nodiscard]] bool inBounds(const Position& position) const {
        return position.row >= 0 && position.row < rows() && position.col >= 0 && position.col < cols();
    }

    // Walls are represented by `#`; everything else is walkable.
    [[nodiscard]] bool isWalkable(const Position& position) const {
        const char cell = cells_[toIndex(position.row)][toIndex(position.col)];
        return cell != '#';
    }

    // Read the raw map character at a position.
    [[nodiscard]] char at(const Position& position) const {
        return cells_[toIndex(position.row)][toIndex(position.col)];
    }

    [[nodiscard]] std::string render(const std::vector<std::vector<bool>>* path_marks,
                                     const std::vector<std::vector<bool>>* frontier_marks,
                                     const std::vector<std::vector<bool>>* closed_marks,
                                     std::optional<Position> current = std::nullopt) const {
        // Rendering returns a string instead of printing directly so the caller
        // can decide when and how to display it.
        std::ostringstream stream;

        for (int row = 0; row < rows(); ++row) {
            for (int col = 0; col < cols(); ++col) {
                const Position position{row, col};
                const char original = at(position);
                char display = original;

                // The overlay order matters. Showing the final path first keeps
                // the end result readable, while frontier/closed markers are
                // more useful during the search animation itself.
                if (original != 'S' && original != 'E' && original != '#') {
                    if (path_marks && (*path_marks)[toIndex(row)][toIndex(col)]) {
                        display = '*';
                    } else if (current && *current == position) {
                        display = '@';
                    } else if (frontier_marks && (*frontier_marks)[toIndex(row)][toIndex(col)]) {
                        display = 'o';
                    } else if (closed_marks && (*closed_marks)[toIndex(row)][toIndex(col)]) {
                        display = 'x';
                    }
                }

                stream << display;
                if (col + 1 < cols()) {
                    // Spaces make the grid easier to read in a terminal.
                    stream << ' ';
                }
            }
            stream << '\n';
        }

        return stream.str();
    }

  private:
    explicit Grid(std::vector<std::string> rows) : cells_(std::move(rows)) {
        if (cells_.empty()) {
            throw std::runtime_error("Grid cannot be empty.");
        }

        // Every row must have equal width so row/column indexing stays valid.
        const std::size_t expected_width = cells_.front().size();
        int start_count = 0;
        int end_count = 0;

        for (std::size_t row = 0; row < cells_.size(); ++row) {
            if (cells_[row].size() != expected_width) {
                throw std::runtime_error("Grid rows must all have the same width.");
            }

            for (std::size_t col = 0; col < cells_[row].size(); ++col) {
                const char cell = cells_[row][col];
                if (cell != '.' && cell != '#' && cell != 'S' && cell != 'E') {
                    throw std::runtime_error("Grid contains invalid character: " + std::string(1, cell));
                }

                if (cell == 'S') {
                    // Remember the unique starting location.
                    start_ = Position{static_cast<int>(row), static_cast<int>(col)};
                    ++start_count;
                } else if (cell == 'E') {
                    // Remember the unique goal location.
                    end_ = Position{static_cast<int>(row), static_cast<int>(col)};
                    ++end_count;
                }
            }
        }

        if (start_count != 1 || end_count != 1) {
            throw std::runtime_error("Grid must contain exactly one S and exactly one E.");
        }
    }

    // Original grid characters loaded from the file.
    std::vector<std::string> cells_;
    // Cached start position.
    Position start_{};
    // Cached goal position.
    Position end_{};
};

// Handles terminal output for search animation and final results.
class TerminalRenderer {
  public:
    explicit TerminalRenderer(int delay_ms) : delay_ms_(delay_ms) {}

    // Draw one intermediate search step.
    void drawSearchStep(const Grid& grid,
                        const std::vector<std::vector<bool>>& frontier_marks,
                        const std::vector<std::vector<bool>>& closed_marks,
                        const Position& current) const {
        // ANSI clear-home sequence redraws the grid in place for a lightweight
        // terminal animation without needing any GUI library.
        std::cout << "\033[2J\033[H";
        std::cout << "A* search in progress\n";
        std::cout << grid.render(nullptr, &frontier_marks, &closed_marks, current) << '\n';
        // A short delay makes the search visually understandable to a human.
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms_));
    }

    // Draw the solved path over the explored grid.
    void drawFinalState(const Grid& grid,
                        const std::vector<Position>& path,
                        const std::vector<std::vector<bool>>& closed_marks) const {
        // A separate overlay map keeps the render logic simple and makes it
        // obvious which cells belong to the final path.
        std::vector<std::vector<bool>> path_marks(
            static_cast<std::size_t>(grid.rows()), std::vector<bool>(static_cast<std::size_t>(grid.cols()), false));

        for (const Position& position : path) {
            path_marks[static_cast<std::size_t>(position.row)][static_cast<std::size_t>(position.col)] = true;
        }

        std::cout << "Path found:\n";
        std::cout << grid.render(&path_marks, nullptr, &closed_marks) << '\n';
    }

    // Draw the explored area when no path exists.
    void drawFailureState(const Grid& grid, const std::vector<std::vector<bool>>& closed_marks) const {
        std::cout << "No path found:\n";
        std::cout << grid.render(nullptr, nullptr, &closed_marks) << '\n';
    }

  private:
    // Milliseconds between animation frames.
    int delay_ms_ = 80;
};

// Implements the A* shortest-path search.
class AStarSolver {
  public:
    // Optional callback used by the terminal animation to observe each step.
    using StepCallback =
        std::function<void(const Position&, const std::vector<std::vector<bool>>&, const std::vector<std::vector<bool>>&)>;

    // Run A* on the given grid and return both the path and visualization data.
    [[nodiscard]] SearchResult solve(const Grid& grid, const StepCallback& on_step = {}) const {
        // One entry in the priority queue.
        struct QueueEntry {
            // Grid position represented by this state.
            Position position;
            // Exact cost from start to this node.
            int g_cost = 0;
            // Heuristic estimate from this node to the goal.
            int h_cost = 0;
            // Total priority used by A*: f = g + h.
            int f_cost = 0;
        };

        // Lower f-cost should come out first, so the comparator reverses the
        // default max-heap behavior of std::priority_queue.
        struct QueueCompare {
            bool operator()(const QueueEntry& left, const QueueEntry& right) const {
                if (left.f_cost == right.f_cost) {
                    // Tie-breaking toward smaller h-cost nudges the search
                    // slightly closer to the goal when priorities are equal.
                    return left.h_cost > right.h_cost;
                }
                return left.f_cost > right.f_cost;
            }
        };

        // Cache grid dimensions up front because they are used repeatedly.
        const int rows = grid.rows();
        const int cols = grid.cols();
        const int infinity = std::numeric_limits<int>::max();

        // `open_queue` is the frontier, `best_cost` stores the cheapest known
        // cost to each cell, and `parents` remembers how to reconstruct the
        // final route once the goal is reached.
        std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open_queue;
        std::vector<std::vector<int>> best_cost(
            static_cast<std::size_t>(rows), std::vector<int>(static_cast<std::size_t>(cols), infinity));
        std::vector<std::vector<std::optional<Position>>> parents(
            static_cast<std::size_t>(rows), std::vector<std::optional<Position>>(static_cast<std::size_t>(cols), std::nullopt));
        std::vector<std::vector<bool>> frontier_marks(
            static_cast<std::size_t>(rows), std::vector<bool>(static_cast<std::size_t>(cols), false));
        std::vector<std::vector<bool>> closed_marks(
            static_cast<std::size_t>(rows), std::vector<bool>(static_cast<std::size_t>(cols), false));

        const Position start = grid.start();
        const Position goal = grid.end();

        // `best_cost` records the shortest confirmed distance from the start to
        // each cell. `parents` is stored separately so the final path can be
        // reconstructed without keeping whole paths inside the priority queue.
        best_cost[static_cast<std::size_t>(start.row)][static_cast<std::size_t>(start.col)] = 0;
        open_queue.push(QueueEntry{start, 0, heuristic(start, goal), heuristic(start, goal)});
        frontier_marks[static_cast<std::size_t>(start.row)][static_cast<std::size_t>(start.col)] = true;

        SearchResult result;
        result.queued_nodes = 1;

        // Continue exploring until there are no more frontier nodes to try.
        while (!open_queue.empty()) {
            const QueueEntry current = open_queue.top();
            open_queue.pop();

            const std::size_t row = static_cast<std::size_t>(current.position.row);
            const std::size_t col = static_cast<std::size_t>(current.position.col);

            if (current.g_cost != best_cost[row][col]) {
                // The priority queue may still contain older, worse entries for
                // the same grid cell. Ignoring them keeps the search efficient
                // and makes the explored-node count reflect meaningful work.
                continue;
            }

            if (closed_marks[row][col]) {
                // Once a node is closed, the best path to it is already known.
                continue;
            }

            // Move this node from the frontier to the explored set.
            frontier_marks[row][col] = false;
            closed_marks[row][col] = true;
            ++result.explored_nodes;

            if (on_step) {
                // The callback is optional so non-animated runs pay almost no
                // extra complexity for the visualization feature.
                on_step(current.position, frontier_marks, closed_marks);
            }

            if (current.position == goal) {
                // Reaching the goal means the shortest path has been found.
                result.path_found = true;
                result.closed_nodes = closed_marks;
                result.path = reconstructPath(parents, start, goal);
                return result;
            }

            for (const Position& neighbor : neighbors(current.position)) {
                if (!grid.inBounds(neighbor) || !grid.isWalkable(neighbor)) {
                    // Ignore moves that leave the grid or hit a wall.
                    continue;
                }

                const std::size_t neighbor_row = static_cast<std::size_t>(neighbor.row);
                const std::size_t neighbor_col = static_cast<std::size_t>(neighbor.col);

                if (closed_marks[neighbor_row][neighbor_col]) {
                    // Closed neighbors already have their best path finalized.
                    continue;
                }

                // Every move in this grid costs exactly one step.
                const int tentative_g = current.g_cost + 1;
                if (tentative_g >= best_cost[neighbor_row][neighbor_col]) {
                    // Ignore routes that are not better than the best one seen.
                    continue;
                }

                // A* only keeps the best-known route to each node. That small
                // rule is what prevents the search from expanding the same cell
                // over and over with worse paths.
                parents[neighbor_row][neighbor_col] = current.position;
                best_cost[neighbor_row][neighbor_col] = tentative_g;

                const int h_cost = heuristic(neighbor, goal);
                open_queue.push(QueueEntry{neighbor, tentative_g, h_cost, tentative_g + h_cost});
                // Keeping a frontier overlay separate from the queue itself
                // makes visualization straightforward.
                frontier_marks[neighbor_row][neighbor_col] = true;
                ++result.queued_nodes;
            }
        }

        // If the queue empties, no route exists from S to E.
        result.closed_nodes = std::move(closed_marks);
        return result;
    }

  private:
    // Manhattan distance is the standard heuristic for 4-direction movement.
    [[nodiscard]] static int heuristic(const Position& left, const Position& right) {
        // Manhattan distance is admissible for 4-direction movement because it
        // never overestimates the number of steps needed when diagonal moves
        // are forbidden.
        return std::abs(left.row - right.row) + std::abs(left.col - right.col);
    }

    // Generate the four orthogonal neighbors of a cell.
    [[nodiscard]] static std::vector<Position> neighbors(const Position& position) {
        return {
            Position{position.row - 1, position.col},
            Position{position.row + 1, position.col},
            Position{position.row, position.col - 1},
            Position{position.row, position.col + 1},
        };
    }

    // Rebuild the final path by walking backward from goal to start.
    [[nodiscard]] static std::vector<Position> reconstructPath(
        const std::vector<std::vector<std::optional<Position>>>& parents, const Position& start, const Position& goal) {
        std::vector<Position> reversed_path;
        Position current = goal;

        // Start from the goal and follow parent pointers until the start is reached.
        reversed_path.push_back(current);
        while (!(current == start)) {
            const std::optional<Position>& parent =
                parents[static_cast<std::size_t>(current.row)][static_cast<std::size_t>(current.col)];
            if (!parent) {
                throw std::runtime_error("Path reconstruction failed even though the goal was reached.");
            }

            current = *parent;
            reversed_path.push_back(current);
        }

        // The path was collected goal-to-start, so flip it before returning.
        return std::vector<Position>(reversed_path.rbegin(), reversed_path.rend());
    }
};

// Print short usage text for the CLI.
void printUsage() {
    std::cout << "Usage: ./bin/pathfinder <grid-file> [--animate] [--delay-ms N]\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc < 2) {
            printUsage();
            return 1;
        }

        // The first argument is always the grid file path.
        fs::path grid_file = argv[1];
        // Animation is optional so the same executable works for both fast runs
        // and more visual demos.
        bool animate = false;
        int delay_ms = 80;

        for (int index = 2; index < argc; ++index) {
            const std::string argument = argv[index];
            if (argument == "--animate") {
                animate = true;
            } else if (argument == "--delay-ms") {
                if (index + 1 >= argc) {
                    throw std::runtime_error("--delay-ms requires a number.");
                }
                // The next CLI token becomes the animation frame delay.
                delay_ms = std::stoi(argv[++index]);
                if (delay_ms < 0) {
                    throw std::runtime_error("--delay-ms must be non-negative.");
                }
            } else {
                throw std::runtime_error("Unknown option: " + argument);
            }
        }

        // Load the grid and create the helper objects that operate on it.
        const Grid grid = Grid::loadFromFile(grid_file);
        const TerminalRenderer renderer(delay_ms);
        const AStarSolver solver;

        // The step callback is supplied only when animation is enabled.
        const SearchResult result = solver.solve(
            grid,
            animate ? AStarSolver::StepCallback([&](const Position& current,
                                                    const std::vector<std::vector<bool>>& frontier_marks,
                                                    const std::vector<std::vector<bool>>& closed_marks) {
                        renderer.drawSearchStep(grid, frontier_marks, closed_marks, current);
                    })
                    : AStarSolver::StepCallback{});

        if (!result.path_found) {
            // Even failed searches still have useful visualization data.
            renderer.drawFailureState(grid, result.closed_nodes);
            std::cout << "Explored nodes: " << result.explored_nodes << '\n';
            std::cout << "Queued states: " << result.queued_nodes << '\n';
            return 0;
        }

        // Successful searches print both the solved path and some metrics.
        renderer.drawFinalState(grid, result.path, result.closed_nodes);
        std::cout << "Path length: " << (result.path.empty() ? 0 : result.path.size() - 1) << '\n';
        std::cout << "Explored nodes: " << result.explored_nodes << '\n';
        std::cout << "Queued states: " << result.queued_nodes << '\n';
        return 0;
    } catch (const std::exception& error) {
        // Centralized error reporting keeps parsing and solver code cleaner.
        std::cerr << "pathfinder error: " << error.what() << '\n';
        return 1;
    }
}
