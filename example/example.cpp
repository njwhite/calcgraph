#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <set>
#include <string>
#include <sys/socket.h>
#include <thread>

#include <calcgraph.h>

using clock_tick = std::chrono::time_point<std::chrono::high_resolution_clock>;
static const double EPSILON = 0.0001;

struct Vec2 final {
    double x;
    double y;
    Vec2(double x, double y) : x(x), y(y) {}

    inline double norm() const {
        return sqrt(this->x * this->x + this->y * this->y);
    }
    inline void resize(double to) {
        double n = this->norm();
        if (n != 0) {
            x *= to / n;
            y *= to / n;
        }
    }
    inline void cap(double to) {
        double n = this->norm();
        if (n > to) {
            x *= to / n;
            y *= to / n;
        }
    }
};

inline bool operator==(const Vec2 &lhs, const Vec2 &rhs) {
    return (std::abs(lhs.x - rhs.x) < EPSILON) &&
           (std::abs(lhs.y - rhs.y) < EPSILON);
}
inline bool operator!=(const Vec2 &lhs, const Vec2 &rhs) {
    return !(lhs == rhs);
}
inline Vec2 operator+(const Vec2 &lhs, const Vec2 &rhs) {
    return Vec2(lhs.x + rhs.x, lhs.y + rhs.y);
}
inline Vec2 &operator+=(Vec2 &lhs, const Vec2 &rhs) {
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    return lhs;
}
inline Vec2 &operator-=(Vec2 &lhs, const Vec2 &rhs) {
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
    return lhs;
}
inline Vec2 operator-(const Vec2 &lhs, const Vec2 &rhs) {
    return Vec2(lhs.x - rhs.x, lhs.y - rhs.y);
}

struct Agent final {
    uint64_t id;
    clock_tick updated;
    Vec2 loc;
    Vec2 velocity;
    Agent(uint64_t id, clock_tick updated, Vec2 loc, Vec2 velocity)
        : id(id), updated(updated), loc(loc), velocity(velocity) {}
};

inline bool operator==(const Agent &lhs, const Agent &rhs) {
    return (lhs.id == rhs.id) && (lhs.loc == rhs.loc) &&
           (lhs.velocity == rhs.velocity);
}
inline bool operator!=(const Agent &lhs, const Agent &rhs) {
    return !(lhs == rhs);
}

struct Counter final {
    uint64_t id;
    clock_tick updated;
    Counter(uint64_t id, clock_tick updated) : id(id), updated(updated) {}
};

using vec2p = std::shared_ptr<Vec2>;
using vec2p_vector = std::shared_ptr<std::vector<vec2p>>;
using agentp = std::shared_ptr<Agent>;
using agentp_vector = std::shared_ptr<std::vector<agentp>>;
using string = std::shared_ptr<std::string>;
using strings = std::shared_ptr<std::forward_list<string>>;
using counterp = std::shared_ptr<Counter>;

struct Scene final {
    /**
     * @brief The center of the cloud of agents; agents will flock
     * towards this
     */
    Vec2 target;

    /**
     * @brief Keep the flock moving in a common direction
     */
    Vec2 heading;
    agentp_vector agents;
    Scene(Vec2& target, Vec2& heading, agentp_vector agents)
        : target(target), heading(heading), agents(agents) {}
};

using scenep = std::shared_ptr<Scene>;

/**
 * @brief Port to listen on
 */
static const short PORT = 8080;

/**
 * @brief Global termination flag, so we can set it in signal handlers
 */
static std::atomic<bool> stop(false);

/**
 * @brief Size of the UDP datagram buffer
 */
static const int buffer_len = 4096;

static calcgraph::Graph g;

static std::ofstream output;

static const double MAX_SPEED = 3;
static const double MAX_ACCELERATION = 1;

/**
 * @brief dump location snapshots at 60fps
 */
static const double FRAME = 1.0 / 60.0;

scenep choose_target(const agentp_vector agents) {
    if (agents->empty()) {
        return std::shared_ptr<Scene>();
    }

    Vec2 target(0, 0);
    Vec2 heading(0, 0);
    double len = 0;

    for (const auto &agent : *agents) {
        if (agent) {
            target += agent->loc;
            heading += agent->velocity;
            len += 1;
        }
    }
    target.x /= len;
    target.y /= len;
    heading.x /= len;
    heading.y /= len;
    heading.resize(MAX_SPEED);

    return std::make_shared<Scene>(target, heading, agents);
}

agentp move_agent(const agentp agent, const scenep scene) {
    // https://betterprogramming.pub/boids-simulating-birds-flock-behavior-in-python-9fff99375118
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed =
        std::chrono::duration<double>(now - agent->updated).count();

    Vec2 acceleration(0, 0);
    if (scene) {
        acceleration += scene->heading - agent->velocity;

        Vec2 dir = scene->target - agent->loc;
        dir.resize(MAX_SPEED);
        Vec2 acc = dir - agent->velocity;
        acc.cap(MAX_ACCELERATION);
        acceleration += acc;

        Vec2 avoid(0, 0);
        size_t len = 0;
        for (const auto &other : *(scene->agents)) {
            if (other and agent->id != other->id) {
                Vec2 dist = other->loc - agent->loc;
                dist.resize(1); // unit vector
                avoid -= dist;
                len += 1;
            }
        }
        if (len > 0) {
            avoid.x /= len;
            avoid.y /= len;
            avoid.resize(MAX_SPEED);
            Vec2 acc = avoid - agent->velocity;
            acc.cap(MAX_ACCELERATION);
            acceleration += acc;
        }
    }

    Vec2 velocity = Vec2(agent->velocity);
    velocity += acceleration;
    velocity.cap(MAX_SPEED);

    return std::make_shared<Agent>(agent->id, now,
                                   Vec2(agent->loc.x + velocity.x * elapsed,
                                        agent->loc.y + velocity.y * elapsed),
                                   velocity);
}

counterp print_movements(const counterp previous,
                         const agentp_vector movements) {
    auto now = std::chrono::high_resolution_clock::now();
    if (!previous) {
        return std::make_shared<Counter>(0, now);
    }
    double elapsed =
        std::chrono::duration<double>(now - previous->updated).count();
    if (elapsed < FRAME || stop.load()) {
        return previous;
    }
    for (const auto &agent : *movements) {
        if (agent) {
            output << previous->id << "\t" << agent->id << "\t" << agent->loc.x
                   << "\t" << agent->loc.y << std::endl;
        }
    }
    output << std::flush;
    return std::make_shared<Counter>(previous->id + 1, now);
}

/**
 * @brief Set up a UDP socket and pass any (complete) received datagrams to the
 * Input.
 * @returns true iff the listening process started correctly
 */
bool listen_to_datagrams(calcgraph::Input<string> &&in) {
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        perror("socket");
        return false;
    }
    int oval = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &oval, sizeof(oval)) < 0) {
        perror("setsockopt SO_REUSEADDR");
        return false;
    }

    // set up a timeout so we check the "stop" flag once a second (to break
    // out
    // of the receive loop)
    struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        perror("setsockopt SO_RCVTIMEO");
        return false;
    }
    struct sockaddr_in myaddr = {.sin_family = AF_INET,
                                 .sin_port = htons(PORT),
                                 .sin_addr = {
                                     .s_addr = htonl(INADDR_ANY),
                                 }};
    if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
        perror("bind");
        return false;
    }

    char buffer[buffer_len];
    struct iovec iov = {.iov_base = buffer, .iov_len = buffer_len};
    struct msghdr msg = {.msg_iov = &iov, .msg_iovlen = 1};
    int byterecv;
    while (!stop.load()) {
        if ((byterecv = recvmsg(fd, &msg, 0)) < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK ||
                errno == EINPROGRESS || errno == EINTR) {
                continue; // probably timeout
            } else {
                perror("recvmsg");
                return false;
            }
        } else if (msg.msg_flags & MSG_TRUNC) {
            continue; // skip broken packets
        } else {
            in.append(g, string(new std::string(buffer, byterecv)));
        }
    }
    return true;
}

void install_sigint_handler() {
    auto handler = [](int sig) {
        std::cerr << "received signal " << sig << ", exiting" << std::endl;
        std::exit(sig);
    };
    signal(SIGINT, handler);
    signal(SIGTERM, handler);
    signal(SIGHUP, handler);
}

int main() {
    install_sigint_handler();
    output.open("output.csv");
    output << "period\tlabel\tx\ty" << std::endl << std::flush;

    std::thread t(calcgraph::evaluate_repeatedly, std::ref(g), std::ref(stop));

    auto scene =
        g.node()
            // take all the agents' locations as a single vector
            .variadic<agentp>()
            .connect(choose_target);

    auto printer =
        g.node()
            // so we don't wake ourselves up when changing the counter
            .propagate<calcgraph::Weak>()
            // we want to loop our output back into our input
            .unconnected<counterp>()
            // take all the agents' locations as a single vector
            .variadic<agentp>()
            .connect(print_movements);
    // connect our output to our 0-th (unconnected<counterp>) input
    printer->connect(printer->input<0>());

    auto dispatcher =
        g.node()
            // process all datatagrams receieved since the last
            // Node evaluation as a batch
            .accumulate(calcgraph::unconnected<string>())
            .connect(
                [&scene, &printer](strings msgs) {
                    for (auto msg : *msgs) {
                        // parse the UDP message
                        std::string s = *msg;
                        uint64_t id = std::stoi(s);
                        s = s.substr(s.find(" ") + 1);
                        double x = std::stod(s);
                        s = s.substr(s.find(" ") + 1);
                        double y = std::stod(s);
                        s = s.substr(s.find(" ") + 1);
                        double xv = std::stod(s);
                        s = s.substr(s.find(" ") + 1);
                        double yv = std::stod(s);
                        agentp new_agent = std::make_shared<Agent>(
                            id, std::chrono::high_resolution_clock::now(),
                            Vec2(x, y), Vec2(xv, yv));

                        auto processor = g.node()
                                             .initialize<agentp>(new_agent)
                                             .latest(scene.get())
                                             .connect(move_agent);

                        // feed my output (the agent's new location) into these
                        // outer nodes
                        processor->connect(processor->input<0>());
                        processor->connect(printer->variadic_add<1>());
                        processor->connect(scene->variadic_add<0>());
                    }
                    return nullptr;
                });

    if (!listen_to_datagrams(dispatcher->input<0>())) {
        stop.store(true);
    }

    t.join();
    output.close();
    return 0;
}