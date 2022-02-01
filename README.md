# CalcGraph {#mainpage}

A C++14 [dataflow programming](https://en.wikipedia.org/wiki/Dataflow_programming) framework, for organising logic in streaming application to minimise end-to-end calculation latency. This is designed for many frequently-updated inputs and allows you to trade off processing each input value against ensuring the application logic output reflects the most recent inputs.

This has several advantages:
- Application logic must be stateless (and so easy to unit-test), and any state maintained between invocations is explicitly managed.
- Parallelism for free: the calculation graph is built with lock-free primitives, so many threads can propagate changes through the graph without blocking or data races, and each node in the graph is guaranteed to only be executed by a single thread at once.
- Compile-time verification that the application logic is connected correctly, and all parameters to a piece of logic have been connected or explicitly ignored.

### Overview

Application logic is broken down into stateless functions, each of which is embedded in a graph "node". The node's "inputs" are connected to the function's parameters and the node's output is connected to the function's return value. The application logic also connects the nodes together (they form a directed cyclic graph), and is responsible for passing external data to the inputs of the relevant nodes. The graph can then be evaluated; all nodes whose inputs have changed have their functions invoked, and their return values are passed into the inputs of any connected nodes. This process continues recursively until there are no nodes left with unprocessed input values.

## Example: Flocking

The `example.cpp` file is an example of agents that exhibit [flocking behaviour](https://en.wikipedia.org/wiki/Flocking_(behavior)) based on code [from here](https://betterprogramming.pub/boids-simulating-birds-flock-behavior-in-python-9fff99375118). It listens on a UDP port for new agent locations, while moving existing agents around the 2D plane according to three rules: (1) head in a common direction (2) head towards the centre of the group (3) don't get too close to other agents. 

```c++
/**
 * @brief Calculate the group's comment heading and "centre of mass"
 *        only once per tick (rather than once per agent per tick).
 */
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

/**
 * @brief The logic for each agent that gets run on each tick. 
          Updates their new position and acceleration vector
          based on their previous state (`agent`) and the other
          agents (`scene`).
 */
agentp move_agent(const agentp agent, const scenep scene) {
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

/**
 * @brief Write the location of every agent out to a file
          once every FRAME seconds (e.g. 1/60 for 60fps). This
          file is read by the `example-run.py` script and fed 
          into the Manim library to visualise the agents.
 */
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

int main() {
    output.open("output.csv");
    output << "period\tlabel\tx\ty" << std::endl << std::flush;

    std::thread t(calcgraph::evaluate_repeatedly, std::ref(g), std::ref(stop));

    // set up the singleton nodes:

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

    // add a node that processes UDP packets
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

                        // add a new node to the graph to
                        // update this new agent,
                        auto processor = g.node()
                                             .initialize<agentp>(new_agent)
                                             .latest(scene.get())
                                             .connect(move_agent);

                        // feed tne output (the agent's new location)
						// into the singleton nodes
                        processor->connect(processor->input<0>());
                        processor->connect(printer->variadic_add<1>());
                        processor->connect(scene->variadic_add<0>());
                    }
                    return nullptr;
                });

    // block on the network interface on this thread
    // (thread `t` runs the graph in the background)
    // and feed any new packets onto the graph via the
    // `dispatcher` node's input.
    if (!listen_to_datagrams(dispatcher->input<0>())) {
        stop.store(true);
    }

    t.join();
    output.close();
    return 0;
}
```

The driver python `example.py` program uses [manim](https://github.com/ManimCommunity/manim) to run the simulation for 5s and then generate a video of the resulting behaviour:

```bash
manim -pql ./examples/example-run.py FlockingOutput
```

And here is the result:

![Flocking Output](FlockingOutput.gif)

## Getting Started

The `Graph` object contains the work queue of nodes that need evaluating due to changes in their inputs. It's also used to construct new nodes in the calculation graph; `Graph::node()` returns you a builder object that allows you to wrap an existing function and customise the input, propagation and output policies (described below).

### Connecting Logic

Blocks of logic are connected via the `Connectable` and `Input` interfaces. All graph nodes constructed using the builder implement `Connectable`, and all nodes have an `Node::input()` method (with the parameter number as a template parameter) that gives you an Input object. You can also push values into a Node directly using the `Input::append` method (which also takes the `Graph` as a parameter to the node that created the Input can be scheduled for re-evaluation). All Node objects are reference-counted (the builder returns a `boost::intrusive_ptr` to a newly-created Node, giving it an initial reference count of one) and `Input` objects hold a (counted) reference to their creating Node, so an Input can never outlive the Node it belongs to. Passing an `Input` to `Connectable::connect` stores the `Input` in the Connectable object, so a Node will never be deleted if it's still connected to anything (and similiarly, a node will be automatically deleted once it's no longer connected to anything, unless you keep additional `boost::intrusive_ptr`s to it).

### Evaluating the Graph's Work Queue

When new values are passed to a graph node via `Input::append`, the Node is scheduled on the graph's work queue. `Graph::operator()()` is a thread-safe method to remove all outstanding work from the queue, and evaluate the "dirty" nodes one by one in the order of their `Work::id` fields. After a dirty node has been evaluated, any connected inputs are always added to the `std::priority_queue` heap of nodes to evaluate (skipping duplicates; i.e. nodes that are already in the heap ready for evaluation), depending on the propagation policy. This node-by-node evaluation continues until the heap is empty, at which point `Graph::operator()()` returns. Now, cycles in the logic graph are expected, so to avoid entering an infinite loop the function only evaluates nodes in strictly monotonically-increasing order. If the next node on the heap has a lower or equal id to the node that was just evaluated, it is removed from the heap and put back on the Graph's work queue.

A helper method `evaluate_repeatedly` repeated calls `Graph::operator()()` on the graph passed as an argument, yielding if the run queue is empty. This method is designed for a dedicated thread to use so it can process graph updates as they come in without blocking, at the cost of fully-utilizing the core the thread is scheduled on.

### Input Policies

Each node in the calculation graph is responsible for storing its own input values. How they're stored, and how the (and which) values are passed to the node's function is determined by the input policy. Each argument to the function has its own independent input policy, and the initial value of the input (that will be passed to the node's function if no other input values have been receieved) is also configurable via the `NodeBuilder` object. The policies include:

- **Latest**, the most frequently-used policy, stores a single parameter value in an atomic variable. New values just replace the existing stored value, so the graph node's function only sees the most up-to-date value (and may not see every value that's ever been fed to the input). The are partial template specializations of the Latest policy to support `std::shared_ptr` and `boost::intrustive_ptr` values if you need to pass objects to the node's function that can't be stored in a `std::atomic` object. The following `NodeBuilder` arguments add a parameter to the builder object with a `Latest` policy:
    - **latest(Connectable*, initial = {})** adds a parameter and connects the parameter of any graph node that the builder creates to the given `Connectable` object. It also sets the parameter's initial value to the supplied value (or a default-constructed value, if not given).
    - **initialize(value)** adds a parameter with the given initial value, but doesn't connect the input to anything
    - **unconnected()** adds a parameter with a default-constructed initial value and doesn't connect the input to anything
- **Accumulate** is a policy that stores every new value in a lock-free single-linked list, and when the node's function is evaluated the current contents of the list is passed to the parameter as a `std::forward_list` args. As this is is thread-safe, the input can be connected to multiple sources, and all collected values are passed in the order they are received. To add a parameter with this policy to a `NodeBuilder` builder object, use the `accumulate(Connectable*)` function (optionally specifying a Connectable to wire the node up to when it's created).
- **Variadic** is for when you want to connect a variable number of inputs to the graph node, but want the values from those inputs to be passed to the node's function as a single `std::vector`. Specifying this policy (via `NodeBuilder::variadic()`) means the created nodes will have `variadic_add` and `variadic_remove` methods, which let you connect and disconnect values from the parameter after the node's constructed.

### Propagation Policies

Once a node's function is calculated, the node's propagation policy is used to determine whether to pass that value on to any inputs, and, if so, whether to schedule those inputs for re-evaluation by adding them to the `Graph`'s work queue. The policy must have two methods:

- **bool push_value(value)**: A Node stores its input values according to its parameters' input policies; this method is used by a node to determine whether to pass the value that it just calculated to the input policies of its connected nodes (the `Input`s that were stored in the node's output policy when they were passed to the node's `Connectable::connect` implementation).
- **bool notify()** If `push_value` returns true for a value it is passed to all the node's connected inputs - then `notify()` is called to determine whether to add those inputs to the graph's work queue for subsequent evaluation.

The propagation policy of a node can be changed using the `NodeBuilder::propagate()` parameterized method (before it's constructed), and available policies are:

- **Always** (the default) just returns true for both methods. It passes all values it sees to connected Inputs without any additional processing.
- **Weak** returns true for `push_value`, so passes every value it sees to the connected Inputs. However, it always returns false for `notify()`, so never schedules downstream nodes for recalcuation.
- **OnChange** is a more complex policy, and is used to coalesce duplicates to reduce the number of times downstream nodes are calculated (by assuming downstream logic is idempotent). It stores the last value the function evaluated to, and if immediately-following values are equal then `push_value` returns false and the duplicates are dropped. There's an partial specialization for `std::shared_ptr` that determines value equality based on the value pointed to (rather than just the `std::shared_ptr` object itself). 

### Output Policies

A node's output policy determines how connected Inputs are stored and how values from the function are passed through to them. They usually embed (one or more copies of) the node's propagation policy to help them make this decision. The policies are:

- **SingleList** (the default) stores the connected Inputs (including duplicates) in a `std::vector`. This means only the most recent value of the output is read by the downstream Node.
- **MultiValued** wraps another output policy (e.g. `MultiValued<SingleList>::type`), and when invoked iterates over the output of the node's function (using `std::begin` and `std::end`), passing each element iterated over to its nested output policy. This is useful when the node operates on a batch of data, but connected nodes only expect to process the data one-by-one.
- **Demultiplexed** is the most complex policy. It works with nodes whose functions output a `std::pair` of values, and treats the first element of each value as a key into the `std::unordered_map` of instances of the SingleList output policy it contains. It passes the second element of the pair to the output policy it finds - or if none exists it passes the whole pair to a separate SingleList policy instance for "unkeyed" items. The Node::connect and Node::disconnect functions delegate through to this "unkeyed" policy. Nodes templated with a Demultiplexed output policy also have a `keyed_output` method that takes a key and returns a Connectable object. The implementation of `keyed_output` looks up the given key in the policy's unordered_map, and if it doesn't find it it creates a new instance of the SingleList policy and adds it to the map. The Connectable object the method returns is connected to this SingleList policy, so passing an Input to its `Connectable::connect` or `Connectable::disconnect` methods adds or removes the Input from the SingleList's `std::vector`. This output policy is conceptually the inverse of a variadic input, and takes care only to schedule downstream nodes connected to keyed inputs (i.e. Inputs stored in the policy's unordered_map) if the node's function outputs a value with that key (so the calculation graph nodes attached to unrelated keys aren't needlessly recalculated).

All data structures in all the output policy implementations are guarded by the containing node's lock, so all modifications of these datastructures spin on the lock until it is free. This is enforced by the Node itself, so the implementations don't contain any locking logic themselves.

## Dependencies

- CalcGraph uses boost intrusive_ptr, a header-only smart pointer library.
- The tests use [cppunit](http://sourceforge.net/projects/cppunit) and [valgrind](http://valgrind.org). On fedora, use `dnf install cppunit-devel`.
- To build the documentation, you need [doxygen](http://www.stack.nl/~dimitri/doxygen) and [pdflatex](https://www.ctan.org/pkg/pdftex).

## Building

Calcgraph uses [cmake](https://cmake.org) as a build system, and uses [pkg-config](https://www.freedesktop.org/wiki/Software/pkg-config) to find the dependencies.

- To build the project, run `cmake . && make`
- To build without examples, run `cmake -D WITH_EXAMPLES=OFF . && make`

## Contributing

Set up the `clang-format` git filter used by the `.gitattributes` to invoke clang-format by running:

```
$ git config filter.clang-format.clean clang-format
$ git config filter.clang-format.smudge cat
```

The canonical version of the code is [hosted on Github](https://github.com/njwhite/calcgraph).
