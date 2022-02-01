
#include "calcgraph.h"

namespace calcgraph {

    void WorkState::add_to_queue(Work &work) {
        // note that the or-equals part of the check is important; if we
        // failed to calculate work this time then Work::id ==
        // WorkState::current_id, and we want to put the work back on the graph
        // queue for later evaluation.
        if (work.id <= current_id) {
            // process it next Graph()
            work.schedule(g);

            if (stats)
                stats->pushed_graph++;
        } else {
            // keep anything around that's going on the heap - we remove a
            // reference after popping them off the heap and eval()'ing them
            intrusive_ptr_add_ref(&work);

            q.push(&work);

            if (stats)
                stats->pushed_heap++;
        }
    }

    constexpr bool WorkQueueCmp::operator()(const Work *a,
                                            const Work *b) const {
        return a->id > b->id;
    }

    bool Graph::operator()(struct Stats *stats) {
        if (stats)
            *stats = EmptyStats;

        auto head = work_queue.exchange(&tombstone, std::memory_order_acq_rel);
        if (head == &tombstone)
            return false;

        WorkState work(*this, stats);
        Work *w = head;
        while (w != &tombstone) {
            // remove us from the work queue. Note that this is slightly
            // inefficient, as w could be put back on the Graph's work_queue
            // before it's been evaluated in this function call, and so is
            // needlessly evaluated a second time.
            Work *next = w->dequeue();

            work.q.push(w);
            if (stats)
                stats->queued++;

            w = next;
        }

        while (!work.q.empty()) {

            Work *w = work.q.top();
            work.q.pop();

            // remove any duplicates, we only need to
            // calculate things once.
            while (!work.q.empty() && work.q.top()->id == w->id) {
                intrusive_ptr_release(work.q.top());
                work.q.pop();
                if (stats)
                    stats->duplicates++;
            }

            work.current_id = w->id;
            w->eval(work);
            if (stats)
                stats->worked++;

            // finally finished with this Work - it's not on the Graph queue
            // or the heap
            intrusive_ptr_release(w);
        }

        return true;
    }

    void Work::schedule(Graph &g) {
        if (id == flags::DONT_SCHEDULE)
            return;

        // don't want work to be deleted while queued
        intrusive_ptr_add_ref(this);

        bool first_time = true;
        while (true) {
            std::uintptr_t current = next.load(std::memory_order_acquire);
            bool locked = current & flags::LOCK;

            if (first_time && (current & ~flags::LOCK)) {
                // we're already on the work queue, as we're pointing to a
                // non-zero pointer
                intrusive_ptr_release(this);
                return;
            }

            // add w to the queue by chaning its next pointer to point
            // to the head of the queue
            Work *head = g.work_queue.load(std::memory_order_acquire);
            if (!next.compare_exchange_weak(
                    current, reinterpret_cast<std::uintptr_t>(head) | locked)) {
                // next was updated under us, retry from the start
                continue;
            }

            if (g.work_queue.compare_exchange_weak(head, this)) {
                // success! but keep the intrustive reference active
                return;
            }

            // if we're here we pointed w::next to the head of the queue,
            // but something changed the queue before we could finish. The
            // next time round the loop we know current will not be nullptr,
            // so set a flag to skip the are-we-already-queued check.
            first_time = false;
        }
    }

    NodeBuilder<Always, SingleList> Graph::node() {
        return NodeBuilder<Always, SingleList>(*this);
    }

    void evaluate_repeatedly(Graph &g, std::atomic<bool> &stop) {
        while (!stop.load(std::memory_order_consume)) {
            while (g()) {
            }
            std::this_thread::yield();
        }
    }
}
