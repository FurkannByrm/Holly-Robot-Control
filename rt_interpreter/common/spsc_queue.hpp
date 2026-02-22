#pragma once
#include <atomic>
#include <vector>
#include <optional>

template <typename T, size_t Size>
class SPSCQueue {
public:
    static_assert((Size & (Size - 1)) == 0, "for masking speed)");

    SPSCQueue() : head(0), tail(0) {
        buffer.resize(Size);
    }

    // Producer
    bool push(const T& data) {
        size_t current_head = head.load(std::memory_order_relaxed);
        size_t next_head = (current_head + 1) & (Size - 1);

        if (next_head == tail.load(std::memory_order_acquire)) {
            return false; 
        }

        buffer[current_head] = data;
        head.store(next_head, std::memory_order_release);
        return true;
    }

    // Consumer 
    std::optional<T> pop() {
       const size_t current_tail = tail.load(std::memory_order_relaxed);

        if (current_tail == head.load(std::memory_order_acquire)) {
            return std::nullopt; 
        }

        T data = buffer[current_tail];
        tail.store((current_tail + 1) & (Size - 1), std::memory_order_release);
        return data;
    }

private:
    std::vector<T> buffer;

    alignas(64) std::atomic<size_t> head;
    alignas(64) std::atomic<size_t> tail;
};
