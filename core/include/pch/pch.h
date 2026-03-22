#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <execution>

#if defined(__APPLE__) && defined(__clang__)
#include <future>
#include <thread>
#include <vector>

// Polyfill for missing std::execution policies on macOS with rudimentary parallelism
namespace std
{
    namespace execution
    {
        struct sequenced_policy
        {
        };
        struct parallel_policy
        {
        };
        struct parallel_unsequenced_policy
        {
        };
        inline constexpr sequenced_policy seq;
        inline constexpr parallel_policy par;
        inline constexpr parallel_unsequenced_policy par_unseq;
    } // namespace execution

    template<class ExecutionPolicy, class RandomAccessIterator, class UnaryFunction>
    void for_each(ExecutionPolicy&&, RandomAccessIterator first, RandomAccessIterator last, UnaryFunction f)
    {
        auto length = std::distance(first, last);
        if (length < 2048)
        { // Threshold for parallelism
            std::for_each(first, last, f);
            return;
        }

        unsigned int nb_threads = std::thread::hardware_concurrency();
        if (nb_threads == 0)
            nb_threads = 4;

        // Avoid creating more threads than elements
        if (static_cast<unsigned int>(length) < nb_threads)
            nb_threads = static_cast<unsigned int>(length);

        auto block_size = length / nb_threads;
        std::vector<std::future<void>> futures;
        futures.reserve(nb_threads);

        auto block_start = first;
        for (unsigned int i = 0; i < nb_threads; ++i)
        {
            auto block_end = (i == nb_threads - 1) ? last : (block_start + block_size);

            futures.push_back(
                std::async(
                    std::launch::async,
                    [block_start, block_end, f]()
                    {
                        std::for_each(block_start, block_end, f);
                    }));

            block_start = block_end;
        }

        for (auto& fut : futures)
            fut.get();
    }

    template<class ExecutionPolicy, class ForwardIt1, class ForwardIt2, class UnaryOperation>
    ForwardIt2 transform(ExecutionPolicy&&, ForwardIt1 first1, ForwardIt1 last1, ForwardIt2 d_first, UnaryOperation unary_op)
    {
        // Assumption: Random access iterators or cheap std::distance
        auto length = std::distance(first1, last1);
        if (length < 2048)
        {
            return std::transform(first1, last1, d_first, unary_op);
        }

        unsigned int nb_threads = std::thread::hardware_concurrency();
        if (nb_threads == 0)
            nb_threads = 4;
        if (static_cast<unsigned int>(length) < nb_threads)
            nb_threads = static_cast<unsigned int>(length);

        auto block_size = length / nb_threads;
        std::vector<std::future<void>> futures;
        futures.reserve(nb_threads);

        auto src_start = first1;
        auto dst_start = d_first;

        for (unsigned int i = 0; i < nb_threads; ++i)
        {
            auto src_end = (i == nb_threads - 1) ? last1 : (src_start + block_size);
            auto dist = std::distance(src_start, src_end); // Should be fast for vector iterators

            futures.push_back(
                std::async(
                    std::launch::async,
                    [src_start, src_end, dst_start, unary_op]()
                    {
                        std::transform(src_start, src_end, dst_start, unary_op);
                    }));

            src_start = src_end;
            std::advance(dst_start, dist);
        }

        for (auto& fut : futures)
            fut.get();
        return d_first + length;
    }

    template<class ExecutionPolicy, class ForwardIt1, class ForwardIt2, class ForwardIt3, class BinaryOperation>
    ForwardIt3 transform(
        ExecutionPolicy&&, ForwardIt1 first1, ForwardIt1 last1, ForwardIt2 first2, ForwardIt3 d_first, BinaryOperation binary_op)
    {
        auto length = std::distance(first1, last1);
        if (length < 2048)
        {
            return std::transform(first1, last1, first2, d_first, binary_op);
        }

        unsigned int nb_threads = std::thread::hardware_concurrency();
        if (nb_threads == 0)
            nb_threads = 4;
        if (static_cast<unsigned int>(length) < nb_threads)
            nb_threads = static_cast<unsigned int>(length);

        auto block_size = length / nb_threads;
        std::vector<std::future<void>> futures;
        futures.reserve(nb_threads);

        auto src1_start = first1;
        auto src2_start = first2;
        auto dst_start = d_first;

        for (unsigned int i = 0; i < nb_threads; ++i)
        {
            auto src1_end = (i == nb_threads - 1) ? last1 : (src1_start + block_size);
            auto dist = std::distance(src1_start, src1_end);

            futures.push_back(
                std::async(
                    std::launch::async,
                    [src1_start, src1_end, src2_start, dst_start, binary_op]()
                    {
                        std::transform(src1_start, src1_end, src2_start, dst_start, binary_op);
                    }));

            src1_start = src1_end;
            std::advance(src2_start, dist);
            std::advance(dst_start, dist);
        }

        for (auto& fut : futures)
            fut.get();
        return d_first + length;
    }
} // namespace std
#endif

#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <typeindex>
#include <variant>
#include <vector>