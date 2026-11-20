/**
 * @file parallel.hpp
 * @brief 使用共享任务调度器执行有界并行批次，并按输入顺序传播异常。
 */
#pragma once

#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

#include <algorithm>
#include <cstddef>
#include <exception>
#include <limits>
#include <vector>

namespace backend::detail {

/// 每个输入恰好执行一次；等待全部输入完成后重抛序号最小的任务异常。
/// workers 不大于 1 时在调用线程串行执行；结果由调用方按输入序号存放。
template <class Function>
void parallelFor(std::size_t count, std::size_t workers, Function function) {
    if (count == 0)
        return;
    std::vector<std::exception_ptr> errors(count);
    // 每项独立保存异常，避免任务组取消尚未执行的输入。
    const auto execute = [&](std::size_t index) {
        try {
            function(index);
        } catch (...) {
            errors[index] = std::current_exception();
        }
    };
    if (workers <= 1) {
        for (std::size_t index = 0; index < count; ++index)
            execute(index);
    } else {
        const auto concurrency =
            std::min({count, workers, static_cast<std::size_t>(std::numeric_limits<int>::max())});
        tbb::task_arena arena(static_cast<int>(concurrency));
        arena.execute([&] { tbb::parallel_for(std::size_t{0}, count, execute); });
    }
    for (const auto& error : errors) {
        if (error)
            std::rethrow_exception(error);
    }
}

}  // namespace backend::detail
