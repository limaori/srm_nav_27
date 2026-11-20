/**
 * @file observation.hpp
 * @brief 定义单周期全局观测集合及独立于状态提交的统计接纳策略。
 */
#pragma once

#include <string>
#include <vector>

#include "backend/factors.hpp"

namespace backend {

/// 因子提交端使用的观测身份；共同统计策略不按来源分支。
enum class ObservationSource { PriorMap, Uwb };

/// 同一来源的线性化输入、诊断名称及接纳结果。
struct ObservationEntry {
    ObservationSource source;
    std::string label;
    InnovationTerm innovation;
    bool accepted = false;
};

/// 同周期证据对定位健康状态的影响，不直接修改状态机。
enum class ObservationOutcome { Unavailable, Accepted, Conflict };

struct ObservationConclusion {
    ObservationOutcome outcome = ObservationOutcome::Unavailable;
    bool had_conflict = false;
    std::string reason;
};

/// 仅在一次提交内存在的观测集合；来源适配与消费水位由调用方持有。
struct ObservationBatch {
    std::vector<ObservationEntry> entries;
    std::vector<std::string> accepted_evidence;
    std::vector<std::string> conflicts;
    std::vector<std::string> unavailable;

    /// 按输入顺序复制图接口所需的线性化项，保持统计结果与条目一一对应。
    std::vector<InnovationTerm> terms() const;
    /// 检查联合统计及逐项数量；不可用诊断不产生观测冲突。
    bool validate(const InnovationBatchAssessment& assessment);
    /// 在 validate 成功且来源消费已完成后归约接纳结果与诊断。
    void classify(const InnovationBatchAssessment& assessment, double rejection_probability);
    /// 汇总接受、冲突和不可用证据，供唯一提交端推进状态机。
    ObservationConclusion conclusion() const;
};

}  // namespace backend
