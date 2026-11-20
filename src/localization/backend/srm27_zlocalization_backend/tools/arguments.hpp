/**
 * @file arguments.hpp
 * @brief 绑定地图工具的命令行选项、取值转换及缺值检查。
 */
#pragma once

#include <charconv>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace backend::tools::cli {
/// 按选项别名、取值数量和设置回调统一解析地图工具命令行。
class Arguments {
    using Names = std::initializer_list<const char*>;
    struct Option {
        int count;
        std::function<bool(const char* const*)> set;
    };
    std::unordered_map<std::string, Option> options_;
    /// 将一组选项别名绑定到相同的取值数量和设置操作。
    void bind(Names names, Option option) {
        for (const auto name : names)
            options_.emplace(name, option);
    }

    /// 完整读取有限数值并按目标类型检查范围，拒绝尾随字符及空取值。
    template <class T>
    static T number(std::string_view text) {
        if (!text.empty() && text.front() == '+') {
            text.remove_prefix(1);
            if (!text.empty() && text.front() == '-')
                throw std::invalid_argument("invalid numeric argument sign");
        }
        if (text.empty())
            throw std::invalid_argument("empty numeric argument");
        T value{};
        const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
        if (result.ec != std::errc{} || result.ptr != text.data() + text.size())
            throw std::invalid_argument("invalid numeric argument: " + std::string(text));
        if constexpr (std::is_floating_point_v<T>) {
            if (!std::isfinite(value))
                throw std::invalid_argument("numeric argument must be finite");
        }
        return value;
    }

public:
    /// 注册接收单个文本参数的选项及其设置回调。
    void add(Names names, std::function<bool(const std::string&)> setter) {
        bind(names, {1, [setter](const char* const* values) { return setter(values[0]); }});
    }
    /// 绑定字符串、布尔或数值选项，并在成功赋值后设置显式出现标记。
    template <class T>
    void value(Names names, T& target, std::initializer_list<bool*> flags = {}) {
        // 类型转换成功后再记录显式出现标记，供互斥模式与完整性检查使用。
        add(names, [&target, flags = std::vector<bool*>(flags)](const std::string& text) {
            if constexpr (std::is_same_v<T, std::string>)
                target = text;
            else if constexpr (std::is_same_v<T, bool>) {
                if (text == "true" || text == "1")
                    target = true;
                else if (text == "false" || text == "0")
                    target = false;
                else
                    return false;
            } else
                target = number<T>(text);
            for (auto flag : flags)
                *flag = true;
            return true;
        });
    }
    /// 绑定无需取值的开关，并可同时标记相关能力选项已经出现。
    void flag(Names names, bool& target, bool* related = nullptr) {
        bind(names, {0, [&target, related](const char* const*) {
                         target = true;
                         if (related)
                             *related = true;
                         return true;
                     }});
    }
    /// 绑定固定数量的浮点取值及其显式出现标记。
    void sequence(Names names, std::initializer_list<double*> fields,
                  std::initializer_list<bool*> flags) {
        // 按声明次序读取整组数值，完成后一次更新相关出现标记。
        bind(names, {static_cast<int>(fields.size()),
                     [fields = std::vector<double*>(fields),
                      flags = std::vector<bool*>(flags)](const char* const* values) {
                         std::vector<double> parsed;
                         parsed.reserve(fields.size());
                         for (std::size_t i = 0; i < fields.size(); ++i)
                             parsed.push_back(number<double>(values[i]));
                         for (std::size_t i = 0; i < fields.size(); ++i)
                             *fields[i] = parsed[i];
                         for (auto flag : flags)
                             *flag = true;
                         return true;
                     }});
    }
    /// 依次解析选项并统一报告未知名称、缺少取值及数值转换错误。
    bool parse(int argc, char** argv, bool& help) const {
        try {
            for (int i = 1; i < argc; ++i) {
                const std::string name = argv[i];
                if (name == "--help" || name == "-h") {
                    help = true;
                    return true;
                }
                const auto found = options_.find(name);
                if (found == options_.end()) {
                    std::cerr << "Unknown argument: " << name << "\n";
                    return false;
                }
                const auto& option = found->second;
                if (i + option.count >= argc) {
                    std::cerr << "Missing value for " << name << "\n";
                    return false;
                }
                if (!option.set(argv + i + 1))
                    return false;
                i += option.count;
            }
            return true;
        } catch (const std::exception& error) {
            std::cerr << "Invalid argument: " << error.what() << "\n";
            return false;
        }
    }
};
}  // namespace backend::tools::cli
