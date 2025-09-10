#ifndef PR_PALETTE_DEBUG_HPP
#define PR_PALETTE_DEBUG_HPP

// #include <algorithm>
#include <ros/console.h>

#include <cstring>
// #include <sstream>
// #include <vector>

// ANSI color definitions
#define ANSI_COLOR_RED "\033[0;31m"
#define ANSI_COLOR_GREEN "\033[0;32m"
#define ANSI_COLOR_YELLOW "\033[0;33m"
#define ANSI_COLOR_BLUE "\033[0;34m"
#define ANSI_COLOR_MAGENTA "\033[0;35m"
#define ANSI_COLOR_CYAN "\033[0;36m"
#define ANSI_COLOR_WHITE "\033[0;37m"
#define ANSI_COLOR_RESET ANSI_COLOR_GREEN

// Color stream
#define RED(x) ANSI_COLOR_RED << x << ANSI_COLOR_RESET
#define GREEN(x) ANSI_COLOR_GREEN << x << ANSI_COLOR_RESET
#define YELLOW(x) ANSI_COLOR_YELLOW << x << ANSI_COLOR_RESET
#define BLUE(x) ANSI_COLOR_BLUE << x << ANSI_COLOR_RESET
#define MAGENTA(x) ANSI_COLOR_MAGENTA << x << ANSI_COLOR_RESET
#define CYAN(x) ANSI_COLOR_CYAN << x << ANSI_COLOR_RESET
#define WHITE(x) ANSI_COLOR_WHITE << x << ANSI_COLOR_RESET
//
// inline std::string prettyName(const std::string &prettyFunction) {
//   return prettyFunction.substr();
// }
// #define __FUNC_NAME__ prettyName(__PRETTY_FUNCTION__)
//
// // Вспомогательная печать одной пары имя-значение
// inline bool is_valid_var_name(const std::string &name) {
//   if (name.empty())
//     return false;
//   if (!(std::isalpha(name[0]) || name[0] == '_'))
//     return false;
//   return std::all_of(name.begin() + 1, name.end(),
//                      [](char c) { return std::isalnum(c) || c == '_'; });
// }
//
// // Вспомогательная печать одной пары имя-значение
// template <typename T>
// inline void debug_print_single(std::ostringstream &oss, const std::string
// &name,
//                                T &&value) {
//   if (is_valid_var_name(name))
//     oss << YELLOW(name << "=" << std::forward<T>(value));
//   else
//     oss << YELLOW(std::forward<T>(value));
// }
//
// inline void debug_print_impl(std::ostringstream &oss,
//                              const std::vector<std::string> & /*names*/,
//                              size_t /*idx*/) {}
//
// template <typename T, typename... Args>
// inline void debug_print_impl(std::ostringstream &oss,
//                              const std::vector<std::string> &names, size_t
//                              idx, T &&value, Args &&...args) {
//   debug_print_single(oss, names[idx], std::forward<T>(value));
//   if (sizeof...(args) > 0) {
//     oss << ", ";
//     debug_print_impl(oss, names, idx++, std::forward<Args>(args)...);
//   }
// }
//
// inline std::vector<std::string> parse_names(std::string &names_str) {
//   std::replace(names_str.begin(), names_str.end(), ',', ' ');
//   std::istringstream iss(names_str);
//   std::vector<std::string> names_;
//   std::string name_;
//   while (iss >> name_)
//     names_.emplace_back(name_);
//   return names_;
// }
//
// template <typename... Args>
// inline void debug_print_args(std::ostringstream &oss, std::string names_str,
//                              Args &&...args) {
//   debug_print_impl(oss, parse_names(names_str), 0,
//   std::forward<Args>(args)...);
// }
//
// template <typename OutFunc, typename... Args>
// inline void log_vars(const std::string func_name, OutFunc outfunc,
//                      const std::string names, Args &&...args) {
//   std::ostringstream oss;
//   if (func_name != "")
//     oss << "[" << MAGENTA(func_name) << "] : ";
//   debug_print_args(oss, names, args...);
//   outfunc(oss.str());
// }
// // Non-void return type version
// template <typename Func, typename OutFunc, typename... Args>
// inline auto log_func(const std::string func_name, Func func, OutFunc outfunc,
//                      const std::string names, Args &&...args) ->
//     typename std::enable_if<
//         !std::is_void<typename std::result_of<Func(Args
//         &&...)>::type>::value, typename std::result_of<Func(Args
//         &&...)>::type>::type {
//   std::ostringstream oss;
//   auto result_ = func(std::forward<Args>(args)...);
//
//   oss << "[" << MAGENTA(func_name) << "](";
//   debug_print_args(oss, names, args...);
//   oss << ") : " << WHITE(result_);
//   outfunc(oss.str());
//
//   return result_;
// }
//
// // Void return type version
// template <typename Func, typename OutFunc, typename... Args>
// inline typename std::enable_if<
//     std::is_void<typename std::result_of<Func(Args &&...)>::type>::value,
//     void>::type
// log_func(const std::string func_name, Func func, OutFunc outfunc,
//          const std::string names, Args &&...args) {
//   std::ostringstream oss;
//   oss << "[" << MAGENTA(func_name) << "](";
//   debug_print_args(oss, names, args...);
//   oss << ") : " << WHITE("(void)");
//   outfunc(oss.str());
// }
//
// #define LOG_VAR_(func_name, outfunc, ...) \
//   log_vars(func_name, outfunc, #__VA_ARGS__, __VA_ARGS__)
// #define LOG_VAR(...) \
//   LOG_VAR_( \
//       , [](const std::string &msg) { std::cout << msg << std::endl; }, \
//       __VA_ARGS__)
// #define LOG_CLASS_VAR(...) \
//   LOG_VAR_( \
//       __FUNC_NAME__, \
//       [](const std::string &msg) { std::cout << msg << std::endl; }, \
//       __VA_ARGS__)
//
// #define LOG_FUNC_(func, outfunc, ...) \
//   log_func( \
//       #func, \
//       [&](auto &&...args) -> decltype(auto) { \
//         return func(std::forward<decltype(args)>(args)...); \
//       }, \ outfunc, #__VA_ARGS__, __VA_ARGS__)
// #define LOG_CLASS_FUNC_(func, outfunc, ...) \
//   log_func( \
//       __FUNC_NAME__ + "->" #func, \
//       [&](auto &&...args) -> decltype(auto) { \
//         return func(std::forward<decltype(args)>(args)...); \
//       }, \ outfunc, #__VA_ARGS__, __VA_ARGS__)
// #define LOG_FUNC(func, ...) \
//   LOG_FUNC_( \
//       func, [](const std::string &msg) { std::cout << msg << std::endl; }, \
//       __VA_ARGS__)
// #define LOG_CLASS_FUNC(func, ...) \
//   LOG_CLASS_FUNC_( \
//       func, [](const std::string &msg) { std::cout << msg << std::endl; }, \
//       __VA_ARGS__)

#endif  // PR_PALETTE_DEBUG_HPP
