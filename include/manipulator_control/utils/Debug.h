#ifndef PR_MANIPULATOR_CONTROL_DEBUG
#define PR_MANIPULATOR_CONTROL_DEBUG

#include <algorithm>
#include <cstring>
#include <sstream>
#include <vector>

// ANSI color definitions
#define ANSI_COLOR_RED "\033[0;31m"
#define ANSI_COLOR_GREEN "\033[0;32m"
#define ANSI_COLOR_YELLOW "\033[0;33m"
#define ANSI_COLOR_BLUE "\033[0;34m"
#define ANSI_COLOR_MAGENTA "\033[0;35m"
#define ANSI_COLOR_CYAN "\033[0;36m"
#define ANSI_COLOR_WHITE "\033[0;37m"
#define ANSI_COLOR_RESET ANSI_COLOR_GREEN

// Color stream manipulators
#define RED(x) ANSI_COLOR_RED << x << ANSI_COLOR_RESET
#define GREEN(x) ANSI_COLOR_GREEN << x << ANSI_COLOR_RESET
#define YELLOW(x) ANSI_COLOR_YELLOW << x << ANSI_COLOR_RESET
#define BLUE(x) ANSI_COLOR_BLUE << x << ANSI_COLOR_RESET
#define MAGENTA(x) ANSI_COLOR_MAGENTA << x << ANSI_COLOR_RESET
#define CYAN(x) ANSI_COLOR_CYAN << x << ANSI_COLOR_RESET
#define WHITE(x) ANSI_COLOR_WHITE << x << ANSI_COLOR_RESET

// Вспомогательная печать одной пары имя-значение
template <typename T>
inline void debug_print_single(std::ostringstream &oss, const std::string &name,
                               T &&value) {
  oss << YELLOW(name << "=" << std::forward<T>(value));
}

inline void debug_print_impl(std::ostringstream &oss,
                             const std::vector<std::string> & /*names*/,
                             size_t /*idx*/) {}

template <typename T, typename... Args>
inline void debug_print_impl(std::ostringstream &oss,
                             const std::vector<std::string> &names, size_t idx,
                             T &&value, Args &&...args) {
  debug_print_single(oss, names[idx], std::forward<T>(value));
  if (sizeof...(args) > 0) {
    oss << ", ";
    debug_print_impl(oss, names, idx + 1, std::forward<Args>(args)...);
  }
}

template <typename... Args>
void debug_print_args(std::ostringstream &oss, std::string names_str,
                      Args &&...args) {
  std::vector<std::string> names;
  // Replace commas with spaces to normalize separators
  std::replace(names_str.begin(), names_str.end(), ',', ' ');
  std::istringstream iss{names_str};
  std::string name;
  while (iss >> name) { // Skip spaces and tabs between names
    names.emplace_back(name);
  }
  debug_print_impl(oss, names, 0, std::forward<Args>(args)...);
}

// Non-void return type version
template <typename Func, typename OutFunc, typename... Args>
inline auto log_wrapper(const std::string func_name, Func func, OutFunc outfunc,
                        const std::string names, Args &&...args) ->
    typename std::enable_if<
        !std::is_void<typename std::result_of<Func(Args &&...)>::type>::value,
        typename std::result_of<Func(Args &&...)>::type>::type {
  std::ostringstream oss;
  oss << "[" << MAGENTA(func_name) << "](";
  debug_print_args(oss, names, std::forward<Args>(args)...);
  auto result = func(std::forward<Args>(args)...);
  oss << ") : " << WHITE(result);
  outfunc(oss.str());
  return result;
}

// Void return type version
template <typename Func, typename OutFunc, typename... Args>
inline typename std::enable_if<
    std::is_void<typename std::result_of<Func(Args &&...)>::type>::value,
    void>::type
log_wrapper(const char *func_name, Func func, OutFunc outfunc,
            const char *names, Args &&...args) {
  std::ostringstream oss;
  oss << "[" << MAGENTA(func_name) << "](";
  debug_print_args(oss, names, std::forward<Args>(args)...);
  func(std::forward<Args>(args)...);
  oss << ") : " << WHITE("(void)");
  outfunc(oss.str());
}

#define DEBUG_FUNC(func, outfunc, ...)                                         \
  log_wrapper(                                                                 \
      #func,                                                                   \
      [&](auto &&...args) -> decltype(auto) {                                  \
        return func(std::forward<decltype(args)>(args)...);                    \
      },                                                                       \
      outfunc, #__VA_ARGS__, __VA_ARGS__)

#define DEBUG_WRAP_FUNC(func, ...)                                             \
  DEBUG_FUNC(                                                                  \
      func, [](const std::string &msg) { std::cout << msg << std::endl; },     \
      __VA_ARGS__)

#define DEBUG_IN_FUNC(func, output, ...)                                       \
  log_wrapper(                                                                 \
      __func__, [](const std::string &msg) { std::cout << msg << std::endl; }, \
      output, __VA_ARGS__)

#endif // !PR_MANIPULATOR_CONTROL_DEBUG_H
