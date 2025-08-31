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
  oss << YELLOW(name + "=" << std::forward<T>(value));
}

inline void debug_print_impl(std::ostringstream &oss,
                             const std::vector<std::string> &names,
                             size_t /*idx*/) {
  // базовый случай — ничего не делаем
}

template <typename T, typename... Args>
void debug_print_impl(std::ostringstream &oss,
                      const std::vector<std::string> &names, size_t idx,
                      T &&value, Args &&...args) {
  debug_print_single(oss, names[idx], std::forward<T>(value));
  if (sizeof...(args) > 0) {
    oss << ", ";
    debug_print_impl(oss, names, idx + 1, std::forward<Args>(args)...);
  }
}

template <typename... Args>
void somefunc(std::ostringstream &oss, std::string names_str, Args &&...args) {
  std::vector<std::string> names;

  // Заменяем запятую на пробел, чтобы считать \t и пробелы одинаково
  std::replace(names_str.begin(), names_str.end(), ',', ' ');

  std::istringstream iss(names_str);
  std::string name;
  while (iss >> name) { // Так пропускаются пробелы и табы между именами
    names.emplace_back(name);
  }

  debug_print_impl(oss, names, 0, std::forward<Args>(args)...);
}

#define _LOG_FUNC(func, output, ...)                                           \
  do {                                                                         \
    std::ostringstream oss;                                                    \
    oss << MAGENTA(__func__) << " ";                                           \
    somefunc(oss, #__VA_ARGS__, __VA_ARGS__);                                  \
    oss << " : " << WHITE(output);                                             \
    func(oss.str());                                                           \
  } while (0)

// Simple debug function that outputs to std::cout
#define DEBUG_FUNC(output, ...)                                                \
  _LOG_FUNC([](const std::string &msg) { std::cout << msg << std::endl; },     \
            output, __VA_ARGS__)

#endif // !PR_MANIPULATOR_CONTROL_DEBUG_H
