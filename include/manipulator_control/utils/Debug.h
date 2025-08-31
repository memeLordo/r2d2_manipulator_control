#ifndef PR_MANIPULATOR_CONTROL_DEBUG
#define PR_MANIPULATOR_CONTROL_DEBUG

#include <cstring>
#include <sstream>

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

// Base case for recursion
inline void debug_print_args(std::ostringstream &oss, const char *names) {
  // End of recursion - no more arguments
}

// Recursive template to parse and print variable names with values
template <typename T, typename... Args>
void debug_print_args(std::ostringstream &oss, const char *names, T &&value,
                      Args &&...args) {
  // Extract first variable name from comma-separated string
  const char *comma = strchr(names, ',');
  std::string firstName;
  const char *nextNames = nullptr;

  if (comma) {
    firstName = std::string(names, comma - names);
    nextNames = comma + 1;
    // Skip whitespace after comma
    while (*nextNames == ' ' || *nextNames == '\t')
      ++nextNames;
  } else {
    firstName = std::string(names);
  }

  // Trim leading and trailing spaces
  size_t start = firstName.find_first_not_of(" \t");
  size_t end = firstName.find_last_not_of(" \t");
  if (start != std::string::npos && end != std::string::npos) {
    firstName = firstName.substr(start, end - start + 1);
  }

  // Add colored variable name and value
  oss << YELLOW(firstName + " = " << value);

  // Continue recursion if more arguments exist
  if (nextNames) {
    oss << ", ";
    debug_print_args(oss, nextNames, std::forward<Args>(args)...);
  }
}

// Main logging function macro - accepts custom output function
#define _LOG_FUNC(func, output, ...)                                           \
  do {                                                                         \
    std::ostringstream oss;                                                    \
    oss << MAGENTA("[" << __func__ << "]") << "(";                             \
    debug_print_args(oss, #__VA_ARGS__, __VA_ARGS__);                          \
    oss << ") : " << WHITE(output);                                            \
    func(oss.str());                                                           \
  } while (0)

// Simple debug function that outputs to std::cout
#define DEBUG_FUNC(output, ...)                                                \
  _LOG_FUNC([](const std::string &msg) { std::cout << msg << std::endl; },     \
            output, __VA_ARGS__)

// Alternative macro for stream-based output
#define DEBUG_STREAM(stream, output, ...)                                      \
  do {                                                                         \
    std::ostringstream oss;                                                    \
    oss << MAGENTA(__func__);                                                  \
    debug_print_args(oss, #__VA_ARGS__, __VA_ARGS__);                          \
    oss << " : " << WHITE(output);                                             \
    stream << oss.str() << std::endl;                                          \
  } while (0)

#endif // !PR_MANIPULATOR_CONTROL_DEBUG_H
