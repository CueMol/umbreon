// Minimal dependency-free test harness.
#pragma once

#include <cstdio>
#include <sstream>
#include <string>
#include <utility>

namespace umbreon {
namespace test {

class Suite {
 public:
  explicit Suite(std::string name) : name_(std::move(name)) {}

  void check(const std::string& what, bool ok) {
    ++total_;
    if (ok) {
      std::printf("  ok   - %s\n", what.c_str());
    } else {
      ++failed_;
      std::printf("  FAIL - %s\n", what.c_str());
    }
  }

  template <class A, class B>
  void check_eq(const std::string& what, const A& got, const B& expected) {
    ++total_;
    if (got == expected) {
      std::printf("  ok   - %s\n", what.c_str());
    } else {
      ++failed_;
      std::ostringstream oss;
      oss << what << " (got " << got << ", expected " << expected << ")";
      std::printf("  FAIL - %s\n", oss.str().c_str());
    }
  }

  int report() const {
    std::printf("[%s] %d/%d checks passed\n", name_.c_str(), total_ - failed_,
                total_);
    return failed_ == 0 ? 0 : 1;
  }

 private:
  std::string name_;
  int total_ = 0;
  int failed_ = 0;
};

}  // namespace test
}  // namespace umbreon
