#pragma once
#include <stddef.h>
#include <stdint.h>

enum class Turn : uint8_t { Left, Straight, Right, UTurn, Stop };

// route = action per intersection, e.g. "SLRUF"
class Route {
 public:
  static constexpr size_t kMax = 32;
  bool parse(const char* text);
  Turn next();
  void rewind() { idx_ = 0; }
  size_t size() const { return n_; }
  size_t index() const { return idx_; }
  const char* str() const { return text_; }

 private:
  Turn steps_[kMax] = {};
  char text_[kMax + 1] = {};
  size_t n_ = 0;
  size_t idx_ = 0;
};
