#include "route.h"
#include <ctype.h>

bool Route::parse(const char* text) {
  Turn tmp[kMax];
  char chars[kMax + 1] = {};
  size_t n = 0;
  for (const char* p = text; p && *p; ++p) {
    char c = static_cast<char>(toupper(static_cast<unsigned char>(*p)));
    if (c == ' ' || c == ',' || c == '-') continue;
    if (n >= kMax) return false;
    switch (c) {
      case 'L': tmp[n] = Turn::Left; break;
      case 'S': tmp[n] = Turn::Straight; break;
      case 'R': tmp[n] = Turn::Right; break;
      case 'U': tmp[n] = Turn::UTurn; break;
      case 'F': tmp[n] = Turn::Stop; break;
      default: return false;
    }
    chars[n++] = c;
  }
  if (n == 0) return false;
  for (size_t i = 0; i < n; ++i) steps_[i] = tmp[i];
  for (size_t i = 0; i <= n; ++i) text_[i] = chars[i];
  n_ = n;
  idx_ = 0;
  return true;
}

Turn Route::next() { return idx_ < n_ ? steps_[idx_++] : Turn::Stop; }
