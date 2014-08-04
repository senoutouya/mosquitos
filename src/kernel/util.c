#include "util.h"
#include "text_output.h"

#include <stdarg.h>

void _panic(char *format, ...) {
  va_list arg_list;
  va_start(arg_list, format);

  text_output_vprintf(format, arg_list);

  va_end(arg_list);

  __asm__ ("cli \n\t hlt");
}

int int2str(uint64_t n, char *buf, int buf_len, int radix) {
  static char *digit_lookup = "0123456789abcdef";

  // Compute log base `radix` of the number
  int len = 0;
  for (uint64_t tmp = n; tmp > 0; tmp /= radix, len++);

  // Special case for zero
  if (n == 0) len = 1;

  if (len + 1 > buf_len) return -1; // Don't overflow

  buf[len] = '\0';
  for (int i = len-1; i >= 0; --i) {
    buf[i] = digit_lookup[(n % radix)];
    n /= radix;
  }

  return 0;
}

void sti() {
  __asm__ ("sti");
}

void cli() {
  __asm__ ("cli");
}

uint8_t inb(uint16_t port) {
    uint8_t ret;
    __asm__ volatile ( "inb %1, %0" : "=a"(ret) : "d"(port) );
    return ret;
}

void outb(uint16_t port, uint8_t val) {
  __asm__ volatile ( "outb %0, %1" : : "a"(val), "d"(port) );
}

void write_msr(uint64_t index, uint64_t value) {
  uint64_t high = value >> 32;
  uint64_t low  = (value & 0x0000000000000000ffffffffffffffff);
  __asm__ ("rdmsr" : : "a" (low), "d" (high), "c" (index));
}

uint64_t read_msr(uint64_t index) {
  uint64_t high, low;
  __asm__ ("rdmsr" : "=a" (low), "=d" (high) : "c" (index));

  return high << 32 | low;
}