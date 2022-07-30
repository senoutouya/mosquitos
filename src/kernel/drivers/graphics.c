#include <kernel/drivers/graphics.h>

static struct {
  EFI_GRAPHICS_OUTPUT_PROTOCOL *gop;
  uint32_t *frame_buffer_base;
  uint32_t pixels_per_line;
  uint32_t horizontal_resolution;
  uint32_t vertical_resolution;
} graphics_data;

void graphics_init(EFI_GRAPHICS_OUTPUT_PROTOCOL *gop) {
  graphics_data.gop = gop;
  graphics_data.frame_buffer_base = (uint32_t *)gop->Mode->FrameBufferBase;
  graphics_data.pixels_per_line = gop->Mode->Info->PixelsPerScanLine;
  graphics_data.horizontal_resolution = gop->Mode->Info->HorizontalResolution;
  graphics_data.vertical_resolution = gop->Mode->Info->VerticalResolution;

  REGISTER_MODULE("graphics");
}

void graphics_clear_screen(uint32_t color) {
  graphics_fill_rect(0, 0, graphics_data.gop->Mode->Info->HorizontalResolution,
                     graphics_data.gop->Mode->Info->VerticalResolution, color);
}

void graphics_fill_rect(int x, int y, int w, int h, uint32_t color) {
  for (int i = y; i < y + h; ++i) {
    for (int j = x; j < x + w; ++j) {
      graphics_draw_pixel(j, i, color);
    }
  }
}

void graphics_draw_pixel(int x, int y, uint32_t color) {
  graphics_data.frame_buffer_base[y * graphics_data.pixels_per_line + x] =
      color;
}

uint32_t graphics_resolution_horizon() { return graphics_data.horizontal_resolution; }

uint32_t graphics_resolution_vert() { return graphics_data.vertical_resolution; }

