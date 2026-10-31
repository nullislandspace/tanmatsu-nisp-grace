// =====================================================================
//  Null Island Space Program  --  the legend's part icons (see icons.h)
// =====================================================================

#include "engineering/icons.h"

#include "engineering/sheet.h"
#include "se_direct565.h"

static void line(pax_buf_t* fb, uint16_t c, int x0, int y0, int x1, int y1) {
    direct_565_line((uint16_t*)pax_buf_get_pixels(fb), x0, y0, x1, y1, c);
}

static void box(pax_buf_t* fb, uint16_t c, int x, int y, int w, int h) {
    if (w < 2 || h < 2) return;
    line(fb, c, x, y, x + w - 1, y);
    line(fb, c, x, y + h - 1, x + w - 1, y + h - 1);
    line(fb, c, x, y, x, y + h - 1);
    line(fb, c, x + w - 1, y, x + w - 1, y + h - 1);
}

void nisp_icon_draw(pax_buf_t* fb, int x, int y, int kind, bool hot) {
    if (fb == NULL) return;
    uint16_t const c = direct_565_pack_for(fb, hot ? NISP_MARK_ARGB : NISP_PENCIL_ARGB);
    int const cx = x + NISP_ICON_PX / 2;
    int const top = y + 3;
    int const bot = y + NISP_ICON_PX - 7;  // the last 7 px are the bell

    switch (kind) {
        case NISP_KIND_CAPSULE:
            line(fb, c, cx, top, cx - 8, top + 10);
            line(fb, c, cx, top, cx + 8, top + 10);
            line(fb, c, cx - 8, top + 10, cx - 8, bot);
            line(fb, c, cx + 8, top + 10, cx + 8, bot);
            line(fb, c, cx - 10, bot, cx + 10, bot);  // the shield
            break;

        case NISP_KIND_UPPER:
            box(fb, c, cx - 5, top, 11, bot - top + 1);
            line(fb, c, cx - 3, bot + 1, cx - 6, bot + 6);
            line(fb, c, cx + 3, bot + 1, cx + 6, bot + 6);
            line(fb, c, cx - 6, bot + 6, cx + 6, bot + 6);
            break;

        case NISP_KIND_BOOSTER:
            for (int s = -1; s <= 1; s += 2) {
                int const bx = cx + s * 8;
                line(fb, c, bx - 5, top + 4, bx, top);
                line(fb, c, bx, top, bx + 5, top + 4);
                box(fb, c, bx - 5, top + 4, 11, bot - top - 3);
                line(fb, c, bx - 5, bot + 1, bx + 5, bot + 1);
            }
            break;

        case NISP_KIND_CARGO:
            box(fb, c, cx - 9, top, 19, bot - top + 1);
            // A cargo section is a tank with something in it: the hatch is
            // the drawing's way of saying "no engine here".
            line(fb, c, cx - 9, top + 6, cx + 9, top + 6);
            line(fb, c, cx - 9, bot - 5, cx + 9, bot - 5);
            line(fb, c, cx - 4, top + 6, cx + 4, bot - 5);
            break;

        case NISP_KIND_MAIN:
        default:
            box(fb, c, cx - 9, top, 19, bot - top + 1);
            line(fb, c, cx - 4, bot + 1, cx - 8, bot + 6);
            line(fb, c, cx + 4, bot + 1, cx + 8, bot + 6);
            line(fb, c, cx - 8, bot + 6, cx + 8, bot + 6);
            break;
    }
}
