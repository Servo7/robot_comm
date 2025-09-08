#!/usr/bin/env bash
#
# Isaac-kiosk: force any window whose title contains “Isaac” (any case)
# into border-less, always-on-top fullscreen, and suppress the usual
# key combos that could break out.  Needs X11, xdotool and wmctrl.

TITLE_RE='[Ii][Ss][Aa][Aa][Cc]'   # case-insensitive “Isaac”
INTERVAL=1                        # seconds between scans

##############################################################################
# Safety cage – disable Alt+F4 and Alt+Tab while the script runs.
# Reverts automatically on exit so you never lock yourself out permanently.
disable_hotkeys() {
  # exact keycodes differ per layout, so clear the whole Alt (Mod1) modifier
  xmodmap -e "clear mod1"
  # neutralise F4 and Tab keys themselves
  xmodmap -e "keycode  70 = NoSymbol"   # F4
  xmodmap -e "keycode  23 = NoSymbol"   # Tab
}
restore_hotkeys() { setxkbmap; }        # reload full layout

trap restore_hotkeys EXIT
disable_hotkeys
##############################################################################

echo "[$(date '+%F %T')] Isaac-kiosk started – waiting for Isaac window"

while true; do
  # grab first matching window ID, if any
  wid=$(xdotool search --name --limit 1 --sync "$TITLE_RE" 2>/dev/null | head -n 1)
  if [[ -n "$wid" ]]; then
    # 1. Add EWMH fullscreen & always-on-top flags
    wmctrl -ir "$wid" -b add,fullscreen,above
    # 2. Tell most WMs to drop decorations (Motif hint)
    xprop -id "$wid" -f _MOTIF_WM_HINTS 32c \
          -set _MOTIF_WM_HINTS "0x2, 0x0, 0x0, 0x0, 0x0" >/dev/null 2>&1
  fi
  sleep "$INTERVAL"
done

# robot_comm
