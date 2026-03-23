#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESS_DIR="${ROOT_DIR}/sessions"
DIAG_DIR="${ROOT_DIR}/diagnostics"
TRACK_DIR="${DIAG_DIR}/track_exports"

MOVED=0
SKIPPED=0

usage() {
  cat <<'EOF'
Usage: ./scripts/organize_field_data.sh

Organizes repository-level field data into date-based folders:

  sessions/
    YYYY/MM/DD/sess_YYYYmmdd_HHMMSS/...

  diagnostics/
    by_date/YYYY/MM/DD/archive/rover_diag_YYYYmmdd_HHMMSS.tar.gz
    by_date/YYYY/MM/DD/extracted/rover_diag_YYYYmmdd_HHMMSS/...
    track_exports/YYYY/MM/DD/sess_YYYYmmdd_HHMMSS_*.{csv,svg,kml,gpx,geojson}
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

mkdir -p "${SESS_DIR}" "${DIAG_DIR}" "${TRACK_DIR}"

log() {
  echo "[organize] $*"
}

move_into() {
  local src="$1"
  local dst_dir="$2"
  local base dst
  base="$(basename "${src}")"
  dst="${dst_dir}/${base}"
  mkdir -p "${dst_dir}"

  if [[ "$(cd "$(dirname "${src}")" && pwd)/${base}" == "$(cd "${dst_dir}" && pwd)/${base}" ]]; then
    ((SKIPPED+=1))
    return
  fi
  if [[ -e "${dst}" ]]; then
    log "skip (exists): ${dst}"
    ((SKIPPED+=1))
    return
  fi

  mv "${src}" "${dst}"
  ((MOVED+=1))
  log "moved: ${src} -> ${dst}"
}

organize_sessions() {
  local d b stamp yyyy mm dd target
  shopt -s nullglob
  for d in "${SESS_DIR}"/sess_*; do
    [[ -d "${d}" ]] || continue
    b="$(basename "${d}")"
    if [[ "${b}" =~ ^sess_([0-9]{8})_[0-9]{6}$ ]]; then
      stamp="${BASH_REMATCH[1]}"
      yyyy="${stamp:0:4}"
      mm="${stamp:4:2}"
      dd="${stamp:6:2}"
      target="${SESS_DIR}/${yyyy}/${mm}/${dd}"
      move_into "${d}" "${target}"
    else
      log "skip (unexpected session name): ${d}"
      ((SKIPPED+=1))
    fi
  done
  shopt -u nullglob
}

organize_diagnostics() {
  local p b stamp yyyy mm dd target
  shopt -s nullglob

  for p in "${DIAG_DIR}"/rover_diag_*.tar.gz; do
    [[ -f "${p}" ]] || continue
    b="$(basename "${p}")"
    if [[ "${b}" =~ ^rover_diag_([0-9]{8})_[0-9]{6}\.tar\.gz$ ]]; then
      stamp="${BASH_REMATCH[1]}"
      yyyy="${stamp:0:4}"
      mm="${stamp:4:2}"
      dd="${stamp:6:2}"
      target="${DIAG_DIR}/by_date/${yyyy}/${mm}/${dd}/archive"
      move_into "${p}" "${target}"
    else
      log "skip (unexpected diagnostics archive name): ${p}"
      ((SKIPPED+=1))
    fi
  done

  for p in "${DIAG_DIR}"/rover_diag_*; do
    [[ -d "${p}" ]] || continue
    b="$(basename "${p}")"
    if [[ "${b}" =~ ^rover_diag_([0-9]{8})_[0-9]{6}$ ]]; then
      stamp="${BASH_REMATCH[1]}"
      yyyy="${stamp:0:4}"
      mm="${stamp:4:2}"
      dd="${stamp:6:2}"
      target="${DIAG_DIR}/by_date/${yyyy}/${mm}/${dd}/extracted"
      move_into "${p}" "${target}"
    else
      log "skip (unexpected diagnostics dir name): ${p}"
      ((SKIPPED+=1))
    fi
  done

  for p in "${TRACK_DIR}"/sess_*; do
    [[ -f "${p}" ]] || continue
    b="$(basename "${p}")"
    if [[ "${b}" =~ ^sess_([0-9]{8})_[0-9]{6} ]]; then
      stamp="${BASH_REMATCH[1]}"
      yyyy="${stamp:0:4}"
      mm="${stamp:4:2}"
      dd="${stamp:6:2}"
      target="${TRACK_DIR}/${yyyy}/${mm}/${dd}"
      move_into "${p}" "${target}"
    else
      log "skip (unexpected track export name): ${p}"
      ((SKIPPED+=1))
    fi
  done

  shopt -u nullglob
}

write_indexes() {
  python3 - "${ROOT_DIR}" <<'PY'
from pathlib import Path
import json
import datetime

root = Path(__import__("sys").argv[1])

sessions_dir = root / "sessions"
diagnostics_dir = root / "diagnostics"

session_rows = []
for p in sessions_dir.rglob("session_manifest.json"):
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        sid = data.get("session_id", p.parent.name)
        start = data.get("start_utc", "")
        dur = data.get("duration_s")
        state = data.get("state", "")
        gnss = data.get("gnss_lock_at_start")
        corr = data.get("corrections_active_at_start")
        session_rows.append((start, sid, dur, state, gnss, corr, str(p.parent.relative_to(root))))
    except Exception:
        continue
session_rows.sort(reverse=True)

session_index = sessions_dir / "INDEX.md"
session_index.parent.mkdir(parents=True, exist_ok=True)
with session_index.open("w", encoding="utf-8") as f:
    f.write("# Sessions Index\n\n")
    f.write("| Start UTC | Session | Duration (s) | State | GNSS at Start | Corrections at Start | Path |\n")
    f.write("|---|---|---:|---|---|---|---|\n")
    for row in session_rows[:200]:
        start, sid, dur, state, gnss, corr, path = row
        dur_txt = "" if dur is None else f"{dur:.1f}" if isinstance(dur, (int, float)) else str(dur)
        f.write(f"| {start} | {sid} | {dur_txt} | {state} | {gnss} | {corr} | `{path}` |\n")

diag_rows = []
for p in (diagnostics_dir / "by_date").rglob("rover_diag_*.tar.gz"):
    mtime = datetime.datetime.fromtimestamp(
        p.stat().st_mtime, tz=datetime.timezone.utc
    ).isoformat().replace("+00:00", "Z")
    diag_rows.append((mtime, p.name, str(p.relative_to(root))))
diag_rows.sort(reverse=True)

diag_index = diagnostics_dir / "INDEX.md"
diag_index.parent.mkdir(parents=True, exist_ok=True)
with diag_index.open("w", encoding="utf-8") as f:
    f.write("# Diagnostics Index\n\n")
    f.write("| Modified UTC | Bundle | Path |\n")
    f.write("|---|---|---|\n")
    for row in diag_rows[:200]:
        mtime, name, path = row
        f.write(f"| {mtime} | {name} | `{path}` |\n")

print(session_index)
print(diag_index)
PY
}

log "organizing sessions ..."
organize_sessions

log "organizing diagnostics ..."
organize_diagnostics

log "writing indexes ..."
write_indexes

echo
echo "Done."
echo "Moved items:   ${MOVED}"
echo "Skipped items: ${SKIPPED}"
echo "Sessions index:    ${SESS_DIR}/INDEX.md"
echo "Diagnostics index: ${DIAG_DIR}/INDEX.md"
