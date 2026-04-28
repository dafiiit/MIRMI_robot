#!/usr/bin/env bash
set -euo pipefail

# Collect one sample message per topic and store interface details.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +"%Y-%m-%d_%H-%M-%S")"

MODE="${1:-fmu}"
case "$MODE" in
  --all|all)
    FILTER_DESC="all topics"
    OUT_ROOT="all_topic_interfaces"
    ;;
  --fmu|fmu|"")
    FILTER_DESC="topics starting with /fmu"
    OUT_ROOT="fmu_topic_interfaces"
    ;;
  *)
    echo "Usage: $0 [--fmu|--all]"
    exit 2
    ;;
esac

OUT_DIR="$SCRIPT_DIR/$OUT_ROOT/$TIMESTAMP"
MSG_DIR="$OUT_DIR/messages"
IFACE_DIR="$OUT_DIR/interfaces"
OVERVIEW_FILE="$OUT_DIR/topic_overview.csv"

mkdir -p "$MSG_DIR" "$IFACE_DIR"

printf 'topic,type,message_file,interface_file,status\n' > "$OVERVIEW_FILE"

if [[ "$OUT_ROOT" == "fmu_topic_interfaces" ]]; then
  mapfile -t TOPICS < <(ros2 topic list | grep '^/fmu' || true)
else
  mapfile -t TOPICS < <(ros2 topic list || true)
fi

if [[ ${#TOPICS[@]} -eq 0 ]]; then
  echo "Keine Topics gefunden (Filter: $FILTER_DESC)."
  echo "Ausgabeordner wurde trotzdem erstellt: $OUT_DIR"
  exit 0
fi

for TOPIC in "${TOPICS[@]}"; do
  SAFE_NAME="$(echo "$TOPIC" | sed 's#^/##; s#[^A-Za-z0-9._-]#_#g')"
  MSG_FILE="$MSG_DIR/${SAFE_NAME}.msg.txt"
  ERR_FILE="$MSG_DIR/${SAFE_NAME}.err.txt"

  TOPIC_TYPE="$(ros2 topic type "$TOPIC" 2>/dev/null || true)"
  if [[ -z "$TOPIC_TYPE" ]]; then
    TOPIC_TYPE="UNKNOWN"
  fi

  IFACE_FILE=""
  STATUS="ok"

  if ! timeout 8s ros2 topic echo --once "$TOPIC" > "$MSG_FILE" 2> "$ERR_FILE"; then
    STATUS="no_message_timeout_or_error"
    {
      echo "# No sample message captured for topic: $TOPIC"
      echo "# Reason: timeout or ros2 topic echo returned an error"
      if [[ -s "$ERR_FILE" ]]; then
        echo
        echo "# stderr"
        cat "$ERR_FILE"
      fi
    } > "$MSG_FILE"
  fi

  if [[ "$TOPIC_TYPE" != "UNKNOWN" ]]; then
    IFACE_FILE="$IFACE_DIR/${SAFE_NAME}.interface.txt"
    if ! ros2 interface show "$TOPIC_TYPE" > "$IFACE_FILE" 2>/dev/null; then
      IFACE_FILE=""
    fi
  fi

  rm -f "$ERR_FILE"

  printf '%s,%s,%s,%s,%s\n' \
    "$TOPIC" \
    "$TOPIC_TYPE" \
    "messages/${SAFE_NAME}.msg.txt" \
    "${IFACE_FILE#"$OUT_DIR"/}" \
    "$STATUS" >> "$OVERVIEW_FILE"
done

cat > "$OUT_DIR/README.txt" <<EOF
Topic snapshot
==============

Created at: $(date)
Filter: $FILTER_DESC

Contents:
- topic_overview.csv: Summary of all captured topics
- messages/: one sample message per topic (or error/timeout note)
- interfaces/: ROS interface definition files (when type could be resolved)
EOF

echo "Fertig. Ergebnisse gespeichert in: $OUT_DIR"
