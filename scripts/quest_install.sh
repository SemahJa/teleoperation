#!/usr/bin/env bash
set -euo pipefail

apk="android/quest_teleop/app/build/outputs/apk/debug/app-debug.apk"
if [[ ! -f "${apk}" ]]; then
  echo "APK not found: ${apk}" >&2
  echo "Build it with: docker compose -f docker/android-compose.yaml run --rm android-build gradle assembleDebug" >&2
  exit 2
fi

if ! docker compose -f docker/android-compose.yaml run --rm quest-adb \
  adb install -r /workspace/android/quest_teleop/app/build/outputs/apk/debug/app-debug.apk; then
  echo "Install failed. If this is INSTALL_FAILED_UPDATE_INCOMPATIBLE, the installed app was signed with a different debug key." >&2
  echo "After confirming its app data can be discarded, run: docker compose -f docker/android-compose.yaml run --rm quest-adb adb uninstall org.franka.questteleop" >&2
  echo "Then rerun this script. Debug signing is now persisted in the android-signing volume." >&2
  exit 1
fi
docker compose -f docker/android-compose.yaml run --rm quest-adb \
  adb shell am start -n org.franka.questteleop/android.app.NativeActivity
