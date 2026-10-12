#pragma once
// Standalone transport tests replace only Android logging, never transport logic.
enum { ANDROID_LOG_INFO, ANDROID_LOG_WARN, ANDROID_LOG_ERROR };
inline int __android_log_print(int, const char*, const char*, ...) { return 0; }
