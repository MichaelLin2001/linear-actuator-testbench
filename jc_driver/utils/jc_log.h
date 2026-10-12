#ifndef JC_LOG_H
#define JC_LOG_H

#ifdef __cplusplus
extern "C" {
#endif
typedef enum {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    FATAL
} Level;

void _JC_LOG(Level level, int line, const char *func, const char *format, ...);
void initLogger(const char *filePath);
void deInitLogger(void);

#define JC_LOG_DEBUG(format, ...) _JC_LOG(DEBUG, __LINE__, __func__, format, ##__VA_ARGS__)
#define JC_LOG_INFO(format, ...) _JC_LOG(INFO, __LINE__, __func__, format, ##__VA_ARGS__)
#define JC_LOG_WARN(format, ...) _JC_LOG(WARNING, __LINE__, __func__, format, ##__VA_ARGS__)
#define JC_LOG_ERROR(format, ...) _JC_LOG(ERROR, __LINE__, __func__, format, ##__VA_ARGS__)
#define JC_LOG_FATAL(format, ...) _JC_LOG(FATAL, __LINE__, __func__, format, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif  // JC_LOG_H