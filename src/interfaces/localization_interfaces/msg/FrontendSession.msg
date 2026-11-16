# 在启动、会话重建、输入中断和处理失败时发布。
uint64 instance_id
uint64 epoch
# 当前代次发生可恢复输入中断，要求后端协调冷启动；成功换代后清除。
bool reset_required
# 当前会话已因处理错误停止，禁止自动重试；成功换代后清除。
bool failed
