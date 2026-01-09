#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <mutex>
#include <chrono>

namespace referee_node {

/**
 * @brief 数据记录格式：[时间戳(8字节)][数据长度(4字节)][数据内容]
 * 时间戳：从记录开始的微秒数（uint64_t）
 * 数据长度：数据字节数（uint32_t）
 * 数据内容：实际的字节流数据
 */

/**
 * @brief 字节流记录器类，用于将原始字节流数据记录到文件
 */
class ByteStreamRecorder {
 public:
  /**
   * @brief 构造函数
   * @param file_path 要写入的文件路径
   */
  explicit ByteStreamRecorder(const std::string& file_path);

  /**
   * @brief 析构函数，自动关闭文件
   */
  ~ByteStreamRecorder();

  // noncopyable
  ByteStreamRecorder(const ByteStreamRecorder&) = delete;
  ByteStreamRecorder& operator=(const ByteStreamRecorder&) = delete;

  /**
   * @brief 写入数据到文件
   * @param data 要写入的字节数据
   * @return 成功返回true，失败返回false
   */
  bool Write(const std::vector<uint8_t>& data);

  /**
   * @brief 写入数据到文件
   * @param data 要写入的字节数据（std::string版本）
   * @return 成功返回true，失败返回false
   */
  bool Write(const std::string& data);

  /**
   * @brief 写入数据到文件（指针版本）
   * @param data 数据指针
   * @param size 数据大小
   * @return 成功返回true，失败返回false
   */
  bool Write(const uint8_t* data, size_t size);

  /**
   * @brief 检查文件是否成功打开
   * @return 文件打开返回true，否则返回false
   */
  bool IsOpen() const;

  /**
   * @brief 获取文件路径
   * @return 文件路径字符串
   */
  const std::string& GetFilePath() const { return file_path_; }

  /**
   * @brief 手动关闭文件
   */
  void Close();

 private:
  std::string file_path_;                             ///< 文件路径
  std::ofstream file_stream_;                         ///< 文件输出流
  mutable std::mutex mutex_;                          ///< 保护文件操作的互斥锁
  std::chrono::steady_clock::time_point start_time_;  ///< 记录开始的时间点
};

}  // namespace referee_node
