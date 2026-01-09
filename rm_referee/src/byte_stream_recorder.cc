
#include "referee_node/byte_stream_recorder.hpp"

namespace referee_node {

ByteStreamRecorder::ByteStreamRecorder(const std::string& file_path)
    : file_path_(file_path), start_time_(std::chrono::steady_clock::now()) {
  file_stream_.open(file_path_, std::ios::binary | std::ios::out);
}

ByteStreamRecorder::~ByteStreamRecorder() { Close(); }

bool ByteStreamRecorder::Write(const std::vector<uint8_t>& data) { return Write(data.data(), data.size()); }

bool ByteStreamRecorder::Write(const std::string& data) {
  return Write(reinterpret_cast<const uint8_t*>(data.data()), data.size());
}

bool ByteStreamRecorder::Write(const uint8_t* data, size_t size) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!file_stream_.is_open() || size == 0) {
    return false;
  }

  // 计算从开始记录到现在的微秒数
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
  uint64_t timestamp_us = static_cast<uint64_t>(elapsed.count());

  // 记录数据长度
  uint32_t data_size = static_cast<uint32_t>(size);

  // 写入格式：[时间戳(8字节)][数据长度(4字节)][数据内容]
  file_stream_.write(reinterpret_cast<const char*>(&timestamp_us), sizeof(timestamp_us));
  file_stream_.write(reinterpret_cast<const char*>(&data_size), sizeof(data_size));
  file_stream_.write(reinterpret_cast<const char*>(data), size);
  file_stream_.flush();

  return file_stream_.good();
}

bool ByteStreamRecorder::IsOpen() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return file_stream_.is_open();
}

void ByteStreamRecorder::Close() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (file_stream_.is_open()) {
    file_stream_.close();
  }
}

}  // namespace referee_node