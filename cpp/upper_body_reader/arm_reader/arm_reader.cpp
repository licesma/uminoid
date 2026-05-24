#include "arm_reader.hpp"

#include <string>

namespace {

std::string format_line(int collection_id, const ArmLine& line) {
  std::string s = std::to_string(collection_id) + "," +
                  std::to_string(line.timestamp) + "," +
                  std::to_string(line.host_timestamp);
  for (const auto v : line.data) s += "," + std::to_string(v);
  return s;
}

std::string format_command_line(int collection_id, const ArmLine& line,
                                const ArmAngleConverter& converter,
                                bool from_left) {
  std::string s = std::to_string(collection_id) + "," +
                  std::to_string(line.timestamp) + "," +
                  std::to_string(line.host_timestamp);
  for (const double a : converter.convert(line, from_left))
    s += "," + std::to_string(a);
  return s;
}

}  // namespace

ArmReader::ArmReader(std::unique_ptr<SkeletonArm> arm,
                     const std::string& csv_path,
                     const std::function<void(const std::string&)>& raise_error,
                     const ArmAngleConverter* converter, bool from_left,
                     const std::string& command_csv_path)
    : arm_(std::move(arm)),
      raise_error_(raise_error),
      csv_(csv_path.empty() ? CsvSaver{}
                            : CsvSaver(csv_path, raw_arm_csv_header())),
      command_csv_(command_csv_path.empty()
                       ? CsvSaver{}
                       : CsvSaver(command_csv_path, arm_csv_header(from_left))),
      converter_(converter),
      from_left_(from_left) {
  if (!arm_) {
    stopped_ = true;
    return;
  }
  thread_ = std::thread(&ArmReader::read_loop, this);
}

ArmReader::~ArmReader() {
  stop();
}

ArmSnapshot ArmReader::snapshot_with_counter() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return {latest_, frame_seq_};
}

void ArmReader::stop() {
  if (arm_) arm_->Stop();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}


std::optional<ArmLine> ArmReader::wait_for_next() {
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait(lock, [&] { return stopped_ || frame_seq_ > consumed_seq_; });

  if (frame_seq_ > consumed_seq_) {
    consumed_seq_ = frame_seq_;
    return latest_;
  }
  return std::nullopt;
}


void ArmReader::collect_loop(const std::function<int()>&  collection_id,
                             const std::function<bool()>& stop,
                             const std::function<bool()>& pause) {
  while (!stop()) {
    auto reading = wait_for_next();
    if (!reading) break;
    if (!pause()) {
      csv_.write_line(format_line(collection_id(), *reading));
      if (converter_ && command_csv_)
        command_csv_.write_line(format_command_line(
            collection_id(), *reading, *converter_, from_left_));
    }
  }

  csv_.close();
  command_csv_.close();
}

void ArmReader::read_loop() {
  while (auto frame = arm_->GetNextLine(raise_error_)) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_ = *frame;
      ++frame_seq_;
    }
    cv_.notify_all();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
  }
  cv_.notify_all();
}
