#include "PicoColorSensor.h"

#include <atomic>
#include <thread>
#include <wpi/mutex.h>
#include <frc/SerialPort.h>
#include <wpi/StringExtras.h>
#include <wpi/SmallVector.h>
#include <frc/Timer.h>

using namespace pico;

struct ColorSensor::Impl {
 public:
  Impl() {
    thread = std::thread([&] { ThreadMain(); });
  }
  ~Impl() {
    threadRunning = false;
    thread.join();
  };

  void ThreadMain() {
    frc::SerialPort serialPort{115200, frc::SerialPort::kMXP};
    wpi::SmallVector<std::string_view, 16> splits;
    serialPort.SetTimeout(1_s);
    serialPort.EnableTermination('\n');
    char buf[257];
    while (threadRunning.load()) {
      size_t read = serialPort.Read(buf, sizeof(buf) - 1);
      if (read <= 0) {
        std::scoped_lock lock{mutex};
        this->has0 = false;
        this->has1 = false;
        continue;
      }
      if (!threadRunning.load()) {
        break;
      }

      // Trim trailing newline if exists
      if (buf[read - 1] == '\n') {
        read--;
      }

      if (read == 0) {
        continue;
      }

      if (debug.load()) {
        buf[read] = '\0';
        fmt::print("{}\n", (char*)buf);
      }

      splits.clear();
      wpi::split(std::string_view{buf, read}, splits, ',');
      if (splits.size() < 2) {
        continue;
      }

      bool hasColor0 =
          wpi::parse_integer<uint32_t>(splits[0], 10).value_or(0) != 0;
      bool hasColor1 =
          wpi::parse_integer<uint32_t>(splits[1], 10).value_or(0) != 0;

      RawColor color0;
      uint32_t prox0 = 0;
      if (hasColor0 && splits.size() >= 7) {
        color0.red = wpi::parse_integer<uint32_t>(splits[2], 10).value_or(0);
        color0.green = wpi::parse_integer<uint32_t>(splits[3], 10).value_or(0);
        color0.blue = wpi::parse_integer<uint32_t>(splits[4], 10).value_or(0);
        color0.ir = wpi::parse_integer<uint32_t>(splits[5], 10).value_or(0);
        prox0 = wpi::parse_integer<uint32_t>(splits[6], 10).value_or(0);
      }

      RawColor color1;
      uint32_t prox1 = 0;
      if (hasColor1 && splits.size() >= 12) {
        color1.red = wpi::parse_integer<uint32_t>(splits[7], 10).value_or(0);
        color1.green = wpi::parse_integer<uint32_t>(splits[8], 10).value_or(0);
        color1.blue = wpi::parse_integer<uint32_t>(splits[9], 10).value_or(0);
        color1.ir = wpi::parse_integer<uint32_t>(splits[10], 10).value_or(0);
        prox1 = wpi::parse_integer<uint32_t>(splits[11], 10).value_or(0);
      }

      auto ts = frc::Timer::GetFPGATimestamp();
      std::scoped_lock lock{mutex};
      this->has0 = hasColor0;
      this->has1 = hasColor1;
      this->lastReadTime = ts;
      if (hasColor0) {
        this->color0 = color0;
        this->prox0 = prox0;
      }
      if (hasColor1) {
        this->color1 = color1;
        this->prox1 = prox1;
      }
    }
  }

  std::atomic_bool threadRunning{true};
  std::thread thread;
  wpi::mutex mutex;
  RawColor color0;
  RawColor color1;
  uint32_t prox0 = 0;
  uint32_t prox1 = 0;
  bool has0 = false;
  bool has1 = false;
  std::atomic_bool debug{false};
  units::second_t lastReadTime = 0_s;
};

ColorSensor::ColorSensor() {
  pImpl = std::make_unique<Impl>();
}

ColorSensor::~ColorSensor() {}

bool ColorSensor::IsSensor0Connected() {
  std::scoped_lock lock{pImpl->mutex};
  return pImpl->has0;
}

bool ColorSensor::IsSensor1Connected() {
  std::scoped_lock lock{pImpl->mutex};
  return pImpl->has1;
}

ColorSensor::RawColor ColorSensor::GetRawColor0() {
  std::scoped_lock lock{pImpl->mutex};
  return pImpl->color0;
}

uint32_t ColorSensor::GetProximity0() {
  std::scoped_lock lock{pImpl->mutex};
  return pImpl->prox0;
}

ColorSensor::RawColor ColorSensor::GetRawColor1() {
  std::scoped_lock lock{pImpl->mutex};
  return pImpl->color1;
}

uint32_t ColorSensor::GetProximity1() {
  std::scoped_lock lock{pImpl->mutex};
  return pImpl->prox1;
}

units::second_t ColorSensor::GetLastReadTimestamp() {
  std::scoped_lock lock{pImpl->mutex};
  return pImpl->lastReadTime;
}

void ColorSensor::SetDebugPrints(bool debug) {
  pImpl->debug = debug;
}
