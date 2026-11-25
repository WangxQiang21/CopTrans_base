#pragma once

#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace bridge_take_trace {
struct Take {
  std::int64_t robot, steady_ns, realtime_ns, source_ns, received_ns;
  double vx, vy, wz;
};
struct Slot {
  std::uint64_t committed;
  Take take;
};
struct Journal {
  char magic[8];
  std::uint64_t capacity, claimed;
  Slot slots[65536];
};
static_assert(sizeof(Take) == 64 && sizeof(Slot) == 72 && offsetof(Journal, slots) == 24);
static_assert(__atomic_always_lock_free(sizeof(std::uint64_t), nullptr));

struct Store {
  Journal* journal{};
  char filename[4096]{};

  void open(const char* directory) noexcept {
    if (!directory) return;
    const int n = std::snprintf(filename, sizeof(filename), "%s/bridge_take_%d.journal", directory, getpid());
    if (n < 0 || static_cast<std::size_t>(n) >= sizeof(filename)) return;
    const int fd = ::open(filename, O_CREAT | O_EXCL | O_RDWR | O_CLOEXEC, 0600);
    if (fd < 0) return;
    if (posix_fallocate(fd, 0, sizeof(Journal)) == 0) {
      void* ptr = mmap(nullptr, sizeof(Journal), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_POPULATE, fd, 0);
      if (ptr != MAP_FAILED) {
        journal = static_cast<Journal*>(ptr);
        std::memcpy(journal->magic, "CTTAKE01", 8);
        journal->capacity = sizeof(journal->slots) / sizeof(Slot);
      }
    }
    close(fd);
  }

  void record(const Take& row) noexcept {
    if (!journal) return;
    const auto index = __atomic_fetch_add(&journal->claimed, 1, __ATOMIC_RELAXED);
    if (index >= journal->capacity) return;
    auto& slot = journal->slots[index];
    slot.take = row;
    // The runner can recover exactly the committed prefix after the bridge dies.
    __atomic_store_n(&slot.committed, index + 1, __ATOMIC_RELEASE);
  }

  void finish() noexcept {
    if (!journal) return;
    munmap(journal, sizeof(Journal));
    journal = nullptr;
  }
};
}  // namespace bridge_take_trace
